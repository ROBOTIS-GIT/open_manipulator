// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__CALIBRATED_FRICTION_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__CALIBRATED_FRICTION_HPP_
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <optional>
#include "friction_compensation_controller/modified_lugre.hpp"

namespace friction_compensation_controller
{
enum FrictionFlag : uint32_t
{
  Disabled = 1, Fault = 2, LoadOutsideRange = 4, Rest = 8, Overspeed = 16,
  LowSpeedFade = 32, HighSpeedFade = 64, TorqueCap = 128, Slew = 256, Reversal = 512,
  OutsideIdentificationRange = 1024, AccelerationTrip = 2048, LuGreDampingBound = 4096
};

struct FrictionDiagnostics
{
  // NaN means the gate prevented model evaluation, not that the fitted model is zero.
  double model_magnitude = std::numeric_limits<double>::quiet_NaN();
  double requested = 0.;
  double target = 0.;
  double applied = 0.;
  uint32_t flags = 0;
};

struct FrictionWindow
{
  uint64_t cycles = 0, assist_cycles = 0;
  double sum_abs = 0., peak_abs = 0.;
  uint32_t flags = 0;
  void add(const FrictionDiagnostics & d)
  {
    ++cycles;
    assist_cycles += d.applied != 0.;
    sum_abs += std::abs(d.applied);
    peak_abs = std::max(peak_abs, std::abs(d.applied));
    flags |= d.flags;
  }
};

// Bounded velocity/load-dependent assistance, NOT a LuGre/passivity guarantee.
// Fs is a curve parameter; at rest no intended direction is inferred.
struct FrictionParameters
{
  double fc, fs, vs, viscous, gain, maximum, slew, deadband, v_min, v_max;
  double load_gain, load_min, load_max;
  bool enforce_load_bounds = true;
  // Explicit research mode: the identified function is passed through unchanged.
  // It does not assert passivity or validate extrapolation beyond the experiment.
  bool unscaled = false;
  double load_peak = 0., load_vs = .01;
  double position_peak = 0., position_order = 1., position_phase = 0.;
  double context_peak = 0., context_vs = .01, context_power = 2.;
  bool lugre = false;
  double sigma0 = 0., sigma1 = 0., smoothing_k = 0., maximum_acceleration = 0.;
  void validate() const
  {
    const double values[] = {fc, fs, vs, viscous, gain, maximum, slew,
      deadband, v_min, v_max, load_gain, load_min, load_max,
      load_peak, load_vs, position_peak, position_order, context_peak, context_vs, context_power};
    for (double value : values) {
      if (!std::isfinite(value) || value < 0) {
        throw std::invalid_argument("Nonfinite/negative calibrated friction parameter");
      }
    }
    if (fs < fc || vs <= 0 || gain <= 0 || gain > 1. ||
      (!unscaled && (gain > .7 || maximum <= 0 || slew <= 0 || deadband <= 0)) ||
      (unscaled && (gain != 1. || maximum != 0. || slew != 0. || deadband != 0. ||
      enforce_load_bounds)) || v_min <= deadband || v_max <= v_min || load_max <= load_min ||
      (!unscaled && (load_gain > 1. || (!enforce_load_bounds && load_gain != 0.) ||
      load_peak != 0. || position_peak != 0. || context_peak != 0.)) ||
      context_vs <= 0. || (context_power != 2. && context_power != 4. && context_power != 6.) ||
      load_vs <= 0. || position_order <= 0. ||
      !std::isfinite(position_phase))
    {
      throw std::invalid_argument("Invalid calibrated friction ranges");
    }
    if (lugre && (!unscaled || load_gain != 0. || load_peak != 0. ||
      position_peak != 0. || context_peak != 0. ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration <= 0.))
    {
      throw std::invalid_argument("LuGre research mode requires separate velocity-only parameters");
    }
  }
};

class CalibratedFriction
{
public:
  // Not a ROS parameter: raw static laws are retained only for offline reproduction.
  enum class Use {Control, OfflineReplay};
  explicit CalibratedFriction(
    const FrictionParameters & parameters, Use use = Use::Control) : p_(parameters)
  {
    p_.validate();
    if (p_.unscaled && !p_.lugre && use != Use::OfflineReplay) {
      throw std::invalid_argument(
        "Static unscaled friction is an offline model, not a qualified freedrive controller");
    }
    if (p_.lugre) {lugre_.emplace(p_.fc, p_.fs, p_.vs, p_.viscous,
      p_.sigma0, p_.sigma1, p_.smoothing_k);}
  }
  void reset() {last_ = 0.; fault_ = false; diagnostics_ = {}; have_velocity_ = false;
    if (lugre_) {lugre_->reset();}}
  bool fault() const {return fault_;}
  const FrictionDiagnostics & diagnostics() const {return diagnostics_;}
  double step(double q, double velocity, double load, double period, bool enabled,
    double context_q = 0.)
  {
    diagnostics_ = {};
    if (!std::isfinite(q) || !std::isfinite(velocity) || !std::isfinite(load) || !std::isfinite(period) ||
      period <= 0 || period > .02 || !std::isfinite(context_q))
    {
      fault_ = true;
    }
    // Removing assistance is immediate on a stop/fault. No stale signed torque at rest.
    const double speed = std::abs(velocity);
    load = std::abs(load);
    if (lugre_) {
      if (speed > p_.v_max) {fault_ = true; diagnostics_.flags |= Overspeed;}
      if (have_velocity_ && std::abs(velocity-previous_velocity_)/period > p_.maximum_acceleration) {
        fault_ = true; diagnostics_.flags |= AccelerationTrip;
      }
      if (fault_) {diagnostics_.flags |= Fault;}
      if (!enabled) {diagnostics_.flags |= Disabled;}
      if (fault_ || !enabled) {
        lugre_->reset(); last_ = 0.; have_velocity_ = false; return 0.;
      }
      // Prime the velocity history without issuing a first-sample torque jump.
      if (!have_velocity_) {
        previous_velocity_ = velocity; have_velocity_ = true; return 0.;
      }
      last_ = lugre_->step((velocity+previous_velocity_)/2., period);
      previous_velocity_ = velocity;
      if (!std::isfinite(last_)) {fault_ = true; diagnostics_.flags |= Fault; last_ = 0.;}
      if (lugre_->damping() < p_.sigma1) {diagnostics_.flags |= LuGreDampingBound;}
      diagnostics_.model_magnitude = std::abs(last_);
      diagnostics_.requested = diagnostics_.target = diagnostics_.applied = last_;
      return last_;
    }
    if (p_.unscaled) {
      if (fault_) {diagnostics_.flags |= Fault;}
      if (!enabled) {diagnostics_.flags |= Disabled;}
      if (speed == 0.) {diagnostics_.flags |= Rest;}
      if (speed < p_.v_min || speed > p_.v_max) {
        diagnostics_.flags |= OutsideIdentificationRange;
      }
      if (fault_ || !enabled || speed == 0.) {last_ = 0.; return 0.;}
      const double ratio = speed / p_.vs;
      const double harmonic = std::sin(p_.position_order*q+p_.position_phase);
      const double load_ratio = speed/p_.load_vs;
      const double context_ratio = speed/p_.context_vs;
      const double magnitude = p_.fc +
        (p_.fs-p_.fc+p_.position_peak*harmonic*harmonic)*std::exp(-ratio*ratio) +
        p_.viscous*speed + (p_.load_gain+p_.load_peak*std::exp(-load_ratio*load_ratio))*load +
        p_.context_peak*load*std::pow(std::abs(std::sin(context_q)),p_.context_power)*
        std::exp(-context_ratio*context_ratio);
      if (!std::isfinite(magnitude)) {
        fault_ = true; diagnostics_.flags |= Fault; last_ = 0.; return 0.;
      }
      diagnostics_.model_magnitude = magnitude;
      diagnostics_.requested = diagnostics_.target = diagnostics_.applied =
        std::copysign(magnitude, velocity);
      last_ = diagnostics_.applied;
      return last_;
    }
    if (fault_) {diagnostics_.flags |= Fault;}
    if (!enabled) {diagnostics_.flags |= Disabled;}
    if (p_.enforce_load_bounds && (load < p_.load_min || load > p_.load_max)) {
      diagnostics_.flags |= LoadOutsideRange;
    }
    if (speed <= p_.deadband) {diagnostics_.flags |= Rest;}
    if (speed > p_.v_max) {diagnostics_.flags |= Overspeed;}
    if (fault_ || !enabled || (p_.enforce_load_bounds && (load < p_.load_min || load > p_.load_max)) ||
      speed <= p_.deadband || speed > p_.v_max)
    {
      last_ = 0.;
      return 0.;
    }
    const double observed_speed = std::max(speed, p_.v_min);
    const double ratio = observed_speed / p_.vs;
    diagnostics_.model_magnitude = p_.fc + (p_.fs-p_.fc)*std::exp(-ratio*ratio) +
      p_.viscous*observed_speed + p_.load_gain*load;
    double target = p_.gain * diagnostics_.model_magnitude;
    diagnostics_.requested = std::copysign(target, velocity);
    // Do not extrapolate the fitted low-speed curve below the measured range.
    target *= std::clamp((speed-p_.deadband)/(p_.v_min-p_.deadband), 0., 1.);
    if (speed < p_.v_min) {diagnostics_.flags |= LowSpeedFade;}
    // Smooth fade near the upper edge of the measured range.
    target *= std::clamp((p_.v_max-speed)/(.2*p_.v_max), 0., 1.);
    if (speed > .8*p_.v_max) {diagnostics_.flags |= HighSpeedFade;}
    if (target > p_.maximum) {diagnostics_.flags |= TorqueCap;}
    target = std::copysign(std::min(target, p_.maximum), velocity);
    diagnostics_.target = target;
    if (last_*target < 0) {last_ = 0.; diagnostics_.flags |= Reversal;}
    if (std::abs(target-last_) > p_.slew*period) {diagnostics_.flags |= Slew;}
    last_ += std::clamp(target-last_, -p_.slew*period, p_.slew*period);
    diagnostics_.applied = last_;
    return last_;
  }
private:
  FrictionParameters p_;
  double last_ = 0.;
  bool fault_ = false;
  FrictionDiagnostics diagnostics_;
  std::optional<ModifiedLuGre> lugre_;
  double previous_velocity_ = 0.;
  bool have_velocity_ = false;
};
}  // namespace friction_compensation_controller
#endif
