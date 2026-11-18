// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__STATEFUL_FRICTION_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__STATEFUL_FRICTION_HPP_
#include "friction_compensation_controller/calibrated_friction.hpp"
#include "friction_compensation_controller/elastoplastic_replay.hpp"
#include "friction_compensation_controller/kinetic_curve.hpp"
#include "friction_compensation_controller/rest_dither.hpp"
#include "friction_compensation_controller/directional_load_law.hpp"
#include <optional>

namespace friction_compensation_controller
{
// Hand-guiding candidate, separate from the retired instantaneous sign(v) law.
// Stored identification coefficients are unchanged. The optional kinetic-curve
// experiment has a separately recorded output scale, requested for coast tests.
// Decay and damping are explicit controller design parameters, not friction fits.
// This state machine does not constitute a robot passivity certificate.
struct StatefulParameters
{
  double positive_yield, negative_yield, positive_viscous, negative_viscous;
  double stiffness, decay, damping, maximum_velocity;
  double load_transition_distance=0.;
  void validate() const
  {
    const double positive[] = {positive_yield, negative_yield, stiffness, decay};
    for (double value : positive) {
      if (!std::isfinite(value) || value <= 0.) {
        throw std::invalid_argument("Invalid stateful friction parameter");
      }
    }
    for (double value : {positive_viscous, negative_viscous, damping, maximum_velocity, load_transition_distance}) {
      if (!std::isfinite(value) || value < 0.) {
        throw std::invalid_argument("Invalid stateful friction rate parameter");
      }
    }
  }
};

class StatefulFriction
{
public:
  enum FaultReason : unsigned int
  {
    NoFault = 0, InvalidSample = 1, SpeedEnvelope = 2,
    PositionStep = 4, InvalidRateLaw = 8, InvalidObservation = 16
  };
  explicit StatefulFriction(const StatefulParameters & parameters,
    std::optional<DirectionalKineticCurve> curve = std::nullopt, double output_scale = 1.,
    std::optional<RestDither> dither = std::nullopt, bool dither_with_elastic = false,
    std::optional<DirectionalLoadLaw> load_law = std::nullopt)
  : parameters_(parameters), state_(parameters.stiffness, parameters.decay, parameters.damping),
    curve_(std::move(curve)), output_scale_(output_scale), dither_(std::move(dither)),
    dither_with_elastic_(dither_with_elastic), load_law_(std::move(load_law))
  {
    parameters_.validate();
    if(parameters_.load_transition_distance>0. && !load_law_)
    {throw std::invalid_argument("Load transition distance requires a load law");}
    if (load_law_ && (curve_ || (dither_ && !dither_with_elastic_) || parameters_.damping != 0. ||
      parameters_.positive_viscous != 0. || parameters_.negative_viscous != 0. ||
      std::abs(load_law_->yield(true, 0.)-parameters_.positive_yield) > 1e-10 ||
      std::abs(load_law_->yield(false, 0.)-parameters_.negative_yield) > 1e-10))
    {throw std::invalid_argument("Loaded experiment requires matching yields and elastic mode for rest dither");}
    if (!std::isfinite(output_scale_) || output_scale_ <= 0. || output_scale_ > 1. ||
      (!curve_ && !dither_with_elastic_ && !load_law_ && output_scale_ != 1.))
    {throw std::invalid_argument("Invalid kinetic curve output scale");}
    if (curve_ && (std::abs(curve_->yield(true)-parameters_.positive_yield) > 1e-10 ||
      std::abs(curve_->yield(false)-parameters_.negative_yield) > 1e-10 ||
      parameters_.damping != 0.))
    {throw std::invalid_argument("Coasting curve requires matching yields and zero extra damping");}
    if (dither_ && ((curve_ && (!dither_with_elastic_ || !curve_->holds_endpoint())) ||
      (!dither_with_elastic_ && output_scale_ != 1.) || parameters_.damping != 0. ||
      parameters_.positive_viscous != 0. || parameters_.negative_viscous != 0.))
    {throw std::invalid_argument("Rest dither requires constant curve extension and no added damping/slopes");}
    if (dither_with_elastic_ && !dither_) {
      throw std::invalid_argument("Elastic/dither combination requires a waveform");
    }
  }

  void reset()
  {
    state_.reset(); fault_ = false; reason_ = NoFault; primed_ = false;
    if (dither_) {dither_->reset();}
    diagnostics_ = {}; output_ = {}; dither_output_ = {};
  }
  bool fault() const {return fault_;}
  unsigned int fault_reason() const {return reason_;}
  const FrictionDiagnostics & diagnostics() const {return diagnostics_;}
  const ElastoplasticReplay::Output & components() const {return output_;}
  const RestDither::Output & dither_components() const {return dither_output_;}

  double step(double q, double velocity, double dt, bool observation_ready, bool enabled,
    double nominal_gravity = 0.)
  {
    diagnostics_ = {}; output_ = {}; dither_output_ = {};
    // A bad timing/state sample latches assistance OFF. The owner can continue
    // computing valid original gravity; no motor-disable operation occurs here.
    if (!std::isfinite(q) || !std::isfinite(dt) || dt <= 0. || dt > .02 ||
      (observation_ready && !std::isfinite(velocity)) ||
      (load_law_ && !std::isfinite(nominal_gravity)))
    {return trip(InvalidSample);}
    if (fault_) {return trip(reason_);}
    if (!enabled || !observation_ready) {
      state_.reset(); primed_ = false;
      if (dither_) {dither_->reset();}
      diagnostics_.flags = !enabled ? Disabled : Rest; return 0.;
    }
    // Zero explicitly disables the host speed and position-increment trips.
    // Firmware limits are independent; no output cap or slew limiter is added.
    if (parameters_.maximum_velocity > 0. &&
      std::abs(velocity) > parameters_.maximum_velocity) {
      return trip(SpeedEnvelope);
    }
    if (parameters_.maximum_velocity > 0. && primed_ && std::abs(q-previous_position_) >
      parameters_.maximum_velocity*dt+4.*.000011984224905356572) {
      return trip(PositionStep);
    }
    previous_position_ = q; primed_ = true;
    if (dither_) {
      try {dither_output_ = dither_->step(velocity, dt);}
      catch (const std::invalid_argument &) {return trip(InvalidSample);}
      if (!dither_with_elastic_) {
        // Explicit dither-only mode: no elastic, kinetic-rate, or damping output.
        diagnostics_.requested = diagnostics_.target = diagnostics_.applied = dither_output_.torque;
        output_.total = dither_output_.torque;
        if (output_.total == 0.) {diagnostics_.flags = Rest;}
        return output_.total;
      }
    }
    const double slope = velocity >= 0. ? parameters_.positive_viscous : parameters_.negative_viscous;
    const double positive_yield = load_law_ ? load_law_->yield(true, nominal_gravity,q) : parameters_.positive_yield;
    const double negative_yield = load_law_ ? load_law_->yield(false, nominal_gravity,q) : parameters_.negative_yield;
    diagnostics_.model_magnitude = load_law_ ? load_law_->magnitude(velocity, nominal_gravity,q) :
      curve_ ? curve_->magnitude(velocity) :
      (velocity >= 0. ? parameters_.positive_yield : parameters_.negative_yield) +
      slope*std::abs(velocity);
    try {
      output_ = state_.step(q, velocity, dt, positive_yield,
        negative_yield, [this, nominal_gravity,q](double vp) {
          if (load_law_) {return load_law_->rate(vp, nominal_gravity,q);}
          if (curve_) {return curve_->rate(vp);}
          return (vp >= 0. ? parameters_.positive_viscous : parameters_.negative_viscous)*vp;
        },true,parameters_.load_transition_distance>0. ?
        std::max(positive_yield,negative_yield)/parameters_.load_transition_distance : 0.);
    } catch (const std::invalid_argument &) {
      return trip(InvalidRateLaw);
    }
    if (dither_with_elastic_) {
      // Crossfade the rest waveform as the same friction level is represented
      // by the elastic state. Do not count the full rest and sliding terms twice.
      const double yield = output_.elastic >= 0. ? positive_yield : negative_yield;
      const double rest_fraction = std::clamp(1.-std::abs(output_.elastic)/yield, 0., 1.);
      dither_output_.torque *= rest_fraction;
      dither_output_.envelope *= rest_fraction;
    }
    diagnostics_.requested = output_.elastic + output_.rate_term + dither_output_.torque;
    output_.elastic *= output_scale_;
    output_.rate_term *= output_scale_;
    output_.damping *= output_scale_;
    output_.total *= output_scale_;
    output_.total += dither_output_.torque;
    diagnostics_.target = output_.total;
    diagnostics_.applied = output_.total;
    if (output_.total == 0.) {diagnostics_.flags = Rest;}
    return output_.total;
  }
private:
  double trip(unsigned int reason)
  {
    // Retain the cause after the triggering sample and through OFF/ON requests.
    if (!fault_) {reason_ = reason;}
    fault_ = true; state_.reset(); primed_ = false;
    if (dither_) {dither_->reset();}
    dither_output_ = {};
    diagnostics_.flags = Fault;
    if (reason_ & (SpeedEnvelope | PositionStep)) {diagnostics_.flags |= Overspeed;}
    return 0.;
  }
  StatefulParameters parameters_;
  ElastoplasticReplay state_;
  std::optional<DirectionalKineticCurve> curve_;
  double output_scale_;
  std::optional<RestDither> dither_;
  RestDither::Output dither_output_;
  bool dither_with_elastic_;
  std::optional<DirectionalLoadLaw> load_law_;
  bool fault_ = false;
  unsigned int reason_ = NoFault;
  bool primed_ = false;
  double previous_position_ = 0.;
  FrictionDiagnostics diagnostics_;
  ElastoplasticReplay::Output output_;
};
}  // namespace friction_compensation_controller
#endif
