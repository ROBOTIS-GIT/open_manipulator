// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__REST_DITHER_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__REST_DITHER_HPP_
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace friction_compensation_controller
{
// Isolated rest-friction experiment, not a passivity or hardware qualification.
// Uses physical seconds and Nm, independent of gravity torque and loop rate.
// Velocity schedules THIS waveform; it never trips the robot or caps its motion.
struct RestDitherParameters
{
  double amplitude, frequency, full_velocity, zero_velocity, ramp_seconds;
  void validate() const
  {
    for (double x : {amplitude, frequency, zero_velocity, ramp_seconds}) {
      if (!std::isfinite(x) || x <= 0.) {throw std::invalid_argument("Invalid rest dither parameter");}
    }
    if (!std::isfinite(full_velocity) || full_velocity < 0. || full_velocity >= zero_velocity) {
      throw std::invalid_argument("Invalid rest dither velocity schedule");
    }
  }
};

class RestDither
{
public:
  struct Output {double torque = 0., envelope = 0.;};
  explicit RestDither(RestDitherParameters parameters) : p_(parameters) {p_.validate();}
  void reset() {phase_ = 0.; envelope_ = 0.;}
  Output step(double velocity, double dt)
  {
    if (!std::isfinite(velocity) || !std::isfinite(dt) || dt <= 0. || dt > .02 ||
      p_.frequency*dt > .25)
    {reset(); throw std::invalid_argument("Invalid rest dither sample");}
    const double x = std::clamp((std::abs(velocity)-p_.full_velocity) /
      (p_.zero_velocity-p_.full_velocity), 0., 1.);
    const double target = 1.-x*x*(3.-2.*x);
    // Remove the rest waveform as motion develops. Its return is gradual.
    envelope_ = std::min(target, envelope_+dt/p_.ramp_seconds);
    constexpr double two_pi = 6.2831853071795864769;
    phase_ = std::fmod(phase_+two_pi*p_.frequency*dt, two_pi);
    return {p_.amplitude*envelope_*std::sin(phase_), envelope_};
  }
private:
  RestDitherParameters p_;
  double phase_ = 0., envelope_ = 0.;
};
}  // namespace friction_compensation_controller
#endif
