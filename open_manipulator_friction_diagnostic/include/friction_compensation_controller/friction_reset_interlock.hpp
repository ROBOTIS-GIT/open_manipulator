// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__FRICTION_RESET_INTERLOCK_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__FRICTION_RESET_INTERLOCK_HPP_
#include <array>
#include <cmath>

namespace friction_compensation_controller
{
// RT-owned rest evidence for an explicitly requested reset, never an automatic
// re-enable. Checks all six joints; a stationary assisted wrist alone is not enough.
class FrictionResetInterlock
{
public:
  using Sample = std::array<double, 6>;
  static constexpr double dwell_s = .5;
  static constexpr double maximum_speed = .003;
  static constexpr double maximum_excursion = 4.*.000011984224905356572;
  void reset() {primed_ = false; elapsed_ = 0.;}
  bool update(const Sample & position, const Sample & velocity, double dt,
    bool valid, bool assistance_off)
  {
    if (!valid || !assistance_off || !std::isfinite(dt) || dt <= 0. || dt > .02) {
      reset(); return false;
    }
    for (size_t i = 0; i < position.size(); ++i) {
      if (!std::isfinite(position[i]) || !std::isfinite(velocity[i]) ||
        std::abs(velocity[i]) > maximum_speed)
      {reset(); return false;}
    }
    if (!primed_) {anchor_ = position; primed_ = true; elapsed_ = 0.; return false;}
    for (size_t i = 0; i < position.size(); ++i) {
      if (std::abs(position[i]-anchor_[i]) > maximum_excursion) {
        anchor_ = position; elapsed_ = 0.; return false;
      }
    }
    elapsed_ = std::fmin(dwell_s, elapsed_+dt);
    return elapsed_ >= dwell_s;
  }
private:
  Sample anchor_{};
  bool primed_ = false;
  double elapsed_ = 0.;
};
}  // namespace friction_compensation_controller
#endif
