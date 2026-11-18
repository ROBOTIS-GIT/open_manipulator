#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace omy_joint_impedance
{
using Vector = std::array<double, 6>;

struct Gains
{
  Vector stiffness{}, damping{}, integral_gain{}, integral_limit{};
  double integral_leak{0.};
};

struct Output
{
  Vector error{}, spring{}, damping{}, integral{}, effort{};
};

class ImpedanceLaw
{
public:
  bool configure(const Gains & gains)
  {
    for (const auto * values : {&gains.stiffness, &gains.damping,
      &gains.integral_gain, &gains.integral_limit})
    {
      for (double value : *values) {
        if (!std::isfinite(value) || value < 0.) {return false;}
      }
    }
    if (!std::isfinite(gains.integral_leak) || gains.integral_leak < 0.) {return false;}
    for (size_t i = 0; i < 6; ++i) {
      if (gains.integral_gain[i] > 0. && gains.integral_limit[i] <= 0.) {return false;}
    }
    gains_ = gains;
    integral_.fill(0.);
    return true;
  }

  bool activate(const Vector & position)
  {
    for (double value : position) {if (!std::isfinite(value)) {return false;}}
    reference_ = position;
    integral_.fill(0.);
    active_ = true;
    return true;
  }

  bool step(const Vector & position, const Vector & velocity, double dt, bool ready, Output & out)
  {
    out = {};
    if (!active_ || !std::isfinite(dt) || dt <= 0.) {return false;}
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(position[i]) || !std::isfinite(velocity[i])) {return false;}
    }
    if (!ready) {integral_.fill(0.); return true;}
    for (size_t i = 0; i < 6; ++i) {
      out.error[i] = reference_[i] - position[i];
      out.spring[i] = gains_.stiffness[i] * out.error[i];
      out.damping[i] = -gains_.damping[i] * velocity[i];
      // Integrate torque directly, with no separate unbounded error accumulator.
      if (gains_.integral_gain[i] == 0.) {
        integral_[i] = 0.;
      } else {
        integral_[i] = std::clamp(integral_[i] + dt *
          (gains_.integral_gain[i] * out.error[i] - gains_.integral_leak * integral_[i]),
          -gains_.integral_limit[i], gains_.integral_limit[i]);
      }
      out.integral[i] = integral_[i];
      out.effort[i] = out.spring[i] + out.damping[i] + out.integral[i];
      if (!std::isfinite(out.effort[i])) {return false;}
    }
    return true;
  }

  const Vector & reference() const {return reference_;}

  bool set_reference(const Vector & position)
  {
    if (!active_) {return false;}
    for (double value : position) {if (!std::isfinite(value)) {return false;}}
    reference_ = position;
    return true;
  }

private:
  Gains gains_;
  Vector reference_{}, integral_{};
  bool active_{false};
};
}  // namespace omy_joint_impedance
