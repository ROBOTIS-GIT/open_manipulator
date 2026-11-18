// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__KINETIC_CURVE_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__KINETIC_CURVE_HPP_
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace friction_compensation_controller
{
// A steady kinetic magnitude, not a direction detector or breakaway model.
// Allocation/validation only at configuration; evaluation does not allocate.
// An explicit endpoint policy distinguishes historical secant extrapolation
// from a constant extension. Neither policy stops or limits robot motion.
class DirectionalKineticCurve
{
public:
  DirectionalKineticCurve(std::vector<double> speed,
    std::vector<double> positive, std::vector<double> negative, bool hold_endpoint = false)
  : speed_(std::move(speed)), positive_(std::move(positive)), negative_(std::move(negative)),
    hold_endpoint_(hold_endpoint)
  {
    if (speed_.size() < 2 || positive_.size() != speed_.size() ||
      negative_.size() != speed_.size())
    {throw std::invalid_argument("Kinetic curve dimensions");}
    for (size_t i = 0; i < speed_.size(); ++i) {
      if (!std::isfinite(speed_[i]) || speed_[i] <= 0. ||
        !std::isfinite(positive_[i]) || positive_[i] <= 0. ||
        !std::isfinite(negative_[i]) || negative_[i] <= 0. ||
        (i && (speed_[i] <= speed_[i-1] || positive_[i] < positive_[i-1] ||
        negative_[i] < negative_[i-1])))
      {throw std::invalid_argument("Invalid monotone kinetic curve");}
    }
  }
  double yield(bool positive) const {return (positive ? positive_ : negative_).front();}
  bool holds_endpoint() const {return hold_endpoint_;}
  double magnitude(double velocity) const
  {
    if (!std::isfinite(velocity)) {throw std::invalid_argument("Invalid kinetic velocity");}
    const auto & values = velocity >= 0. ? positive_ : negative_;
    const double speed = std::abs(velocity);
    if (speed <= speed_.front()) {return values.front();}
    if (hold_endpoint_ && speed >= speed_.back()) {return values.back();}
    size_t i = 1;
    while (i+1 < speed_.size() && speed > speed_[i]) {++i;}
    return values[i-1] + (values[i]-values[i-1]) *
           (speed-speed_[i-1])/(speed_[i]-speed_[i-1]);
  }
  double rate(double plastic_velocity) const
  {
    if (plastic_velocity == 0.) {return 0.;}
    return std::copysign(magnitude(plastic_velocity)-yield(plastic_velocity >= 0.),
      plastic_velocity);
  }
private:
  std::vector<double> speed_, positive_, negative_;
  bool hold_endpoint_;
};
}  // namespace friction_compensation_controller
#endif
