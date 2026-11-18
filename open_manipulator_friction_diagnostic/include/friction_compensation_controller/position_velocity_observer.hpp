// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__POSITION_VELOCITY_OBSERVER_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__POSITION_VELOCITY_OBSERVER_HPP_
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace friction_compensation_controller
{
// Fixed storage, no allocation in update. Estimates the centre of a trailing
// window, NOT current velocity: callers must account for age at reversals.
class PositionVelocityObserver
{
public:
  static constexpr size_t joints = 6;
  static constexpr size_t capacity = 256;
  using Position = std::array<double, joints>;
  enum class Status {Warming, Valid, InvalidInput, ClockReset, Gap, Capacity};
  struct Result
  {
    bool valid = false;
    double age = std::numeric_limits<double>::quiet_NaN();
    Position velocity{}, quantization_bound{};
    Status status = Status::Warming;
    Result()
    {
      velocity.fill(std::numeric_limits<double>::quiet_NaN());
      quantization_bound.fill(std::numeric_limits<double>::quiet_NaN());
    }
  };
  explicit PositionVelocityObserver(double window = .05, double maximum_gap = .05,
    double position_step = .000011984224905356572)
  : window_(window), gap_(maximum_gap), step_(position_step)
  {
    if (!std::isfinite(window) || !std::isfinite(maximum_gap) ||
      !std::isfinite(position_step) || window <= 0. || maximum_gap <= 0. || position_step < 0.)
    {
      throw std::invalid_argument("Invalid position velocity observer settings");
    }
  }
  void reset() {first_ = 0; size_ = 0;}
  Result update(double stamp, const Position & q)
  {
    Result out;
    bool finite = std::isfinite(stamp);
    for (double x : q) {finite = finite && std::isfinite(x);}
    if (!finite) {reset(); out.status = Status::InvalidInput; return out;}
    if (size_ && stamp <= at(size_-1).t) {
      reset(); out.status = Status::ClockReset; return out;
    }
    if (size_ && stamp-at(size_-1).t > gap_) {reset(); out.status = Status::Gap;}
    // Keep a sample bracketing the left edge, matching the Python observer.
    while (size_ > 1 && at(1).t <= stamp-window_) {pop();}
    if (size_ == capacity) {reset(); out.status = Status::Capacity; return out;}
    samples_[(first_+size_)%capacity] = {stamp,q}; ++size_;
    if (size_ < 5 || stamp-at(0).t < .9*window_) {return out;}
    double mean = 0.;
    for (size_t i = 0; i < size_; ++i) {mean += at(i).t-stamp;}
    mean /= size_;
    double denominator = 0., absolute = 0.; Position numerator{};
    for (size_t i = 0; i < size_; ++i) {
      const double x = at(i).t-stamp-mean;
      denominator += x*x; absolute += std::abs(x);
      for (size_t j = 0; j < joints; ++j) {numerator[j] += x*(at(i).q[j]-q[j]);}
    }
    if (denominator <= 1e-14) {return out;}
    for (size_t j = 0; j < joints; ++j) {
      out.velocity[j] = numerator[j]/denominator;
      out.quantization_bound[j] = absolute/denominator*step_;
    }
    out.valid = true; out.status = Status::Valid; out.age = -mean;
    return out;
  }
private:
  struct Sample {double t = 0.; Position q{};};
  std::array<Sample, capacity> samples_{};
  double window_, gap_, step_;
  size_t first_ = 0, size_ = 0;
  const Sample & at(size_t i) const {return samples_[(first_+i)%capacity];}
  void pop() {first_ = (first_+1)%capacity; --size_;}
};
}  // namespace friction_compensation_controller
#endif
