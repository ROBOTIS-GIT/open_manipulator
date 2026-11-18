// Known-input, finite rest-response experiment. Not a freedrive controller.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__SINE_PROBE_SEQUENCE_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__SINE_PROBE_SEQUENCE_HPP_
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace friction_compensation_controller
{
class SineProbeSequence
{
public:
  struct Block {double amplitude, frequency; int repeat;};
  struct Output {double command = 0.; int phase = -1; bool done = false;};
  SineProbeSequence(const std::vector<double> & amplitudes,
    const std::vector<double> & frequencies, int repeats,
    double active_seconds = 6., double rest_seconds = 2., double ramp_seconds = .5)
  : active_(active_seconds), rest_(rest_seconds), ramp_(ramp_seconds)
  {
    if (amplitudes.empty() || amplitudes.size() > 10 || frequencies.empty() ||
      frequencies.size() > 3 || repeats < 2 || repeats > 3 ||
      !std::isfinite(active_+rest_+ramp_) || active_ < 4. || active_ > 10. ||
      rest_ < 1. || rest_ > 5. || ramp_ < .25 || ramp_ > 1.)
    {throw std::invalid_argument("Invalid finite sine experiment schedule");}
    double previous = 0.;
    for (double a : amplitudes) {
      if (!std::isfinite(a) || a <= previous || a > 3.) {
        throw std::invalid_argument("Invalid J6 probe amplitudes");
      }
      previous = a;
      for (double f : frequencies) {
        if (!std::isfinite(f) || f < 5. || f > 20. || (active_-2*ramp_)*f < 10.) {
          throw std::invalid_argument("Invalid sine probe frequency/observation duration");
        }
        for (int repeat = 0; repeat < repeats; ++repeat) {blocks_.push_back({a, f, repeat});}
      }
    }
    if (duration() > 600.) {throw std::invalid_argument("Sine probe exceeds ten minutes");}
  }
  double duration() const {return rest_+blocks_.size()*(active_+rest_);}
  const std::vector<Block> & blocks() const {return blocks_;}
  Output sample(double t) const
  {
    if (!std::isfinite(t) || t < 0.) {throw std::invalid_argument("Invalid probe time");}
    if (t >= duration()) {return {0., -1, true};}
    if (t < rest_) {return {};}
    const auto index = static_cast<size_t>((t-rest_)/(active_+rest_));
    const double local = t-rest_-index*(active_+rest_);
    if (local >= active_) {return {0., -1, false};}
    const auto & block = blocks_[index];
    const double envelope = std::clamp(std::min(local, active_-local)/ramp_, 0., 1.);
    constexpr double two_pi = 6.2831853071795864769;
    return {block.amplitude*envelope*std::sin(two_pi*block.frequency*local),
      static_cast<int>(index), false};
  }
private:
  double active_, rest_, ramp_;
  std::vector<Block> blocks_;
};
}  // namespace friction_compensation_controller
#endif
