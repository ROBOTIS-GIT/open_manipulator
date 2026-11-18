#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace friction_compensation_controller
{
struct EffortReference
{
  std::array<double, 6> values{};
  int64_t stamp_ns{0};
  uint64_t sequence{0};
};

inline bool valid_effort_reference(const EffortReference & reference)
{
  for (double value : reference.values) {if (!std::isfinite(value)) {return false;}}
  return reference.stamp_ns > 0;
}

class EffortReferenceGate
{
public:
  // RT owner consumes one message once, for all six joints together. A gate
  // closure or stale/invalid input also consumes it, preventing later replay.
  std::array<double, 6> consume(const EffortReference & reference,
    int64_t now_ns, int64_t period_ns, bool enabled)
  {
    if (reference.sequence <= consumed_) {return {};}
    consumed_ = reference.sequence;
    const auto age = now_ns - reference.stamp_ns;
    if (!enabled || !valid_effort_reference(reference) || period_ns <= 0 ||
      age < 0 || age > period_ns) {return {};}
    return reference.values;
  }

private:
  uint64_t consumed_{0};
};
}  // namespace friction_compensation_controller
