// Diagnostic pose validation; no change to the runtime friction law or waveform.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__LOADED_REST_POSE_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__LOADED_REST_POSE_HPP_
#include <cmath>
#include <vector>
namespace friction_compensation_controller
{
// The supervisor supplies this exact pose after its geometry review. Only the
// stationary J2 experiment's centre changes; its motion bounds stay unchanged.
inline bool valid_loaded_rest_pose(const std::vector<double> & reviewed,
  const std::vector<double> & expected, bool gravity, bool deferred, int target)
{
  if(!gravity || !deferred || target!=1 || reviewed.size()!=6 || expected.size()!=6)return false;
  for(size_t j=0;j<6;++j)
    if(!std::isfinite(reviewed[j]) || !std::isfinite(expected[j]) ||
       std::abs(reviewed[j]-expected[j])>1e-10 || (j!=1 && std::abs(expected[j])>=.001))return false;
  return true;
}
}
#endif
