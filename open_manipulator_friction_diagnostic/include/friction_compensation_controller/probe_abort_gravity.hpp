#ifndef GRAVITY_COMPENSATION_CONTROLLER__PROBE_ABORT_GRAVITY_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__PROBE_ABORT_GRAVITY_HPP_

namespace friction_compensation_controller {
// Finite diagnostics retain known gravity on motion-bound or sequence abort
// until the supervisor completes torque OFF. Bad state, timing, device, write
// and observer faults must never enter this path. This is not friction assist.
inline bool probe_abort_keeps_gravity(bool finite_gravity_diagnostic, bool deferred, int fault)
{
  return finite_gravity_diagnostic && deferred && (fault==3 || fault==4 || fault==11);
}
}
#endif
