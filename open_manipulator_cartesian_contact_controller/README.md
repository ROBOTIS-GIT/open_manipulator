# OMY Cartesian contact-responsive controller

A separate package alongside joint impedance, joint contact-responsive QP, classical
task-space impedance, and friction calibration/compensation. It extends the contact-responsive
idea to a **Cartesian nominal body with constrained high-gain correction wrench**.
It is not a complete reproduction of the paper or a claim of passivity.

## Control structure

All vectors are expressed in `task.root_link` and act at the `task.tip_link` origin, ordered
`[x, y, z, rx, ry, rz]`.

```
leader mapped joint angles
  -> FK using follower's actual /robot_description
  -> time-based Cartesian slow start -> desired pose
  -> Cartesian nominal spring/damper + virtual diagonal mass
  -> QP adjusts nominal acceleration under wrench/error constraints
  -> high-gain nominal-versus-actual pose/twist correction
  -> J(actual q)^T * (nominal wrench + correction wrench)
  -> existing gravity/friction/dither compensation + optional external joint effort
```

- Actual state: FK pose `X` and twist `V=J(q)dq`.
- Nominal state: `Xn,Vn`; target pose: `Xd`.
- Pose error: `e(Xb,Xa)=[pb-pa; Log(Rb*Ra^T)]`, using the shortest rotation independent
  of quaternion sign.
- Task wrench: `Wtask=clip(Kn*e(Xd,Xn)-Dn*Vn, +/- task_wrench)`.
- Free acceleration: `a_free=Mvirtual^-1 Wtask`.
- QP variables: six nominal accelerations and six nonnegative error slacks.
- First-order prediction of next-cycle correction:
  `error0=e(Xn,X)+dt*(Vn-V)`,
  `Wcorrection=Ki*error0+Di*(Vn-V)+(dt^2*Ki+dt*Di)*a`.
- The QP stays close to free acceleration while satisfying
  `|Wcorrection|<=correction_wrench` and `|Wtask+Wcorrection|<=motion_wrench`.
  `nominal_error` is a soft envelope, not an actual joint-position limit.
- Nominal position uses semi-implicit integration; orientation uses
  `Rn_next=Exp(dt*omega_next)*Rn`, not integration of Euler angles as angular velocity.
- Final torque:
  `tau=existing_compensation+J(q)^T*(Wtask+Wcorrection)+external_joint_effort`.

`nominal.mass` contains user-selected virtual translational mass and rotational inertia.
It is neither operational-space inertia derived from the joint inertia matrix nor identified
robot inertia. The law does not exactly cancel actual robot inertia/Coriolis. Measured
force–displacement/transient response need not match an ideal impedance exactly. Configured
dynamics terms in the existing gravity/friction path remain unchanged.

No inverse Jacobian is used. Calculations remain finite at singularities, but directional
control authority can decrease. There is no null-space posture control or guarantee of
matching the leader's joint configuration. Rotational QP constraints are local first-order
predictions, not global guarantees for large rotations or angles near 180 degrees.

## Run in the robot container

Use the [shared installation instructions](../open_manipulator_friction_diagnostic/README.md#installation-with-the-standard-omy-image)
for first installation on the standard image. With dependencies installed, rebuild just this
package when needed:

```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select open_manipulator_cartesian_contact_controller
source install/setup.bash
```

Terminal 1, follower:

```bash
ros2 launch open_manipulator_cartesian_contact_controller cartesian_contact.launch.py
```

The launch captures the current tip pose as target/nominal pose and uses the existing runtime
for motor power, brake, and gravity-current preparation. Do not run another arm controller.

Terminal 2, leader and trigger/gripper:

```bash
ros2 launch open_manipulator_cartesian_contact_controller omy_l100_cartesian_leader.launch.py
```

This same leader also works with
`open_manipulator_task_space_impedance_controller task_space_impedance.launch.py` without
extra arguments or YAML changes. Both followers share pose input, capture generation, and
readiness conventions. Select only one follower.

If the L100 bringup is already running, use `start_leader:=false`. Do not run the old joint
reference bridge concurrently. Default leader port is `/dev/ttyUSB0`; override with
`port_name:=...`. Stop each terminal with `Ctrl+C`. Stopping the follower shuts down motors;
stopping only the leader leaves the follower holding its last target.

## Leader FK, slow start, and gripper

- `/leader/joint_trajectory` already uses follower names/directions from the L100 broadcaster.
  Do not apply signs/offsets again. The six arm angles are used directly in the follower URDF
  `link0 -> end_effector_link` chain, including origins, axes, lengths, and fixed tip offset.
- The bridge reads transient-local `/robot_description`; no copied/rescaled leader URDF is
  required. Root/tip settings must match. A chain name/order mismatch prevents publishing.
- Startup, communication recovery, and controller reactivation connect the current follower
  reference pose to the leader FK pose using quintic position blending and shortest-path SLERP.
- Synchronization lasts at least 5 s, extended according to initial distance and configured
  translational/rotational reference speeds. It enters `FOLLOWING` when time expires without
  waiting for actual convergence. Keep the leader stationary during synchronization. These
  speeds do not limit all subsequent leader motion.
- Leader/follower telemetry older than 0.3 s or follower fallback returns the bridge to
  `WAITING`. It stops new targets; the follower holds the last target. Recovery restarts slow start.
- `capture_current_pose` resets target/nominal state and QP fault to the current pose. An
  attached leader synchronizes again from that point.
- The shared `l100_gripper_bridge.py` uses `leader_gripper_controller` in position mode.
  It consumes the already-mapped trigger `rh_r1_joint`, primes the actual gripper position
  after follower readiness, and confirms torque ON/readback. Arm wrench control does not
  command the gripper.

## YAML configuration

`config/cartesian_contact.yaml`, or `gains:=/absolute/path/file.yaml`:

| Parameter | Meaning and units |
|---|---|
| `nominal.stiffness`, `nominal.damping` | Nominal target-return stiffness/damping: N/m, Nm/rad; N s/m, Nm s/rad |
| `nominal.mass` | Virtual translational mass (kg) and rotational inertia (kg m²) |
| `inner.stiffness`, `inner.damping` | Nominal-to-actual correction gains, same stiffness/damping units |
| `contact.task_wrench` | Per-axis task wrench bounds, N and Nm |
| `contact.correction_wrench` | Per-axis inner correction bounds, N and Nm |
| `contact.motion_wrench` | Per-axis sum-of-wrenches bounds, N and Nm |
| `contact.nominal_error` | Soft nominal-to-actual error envelope, m and rad |
| `qp.error_slack_weight` | Predicted error slack penalty |
| `qp.tolerance`, `qp.max_iterations` | Solver tolerance and iteration budget, with one retry when applicable |
| `qp.max_period_s` | Maximum supported QP control period; larger periods trigger fallback |
| `fallback.ramp_s` | Transition duration to low-gain hold after QP failure |
| `gravity_compensation_enabled`, `friction_compensation_enabled` | Existing compensation switches |

These bounds do not limit total joint torque or actual contact force. Existing compensation
and optional external joint effort are added separately. Gains require hardware evaluation;
do not copy joint-space Nm/rad gains directly into Cartesian N/m settings. Parameters are
read at configure time, so restart after changes.

`config/l100_cartesian_leader.yaml` (`config:=...`) configures topics, root/tip, publish rate,
minimum synchronization duration, initial reference speeds, and state timeouts.

## Topics and reset

The controller is named `friction_compensation_controller` for runtime compatibility.
All names below are under that namespace.

- `target_pose` (`PoseStamped`): direct target; frame must exactly match the configured root.
  No automatic TF conversion or target smoothing. Held until the next target.
- `leader_target_pose` (`PoseStamped`): dedicated bridge input. **The stamp echoes the
  follower's capture timestamp, not the publication time.** Old-generation packets are
  rejected in both callback and real-time update.
- `current_pose`: FK pose stamped with the current control time.
- `target_pose_state`: applied target; its stamp remains the last capture timestamp until recapture.
- `task_wrench`: delivered task-plus-correction wrench, not measured external force.
- `cartesian_state` (`DynamicJointState`): x/y/z/rx/ry/rz errors, nominal velocity/acceleration,
  wrench, slack, and QP diagnostics. This is not a joint-state measurement.
- `task_state` (`DynamicJointState`): actual joint1..6 position/velocity, task torque,
  enabled/mode, update_sequence, reference_epoch; used for leader/gripper readiness.
- `commands` (`Float64MultiArray`): optional external six-joint effort for one cycle;
  not applied during QP failure.

```bash
ros2 topic echo /l100_cartesian_leader/status
ros2 topic echo /friction_compensation_controller/cartesian_state
ros2 service call /friction_compensation_controller/capture_current_pose std_srvs/srv/Trigger '{}'
```

QP failure latches mode 2 and transitions to a spring–damper hold at the failure pose. Old
QP output is not replayed indefinitely. Diagnose the cause and reset explicitly with the
service above; repeated resets are not a substitute for correcting tuning problems.

## Verification scope

Development checks include numerical tests, follower-URDF FK versus KDL, and actual
ros2_control plugins with GenericSystem, synthetic leader input, and the gripper position
controller. Test artifacts are kept outside the repository. Both launches provide
`use_mock_hardware:=true`; a mock harness must open the arm-readiness gate separately.
Mock/synthetic dynamics do not establish physical contact stability or worst-case 400 Hz
performance including Raspberry Pi communication.
