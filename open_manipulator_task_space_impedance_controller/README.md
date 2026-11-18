# OMY task-space impedance controller

A separate ros2_control plugin alongside the existing joint-impedance, contact-responsive,
and friction calibration/compensation packages. It implements a **Cartesian spring–damper
using the Jacobian transpose, without a QP**.

## Control law and frames

```
e_position = p_target - p_actual
e_rotation = Log(R_target * R_actual.transpose())
twist = J(q) * dq
wrench_task = K * [e_position; e_rotation] - D * twist
tau_task = J(q).transpose() * wrench_task
tau_command = existing_compensation + tau_task + external_joint_effort
```

- `J` is the geometric Jacobian at the `task.tip_link` origin. All vectors are expressed in
  `task.root_link`. Rotation error is the shortest rotation vector; quaternion sign changes
  do not change the target.
- The existing compensation path reads this OMY unit's latest `current.yaml`. Ratios and
  dithering come from the shared `controllers` YAML. Configured dynamics terms in that
  compensation path remain unchanged.
- Activation captures the current end-effector pose as the initial target. Motor readiness
  and reactivation recapture the current pose instead of replaying an old target. With no
  input, the captured pose remains the reference.
- Each `accept_external_effort: true` request schedules a new capture, including an OFF/ON
  pair handled between updates. Do not call it periodically during normal operation.
- At a stationary initial pose, task spring torque is zero. Damping may produce torque if
  the robot is already moving.
- There is no virtual inertia, QP, integral term, disturbance observer, null-space posture
  control, or Jacobian inversion. Loss of control authority near singularities still applies.
- Rotational stiffness is locally interpretable for small errors. The shortest rotation
  direction can be discontinuous near 180 degrees.
- This is not force-sensor feedback and does not bound actual contact forces. It adds no
  separate task-torque, position, or speed limits; existing hardware protection/runtime remain.

## Build and run

For first-time installation on the standard OMY image, follow the
[shared installation instructions](../open_manipulator_friction_diagnostic/README.md#installation-with-the-standard-omy-image).
With dependencies and the friction package already installed:

```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select open_manipulator_task_space_impedance_controller
source install/setup.bash
ros2 launch open_manipulator_task_space_impedance_controller task_space_impedance.launch.py
```

Do not run another arm bringup/controller simultaneously. This launch owns the existing
runtime's motor ON/OFF, brake transition, and gravity-current preparation. Stop with `Ctrl+C`.
Installation alone does not run the robot.

In an isolated mock environment, add `use_mock_hardware:=true`. Mock verifies plugin,
message, and torque paths, not physical friction/contact stability. Mock runtime does not
open the motor-readiness gate automatically; a test harness can call `~/accept_external_effort`.

## Gains

Example parameters in `config/task_space_impedance.yaml`:

```yaml
task.stiffness: [100.0, 100.0, 100.0, 3.0, 3.0, 3.0]
task.damping: [20.0, 20.0, 20.0, 1.0, 1.0, 1.0]
gravity_compensation_enabled: true
friction_compensation_enabled: true
```

The first three entries are root-frame x/y/z translation; the last three are rotation about
x/y/z. Stiffness units are N/m and Nm/rad; damping units are N s/m and Nm s/rad.
At 100 N/m, a 1 cm translational error produces 1 N of spring wrench in that direction.
Example gains are not a guarantee of tuned hardware behavior.

Parameters are read at configure time; restart the launch to apply edits. Use
`gains:=/absolute/path/task_space_impedance.yaml`, `controllers:=...`, or
`calibration_file:=...` for explicit files. Other controllers' launches/YAML are not overwritten.

## Targets and diagnostics

The controller instance is named `friction_compensation_controller` for motor-supervisor
compatibility. All entries below are under `/friction_compensation_controller`.

| Name | Type | Meaning |
|---|---|---|
| `target_pose` | `geometry_msgs/msg/PoseStamped` | Direct end-effector target |
| `leader_target_pose` | `geometry_msgs/msg/PoseStamped` | Shared Cartesian leader input; requires the current capture timestamp |
| `current_pose` | `geometry_msgs/msg/PoseStamped` | Current pose from FK |
| `target_pose_state` | `geometry_msgs/msg/PoseStamped` | Applied target; stamp identifies the current capture, not publication time |
| `task_wrench` | `geometry_msgs/msg/WrenchStamped` | Commanded spring–damper wrench, not measured external force |
| `task_state` | `control_msgs/msg/DynamicJointState` | Joint position/velocity, task/external torques, enabled, update_sequence, reference_epoch |
| `commands` | `std_msgs/msg/Float64MultiArray` | Optional six-joint external effort, Nm, consumed for one cycle |
| `capture_current_pose` | `std_srvs/srv/Trigger` | Capture the actual pose as the target on the next update |

`target_pose.header.frame_id` must exactly match `task.root_link` (default `link0`). No
implicit TF conversion is performed. Quaternions must be finite and nonzero and are normalized.
Invalid targets are rejected while the previous target remains active.

A target is applied in the control update and held until another valid target arrives.
Its timestamp does not schedule a trajectory. There is no target smoothing or velocity/
acceleration feedforward; an upstream node must provide smooth pose references for motion.
Damping uses measured end-effector twist.

```bash
ros2 topic echo /friction_compensation_controller/current_pose --once
ros2 topic echo /friction_compensation_controller/task_wrench
ros2 service call /friction_compensation_controller/capture_current_pose std_srvs/srv/Trigger '{}'
```

## L100 Cartesian leader

Follower terminal:

```bash
ros2 launch open_manipulator_task_space_impedance_controller task_space_impedance.launch.py
```

In another terminal, use the same leader as the Cartesian contact controller:

```bash
ros2 launch open_manipulator_cartesian_contact_controller omy_l100_cartesian_leader.launch.py
```

FK uses follower link geometry with L100 joint angles. A time-based slow start connects the
current reference to the leader target; it does not wait for actual pose convergence. The
shared bridge sends the trigger to the gripper position controller. The leader launch/YAML
also works with `cartesian_contact.launch.py`; run only one follower at a time.

Capture and motor reactivation update `reference_epoch` and the capture timestamp, causing
the leader to synchronize again. Old-generation targets are rejected in both receipt and
control update paths. Direct `target_pose` timestamp behavior and the PD/Jacobian-transpose
law are unchanged. `current_pose` and `task_state` stamps are publication timestamps.
