# Dual OMX leader and follower bringup

## Roles and launch names

| Role | Launch | ROS namespace | Default hardware mode |
| --- | --- | --- | --- |
| Follower (two OMX-F arms) | `omx_dual_follower.launch.py` | `/follower` | Real |
| Leader (two OMX-L devices) | `omx_dual_leader.launch.py` | `/leader` | Mock |
| Both roles (four devices) | `omx_dual.launch.py` | `/leader` and `/follower` | Mock |

`omx_dual.launch.py` now includes both role-specific launches. It replaces the
previous follower-only alias; use `omx_dual_follower.launch.py` for just followers.
Each role has one controller manager, one robot description and one joint-state
topic. Joint names retain `left_` / `right_` prefixes inside each description.
TF frames use `leader/` / `follower/` prefixes. Both roles publish on the shared
`/tf` and `/tf_static` topics; both RViz instances subscribe to those topics.
RViz TF Prefix is `leader` or `follower` (RViz adds the slash), and Fixed Frame
is `leader/world` or `follower/world`. This avoids the old underscore/slash
mismatch. The two role-specific `world` frames are independent and are not
calibrated to each other. Each RobotModel can render within its own tree; a TF
display requesting frames from the other tree may report missing transforms.
Displaying both robots in a common fixed frame requires a measured transform
between the trees; joint-angle teleoperation does not require that transform.

## Combined bringup

All dual-arm launch code lives in `open_manipulator_bringup/launch/`:

```text
omx_dual.launch.py
├── includes omx_dual_follower.launch.py
└── includes omx_dual_leader.launch.py (after follower initialization)

```

The parent starts the two role-specific launch files, like stock `omx_ai.launch.py`.
Each role launch directly defines its arguments, Xacro expansion, nodes,
controllers, RViz and role-specific initialization. There is no shared launch
module. The parent handles argument forwarding and startup order using process
exit events: both follower init nodes must finish successfully before leaders
start. With initialization disabled, it waits for the follower controller spawner.

```bash
# Both roles in mock mode (default)
ros2 launch open_manipulator_bringup omx_dual.launch.py

# Both roles on real hardware, using the four configured USB paths
ros2 launch open_manipulator_bringup omx_dual.launch.py use_mock_hardware:=false
```

The parent includes `omx_dual_follower.launch.py` and `omx_dual_leader.launch.py`
in separate launch scopes. Followers start first. After both follower initial-pose
actions succeed, the parent starts leaders; if `init_position:=false`, it starts
leaders after follower controller activation. Initialization failure shuts down
the launch without starting leaders. Ctrl+C stops both roles.

`teleop:=true` is the combined-launch default. As in stock OMX, each follower
uses a six-joint trajectory controller (five arm joints plus the gripper) instead
of separate arm/gripper controllers. Each controller subscribes directly to its
matching leader trajectory stream. The leader broadcaster already reverses the
trigger sign; no second reversal or additional scaling is applied.

Set `teleop:=false` to bring up both roles without connecting commands, keeping
the separate follower gripper action controllers. Standalone follower bringup
also defaults to `teleop:=false`; pass `teleop:=true` to connect to separately
started leaders. The combined launch provides startup ordering; standalone
launches do not coordinate with each other. When launching separately, finish
follower initialization before starting leaders.

`use_mock_hardware` and `start_rviz` apply to both roles; override one with
`leader_use_mock_hardware`, `follower_use_mock_hardware`, `leader_start_rviz` or
`follower_start_rviz`. By default, RViz opens one window for each role.
`init_position`, `init_position_file` apply only to followers; `trigger_preload`
applies only to leaders. Real follower startup moves the arms to the initial
poses unless `init_position:=false` is passed.

Port and mount overrides have role prefixes, for example `leader_left_port`,
`follower_right_port`, `leader_left_xyz` and `follower_right_rpy`. All real-device
paths are checked for duplicates across both roles before either child starts.
Do not also run the individual role launches while the combined launch is running.

## Leader bringup

```bash
ros2 launch open_manipulator_bringup omx_dual_leader.launch.py use_mock_hardware:=true
```

Leader defaults use these persistent paths under `/dev/serial/by-id/`:

| Leader | Device basename |
| --- | --- |
| Left | `usb-ROBOTIS_OpenRB-150_228BDD7B503059384C2E3120FF0A2B19-if00` |
| Right | `usb-ROBOTIS_OpenRB-150_895EBFC8503059384C2E3120FF08031C-if00` |

Mock mode remains the default and does not open ports. To use the configured
real leader devices:

```bash
ros2 launch open_manipulator_bringup omx_dual_leader.launch.py \
  use_mock_hardware:=false
```

Override devices with `left_port` and `right_port` launch arguments; real mode
rejects empty paths or two paths resolving to the same device.
Leader base positions default to +/-184 mm for display only; use `left_xyz`,
`right_xyz`, `left_rpy`, `right_rpy` for the measured leader installation.
The follower aluminium support is not included in the leader model.

Like the stock OMX leader, arm joints 1..5 have state interfaces only and their
motor Torque Enable is set to 0. Each trigger uses a position controller with
the stock current-based position mode. `trigger_preload:=true` (default) sends
one -0.7 rad command per trigger after controllers activate. Use
`trigger_preload:=false` to skip these commands. No arm init-pose node is started
for leaders. IDs 1..6 are reused only across independent USB/DYNAMIXEL buses.

Leader interfaces:

- `/leader/controller_manager`
- `/leader/robot_description`, `/leader/joint_states`
- `/leader/left_trigger_position_controller/commands`
- `/leader/right_trigger_position_controller/commands`
- `/leader/left_joint_trajectory_command_broadcaster/joint_trajectory`
- `/leader/right_joint_trajectory_command_broadcaster/joint_trajectory`

Each trajectory broadcaster publishes its own six joints, reversing the trigger
sign as in the stock configuration. `use_local_topics: true` keeps the streams
separate. The driver endpoints are `/leader/left/omx_l/...` and
`/leader/right/omx_l/...`. The broadcaster collision flag is routed to
`/leader/collision_flag`; this is not a collision detector or an emergency stop.

The combined launch connects these streams to matching follower controllers
with `teleop:=true` (default). Standalone leader bringup only publishes commands.
Teleoperation uses absolute joint positions, as in stock OMX: the follower moves
toward the leader’s current angles when the stream starts. Position the leaders
near the follower initial pose before real startup. There is no clutch, relative
pose offset, stream-loss watchdog or coordinated fault supervisor in this setup.

## Follower bringup

Two stock OMX-F arms share one robot description, one robot_state_publisher and
one controller_manager. Each arm retains its own USB connection and hardware
component (`LeftOMX`, `RightOMX`). The launch defaults to real hardware with the
configured by-id ports. Set `use_mock_hardware:=true` for a preview without serial
device access. Standalone Xacro expansion still defaults to mock hardware.
This is the model/controller integration stage;
MoveIt, collision avoidance and coordinated fault stopping are not implemented
here. Joint-space teleoperation is available via the combined launch.

## Mounting coordinates

The directly measured gap between the inner edges of the robot bases is
**218 mm**. The stock `follower_01_base.stl` spans Y = -75 to +75 mm with no
visual offset, so the `link0` origins are **368 mm** apart (75 + 218 + 75).
The confirmed long-profile length is **658 mm**, partitioned left to right as
**70 + 150 + 218 + 150 + 70 mm** (end margin, base, gap, base, end margin).
Each base origin is 145 mm from its nearest long-profile end (70 + 75).
Both arms face +X, with +Y toward the robot's left and +Z upward. The default
assumes equal mounting height and no fore/aft offset:

| Argument | Default | Meaning |
| --- | --- | --- |
| `left_xyz` | `0 0.184 0` | Left link0 in workcell_base, metres |
| `right_xyz` | `0 -0.184 0` | Right link0 in workcell_base, metres |
| `left_rpy` | `0 0 0` | Left roll, pitch, yaw, radians |
| `right_rpy` | `0 0 0` | Right roll, pitch, yaw, radians |

`workcell_base` is halfway between the two base origins at the mounting plane.
Left/right are defined while looking in the arms' forward direction. Label the
physical USB devices consistently. Equal mounting height and zero fore/aft offset
are the installation defaults. Four fixed aluminium profiles are included with
outer-envelope collision boxes; the table and fasteners are not modelled.
Mesh units are millimetres with URDF scale 0.001.

`aluminium_profile.urdf.xacro` provides a reusable 30 x 30 mm profile member with
7 mm open T-slot mouths and a solid outer-envelope collision box. Its visual STL
includes four longitudinal T-slots, a hollow centre and chamfered outside corners.
Only the 30 mm envelope and 7 mm mouths are measured; slot chambers (11 mm),
depth (8 mm), lips (2 mm), square centre bore (6 mm) and chamfers (1 mm) are
illustrative, not manufacturing dimensions. The mesh generator is
`open_manipulator_description/meshes/omx_dual/generate_profile.py`.
The mesh is 1 m long along X and scaled to each member length by Xacro;
changing `section` also scales the 7 mm mouths proportionally. Two 658 mm
rails and two 200 mm outer end members are instantiated at the same height,
with their top faces at the robot base underside (Z = 0).
The confirmed clear gap between the two long profiles is 20 mm, giving a
50 mm centre-to-centre spacing (A). The stock STL's two mounting-hole columns
are at Y = +/-62.5 mm, with five holes each at X = -50, -25, 0, 25, 50 mm.
The middle and rearmost hole rows align with the slots, so the front rail centre
is X = 0 and the rear rail centre is X = -50 mm. The base rear edge at X = -60 mm
is therefore 10 mm from the rear slot centre (B), superseding the earlier 15 mm
edge-flush assumption. The base extends 45 mm beyond the front rail's front face;
the rear rail extends 5 mm behind the base.

The 200 mm end members run front to rear as **90 + 30 + 20 + 30 + 30 mm**:
front overhang, front rail, clear gap, rear rail, rear overhang. They span
X = -95 to +105 mm (centre X = +5 mm). Their centres are at Y = +/-344 mm,
outside the 658 mm rails, giving a model envelope of **718 x 200 x 30 mm**.
All coordinates are relative to workcell_base. Arm mounting overrides do not
move the fixed support; update its geometry too if the physical installation changes.

## Build and start with mock hardware

Inside the existing `open_manipulator` Docker container:

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select open_manipulator_description open_manipulator_bringup
source install/setup.bash
ros2 launch open_manipulator_bringup omx_dual_follower.launch.py use_mock_hardware:=true
```

Use the same ROS middleware environment as the rest of the workspace. If using
the configured Zenoh RMW, its router must be running. Pass `start_rviz:=false`
for a headless session. Do not run the single-arm bringup at the same time: this
launch owns the role-specific `/follower/controller_manager`, `/follower/robot_description` and `/follower/joint_states`.

For an isolated local mock preview without a Zenoh router, use:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=173 \
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ros2 launch open_manipulator_bringup omx_dual_follower.launch.py use_mock_hardware:=true
```

Use the same three environment variables in terminals querying that preview.

To override measured mounting transforms:

```bash
ros2 launch open_manipulator_bringup omx_dual_follower.launch.py \
  use_mock_hardware:=true \
  left_xyz:="0 0.184 0" right_xyz:="0 -0.184 0" \
  left_rpy:="0 0 0" right_rpy:="0 0 0"
```

In another terminal with the same environment:

```bash
ros2 control list_hardware_components -c /follower/controller_manager
ros2 control list_controllers -c /follower/controller_manager
ros2 topic echo /follower/joint_states --once
```

Expect two active mock hardware components and five active controllers:
`joint_state_broadcaster`, `left_arm_controller`, `right_arm_controller`,
`left_gripper_controller`, `right_gripper_controller`. The state topic contains
12 actuated joints; robot_state_publisher computes the two passive gripper mimic
joints from the URDF. Do not start joint_state_publisher_gui alongside the
controller-driven state broadcaster.

## Initial pose

Like the stock OMX-F launch, `init_position` defaults to `true`. After the
controller spawner succeeds, the existing `joint_trajectory_executor` starts
once per arm as `left_init_position` and `right_init_position`, alongside RViz.
Both nodes read `config/omx_dual/follower_initial_positions.yaml` and command only their
own five arm joints, through separate FollowJointTrajectory actions. Dual-arm
launches set `wait_for_result:=true`: each step waits for action success, and a
rejected, aborted or canceled goal fails initialization. The legacy executor
mode remains available for other launch files.

The stock sequence is `[0, 0, 0, 0, 0]`, then
`[0, -1.57, 1.57, 1.57, 0]` radians, with 5 seconds per trajectory and the stock
0.15 rad legacy step-completion tolerance. Dual-arm startup instead waits for
the controller action result. Gripper positions are unchanged during initialization. The two
executors run independently; there is no inter-arm synchronization or collision
checking. This is a move to configured joint poses, not a calibration procedure.

Disable automatic pose commands with `init_position:=false`. Override the YAML
with `init_position_file:=/absolute/path/poses.yaml`, retaining both node keys.
The real-hardware default now sends these pose commands automatically once all
controllers are active; use mock mode first to review the sequence.

## Real hardware configuration (not validated on motors)

The configured defaults use these persistent `/dev/serial/by-id/` paths:

| Arm | Device basename under `/dev/serial/by-id/` |
| --- | --- |
| Left | `usb-ROBOTIS_OpenRB-150_6D5D4B68503059384C2E3120FF05133E-if00` |
| Right | `usb-ROBOTIS_OpenRB-150_23D2E9915157375037202020FF0F2510-if00` |

The launch accepts `left_port` and `right_port` overrides. Empty arguments or two
paths resolving to the same device are rejected in real mode. Device presence
and communication are checked by the driver, not by this argument validation.
To start real hardware using the configured ports:

```bash
ros2 launch open_manipulator_bringup omx_dual_follower.launch.py \
  use_mock_hardware:=false init_position:=true
```

Both by-id symlinks and their target devices must be accessible inside the
container. Each physical bus keeps motor IDs 11..16. Do not join the two buses
with duplicate IDs.
Hardware GPIO resource names are prefixed `left_`/`right_`; driver endpoints are
under `/follower/left/dynamixel_hardware_interface/` and
`/follower/right/dynamixel_hardware_interface/`. The upstream driver currently constructs
its internal ROS nodes with the same name for both instances; endpoint separation
is explicit, but real multi-instance driver behavior still needs bench validation.

Set `init_position:=false` to skip the initial pose trajectories. Even with
that option, real driver initialization can change torque state and activate
motors; it is not a read-only probe. Confirm
mounting, joint limits and current posture before real bringup. The stock arm
URDF's broad joint/effort limits are inherited, not validated physical limits.
There is no coupled emergency-stop/fault supervisor in this initial integration.

## Follow-up work

1. Confirm base dimensions and the common frame against the installed robot.
2. Verify distinct serial paths and each real arm independently at low speed.
3. Add table/fastener collision geometry and verify physical joint limits.
4. Add a combined MoveIt SRDF with left_arm, right_arm and both_arms groups.
5. Validate time-aligned trajectories and coordinated stopping before carrying
   one rigid object with both arms. A shared manager does not synchronize USB
   transactions or enforce a relative end-effector pose by itself.
