# OMY joint impedance controller

This separate ros2_control plugin captures current joint angles as the initial reference on
activation. Without another target it holds that reference. In one 400 Hz update it adds
joint-impedance torque to the existing gravity/friction compensation.

For initial installation on the standard OMY image, follow the
[shared installation instructions](../open_manipulator_friction_diagnostic/README.md#installation-with-the-standard-omy-image).
Then run:

```bash
ros2 launch open_manipulator_joint_impedance_controller impedance.launch.py
```

This single launch starts bringup, compensation, and impedance. Do not run another bringup
or the legacy Python joint-impedance node alongside it. The original compensation and
calibration launches remain separate and keep their existing plugins.

## Gains and compensation switches

Edit `config/joint_impedance.yaml` in joint1..joint6 order and restart the launch.
Use `gains:=/absolute/path/gains.yaml` to select another file.
Under `joint_impedance_controller.ros__parameters`:

```yaml
gravity_compensation_enabled: true
friction_compensation_enabled: true
```

Setting either to `false` disables that compensation output on restart, leaving impedance
and external effort active. The friction switch controls both kinetic friction and dithering.
Even with gravity output disabled, the friction load model still uses the original gravity
estimate. Low-stiffness impedance alone may not support the arm's weight without gravity
compensation. These settings are fixed at startup, not dynamically changed by `ros2 param set`.
If friction is enabled in YAML, the existing `set_friction_enabled` service can switch it at
runtime. If YAML disables it, the service cannot re-enable it.

```
tau_impedance = Kp * (q_reference - q) - Kd * q_velocity + tau_integral
tau_command = existing_compensation + tau_impedance + fresh_external_effort
```

Reactivation resets the reference to current angles and the integral state to zero.
Impedance output is zero before motor readiness and during shutdown. The existing supervisor
owns command preparation, brake transition, and shutdown. Internal impedance does not depend
on a ROS topic round trip or Python timer. External effort uses the existing
`/friction_compensation_controller/commands`; without a fresh command next cycle, only that
external term returns to zero while internal impedance continues.

The integral term is disabled by default. Optional per-joint `impedance.integral_gain` can
be set to a small positive value. The torque state itself is bounded by `integral_limit` to
avoid hidden windup and resets on loss of readiness/reactivation. Positive `integral_leak`
gradually forgets the correction. Integral action changes the response to sustained external
force compared with pure PD impedance. Stationarity alone does not identify hand contact.

PD with zero integral gain does not eliminate residual static friction or gravity error.
The existing kinetic-friction ratios and independent dithering settings come from the shared
`controllers.yaml`.

## State

```bash
ros2 topic echo /friction_compensation_controller/impedance_state
ros2 topic echo /friction_compensation_controller/command_state
```

The first topic reports reference, error, spring/damping/integral/impedance torque, and update
sequence. The second reports the commanded sum of gravity, friction, impedance, and external
effort. The controller_manager instance keeps the name `friction_compensation_controller`
for supervisor compatibility; this launch uses plugin type
`open_manipulator_joint_impedance_controller/JointImpedanceController`.

The plugin inherits the existing compensation library.

## External wrench estimated from measured joint effort

```bash
ros2 topic echo /friction_compensation_controller/external_wrench
ros2 topic echo /friction_compensation_controller/external_wrench_valid
```

`external_wrench` is `geometry_msgs/msg/WrenchStamped`. It reads six hardware joint `effort`
states in Nm (the existing driver's current-to-torque conversion) and computes:

```
tau_external = M(q)*ddq + C(q,dq)*dq + g(q) + passive_friction(q,dq,history) - measured_effort
tau_external = J_tool(q)^T * wrench_external
```

The sign denotes force **applied by the environment to the robot**. Commanded effort is not
substituted for measured effort. Impedance torque and external target effort are not subtracted
again: their physical effects are already included in measured current. The observer retains
gravity/friction modeling independently of compensation output switches. It uses a separate
copy of friction internal state and friction before compensation-ratio scaling. The commanded
dither waveform and artificial control damping are not treated as passive friction.

By default the wrench is expressed at the **origin and axes** of `end_effector_link`, also
used in `header.frame_id`. Force is in N and torque in Nm. Contacts elsewhere on the arm are
represented as one equivalent tip wrench; contact locations and multiple contacts are not
identified. The URDF chain between `wrench.root_link` and `wrench.tip_link` must contain the
same six controlled joints in the same order.

**This is an estimate, not a force-sensor measurement.** URDF inertial error, torque conversion
error, current filtering/delay, and static-friction uncertainty enter the residual. Accuracy
may decrease with an extended arm or during acceleration, particularly where J2/J3 inertial
parameters are inaccurate. Acceleration from differentiated hardware velocity uses a default
0.05 s filter; torque residual uses a 0.1 s filter. These filters delay transients; they do not
automatically identify or cancel sensor delay. Observer state resets after activation or sensor
fault. Independent force accuracy has not been established, so this topic is not a validated
contact-safety signal or force-control sensor.

Near singularities, damped least squares provides a finite approximation and
`external_wrench_valid` becomes `false`. This Boolean indicates finite sensor values, motor
readiness, and kinematic conditioning, not physical accuracy. Before readiness or during sensor
faults, validity is false and no new wrench is published. Consumers must check both validity
and timestamp. The timestamp is controller time, not a separately verified current-acquisition time.

Configure `wrench.*` in the same YAML. Rigid-body dynamics/Jacobian/SVD calculation and publication
default to 50 Hz; friction state and residual filters update in the control loop. Lower publication
rates do not let the dynamics update interval exceed 20 ms. `wrench.enabled: false` disables
calculation/publication and the additional effort state request. This diagnostic path does not
change impedance, gravity, or friction commands.

In `impedance_state`, `measured_effort_nm` is measured input and `external_joint_torque_nm` is
the filtered external residual. `external_effort_nm` is a commanded target torque from another
topic, not a force-sensor measurement.

To avoid mixed-unit scaling, the estimator uses `z=[F,M/L]`, `A=[J_v^T,L*J_w^T]` and solves
`z=(A^T A+lambda^2 I)^(-1) A^T tau_external` using SVD.
Here `L=wrench.characteristic_length_m` and `lambda=wrench.damping_m`. The kinematic mapping
is marked valid when the normalized matrix's smallest singular value is at least lambda.

## OMY L100 leader to impedance follower

Start the follower with its normal launch. Start the L100 leader in a separate terminal or on
the computer connected to it. If using two computers, their ROS communication settings must
allow topic discovery. Each command below starts its respective hardware.

```bash
# Follower
ros2 launch open_manipulator_joint_impedance_controller impedance.launch.py

# L100 leader, including its bringup
ros2 launch open_manipulator_joint_impedance_controller omy_l100_leader.launch.py port_name:=/dev/ttyUSB0
```

If the L100 bringup is already active, add `start_leader:=false` to avoid two hardware nodes
opening the same port. This launch does not start/switch the follower arm controller. By default
it adds `leader_gripper_controller` to the follower manager and connects the trigger; use
`gripper_enabled:=false` to disable that part.

Configure topics and synchronization timing/speeds in `config/l100_impedance_leader.yaml`, or
select `config:=/absolute/path/config.yaml`. The default reference path is:

```
/leader/joint_trajectory (existing broadcaster; stops publishing on detected collision)
    -> l100_impedance_leader
    -> /friction_compensation_controller/reference (Float64MultiArray, rad, joint1..joint6)
```

1. `WAITING`: wait for leader commands and fresh active-follower telemetry.
2. `SYNCING`: publish quintic-blended references at 100 Hz from the current follower reference
   to the latest leader posture. Duration is at least 5 s and increases for larger initial
   differences, based on a stationary-leader reference speed of 0.2 rad/s. Moving the leader
   during synchronization changes the interpolation; 0.2 rad/s is not a global target-speed limit.
3. Enter `FOLLOWING` when the interpolation time expires, regardless of actual follower error.
   Low-stiffness steady-state error therefore cannot cause indefinite synchronization.
4. `FOLLOWING`: forward leader angles as impedance references. Follower gains and compensation
   settings are unchanged; the arm does not switch to position-control mode.

Synchronization completion means **the reference connection has completed**, not that actual
poses match. The interpolation is not a collision-avoidance planner; clear the space between
starting poses.

```bash
ros2 topic echo /l100_impedance_leader/status
```

Input/telemetry loss exceeding 0.3 s or follower deactivation stops new reference publication.
The follower holds its last impedance target; it does not automatically turn torque OFF or
switch to freedrive. Recovery/reactivation starts another slow start. Stopping only the leader
has the same hold behavior. Stop the follower launch to shut down the robot.

`~/reference` accepts six finite angles applied in the real-time update. Without input, the
activation pose remains the target. Reactivation captures the current pose and clears old input.
This is separate from the one-cycle external-torque policy of `~/commands`. Do not use multiple
position-reference publishers simultaneously.

### QP follower and gripper

The same leader works with the joint contact-responsive follower. Replace only the follower command:

```bash
ros2 launch open_manipulator_contact_responsive_controller contact_responsive.launch.py
```

The broadcaster already applies direction reversal and the +0.2 offset to `rh_r1_joint` in
`/leader/joint_trajectory`. `l100_gripper_bridge.py` forwards it without repeating that conversion
to `/leader_gripper_controller/joint_trajectory`, using the URDF range 0..1.13514578304 rad.
This gripper range is separate from arm position/speed constraints.

The position controller is first primed with the current gripper position. An end-unit-specific
service enables torque and ID 7 ON readback is checked. After a gradual trigger/gripper
synchronization it follows position commands with `time_from_start=0`, matching the standard OMY
leader. No extra interpolation duration is inserted into each message; startup/reconnection
slow start is implemented by the bridge. Joint and Cartesian contact leaders share this bridge.
It does not call arm torque-enable services.

On input loss, fresh gripper state is used to send a hold at the current position. Stopping the
leader leaves follower arm/gripper controllers holding their references; stop the follower for
full shutdown. Another `gripper_controller` cannot simultaneously claim the same position
interface. This setup assumes only the selected follower bringup is running.
