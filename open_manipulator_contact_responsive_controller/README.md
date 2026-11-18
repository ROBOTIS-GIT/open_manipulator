# Contact-responsive QP controller for OMY

Independent, experimental ros2_control plugin inspired by C-NRIC / constrained disturbance observer control. The existing joint impedance control law and friction calibration pipeline are retained; the shared L100 launch adds a separate gripper bridge and virtual end-unit torque-enable support. This is an adaptation, not a reproduction of the paper's equations or stability proof.

Build with the existing OMY workspace and ROS Jazzy `osqp_vendor` dependency, then:

```bash
ros2 launch open_manipulator_contact_responsive_controller contact_responsive.launch.py
# Same plugin and lifecycle with fake hardware (no physical dynamics):
ros2 launch open_manipulator_contact_responsive_controller contact_responsive.launch.py use_mock_hardware:=true
```

Do not run alongside another bringup controlling the same hardware. The launch reuses the friction package's motor preparation, readiness, gravity/current priming, stop and brake lifecycle. Activation and motor-ready transitions capture the current pose as the target. No home motion is requested. It loads the same current calibration and compensation ratios as the existing friction bringup. There is no calibration, trajectory review or simulation prerequisite in hardware startup. GenericSystem deliberately leaves the motor-ready gate disabled; for an offline mock exercise only, call `ros2 service call /friction_compensation_controller/accept_external_effort std_srvs/srv/SetBool "{data: true}"` after startup. Hardware startup manages that gate itself.

## Inputs and configuration

`config/contact_responsive.yaml` supplies six-element J1–J6 gains and QP settings. Pass an alternative with `gains:=/absolute/path.yaml`. Existing compensation ratios and dither switches remain in the friction package's `controllers.yaml`, overridable using `controllers:=...`. `calibration_file:=...` selects a calibration explicitly. Parameters are configure-time, not live-tunable. Numeric gains may use integer or decimal YAML literals (for example `[30, 30, 30, 20, 10, 5]`); the launch normalizes double-valued parameters before ROS loads the plugin. Six-element numeric arrays are required for per-joint settings.

The runtime instance name remains `friction_compensation_controller` for compatibility with the motor supervisor:

* `/friction_compensation_controller/reference`: `std_msgs/msg/Float64MultiArray`, six target positions [rad]. A valid target is held; malformed and stale queued targets are ignored. On re-enable/reset, capture current position and discard earlier targets.
* `/friction_compensation_controller/commands`: six external efforts [Nm]. Each message is consumed for one control cycle only. It enters both nominal dynamics and real motion torque, within the QP motion bounds. No 0.5 s hold.
* `/friction_compensation_controller/reset_contact_controller`: `std_srvs/srv/Trigger`. Recover a latched QP failure by capturing the current pose; old targets are discarded.
* `/friction_compensation_controller/impedance_state`: `control_msgs/msg/DynamicJointState`, approximately 50 Hz. Contains target/error, nominal state, task/correction/motion effort, solver status, iterations and solve time. `mode`: 0 not ready, 1 QP, 2 fallback. `failure_reason`: 0 none, 1 invalid input/model/period, 2 infeasible hard bounds, 3 solver failure. `enabled` is false in fallback. Field names used by the existing L100 bridge are retained; this package does not start a leader.

## Control law

Let q,v be measured state; qn,vn nominal state; r the target. At each 400 Hz cycle:

1. Compute bounded low-gain task torque `t = clip(K(r-qn)-D*vn, +/-task_limit)`.
2. Evaluate full KDL mass M(qn) and Coriolis c(qn,vn). A configurable diagonal inertia regularization is added to M; it is not identified motor inertia. Gravity is compensated separately at actual measured q by the existing base controller.
3. Free nominal acceleration is `afree = M^-1 (t + external_effort)`.
4. Solve a 12-variable QP over nominal acceleration a and nonnegative slack z:
   `min 0.5*(a-afree)' W (a-afree) + 0.5*slack_weight*||z||²`, where `W=M/(trace(M)/6)`.
5. Semi-implicit nominal update: `vn_next=vn+dt*a`, `qn_next=qn+dt*vn_next`.
6. Inner correction uses predicted actual position `q+dt*v`:
   `u = Ki*(qn_next-q-dt*v) + Di*(vn_next-v)`.
   Real motion effort is `t + c + external_effort + u`.
7. Existing gravity, identified friction and dithering are added by the base controller. No online correction is written to the friction model.

QP hard bounds constrain inner correction torque and total **motion** effort (task + Coriolis + external effort + correction). A soft nominal/actual error envelope penalizes nominal separation during contact; its physical slack is `dt²*z`. These are contact-response control parameters. Nominal acceleration is shaped by the objective, without a separate hard cap that could prevent the nominal robot from yielding to hand motion. **No self-collision, absolute joint-position or joint-velocity constraints are implemented.** Motion limits exclude gravity/friction/dither and do not constitute actuator torque limits. Existing hardware protections remain unchanged.

The fixed OSQP 0.6 workspace is allocated at configure time; its sparsity is constant. Each solve uses at most `qp.max_iterations`. On iteration exhaustion only, one retry resets the dual iterate and ADMM penalty and starts from a feasible point of the current QP; at most twice that iteration budget is used. Telemetry `qp_iterations` and `qp_time_s` include both attempts. `OSQP_SOLVED_INACCURATE` (status 2) is accepted only when finite primal/dual residuals meet 10 times `qp.tolerance` and every acceleration-box and soft-envelope constraint independently passes the same numerical tolerance. Infeasibility and unconverged results are rejected; previous torque solutions are never used as a substitute. Nonfinite state or nonpositive/nonfinite period returns controller ERROR; invalid dynamics, positive periods outside the supported range, infeasible constraints or solver failure latch a low-gain hold at the failure pose, blended from the last output over `fallback.ramp_s`. Gravity/friction remain active in that fallback. It never keeps an old QP solution as a permanent command. Reset is explicit. OSQP status and `mode` must be monitored during evaluation. Status 2 with `mode=1` now indicates an accepted, independently checked solution, not fallback.

This inner feedback rejects residual model/friction error rather than estimating friction separately from human contact. Exact requested mechanical stiffness and torque transparency are not guaranteed. Compare contact force/deflection and release response as well as target error. URDF error, delay and dithering can affect that result. Neither a fake-hardware launch nor desktop timing establishes real robot stability or Raspberry Pi real-time performance. Closed-loop stability, torque transparency and requested mechanical stiffness require independent hardware evaluation.

## Basis

[Contact-responsive high-gain robot motion controllers using a constrained disturbance observer](https://link.springer.com/article/10.1007/s11370-026-00744-z) motivates a nominal robot, high-gain inner tracking and constrained response under contact. The adaptation above uses the OMY gravity-compensated torque interface and soft error constraints; its properties require independent validation.

[ROBOTIS cyclo_control](https://github.com/ROBOTIS-GIT/cyclo_control), inspected at `f4ee93577327976ef465c94e6b8cef9a4469bbc2`, uses velocity-level MoveJ optimization. This package uses OSQP directly for a small, fixed-sparsity acceleration/torque QP instead of bringing its motion-planning stack into the 400 Hz effort loop.

## L100 leader

Use the existing `open_manipulator_joint_impedance_controller omy_l100_leader.launch.py`
after starting this follower. Its six-axis reference and telemetry contract is shared.
The leader launch also starts an independent follower gripper position controller and
trigger bridge; `gripper_enabled:=false` disables that feature. A QP fallback publishes
`enabled=0`, so the leader stops sending new references until explicit recovery.
