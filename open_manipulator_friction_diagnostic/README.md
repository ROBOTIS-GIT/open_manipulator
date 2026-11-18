# OMY automatic friction calibration and compensation

This package provides automatic position-controlled data collection, joint-group friction
identification, independent dithering selection, and a gravity/friction ros2_control plugin.
The original `om_gravity_compensation_controller` remains a separate, unchanged controller.
The supported hardware is the same six-axis OMY F3M configuration, including different
physical units. This is not a general seven-axis or dual-arm implementation.

## Installation with the standard OMY image

Use the repository branch containing these packages. The published upstream branch/image
may not contain them yet. No custom Docker image, Dockerfile changes, Compose overrides,
or changes to `docker/container.sh` are required. Keep the standard OMY Docker workflow.

On the **robot host**, from this repository:

```bash
./docker/container.sh start
./docker/container.sh enter
```

`start` uses the standard published image. It does not build these additional packages.
Inside the **container**, install dependencies and build them once:

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

packages=(
  open_manipulator_friction_diagnostic
  open_manipulator_joint_impedance_controller
  open_manipulator_contact_responsive_controller
  open_manipulator_task_space_impedance_controller
  open_manipulator_cartesian_contact_controller
)
paths=()
for package in "${packages[@]}"; do
  paths+=("src/open_manipulator/$package")
done

apt-get update
rosdep install --from-paths "${paths[@]}" --ignore-src -r -y
colcon build --symlink-install --executor sequential \
  --packages-select "${packages[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

The standard container already sources `~/ros2_ws/install/setup.bash` in `.bashrc`.
New terminals therefore need no dated experiment overlays or `env.sh` files.
On a native ROS Jazzy installation, use the same dependency/build commands in your workspace.
Building packages does not start any motors.

A container restart preserves its installed packages; removing/recreating the container does
not. Repeat dependency installation and the build after recreation. Do not run
`container.sh stop` merely to close a terminal: it removes the container.

### One-time unit identity and persistent storage

The stock Compose file already mounts `docker/workspace/` at `/workspace`. Use that existing
mount for calibration data and the unit environment file. In the container, replace
`YOUR_OMY_ID` with a stable, unique ID for this physical arm:

```bash
mkdir -p /workspace/omy_friction
# Create once. Inspect an existing file instead of replacing another unit's identity.
if [ ! -e /workspace/omy_friction/environment.bash ]; then
  cat > /workspace/omy_friction/environment.bash <<'ENV'
export OMY_ROBOT_UNIT=YOUR_OMY_ID
export OMY_FRICTION_DATA_DIR=/workspace/omy_friction
ENV
fi

# Add once per container; repeat this step after container recreation.
grep -qxF 'source /workspace/omy_friction/environment.bash' ~/.bashrc || \
  echo 'source /workspace/omy_friction/environment.bash' >> ~/.bashrc
source /workspace/omy_friction/environment.bash

ros2 run open_manipulator_friction_diagnostic friction_device show
```

Verify the ID before collection. Identity comes from `OMY_ROBOT_UNIT`, an optional host
identity mount, or the hostname. A random Docker hostname is rejected. This identifies the
computer/arm pairing; it is **not an actuator serial-number read**. Use a new ID if the computer
moves to another arm. Do not distribute `docker/workspace/`, previous calibration files,
`build/`, `install/`, or `log/` with source code for another robot.

With the setup above, per-unit data lives in `/workspace/omy_friction/units/<ID>/`:

- `current.yaml`: the latest successfully saved friction models and dither parameters.
- `device.yaml`: optional measured sensor timing settings.
- `collection/`: acquisition data, checkpoints, and calculation reports.

These files survive container recreation through the existing host mount. Without
`OMY_FRICTION_DATA_DIR`, the default is `~/.ros/omy_friction/units/<ID>/`; the standard
container does not persist that directory automatically. A specific model file can be selected
with `OM_FRICTION_CALIBRATION_FILE` or `calibration_file:=...`. Explicit selection still checks
unit identity on hardware.

All compensation, impedance, and contact-controller launches use the same unit model file.
The package installs no unit-specific coefficients. A matching legacy package
`calibration/current.yaml` is copied once to the unit store if the destination is absent;
its source is preserved. To import a separately backed-up model for the **same** unit:

```bash
ros2 run open_manipulator_friction_diagnostic friction_device import-calibration /path/to/current.yaml
```

Import refuses another unit's model and never overwrites an existing destination.

### Sensor timing

Optional per-unit sensor settings can be initialized without opening hardware:

```bash
ros2 run open_manipulator_friction_diagnostic friction_device init
```

Edit the reported `device.yaml`, or select another file with
`calibrate.launch.py device_config:=/absolute/path/device.yaml`. Existing settings are not
overwritten by `init`. Current-filter time constants and measurement delays default to zero
with `sensor_timing_verified: false`: these are **unmeasured assumptions**, not proof that a
new unit has no delay. Measured settling times must cover five filter time constants plus
the measurement delay. Position-command tracking lag is estimated during each acquisition
and is separate from current-sensor delay. Torque conversion uses the existing OMY driver
and actuator model; changed firmware, gearing, or actuators require checking that configuration.

## 1. Automatic identification

Stop any other arm bringup/controller first. Clear the workspace and keep the emergency stop
available. Do not touch the arm during collection. The robot moves automatically; hand-guided
data collection is not required.

```bash
ros2 launch open_manipulator_friction_diagnostic calibrate.launch.py
```

The default selects all six joints. To collect and calculate only selected joints:

```bash
ros2 launch open_manipulator_friction_diagnostic calibrate.launch.py joints:=2,3
ros2 launch open_manipulator_friction_diagnostic calibrate.launch.py joints:=5
```

Alternatively, set `joints: [2, 3]` in `config/calibration.yaml`. An explicit launch argument
overrides the YAML selection. Other joints may move to establish the measurement posture.
The selected joints retain their full speed/posture conditions. Successful partial calibration
replaces only those joints in `current.yaml`, preserving other models, coverage, and compatible
dither parameters. Unit, URDF, and gravity-reference mismatches cannot be merged.
A new unit can accumulate results joint by joint, but compensation bringup requires all six models.

The sequence is:

1. Load the fixed trajectory and matching geometry review.
2. Prepare position-control bringup with torque OFF and review the current-pose-to-zero path.
3. Prepare the hold command, enable motors, and confirm drive readiness.
4. Move slowly to `[0, 0, 0, 0, 0, 0]`.
5. Collect joint-by-joint forward/reverse measurements.
6. Return to zero and confirm motor OFF.
7. Calculate and save the model. Freedrive does not start automatically.

Progress includes `CALCULATING`, `EXTRACTING_MEASUREMENTS`, `FITTING_JOINT`, `SAVING`, and
`COMPLETE`. During long calculations, a message every 30 seconds reports the active joint,
elapsed time, and asks you not to stop the process before saving finishes.
`calibration_complete: true` means all six joint models are present. Failed or interrupted
calculations do not replace the previous compensation file.

### Precomputed motion and coverage

Fixed zero-to-collection-to-zero motions are distributed as per-joint position, velocity,
acceleration, and timing samples. `TRAJECTORY loaded (packaged)` confirms reuse. Selected
blocks are joined with duplicate holds removed and boundary derivative samples adjusted to
match the original full generator exactly. The URDF fingerprint, effective collection
settings, conditions, and generator code must match. Changed inputs regenerate the affected
blocks into `trajectory_cache/`; a different unit ID alone does not regenerate them.
No measured friction coefficients are included in these trajectory assets.

Fixed geometry reviews are also reused when the trajectory, URDF kinematics, collision meshes,
review code, and library versions match. `GEOMETRY cache hit` or `cache miss` reports the result;
new successful reviews are cached under `~/.ros/omy_friction/geometry_cache`.
The actual-current-pose-to-zero connection is still reviewed at runtime. Reusing motion and
geometry does not skip fitting the newly measured data.

The default physical collection trajectory takes **109 minutes 42 seconds**, plus initial
homing and calculation. It includes forward/reverse pairs and per-joint reference measurements
before and after collection. Independent validation motion and dithering selection are off by default.

| Joint | Collection speed range | Peak speed |
|---|---|---|
| J1 | 0.004–2.0 rad/s | about 115 deg/s |
| J2 | 0.004–1.3 rad/s | about 74 deg/s |
| J3 | 0.004–2.0 rad/s | about 115 deg/s |
| J4 | 0.001745–2.0 rad/s | about 115 deg/s |
| J5 | 0.001745–1.5 rad/s | about 86 deg/s |
| J6 | 0.004–2.0 rad/s | about 115 deg/s |

Intermediate speed points are retained. Low-speed motions provide sufficient constant-speed
samples; higher speeds use additional acceleration/deceleration distance. These are trajectory
coverage limits, not measured actuator maximum capabilities. Extending a range requires
reviewing the modified motion for self-collision and table clearance. Insufficient actual speed
is reported for the affected condition; it is not silently accepted or used to reduce every
joint's range.

J1/J4/J5/J6 use the kinetic-friction selection procedure. J2/J3 share one `loaded_joints.recipe` in
`config/fitting.yaml`: directional speed/load curves, observable nonlinear load correction,
and a mean-preserving periodic position residual. The algorithm is shared; postures and fitted
coefficients differ. This handles posture/load-dependent observed residuals, not inertial
parameter identification. Mass, center of mass, and inertia tensors are not refitted.

Full speed sweeps are combined with several loads at representative speeds rather than every
speed/load combination. Observed coverage is stored separately. Accuracy at every unmeasured
posture/load is not guaranteed. New coefficients come from new measurements; old models are
comparison/recovery references, not a source for filling unmeasured regions.

### Optional dithering and validation

```bash
ros2 launch open_manipulator_friction_diagnostic calibrate.launch.py \
  collect_dither:=true validate:=true
```

Both options apply only to selected joints. `validate:=false` skips extra physical validation
motion, not checks for invalid samples, missing joints, or unidentifiable models.
Dithering is identified separately from kinetic friction and uses one fixed amplitude/frequency
per joint. Selection uses current and position response, not an independent measurement of
human breakaway force. The default dither trials take 26 minutes, plus posture transitions.

A new unit starts with zero dither. `collect_dither:=false` preserves only a compatible dither
result from the same unit/kinematics/gravity reference. To include startup/reversal assistance
on a new unit, perform independent dither collection. Default compensation ratios remain
`[0.9, 0.80, 0.80, 0.95, 0.95, 0.95]`; these do not guarantee the same feel on every OMY.

Collection and fitting settings are in `config/calibration.yaml`, `collection.yaml`,
`conditions.yaml`, `fitting.yaml`, and `dither.yaml`. Changing settings does not refit an already
saved `current.yaml`. The stable output filename is replaced atomically on success.
Large candidate residual arrays remain in `collection/identification.json`; the runtime YAML
contains model parameters, coverage, settings, and a compact result summary.
The previous acquisition is retained as `collection.previous` when replaced.

### Acquisition recovery

Samples with state intervals over 50 ms or arrival delays over 100 ms are excluded from fitting.
After confirmed motor OFF and process shutdown, complete forward/reverse motion groups are
preserved and the incomplete group is reacquired automatically, up to two retries. Disable this
with `max_acquisition_retries:=0`. Failed raw data is retained in `collection.recovery/`.
Service-only delays, drive faults, contact, and user shutdown requests do not automatically
restart motors.

A heartbeat-only delay can be reviewed and explicitly resumed with `resume:=true` if acquisition,
drive state, and OFF confirmation are otherwise valid. Keep the same joint selection:

```bash
ros2 launch open_manipulator_friction_diagnostic calibrate.launch.py joints:=2,3 resume:=true
```

Resume requires the same robot, settings, and full trajectory. Opposite directions of one speed
pair are never combined across attempts. Checkpoints are `automatic_cycles` boundaries including
preparation, stop, paired measurement, and return. Resume first homes slowly to zero, then uses
the existing reviewed zero-to-measurement route. Older data without motion checkpoints can only
resume at completed-joint boundaries. Interruptions are recorded in `collection.json` and the
`stamp_s` array in `measurements.npz`. Contact-contaminated data and unconfirmed motor shutdown
cannot be reused through this mechanism.

## 2. Gravity/friction compensation and freedrive

After collection has finished and its launch has stopped:

```bash
ros2 launch open_manipulator_friction_diagnostic bringup.launch.py
```

Compensation starts at the current pose without homing. This launch performs no mock run,
refitting, historical replay, or validation trajectory. Joint ratios and dither switches are in
`config/controllers.yaml`; restart the launch after changing them, or pass `controllers:=...`.

```yaml
friction_compensation_controller:
  ros__parameters:
    compensation_ratios: [0.9, 0.80, 0.80, 0.95, 0.95, 0.95]
    dithering_enabled: [true, true, true, true, true, true]
```

The command is existing gravity compensation + ratio-scaled identified kinetic friction +
independent dithering + optional external effort. Ratios do not modify identified coefficients.

External effort uses `/friction_compensation_controller/commands`,
`std_msgs/msg/Float64MultiArray`, six finite values in joint1..joint6 order, Nm.
A fresh command is used for **one control cycle only**. Without a new command in the next cycle,
only external effort returns to zero; gravity/friction remain active. At 400 Hz this is about
2.5 ms. Old received commands and commands received before readiness are discarded.
`external_effort_timeout_s` has been removed. Timing uses callback receipt time because this
message has no sender timestamp. Total output is published in `~/command_state` effort.

## 3. Torque-based controllers

Each launch includes its own bringup and compensation. Stop the previous follower before
starting another; do not run the compensation-only launch alongside these controllers.

```bash
ros2 launch open_manipulator_joint_impedance_controller impedance.launch.py
ros2 launch open_manipulator_contact_responsive_controller contact_responsive.launch.py
ros2 launch open_manipulator_task_space_impedance_controller task_space_impedance.launch.py
ros2 launch open_manipulator_cartesian_contact_controller cartesian_contact.launch.py
```

Choose one command. They share the unit's `current.yaml` and compensation settings, with separate
controller gain YAML files. See each package README for target inputs, tuning, and L100 support.

## Motor/brake startup and shutdown

The runtime initializes the controller and communication in current mode with torque OFF,
prepares gravity current, then enables motors. Friction and external input are admitted only
after Torque Enable and Controller State readback confirms readiness. Preloaded gravity current
alone does not prove support throughout brake release. Firmware brake/goal delays and watchdog
settings are not arbitrarily changed.

Position, current, Torque Enable, and Controller State are read cyclically. Operating Mode,
Bus Watchdog, Goal Update Delay, and Brake Delay are checked using individual driver services
at startup, not cyclic indirect reads. URDF settings do not substitute for drive readback.
The existing HAT power procedure is retained, so a separate prior OMY bringup is unnecessary.
The package-local DHI plugin includes the individual-read service wait fix.

For collection, the initial geometry review and action preparation finish while torque is OFF.
After `PREPARING_HOMING`, the hold command is rechecked and enable requested.
`SETTLING_AFTER_ENABLE` checks stable position/velocity for 0.25 s, waiting at most 5 s.
Small brake-release pose changes are accepted only within the reviewed collision/table margin.
Homing connects smoothly from the settled actual pose; the full geometry review is not repeated
with motors ON. Details are in `homing.json`. Runtime readiness is checked before and during
homing, so collection cannot continue after runtime shutdown.

Runtime monitoring uses a small 20 Hz heartbeat; services are reserved for lifecycle operations.
Heartbeat generation timestamps prevent delayed messages from extending their freshness.
Device tick, sample timing, and tracking checks remain independent. `runtime_monitor.json` and
`runtime/runtime.json` record timing diagnostics. Watchdog and tracking thresholds are not
relaxed to hide communication faults. A normal write response containing only the protocol
Alert bit is handled as success while retaining device diagnostics; one failed OFF write does
not prevent OFF requests to other axes.

Low-speed paired measurements estimate position-command-to-encoder lag before higher-speed
tracking checks. Results are in `position_tracking.json`. This compensates for normal position
profile lag in the tracking check; it changes neither raw data, speed coverage, current-sensor
delay, nor identified friction coefficients.

On `Ctrl+C`, external effort and friction excitation are removed while gravity output supports
the OFF request. After OFF/stop readback, controller and communication processes terminate.
If an emergency stop removed drive power, a requested shutdown cleans up child ROS processes
after the OFF wait (default 8 s). It records `software_stopped: true`,
`all_off_confirmed: false`, and exits with an error rather than claiming physical OFF confirmation.
A communication fault without a shutdown request does not forcibly kill the controller.
Software cannot confirm physical brake state when drive power or communication is lost.

## Gazebo

```bash
ros2 launch open_manipulator_friction_diagnostic calibrate.launch.py \
  use_sim:=true output:=/tmp/omy_calibration_preview
```

The same trajectory/action client drives Gazebo position controllers. `gazebo_overview.mp4`
is an approximately one-minute accelerated recording; `video_summary.json` records simulated
and wall-clock duration and playback speed. Simulated position-servo effort is not real actuator
friction data and cannot overwrite a hardware calibration.

## Package layout

- `launch/`: collection, compensation, legacy impedance, and simulation entry points.
- `open_manipulator_friction_diagnostic/`: planning, acquisition, fitting, storage, lifecycle.
- `src/`, `include/`: real-time compensation and measurement probe controllers.
- `config/`: common collection, fitting, and control settings.
- `calibration/trajectories/`: precomputed per-joint collection/validation trajectories.
- `calibration/geometry_cache/`: reusable fixed-path geometry reviews.
- Unit data directory: runtime models, settings, and acquisition checkpoints outside the package.

Do not delete acquisition data while it is needed for fitting or resume. Keep unit data and
recordings outside the source repository. Mock/simulation results do not establish physical
brake behavior or new-unit compensation quality.

## Regenerating trajectory assets (developers only)

After changing common collection conditions, regenerate and rebuild the package in a ROS environment:

```bash
python3 -m open_manipulator_friction_diagnostic.trajectory_assets \
  --config /path/to/open_manipulator_friction_diagnostic/config/calibration.yaml \
  --output /path/to/open_manipulator_friction_diagnostic/calibration/trajectories
```

Use common sensor/collection settings for distributed assets. Unit-specific timing changes
require matching assets or local regeneration; they do not replace common defaults.
