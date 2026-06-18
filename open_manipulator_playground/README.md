# OpenManipulator Playground

This package keeps experimental OpenManipulator tools that are useful while
validating real hardware flows.

## OMX MoveL hand teleop bridge

`scripts/omx_movel_bridge.py` bridges the Reachy Mini hand-tracking HTTP stream
to the real OpenManipulator-X / OMX-F stack through the cyclo_control MoveL
controller.

Flow:

```text
Reachy Mini teleop --HTTP JSON--> omx_movel_bridge --MoveL / GripperCommand--> cyclo_control --> OMX-F
```

The bridge accepts these JSON POST bodies at `http://<PC>:<port>/`:

```json
{"type":"enable","source":"reachy_mini","value":true,"stamp":0.0}
{"type":"delta","source":"reachy_mini","dx":0.01,"dy":0.0,"dz":0.0,"confidence":1.0,"stamp":0.0}
{"type":"gripper","source":"reachy_mini","aperture":1.0,"stamp":0.0}
```

Every POST returns `200 OK`. Deltas are accumulated as absolute end-effector
targets for `/omx_movel_controller/movel` because cyclo_control MoveL does not
accumulate deltas internally.

### PC startup

Source the ROS 2 Jazzy workspace that contains this repository, cyclo_control,
and `robotis_interfaces`, then run:

```bash
source /home/robotis/ros2_ws/install/setup.bash
source /home/robotis/open_manipulator/install/setup.bash
ros2 launch open_manipulator_playground omx_hand_teleop.launch.py http_port:=18001
```

Stop it with one `Ctrl+C` in that launch terminal. The launch starts OMX-F
bringup, cyclo MoveL, the local trajectory relay, and the HTTP bridge together.

For manual startup, run these in separate terminals. `main.py` is the single
control entry point: it runs the HTTP bridge (teleop + 5 skills) **and** the
joint-trajectory relay in one process, so no separate relay terminal is needed:

```bash
source /home/robotis/ros2_ws/install/setup.bash
source /home/robotis/open_manipulator/install/setup.bash
ros2 launch open_manipulator_bringup omx_f.launch.py
ros2 launch cyclo_motion_controller_ros omx_controller.launch.py controller_type:=movel
/usr/bin/python3 open_manipulator_playground/scripts/main.py --ros-args -p http_port:=18001
```

After installing the package, the control side can also be started with:

```bash
ros2 run open_manipulator_playground main.py --ros-args -p http_port:=8000
```

### Pi startup

```bash
python teleop/teleop_runtime.py --sdk-camera --host <PC_IP> --port 18001
```

### Hardware checks

Confirm the controller is publishing the current end-effector pose:

```bash
ros2 topic echo /omx_movel_controller/current_pose
```

Confirm the bridge accepts HTTP:

```bash
curl -s -o /dev/null -w "%{http_code}\n" \
  -X POST localhost:8000 \
  -d '{"type":"enable","value":true}'
```

Confirm MoveL targets move in absolute pose coordinates:

```bash
curl -s -o /dev/null -w "%{http_code}\n" \
  -X POST localhost:8000 \
  -d '{"type":"delta","dx":0.01,"dy":0.0,"dz":0.0}'

ros2 topic echo /omx_movel_controller/movel
```

Confirm the gripper action path:

```bash
curl -s -o /dev/null -w "%{http_code}\n" \
  -X POST localhost:8000 \
  -d '{"type":"gripper","aperture":0.0}'

curl -s -o /dev/null -w "%{http_code}\n" \
  -X POST localhost:8000 \
  -d '{"type":"gripper","aperture":1.0}'
```

### High-level skill commands

On top of the teleop `delta`/`absolute`/`gripper`/`enable` contract, the bridge
accepts `type:"skill"` commands. Reachy Mini decides *what* to do (from
camera/NL) and POSTs a skill; the PC bridge turns it into a safe absolute MoveL
pose sequence plus GripperCommand actions.

Design rules:

- The HTTP handler thread **never** publishes MoveL or calls the gripper action.
  It only pushes the request onto a thread-safe queue.
- A dedicated worker thread runs the (blocking, timed) skill sequence and is the
  only place skills publish. Teleop is suppressed while a skill runs, then the
  target is reseeded from the live pose to avoid a jump.
- While a skill is running, new skills are **ignored** (logged + recorded in
  `last_error`). Every POST still returns `200 OK`.
- `current_pose` is required before any skill runs; otherwise it errors out.

Supported skills:

| skill | args | sequence |
|-------|------|----------|
| `pick_and_handover` | `{"target":"default_object"}` | ready → approach_pick → lower_pick → close → lift → handover → wait → open → ready |
| `pick_pointed_object` | `{"target_norm":[u,v],"confidence":..}` | norm→pick (linear map, fixed table z) → approach → pick → close → lift |
| `place_to_zone` | `{"zone":"marked_zone"}` | above_marked_zone → place pose → open → retreat |
| `push_object` | `{"direction":"left/right/forward/backward","distance":0.04}` | push approach → push_start → move_delta(direction) → retreat → ready |

Stop the running skill at the next checkpoint:

```bash
curl -X POST http://127.0.0.1:18001/ -d '{"type":"stop"}'
```

Live status (also published on the `~/skill_status` topic as `std_msgs/String`):

```bash
curl -s http://127.0.0.1:18001/
# {"state":"running_skill","current_skill":"pick_and_handover","last_error":null,"queued":0,"enabled":true}
```

`state` is one of `idle`, `teleop_enabled`, `running_skill`, `error`.

Examples:

```bash
curl -X POST http://127.0.0.1:18001/ -H "Content-Type: application/json" \
  -d '{"type":"skill","skill":"pick_and_handover","args":{"target":"default_object"}}'

curl -X POST http://127.0.0.1:18001/ -H "Content-Type: application/json" \
  -d '{"type":"skill","skill":"pick_pointed_object","args":{"target_norm":[0.62,0.48],"confidence":0.9}}'

curl -X POST http://127.0.0.1:18001/ -H "Content-Type: application/json" \
  -d '{"type":"skill","skill":"place_to_zone","args":{"zone":"marked_zone"}}'

curl -X POST http://127.0.0.1:18001/ -H "Content-Type: application/json" \
  -d '{"type":"skill","skill":"push_object","args":{"direction":"right","distance":0.04}}'
```

Skill poses and timings live in [config/omx_skill_poses.yaml](config/omx_skill_poses.yaml)
(in-code defaults are deep-merged with this file). Point the bridge at it with
`-p skill_pose_config:=<path>/omx_skill_poses.yaml`. **The default poses are
conservative placeholders — calibrate them on the real OMX-F.** All skill poses
are clamped to the ROS `x_min`/`x_max`/… workspace parameters.

### Primitive / Sequence executor (Safe Motion Executor)

The bridge is a **safe motion executor**: Reachy Mini (or any client) plans
*what* and *where*; the bridge validates and runs it on Cyclo Control MoveL.
On top of teleop and skills it accepts `type:"primitive"` and `type:"sequence"`.

Primitive commands (`cmd`: `move_l` | `gripper` | `wait`):

```bash
curl -X POST http://127.0.0.1:18001/ -H "Content-Type: application/json" -d '{
  "type":"primitive","robot":"omx","cmd":"move_l","command_id":"cmd_001",
  "args":{"pose":{"x":0.20,"y":0.0,"z":0.16,"qx":0.0,"qy":0.0,"qz":0.0,"qw":1.0},
          "duration":0.8,"frame":"robot_base"}}'

curl -X POST http://127.0.0.1:18001/ -d '{"type":"primitive","cmd":"gripper","args":{"aperture":0.0}}'
curl -X POST http://127.0.0.1:18001/ -d '{"type":"primitive","cmd":"wait","args":{"seconds":0.5}}'
```

Sequence (steps run in order; supported `cmd`: `move_l`, `gripper`, `wait`,
`move_named_pose`, `gripper_open`/`gripper_close`):

```bash
curl -X POST http://127.0.0.1:18001/ -H "Content-Type: application/json" -d '{
  "type":"sequence","robot":"omx","name":"pick_at_pose","command_id":"seq_001",
  "steps":[
    {"cmd":"move_l","pose":{"x":0.21,"y":0.0,"z":0.16},"duration":0.8},
    {"cmd":"move_l","pose":{"x":0.21,"y":0.0,"z":0.085},"duration":0.8},
    {"cmd":"gripper","aperture":0.0},
    {"cmd":"wait","seconds":0.4},
    {"cmd":"move_l","pose":{"x":0.21,"y":0.0,"z":0.18},"duration":0.8}
  ]}'
```

`pose` is in `link0` (frames `robot_base`/`base`/`link0` accepted; others warn
and use the base frame). Orientation is optional — omit it to keep the EE
orientation captured when the command starts.

**Safety (enforced by the executor):** rejects when no `current_pose` yet;
rejects non-finite (NaN/inf) poses; rejects poses outside the `x/y/z`
workspace; rejects pose jumps larger than `sequence_max_jump` (0.5 m);
clamps each step `duration` to `[min_duration, sequence_max_duration]`; clamps
gripper aperture to 0..1; caps a sequence at `sequence_max_steps` (50); ignores
a new command while one runs; suppresses teleop while a command runs; `stop`
aborts at the next checkpoint.

**Status / feedback** (also `~/skill_status` topic):

```bash
curl http://127.0.0.1:18001/status
```
```json
{"robot":"omx","state":"running_sequence","ready":true,
 "current_command_id":"seq_001","current_sequence":"pick_at_pose",
 "current_step":1,"total_steps":5,"teleop_enabled":false,
 "last_event":{"event":"move_l_published","command_id":"seq_001","success":true},
 "last_command_id":"seq_001","last_result":{"success":true,"event":"move_l_published"},
 "last_error":null}
```

`state` ∈ `idle | teleop_enabled | running_skill | running_primitive |
running_sequence | stopped | error`. Events include `command_received`,
`move_l_started/published/reached/failed`, `gripper_started/done/failed`,
`sequence_started/step_started/finished/failed/stopped`, `stop_received`.
(`move_l_reached` currently fires after the commanded duration elapses; a
`current_pose`-error based arrival check can be added later.)

Existing `type:"skill"` (the 11 named skills) is kept for backward
compatibility and runs through the same worker/state machine.

### Tuning skill poses on the real robot (teach, don't guess)

The shipped pose books (`config/omx_skill_poses.yaml`, `config/omy_skill_poses.yaml`)
are conservative placeholders. Tune them per robot by reading the controller's
real end-effector pose instead of guessing:

```bash
# OMY (use /omx_movel_controller/current_pose for OMX)
ros2 run open_manipulator_playground pose_capture.py \
  --ros-args -p current_pose_topic:=/omy_movel_controller/current_pose
```

Jog the arm to each spot, then type a name to capture it:

- Jog with the cyclo interactive marker (drag the EE in rviz):
  `ros2 launch cyclo_motion_controller_ros omy_controller.launch.py
  controller_type:=movel start_interactive_marker:=true`, or just hand teleop.
- In `pose_capture`: type `ready`, `approach_pick`, `lower_pick`, … to capture
  the live pose under that name; `ws` prints the workspace box you swept; `list`
  / `q` print the full pose book. Paste the output into the robot's
  `*_skill_poses.yaml` (orientation is emitted as a comment — add it only if you
  want a fixed tool pose).

This reuses the existing `current_pose` topic, so the numbers are guaranteed
reachable on that specific arm.

### WebSocket (persistent connection, recommended for streaming)

The HTTP POST contract opens a new TCP connection per tick, which can stutter
when Reachy streams at a few Hz. The bridge also accepts a **WebSocket** on the
**same port** (`http_port`), so the Pi connects once and streams the *same* JSON
as text frames over one long-lived connection (full-duplex, low per-message
overhead). Plain HTTP POST keeps working unchanged, so `curl` tests are fine.

- Connect to `ws://<PC>:18001/` and send one JSON object per text frame — the
  exact same `delta` / `absolute` / `gripper` / `enable` / `skill` / `stop`
  payloads as HTTP.
- Send `{"type":"status"}` and the bridge replies with a status frame on the
  same socket (state / current_skill / last_error / ws_clients).
- Ping/pong is handled automatically. Disable WebSocket with
  `-p enable_websocket:=false` (HTTP-only).

Minimal Pi client (`pip install websocket-client`):

```python
import json
import websocket  # websocket-client

ws = websocket.create_connection('ws://<PC_IP>:18001/')
ws.send(json.dumps({'type': 'enable', 'value': True}))
ws.send(json.dumps({'type': 'delta', 'dx': 0.0, 'dy': 0.01, 'dz': 0.0}))   # teleop
ws.send(json.dumps({'type': 'skill', 'skill': 'pick_and_handover', 'args': {}}))
ws.send(json.dumps({'type': 'status'}))
print(ws.recv())   # -> {"state":"running_skill", ...}
```

### Important pitfalls

`cyclo_motion_controller_ros` publishes joint trajectory commands on
`/leader/joint_trajectory`, while `open_manipulator_bringup`'s arm controller
subscribes to `/arm_controller/joint_trajectory`. `main.py` already runs the
relay between them in-process, so the arm moves without an extra terminal. (The
standalone `omx_joint_trajectory_relay.py` is still available if you run the
bridge by itself instead of `main.py`.)

The gripper is separate from MoveL. Arm motion goes through
`/omx_movel_controller/movel`; gripper commands go through the
`/gripper_controller/gripper_cmd` `control_msgs/action/GripperCommand` action.

For first hardware motion, shrink the workspace with parameters such as
`x_min`, `x_max`, `y_min`, `y_max`, `z_min`, and `z_max`, start from a hold
state, move slowly, and keep E-stop ready.

If Conda or another user Python is active, ROS 2 code generation and runtime
imports can resolve the wrong Python environment. Deactivate Conda or put
`/usr/bin` before user Python paths when building/running ROS packages.
