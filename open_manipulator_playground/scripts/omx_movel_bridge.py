#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Daeyeol Kang
#
# omx_movel_bridge
# Bridges a hand-tracking teleop stream to OpenManipulator-X.

import base64
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import queue
import signal
import struct
import threading
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robotis_interfaces.msg import MoveL
from std_msgs.msg import String


def _clamp(value, low, high):
    return low if value < low else high if value > high else value


# WebSocket framing.
_WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
_WS_MAX_PAYLOAD = 1 << 20  # 1 MiB guard against abusive frames
_WS_OP_TEXT = 0x1
_WS_OP_BINARY = 0x2
_WS_OP_CLOSE = 0x8
_WS_OP_PING = 0x9
_WS_OP_PONG = 0xA


def _ws_accept_key(client_key):
    digest = hashlib.sha1((client_key + _WS_GUID).encode('utf-8')).digest()
    return base64.b64encode(digest).decode('ascii')


def _ws_recv_frame(rfile):
    """Read one (unfragmented) client frame. Returns (opcode, bytes) or None."""
    header = rfile.read(2)
    if len(header) < 2:
        return None
    opcode = header[0] & 0x0F
    masked = header[1] & 0x80
    length = header[1] & 0x7F
    if length == 126:
        ext = rfile.read(2)
        if len(ext) < 2:
            return None
        length = struct.unpack('>H', ext)[0]
    elif length == 127:
        ext = rfile.read(8)
        if len(ext) < 8:
            return None
        length = struct.unpack('>Q', ext)[0]
    if length > _WS_MAX_PAYLOAD:
        return None
    mask = rfile.read(4) if masked else b''
    if masked and len(mask) < 4:
        return None
    data = rfile.read(length) if length else b''
    if len(data) < length:
        return None
    if masked:
        data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    return opcode, data


def _ws_build_frame(opcode, data):
    header = bytearray([0x80 | opcode])  # FIN + opcode
    n = len(data)
    if n < 126:
        header.append(n)
    elif n < 65536:
        header.append(126)
        header += struct.pack('>H', n)
    else:
        header.append(127)
        header += struct.pack('>Q', n)
    return bytes(header) + data


# Default high-level skill pose config.
DEFAULT_POSE_CONFIG = {
    'poses': {
        'ready': {'x': 0.20, 'y': 0.0, 'z': 0.18,
                  'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0, 'duration': 1.0},
        'approach_pick': {'x': 0.20, 'y': 0.0, 'z': 0.16, 'duration': 1.0},
        'lower_pick': {'x': 0.20, 'y': 0.0, 'z': 0.09, 'duration': 1.0},
        'lift': {'x': 0.20, 'y': 0.0, 'z': 0.20, 'duration': 1.0},
        'handover': {'x': 0.18, 'y': -0.12, 'z': 0.18, 'duration': 1.2},
        'above_marked_zone': {'x': 0.18, 'y': 0.12, 'z': 0.18, 'duration': 1.0},
        'marked_zone': {'x': 0.18, 'y': 0.12, 'z': 0.10, 'duration': 1.0},
        'push_start': {'x': 0.20, 'y': 0.0, 'z': 0.08, 'duration': 1.0},
    },
    'skill': {
        'default_duration': 1.0,
        'min_duration': 0.3,
        'settle_margin': 0.3,
        'gripper_settle': 0.6,
        'handover_wait': 0.8,
        'approach_offset': 0.04,
        'lift_offset': 0.06,
        'pick_table_z': 0.09,
        'push_max_distance': 0.10,
    },
    'gripper': {'closed': 0.0, 'open': 1.1, 'max_effort': 10.0},
    'workspace': {'x': [0.12, 0.28], 'y': [-0.16, 0.16], 'z': [0.06, 0.28]},
}


def _deep_merge(base, override):
    result = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _load_pose_config(path, logger=None):
    """Return the default pose config deep-merged with an optional file."""
    if not path:
        return DEFAULT_POSE_CONFIG
    try:
        with open(path, 'r', encoding='utf-8') as cfg_file:
            text = cfg_file.read()
        if path.endswith('.json'):
            loaded = json.loads(text)
        else:
            import yaml  # available in a sourced ROS 2 environment
            loaded = yaml.safe_load(text)
        if isinstance(loaded, dict):
            return _deep_merge(DEFAULT_POSE_CONFIG, loaded)
        if logger is not None:
            logger.warn(f'Pose config {path} is not a mapping; using defaults.')
    except Exception as exc:  # noqa: BLE001 - fall back to safe defaults
        if logger is not None:
            logger.warn(f'Failed to load pose config {path} ({exc}); using defaults.')
    return DEFAULT_POSE_CONFIG


def _list_listeners_on_port(port):
    target = f'{int(port):04X}'
    inodes = set()
    for path in ('/proc/net/tcp', '/proc/net/tcp6'):
        try:
            with open(path, 'r', encoding='utf-8') as net_file:
                rows = net_file.read().splitlines()[1:]
        except OSError:
            continue
        for row in rows:
            parts = row.split()
            if len(parts) < 10:
                continue
            local_addr = parts[1]
            state = parts[3]
            inode = parts[9]
            if state == '0A' and local_addr.rsplit(':', 1)[-1].upper() == target:
                inodes.add(inode)

    listeners = []
    for pid in [p for p in os.listdir('/proc') if p.isdigit()]:
        fd_dir = f'/proc/{pid}/fd'
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                link = os.readlink(f'{fd_dir}/{fd}')
            except OSError:
                continue
            if not (link.startswith('socket:[') and link[8:-1] in inodes):
                continue
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as cmd_file:
                    cmdline = cmd_file.read().replace(b'\0', b' ').decode(
                        'utf-8', errors='replace'
                    ).strip()
            except OSError:
                cmdline = ''
            listeners.append((int(pid), cmdline))
            break
    return listeners


class _TeleopHTTPHandler(BaseHTTPRequestHandler):
    """HTTP POST + WebSocket upgrade; both feed the same ingest_payload."""

    # HTTP/1.1 for keep-alive and upgrade.
    protocol_version = 'HTTP/1.1'

    # The owning bridge node is attached to the server instance as `.bridge`.
    def do_POST(self):  # noqa: N802 (http.server API)
        try:
            length = int(self.headers.get('Content-Length', 0) or 0)
            body = self.rfile.read(length) if length > 0 else b''
            self.server.bridge.ingest_payload(body)
        except Exception:  # never let a bad packet break the Pi's urllib
            pass
        self._reply(b'{"ok":true}')

    def do_GET(self):  # noqa: N802 - WebSocket upgrade or status / health
        if (self.server.bridge.enable_websocket
                and self.headers.get('Upgrade', '').lower() == 'websocket'):
            self._serve_websocket()
            return
        try:
            payload = self.server.bridge.status_json().encode('utf-8')
        except Exception:
            payload = b'{"ok":true}'
        self._reply(payload)

    def _reply(self, payload):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        try:
            self.wfile.write(payload)
        except Exception:
            pass

    # ---- WebSocket: one long-lived connection streaming JSON text frames ----
    def _serve_websocket(self):
        key = self.headers.get('Sec-WebSocket-Key')
        if not key:
            self._reply(b'{"error":"missing Sec-WebSocket-Key"}')
            return
        self.send_response(101)
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', _ws_accept_key(key))
        self.end_headers()
        try:
            self.wfile.flush()
        except Exception:
            return
        bridge = self.server.bridge
        bridge.note_ws_connect()
        try:
            while True:
                frame = _ws_recv_frame(self.rfile)
                if frame is None:
                    break
                opcode, payload = frame
                if opcode == _WS_OP_CLOSE:
                    break
                if opcode == _WS_OP_PING:
                    self._ws_send(_WS_OP_PONG, payload)
                elif opcode in (_WS_OP_TEXT, _WS_OP_BINARY):
                    self._handle_ws_payload(payload)
        except (OSError, ConnectionError):
            pass
        finally:
            bridge.note_ws_disconnect()

    def _handle_ws_payload(self, payload):
        bridge = self.server.bridge
        try:
            obj = json.loads(payload.decode('utf-8'))
        except Exception:
            obj = None
        # Handle command and update shared state.
        if isinstance(obj, dict) and obj.get('type') == 'status':
            try:
                self._ws_send(_WS_OP_TEXT, bridge.status_json().encode('utf-8'))
            except OSError:
                pass
            return
        bridge.ingest_payload(payload)

    def _ws_send(self, opcode, data):
        self.connection.sendall(_ws_build_frame(opcode, data))

    def log_message(self, *args):  # silence default stderr access logging
        return


class _TeleopHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class OmxMoveLBridge(Node):
    """HTTP teleop stream -> cyclo_control MoveL + GripperCommand bridge."""

    def __init__(self):
        super().__init__('omx_movel_bridge')

        # --- Parameters (all defaulted so it runs with no args) ---
        self.declare_parameter('movel_topic', '/omx_movel_controller/movel')
        self.declare_parameter('current_pose_topic', '/omx_movel_controller/current_pose')
        self.declare_parameter('gripper_action', '/gripper_controller/gripper_cmd')
        self.declare_parameter('base_frame', 'link0')

        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8000)
        self.declare_parameter('enable_websocket', True)

        self.declare_parameter('time_from_start', 0.08)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('stale_timeout', 0.35)
        self.declare_parameter('publish_epsilon', 1e-5)
        self.declare_parameter('control_mode', 'auto')
        self.declare_parameter('auto_enable_on_input', True)
        self.declare_parameter('delta_policy', 'latest')
        self.declare_parameter('delta_deadband', 0.0002)
        self.declare_parameter('absolute_smoothing_alpha', 0.65)
        self.declare_parameter('absolute_max_step', 0.02)
        self.declare_parameter('absolute_min_confidence', 0.0)
        self.declare_parameter('absolute_input_max_step', 0.025)
        self.declare_parameter('absolute_input_reacquire_after', 0.25)
        self.declare_parameter('absolute_input_reacquire_step', 0.012)
        self.declare_parameter('interpolation_enabled', True)
        self.declare_parameter('interpolation_time_constant', 0.12)
        self.declare_parameter('interpolation_max_speed', 0.18)
        self.declare_parameter('interpolation_arrival_epsilon', 0.0005)

        # Reachy to robot-axis mapping.
        from rcl_interfaces.msg import ParameterDescriptor
        desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('input_axis_x', 'x', descriptor=desc)
        self.declare_parameter('input_axis_y', 'y', descriptor=desc)
        self.declare_parameter('input_axis_z', 'z', descriptor=desc)
        self.declare_parameter('input_invert_x', False)
        self.declare_parameter('input_invert_y', False)
        self.declare_parameter('input_invert_z', False)
        self.declare_parameter('input_center_x', 0.5)
        self.declare_parameter('input_center_y', 0.5)
        self.declare_parameter('input_center_z', 0.5)
        self.declare_parameter('input_gain_x', 1.0)
        self.declare_parameter('input_gain_y', 1.0)
        self.declare_parameter('input_gain_z', 1.0)
        self.declare_parameter('input_deadzone', 0.0)

        self.declare_parameter('scale_x', 1.4)
        self.declare_parameter('scale_y', 1.4)
        self.declare_parameter('scale_z', 1.2)

        self.declare_parameter('x_min', 0.05)
        self.declare_parameter('x_max', 0.32)
        self.declare_parameter('y_min', -0.22)
        self.declare_parameter('y_max', 0.22)
        self.declare_parameter('z_min', 0.03)
        self.declare_parameter('z_max', 0.35)

        self.declare_parameter('gripper_open_position', 1.1)
        self.declare_parameter('gripper_closed_position', 0.0)
        self.declare_parameter('gripper_max_effort', 10.0)

        # Safe-executor limits for primitive / sequence commands.
        self.declare_parameter('robot_name', 'omx')
        self.declare_parameter('sequence_max_steps', 50)
        self.declare_parameter('sequence_max_duration', 10.0)
        self.declare_parameter('sequence_max_wait', 30.0)
        self.declare_parameter('sequence_max_jump', 0.5)

        # High-level skill command support.
        self.declare_parameter('skill_pose_config', '')
        self.declare_parameter('skill_status_topic', '~/skill_status')

        g = self.get_parameter
        self.movel_topic = g('movel_topic').value
        self.current_pose_topic = g('current_pose_topic').value
        self.gripper_action_name = g('gripper_action').value
        self.base_frame = g('base_frame').value
        self.http_host = g('http_host').value
        self.http_port = int(g('http_port').value)
        self.enable_websocket = bool(g('enable_websocket').value)
        self.time_from_start = float(g('time_from_start').value)
        self.publish_rate = float(g('publish_rate').value)
        self.stale_timeout = float(g('stale_timeout').value)
        self.publish_epsilon = float(g('publish_epsilon').value)
        self.control_mode = str(g('control_mode').value).strip().lower()
        self.auto_enable_on_input = bool(g('auto_enable_on_input').value)
        self.delta_policy = str(g('delta_policy').value).strip().lower()
        self.delta_deadband = float(g('delta_deadband').value)
        self.absolute_smoothing_alpha = _clamp(
            float(g('absolute_smoothing_alpha').value), 0.0, 1.0)
        self.absolute_max_step = max(0.0, float(g('absolute_max_step').value))
        self.absolute_min_confidence = _clamp(
            float(g('absolute_min_confidence').value), 0.0, 1.0)
        self.absolute_input_max_step = max(0.0, float(g('absolute_input_max_step').value))
        self.absolute_input_reacquire_after = max(
            0.0, float(g('absolute_input_reacquire_after').value))
        self.absolute_input_reacquire_step = max(
            0.0, float(g('absolute_input_reacquire_step').value))
        self.interpolation_enabled = bool(g('interpolation_enabled').value)
        self.interpolation_time_constant = max(
            0.001, float(g('interpolation_time_constant').value))
        self.interpolation_max_speed = max(0.0, float(g('interpolation_max_speed').value))
        self.interpolation_arrival_epsilon = max(
            0.0, float(g('interpolation_arrival_epsilon').value))
        def _parse_axis(val):
            # Parse axis.
            if isinstance(val, bool):
                return 'y' if val else 'n'
            return str(val).strip().lower()

        self.input_axis = (
            _parse_axis(g('input_axis_x').value),
            _parse_axis(g('input_axis_y').value),
            _parse_axis(g('input_axis_z').value),
        )
        self.input_invert = (
            bool(g('input_invert_x').value),
            bool(g('input_invert_y').value),
            bool(g('input_invert_z').value),
        )
        self.input_center = (
            float(g('input_center_x').value),
            float(g('input_center_y').value),
            float(g('input_center_z').value),
        )
        self.input_gain = (
            float(g('input_gain_x').value),
            float(g('input_gain_y').value),
            float(g('input_gain_z').value),
        )
        self.input_deadzone = max(0.0, float(g('input_deadzone').value))
        self.scale = (
            float(g('scale_x').value),
            float(g('scale_y').value),
            float(g('scale_z').value),
        )
        self.bounds = (
            (float(g('x_min').value), float(g('x_max').value)),
            (float(g('y_min').value), float(g('y_max').value)),
            (float(g('z_min').value), float(g('z_max').value)),
        )
        self.gripper_open = float(g('gripper_open_position').value)
        self.gripper_closed = float(g('gripper_closed_position').value)
        self.gripper_max_effort = float(g('gripper_max_effort').value)
        self.robot_name = str(g('robot_name').value)
        self.sequence_max_steps = max(1, int(g('sequence_max_steps').value))
        self.sequence_max_duration = max(0.1, float(g('sequence_max_duration').value))
        self.sequence_max_wait = max(0.0, float(g('sequence_max_wait').value))
        self.sequence_max_jump = max(0.0, float(g('sequence_max_jump').value))

        # --- High-level skill config (pose book + timings) ---
        self.pose_config = _load_pose_config(g('skill_pose_config').value, self.get_logger())
        self.poses = self.pose_config.get('poses', {})
        sk = self.pose_config.get('skill', {})
        self.skill_default_duration = float(sk.get('default_duration', 1.0))
        self.skill_min_duration = float(sk.get('min_duration', 0.3))
        self.skill_settle_margin = float(sk.get('settle_margin', 0.3))
        self.skill_gripper_settle = float(sk.get('gripper_settle', 0.6))
        self.skill_handover_wait = float(sk.get('handover_wait', 0.8))
        self.skill_approach_offset = float(sk.get('approach_offset', 0.04))
        self.skill_lift_offset = float(sk.get('lift_offset', 0.06))
        self.skill_pick_table_z = float(sk.get('pick_table_z', 0.09))
        self.skill_push_max_distance = float(sk.get('push_max_distance', 0.10))

        # --- Shared state written by HTTP threads, read by the ROS timer ---
        self._lock = threading.Lock()
        self._delta = [0.0, 0.0, 0.0]      # latest or accumulated dx/dy/dz
        self._absolute_request = None       # latest absolute workspace target
        self._gripper_request = None        # latest pending aperture, or None
        self._enable = False
        self._last_input_time = 0.0         # time.monotonic() of last motion input
        self._input_counts = {}
        self._last_input_summary = 'none'
        self._last_input_log_time = 0.0
        self._ws_clients = 0
        self._last_absolute_input = None
        self._last_absolute_input_time = 0.0
        self._absolute_clamp_count = 0
        self._absolute_drop_count = 0

        # --- ROS-timer-only state (single executor thread, no lock needed) ---
        self.current_pose = None            # latest PoseStamped from controller
        self._need_seed = True
        self._target_pos = None             # [x, y, z] desired absolute goal
        self._interp_pos = None             # [x, y, z] target published after interpolation
        self._seed_orient = None            # (x, y, z, w) frozen at seed time
        self._last_pub_pos = None           # last published target, for epsilon gate
        self._last_gripper_pos = None       # last commanded gripper position
        self._last_timer_time = None

        # --- Skill execution state (worker thread + status) ---
        self._skill_lock = threading.RLock()
        self._skill_queue = queue.Queue()
        self._skill_stop = threading.Event()
        self._skill_active = False
        self._worker_shutdown = False
        # idle | teleop_enabled | running_skill | running_primitive |
        # running_sequence | stopped | error
        self._state = 'idle'
        self._current_skill = None
        self._current_skill_phase = None
        self._last_error = None
        self._last_status_payload = ''
        self._skill_pos = None              # last skill-commanded [x, y, z]
        self._skill_orient = (0.0, 0.0, 0.0, 1.0)
        # primitive / sequence executor + command tracking
        self._current_command_id = None
        self._current_sequence = None
        self._current_step = None
        self._total_steps = None
        self._last_event = None
        self._last_command_id = None
        self._last_result = None
        self._command_seq = 0
        self._skill_handlers = {
            'move_to_home': self._skill_move_to_home,
            'pick_and_handover': self._skill_pick_and_handover,
            'pick_pointed_object': self._skill_pick_pointed_object,
            'point_to_detected_object': self._skill_point_to_detected_object,
            'approach_detected_object': self._skill_approach_detected_object,
            'push_detected_object': self._skill_push_detected_object,
            'pick_detected_object': self._skill_pick_detected_object,
            'pick_and_place_detected_object': self._skill_pick_and_place_detected_object,
            'sort_detected_object': self._skill_sort_detected_object,
            'place_to_zone': self._skill_place_to_zone,
            'push_object': self._skill_push_object,
        }

        # --- ROS interfaces ---
        self.movel_pub = self.create_publisher(MoveL, self.movel_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.current_pose_topic, self._on_current_pose, 10
        )
        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_action_name)
        self.status_pub = self.create_publisher(String, g('skill_status_topic').value, 10)

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.02
        self.timer = self.create_timer(period, self._on_timer)

        # Worker thread for blocking skills.
        self._worker_thread = threading.Thread(target=self._skill_worker, daemon=True)
        self._worker_thread.start()

        # --- HTTP server (stdlib only) on a daemon thread ---
        self._stop_stale_bridge_on_port()
        try:
            self.httpd = _TeleopHTTPServer(
                (self.http_host, self.http_port), _TeleopHTTPHandler)
        except OSError as exc:
            self.get_logger().fatal(
                f'Cannot bind HTTP server on {self.http_host}:{self.http_port} ({exc}). '
                f'Is the port already in use? Set a free port with '
                f'-p http_port:=<port>.'
            )
            raise
        self.httpd.bridge = self
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()

        ws_state = 'on' if self.enable_websocket else 'off'
        self.get_logger().info(
            f'omx_movel_bridge up: HTTP+WebSocket({ws_state}) on '
            f'{self.http_host}:{self.http_port} -> '
            f'MoveL {self.movel_topic} (frame {self.base_frame}), '
            f'gripper action {self.gripper_action_name}'
        )
        self.get_logger().info(
            f'control_mode={self.control_mode}, '
            f'absolute workspace x={self.bounds[0]} y={self.bounds[1]} z={self.bounds[2]}, '
            f'interpolation={self.interpolation_enabled} '
            f'tau={self.interpolation_time_constant:.3f}s '
            f'max_speed={self.interpolation_max_speed:.3f}m/s, '
            f'input_step={self.absolute_input_max_step:.3f}m'
        )
        self.get_logger().info(
            f'input map: axis={self.input_axis} invert={self.input_invert} '
            f'gain={self.input_gain} center={self.input_center} '
            f'deadzone={self.input_deadzone}'
        )

    # ------------------------------------------------------------------ HTTP
    def _stop_stale_bridge_on_port(self):
        markers = ('omx_movel_bridge.py', 'main.py')
        for pid, cmdline in _list_listeners_on_port(self.http_port):
            if pid == os.getpid() or not any(m in cmdline for m in markers):
                continue
            self.get_logger().warn(
                f'Port {self.http_port} is held by stale omx_movel_bridge.py '
                f'PID {pid}; sending SIGTERM before rebinding.'
            )
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                continue
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                try:
                    os.kill(pid, 0)
                except OSError:
                    break
                time.sleep(0.1)

    def ingest_payload(self, body):
        """Parse one POST body (HTTP thread) and fold it into shared state."""
        msg = json.loads(body.decode('utf-8'))
        if not isinstance(msg, dict):
            return
        mtype = msg.get('type')
        if mtype is None and all(axis in msg for axis in ('x', 'y', 'z')):
            mtype = 'absolute'

        if mtype == 'delta':
            dx = float(msg.get('dx', 0.0))
            dy = float(msg.get('dy', 0.0))
            dz = float(msg.get('dz', 0.0))
            if abs(dx) < self.delta_deadband:
                dx = 0.0
            if abs(dy) < self.delta_deadband:
                dy = 0.0
            if abs(dz) < self.delta_deadband:
                dz = 0.0
            with self._lock:
                if self.delta_policy == 'accumulate':
                    self._delta[0] += dx
                    self._delta[1] += dy
                    self._delta[2] += dz
                else:
                    self._delta[0] = dx
                    self._delta[1] = dy
                    self._delta[2] = dz
                if self.auto_enable_on_input:
                    self._enable = True
                self._last_input_time = time.monotonic()
                self._note_input_locked('delta', f'dx={dx:.4f} dy={dy:.4f} dz={dz:.4f}')
        elif mtype in ('absolute', 'workspace', 'pose'):
            target = self._absolute_target_from_msg(msg)
            confidence = float(msg.get('confidence', 1.0))
            with self._lock:
                now = time.monotonic()
                guarded = self._guard_absolute_target_locked(target, confidence, now)
                if guarded is None:
                    self._note_input_locked(
                        'absolute_drop',
                        f'conf={confidence:.2f} '
                        f'target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})')
                    return
                self._absolute_request = guarded
                if self.auto_enable_on_input:
                    self._enable = True
                self._last_input_time = now
                if 'aperture' in msg:
                    self._gripper_request = _clamp(float(msg.get('aperture', 1.0)), 0.0, 1.0)
                self._note_input_locked(
                    'absolute',
                    f'x={guarded[0]:.3f} y={guarded[1]:.3f} z={guarded[2]:.3f}')
        elif mtype == 'gripper':
            aperture = _clamp(float(msg.get('aperture', 1.0)), 0.0, 1.0)
            with self._lock:
                self._gripper_request = aperture
                self._note_input_locked('gripper', f'aperture={aperture:.2f}')
        elif mtype == 'enable':
            value = bool(msg.get('value', False))
            with self._lock:
                self._enable = value
                self._note_input_locked('enable', f'value={value}')
        elif mtype == 'skill':
            skill_name = str(msg.get('skill', '')).strip()
            raw_args = msg.get('args')
            skill_args = raw_args if isinstance(raw_args, dict) else {}
            command_id = msg.get('command_id') or self._next_command_id()
            with self._lock:
                self._note_input_locked('skill', f'skill={skill_name}')
            # HTTP thread only enqueues; the worker thread does all ROS work.
            self._enqueue_skill(skill_name, skill_args, command_id=command_id)
        elif mtype == 'primitive':
            self._ingest_primitive(msg)
        elif mtype == 'sequence':
            self._ingest_sequence(msg)
        elif mtype == 'stop':
            with self._lock:
                self._note_input_locked('stop', 'stop')
            self._request_stop()
        else:
            with self._lock:
                self._note_input_locked('ignored', f'type={mtype} keys={sorted(msg.keys())}')

    def _note_input_locked(self, kind, summary):
        self._input_counts[kind] = self._input_counts.get(kind, 0) + 1
        self._last_input_summary = f'{kind}: {summary}'

    def _ingest_primitive(self, msg):
        # HTTP thread: parse + enqueue a one-step job. No ROS work here.
        cmd = str(msg.get('cmd', '')).strip().lower()
        raw_args = msg.get('args')
        args = raw_args if isinstance(raw_args, dict) else {}
        command_id = msg.get('command_id') or self._next_command_id()
        if cmd == 'stop':
            with self._lock:
                self._note_input_locked('primitive', 'cmd=stop')
            self._request_stop()
            return
        if cmd in ('status', ''):
            with self._lock:
                self._note_input_locked('primitive', f'cmd={cmd}')
            return
        step = dict(args)
        step['cmd'] = cmd
        with self._lock:
            self._note_input_locked('primitive', f'cmd={cmd} id={command_id}')
        self._enqueue_job({
            'kind': 'primitive',
            'name': f'primitive:{cmd}',
            'command_id': command_id,
            'steps': [step],
        })

    def _ingest_sequence(self, msg):
        steps = msg.get('steps')
        name = str(msg.get('name', 'sequence'))
        command_id = msg.get('command_id') or self._next_command_id()
        if not isinstance(steps, list) or not steps:
            with self._lock:
                self._note_input_locked('sequence_drop', 'no steps')
            with self._skill_lock:
                self._last_error = f'sequence {name}: no steps'
            return
        if len(steps) > self.sequence_max_steps:
            with self._lock:
                self._note_input_locked('sequence_drop', f'too many steps {len(steps)}')
            with self._skill_lock:
                self._last_error = (
                    f'sequence {name}: {len(steps)} steps > max {self.sequence_max_steps}')
            return
        with self._lock:
            self._note_input_locked('sequence', f'{name} steps={len(steps)} id={command_id}')
        self._enqueue_job({
            'kind': 'sequence',
            'name': name,
            'command_id': command_id,
            'steps': steps,
        })

    def _absolute_target_from_msg(self, msg):
        def read_axis(axis):
            if axis in msg:
                return float(msg[axis])
            key = f'n{axis}'
            if key in msg:
                return float(msg[key])
            return 0.5

        src = {'x': read_axis('x'), 'y': read_axis('y'), 'z': read_axis('z')}
        units = str(msg.get('units', msg.get('space', 'normalized'))).strip().lower()
        if units in ('m', 'meter', 'meters', 'world', 'workspace'):
            # Already robot-frame meters; pass straight through (clamped).
            return [
                _clamp(src['x'], self.bounds[0][0], self.bounds[0][1]),
                _clamp(src['y'], self.bounds[1][0], self.bounds[1][1]),
                _clamp(src['z'], self.bounds[2][0], self.bounds[2][1]),
            ]

        out = []
        for i in range(3):
            raw = src.get(self.input_axis[i], 0.5)      # remap / swap source axis
            centered = raw - self.input_center[i]       # neutral offset
            if abs(centered) < self.input_deadzone:
                centered = 0.0
            unit = 0.5 + self.input_gain[i] * centered  # sensitivity
            if self.input_invert[i]:                    # direction
                unit = 1.0 - unit
            out.append(self._map_unit_axis(unit, self.bounds[i][0], self.bounds[i][1]))
        return out

    def _guard_absolute_target_locked(self, target, confidence, now):
        if confidence < self.absolute_min_confidence:
            self._absolute_drop_count += 1
            return None

        if self._last_absolute_input is None:
            self._last_absolute_input = list(target)
            self._last_absolute_input_time = now
            return list(target)

        elapsed = max(0.0, now - self._last_absolute_input_time)
        limit = self.absolute_input_max_step
        if elapsed >= self.absolute_input_reacquire_after:
            limit = min(limit, self.absolute_input_reacquire_step)

        delta = [target[i] - self._last_absolute_input[i] for i in range(3)]
        distance = sum(d * d for d in delta) ** 0.5
        if limit > 0.0 and distance > limit:
            scale = limit / distance
            target = [self._last_absolute_input[i] + delta[i] * scale for i in range(3)]
            self._absolute_clamp_count += 1

        self._last_absolute_input = list(target)
        self._last_absolute_input_time = now
        return list(target)

    @staticmethod
    def _map_unit_axis(value, low, high):
        unit = _clamp(float(value), 0.0, 1.0)
        return low + unit * (high - low)

    # --------------------------------------------------------------- ROS cbs
    def _on_current_pose(self, msg):
        self.current_pose = msg

    def _on_timer(self):
        # Snapshot + reset accumulators under the lock.
        with self._lock:
            dx = self._delta[0]
            dy = self._delta[1]
            dz = self._delta[2]
            self._delta[0] = self._delta[1] = self._delta[2] = 0.0
            absolute_request = self._absolute_request
            gripper_request = self._gripper_request
            self._gripper_request = None
            enable = self._enable
            last_input_time = self._last_input_time
            input_counts = dict(self._input_counts)
            input_summary = self._last_input_summary

        self._log_input_status(input_counts, input_summary)
        self._update_teleop_state(enable)

        # Drop teleop input during skill execution.
        if self._skill_active:
            self._need_seed = True
            return

        # Disabled: hold the arm, force a fresh seed when re-enabled.
        if not enable:
            self._need_seed = True
            return

        if self.current_pose is None:
            self.get_logger().warn(
                'No /current_pose yet; is cyclo_control + bringup running?',
                throttle_duration_sec=2.0,
            )
            return

        # Gripper is independent of MoveL; service any pending request first.
        if gripper_request is not None:
            self._send_gripper(gripper_request)

        # Seed absolute target from live pose.
        now = time.monotonic()
        if self._last_timer_time is None:
            dt = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
        else:
            dt = max(0.001, now - self._last_timer_time)
        self._last_timer_time = now
        if self._need_seed or (now - last_input_time) > self.stale_timeout:
            self._seed_from_current_pose()
            self._need_seed = False

        if absolute_request is not None and self.control_mode in ('auto', 'absolute', 'workspace'):
            self._target_pos = self._filtered_absolute_target(absolute_request)
        elif self.control_mode in ('auto', 'delta'):
            # Delta input to absolute target.
            self._target_pos[0] += dx * self.scale[0]
            self._target_pos[1] += dy * self.scale[1]
            self._target_pos[2] += dz * self.scale[2]
        elif (self.control_mode in ('absolute', 'workspace')
                and any(abs(v) > 0.0 for v in (dx, dy, dz))):
            self.get_logger().warn(
                'Received delta input while control_mode is absolute; ignoring arm delta. '
                'Reachy must send {"type":"absolute","x":...,"y":...,"z":...}.',
                throttle_duration_sec=2.0,
            )
        for i in range(3):
            self._target_pos[i] = _clamp(self._target_pos[i], self.bounds[i][0], self.bounds[i][1])

        publish_pos = self._next_interpolated_pos(dt)

        # Only publish when the target actually moved (suppress hold spam).
        if self._target_changed(publish_pos):
            self._publish_movel(publish_pos)
            self._last_pub_pos = list(publish_pos)

    # ----------------------------------------------------------- ROS helpers
    def _seed_from_current_pose(self):
        p = self.current_pose.pose.position
        o = self.current_pose.pose.orientation
        self._target_pos = [p.x, p.y, p.z]
        self._interp_pos = list(self._target_pos)
        self._seed_orient = (o.x, o.y, o.z, o.w)
        with self._lock:
            self._last_absolute_input = list(self._target_pos)
            self._last_absolute_input_time = time.monotonic()

    def _log_input_status(self, counts, summary):
        now = time.monotonic()
        if now - self._last_input_log_time < 2.0:
            return
        self._last_input_log_time = now
        if not counts:
            return
        count_text = ', '.join(f'{key}={value}' for key, value in sorted(counts.items()))
        extras = ''
        if self._absolute_clamp_count or self._absolute_drop_count:
            extras = (
                f'; absolute_clamped={self._absolute_clamp_count} '
                f'absolute_dropped={self._absolute_drop_count}'
            )
        self.get_logger().info(f'HTTP input status: {count_text}; last {summary}{extras}')

    def _filtered_absolute_target(self, requested):
        alpha = self.absolute_smoothing_alpha
        target = [
            self._target_pos[i] + alpha * (requested[i] - self._target_pos[i])
            for i in range(3)
        ]
        if self.absolute_max_step <= 0.0:
            return target

        delta = [target[i] - self._target_pos[i] for i in range(3)]
        distance = sum(d * d for d in delta) ** 0.5
        if distance <= self.absolute_max_step:
            return target

        scale = self.absolute_max_step / distance
        return [self._target_pos[i] + delta[i] * scale for i in range(3)]

    def _next_interpolated_pos(self, dt):
        if self._interp_pos is None or not self.interpolation_enabled:
            self._interp_pos = list(self._target_pos)
            return list(self._interp_pos)

        delta = [self._target_pos[i] - self._interp_pos[i] for i in range(3)]
        distance = sum(d * d for d in delta) ** 0.5
        if distance <= self.interpolation_arrival_epsilon:
            self._interp_pos = list(self._target_pos)
            return list(self._interp_pos)

        alpha = _clamp(dt / self.interpolation_time_constant, 0.0, 1.0)
        step = [d * alpha for d in delta]
        step_distance = sum(d * d for d in step) ** 0.5
        max_step = self.interpolation_max_speed * dt if self.interpolation_max_speed > 0.0 else 0.0
        if max_step > 0.0 and step_distance > max_step:
            scale = max_step / step_distance
            step = [d * scale for d in step]

        self._interp_pos = [self._interp_pos[i] + step[i] for i in range(3)]
        return list(self._interp_pos)

    def _target_changed(self, pos):
        if self._last_pub_pos is None:
            return True
        return any(
            abs(pos[i] - self._last_pub_pos[i]) >= self.publish_epsilon
            for i in range(3)
        )

    def _publish_movel(self, pos):
        msg = MoveL()
        msg.pose.header.frame_id = self.base_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.x = float(self._seed_orient[0])
        msg.pose.pose.orientation.y = float(self._seed_orient[1])
        msg.pose.pose.orientation.z = float(self._seed_orient[2])
        msg.pose.pose.orientation.w = float(self._seed_orient[3])
        sec = int(self.time_from_start)
        msg.time_from_start = Duration(
            sec=sec, nanosec=int((self.time_from_start - sec) * 1e9)
        )
        self.movel_pub.publish(msg)

    def _send_gripper(self, aperture):
        pos = self.gripper_closed + aperture * (self.gripper_open - self.gripper_closed)
        if self._last_gripper_pos is not None and abs(pos - self._last_gripper_pos) < 1e-6:
            return
        if not self.gripper_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                f'Gripper action {self.gripper_action_name} unavailable; skipping.',
                throttle_duration_sec=2.0,
            )
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(pos)
        goal.command.max_effort = float(self.gripper_max_effort)
        self.gripper_client.send_goal_async(goal)
        self._last_gripper_pos = pos
        self.get_logger().info(f'Gripper -> {pos:.4f} rad (aperture {aperture:.2f})')

    # ------------------------------------------------------- skill / status
    def status_json(self):
        with self._skill_lock:
            return json.dumps({
                'robot': self.robot_name,
                'state': self._state,
                'ready': self.current_pose is not None,
                'current_pose_received': self.current_pose is not None,
                'teleop_enabled': self._enable,
                'current_command_id': self._current_command_id,
                'current_skill': self._current_skill,
                'current_sequence': self._current_sequence,
                'current_phase': self._current_skill_phase,
                'current_step': self._current_step,
                'total_steps': self._total_steps,
                'last_event': self._last_event,
                'last_command_id': self._last_command_id,
                'last_result': self._last_result,
                'last_error': self._last_error,
                'queued': self._skill_queue.qsize(),
                'enabled': self._enable,
                'ws_clients': self._ws_clients,
            })

    def note_ws_connect(self):
        with self._lock:
            self._ws_clients += 1
        self.get_logger().info('WebSocket client connected.')

    def note_ws_disconnect(self):
        with self._lock:
            self._ws_clients = max(0, self._ws_clients - 1)
        self.get_logger().info('WebSocket client disconnected.')

    _BUSY_STATES = ('running_skill', 'running_primitive', 'running_sequence', 'error')

    def _update_teleop_state(self, enable):
        with self._skill_lock:
            if not self._skill_active and self._state not in self._BUSY_STATES:
                self._state = 'teleop_enabled' if enable else 'idle'
        self._maybe_publish_status()

    def _emit_event(self, event, command_id=None, success=None):
        # Update tracking only (no ROS publish here): the 60 Hz timer pushes the
        # status topic, and GET /status reads live state. Safe from any thread.
        with self._skill_lock:
            self._last_event = {'event': event, 'command_id': command_id, 'success': success}
            if command_id is not None:
                self._last_command_id = command_id
            if success is not None:
                self._last_result = {'success': success, 'event': event}
        self.get_logger().info(f'event={event} cmd={command_id} ok={success}')

    def _next_command_id(self):
        with self._skill_lock:
            self._command_seq += 1
            return f'auto_{self._command_seq}'

    def _maybe_publish_status(self):
        payload = self.status_json()
        if payload != self._last_status_payload:
            self._last_status_payload = payload
            self.status_pub.publish(String(data=payload))

    def _enqueue_skill(self, skill_name, args, command_id=None):
        if not skill_name:
            self.get_logger().warn('Ignoring skill request with empty "skill" name.')
            with self._skill_lock:
                self._last_error = 'empty skill name'
            return
        self._enqueue_job({
            'kind': 'skill',
            'name': skill_name,
            'command_id': command_id or self._next_command_id(),
            'args': dict(args),
        })

    def _enqueue_job(self, job):
        # HTTP-thread safe: validates busy-state and queues; no ROS publish here.
        cid = job.get('command_id')
        with self._skill_lock:
            busy = self._skill_active or not self._skill_queue.empty()
        if busy:
            self.get_logger().warn(f'Busy ({self._state}); ignoring "{job["name"]}".')
            with self._skill_lock:
                self._last_error = f'ignored {job["name"]}: busy'
            self._emit_event('command_rejected', cid, False)
            return
        self.get_logger().info(f'received {job["kind"]}={job["name"]} id={cid}')
        self._emit_event('command_received', cid, None)
        self._skill_queue.put(job)

    def _request_stop(self):
        self._skill_stop.set()
        self._emit_event('stop_received')
        self.get_logger().warn('Stop requested; aborting current command at next checkpoint.')

    # ---- worker thread (the ONLY place commands publish MoveL/gripper) ----
    def _skill_worker(self):
        while not self._worker_shutdown:
            try:
                job = self._skill_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if job is None:
                break
            self._run_job(job)

    def _run_job(self, job):
        kind = job.get('kind', 'skill')
        name = job.get('name', kind)
        cid = job.get('command_id')
        steps = job.get('steps')
        handler = self._skill_handlers.get(name) if kind == 'skill' else None

        if kind == 'skill' and handler is None:
            self.get_logger().error(f'Unknown skill "{name}".')
            with self._skill_lock:
                self._state = 'error'
                self._last_error = f'unknown skill: {name}'
            self._emit_event('sequence_failed', cid, False)
            return
        if self.current_pose is None:
            self.get_logger().error(f'Cannot run "{name}": no /current_pose yet.')
            with self._skill_lock:
                self._state = 'error'
                self._last_error = f'{name}: no current_pose'
            self._emit_event('sequence_failed', cid, False)
            return

        self._skill_stop.clear()
        run_state = {
            'skill': 'running_skill',
            'primitive': 'running_primitive',
            'sequence': 'running_sequence',
        }.get(kind, 'running_sequence')
        with self._skill_lock:
            self._skill_active = True
            self._state = run_state
            self._current_skill = name
            self._current_sequence = name if kind != 'skill' else None
            self._current_command_id = cid
            self._current_skill_phase = 'starting'
            self._current_step = None
            self._total_steps = len(steps) if isinstance(steps, list) else None
            self._last_error = None
        self._maybe_publish_status()
        self._seed_skill_motion()
        self.get_logger().info(f'{kind} started: {name}')
        self._emit_event('sequence_started', cid, None)

        ok = False
        try:
            if kind == 'skill':
                ok = bool(handler(job.get('args', {})))
            else:
                ok = bool(self._run_steps(steps, cid))
        except Exception as exc:  # noqa: BLE001 - report any failure
            self.get_logger().error(f'{kind} {name} failed: {exc}')
            with self._skill_lock:
                self._last_error = f'{name}: {exc}'
        finally:
            stopped = self._skill_stop.is_set()
            with self._skill_lock:
                self._skill_active = False
                self._current_skill = None
                self._current_sequence = None
                self._current_command_id = None
                self._current_skill_phase = None
                self._current_step = None
                self._total_steps = None
                if stopped:
                    self._state = 'stopped'
                    self._last_error = f'{name}: stopped'
                elif self._last_error is not None:
                    self._state = 'error'
                else:
                    self._state = 'idle'
            self._need_seed = True  # reseed teleop from the live pose
            outcome = 'stopped' if stopped else 'ok' if ok else 'error'
            self.get_logger().info(f'{kind} finished: {name} ({outcome})')
            event = ('sequence_stopped' if stopped
                     else 'sequence_finished' if ok else 'sequence_failed')
            self._emit_event(event, cid, ok and not stopped)
            self._maybe_publish_status()

    # ---- primitive / sequence step executor (worker thread only) ----
    def _run_steps(self, steps, command_id):
        if not isinstance(steps, list):
            with self._skill_lock:
                self._last_error = 'sequence steps not a list'
            return False
        for i, step in enumerate(steps):
            if self._skill_stop.is_set():
                return False
            if not isinstance(step, dict):
                self.get_logger().warn(f'sequence step {i} is not an object; skipping.')
                continue
            with self._skill_lock:
                self._current_step = i
                self._current_skill_phase = f'step{i}:{step.get("cmd", "?")}'
            self._maybe_publish_status()
            self._emit_event('sequence_step_started', command_id, None)
            if not self._run_step(step, command_id):
                return False
        return True

    def _run_step(self, step, command_id):
        cmd = str(step.get('cmd', '')).strip().lower()
        if cmd in ('move_l', 'movel', 'move'):
            return self._step_move_l(step, command_id)
        if cmd == 'gripper':
            return self._step_gripper(step, command_id)
        if cmd in ('wait', 'sleep'):
            seconds = _clamp(
                float(step.get('seconds', step.get('duration', 0.5))),
                0.0, self.sequence_max_wait)
            return self.sleep_or_timer_wait(seconds)
        if cmd in ('move_named_pose', 'named_pose', 'home', 'move_to_home'):
            name = str(step.get('pose', step.get('name', 'ready')))
            if name not in self.poses:
                name = 'ready'
            return self.move_to_named_pose(name)
        if cmd in ('gripper_open', 'open'):
            return self._step_gripper({'aperture': 1.0}, command_id)
        if cmd in ('gripper_close', 'close'):
            return self._step_gripper({'aperture': 0.0}, command_id)
        if cmd == 'stop':
            self._skill_stop.set()
            return False
        if cmd in ('status', 'noop', ''):
            return True
        self.get_logger().warn(f'Unknown step cmd "{cmd}"; skipping.')
        return True

    def _step_move_l(self, step, command_id):
        pose = step.get('pose') if isinstance(step.get('pose'), dict) else step
        frame = str(step.get('frame', '') or (
            pose.get('frame', '') if isinstance(pose, dict) else '')).strip().lower()
        valid = ('', 'robot_base', 'base', 'base_link', 'link0', self.base_frame.lower())
        if frame not in valid:
            self.get_logger().warn(
                f'move_l frame "{frame}" unsupported; commanding in base frame '
                f'{self.base_frame}.')
        try:
            x = float(pose['x'])
            y = float(pose['y'])
            z = float(pose['z'])
        except (KeyError, TypeError, ValueError):
            return self._reject_move_l('missing/invalid x/y/z', command_id)
        ok, reason = self._validate_pose(x, y, z)
        if not ok:
            return self._reject_move_l(f'{reason} ({x:.3f},{y:.3f},{z:.3f})', command_id)
        qx, qy, qz, qw = self._skill_orient
        if isinstance(pose, dict) and all(k in pose for k in ('qx', 'qy', 'qz', 'qw')):
            cand = (float(pose['qx']), float(pose['qy']),
                    float(pose['qz']), float(pose['qw']))
            if all(math.isfinite(v) for v in cand) and any(abs(v) > 1e-9 for v in cand):
                qx, qy, qz, qw = cand
        duration = _clamp(
            float(step.get('duration', self.skill_default_duration)),
            self.skill_min_duration, self.sequence_max_duration)
        self._emit_event('move_l_started', command_id, None)
        self.publish_movel_pose(x, y, z, qx, qy, qz, qw, duration)
        self._emit_event('move_l_published', command_id, True)
        if not self.sleep_or_timer_wait(duration + self.skill_settle_margin):
            return False
        self._emit_event('move_l_reached', command_id, True)
        return True

    def _reject_move_l(self, reason, command_id):
        self.get_logger().error(f'move_l rejected: {reason}')
        with self._skill_lock:
            self._last_error = f'move_l rejected: {reason}'
        self._emit_event('move_l_failed', command_id, False)
        return False

    def _step_gripper(self, step, command_id):
        aperture = _clamp(float(step.get('aperture', 1.0)), 0.0, 1.0)
        self._emit_event('gripper_started', command_id, None)
        self._command_gripper(aperture, force=True)
        done = self.sleep_or_timer_wait(self.skill_gripper_settle)
        self._emit_event('gripper_done' if done else 'gripper_failed', command_id, done)
        return done

    def _validate_pose(self, x, y, z):
        for v in (x, y, z):
            if not math.isfinite(v):
                return False, 'non-finite pose'
        bx, by, bz = self.bounds
        tol = 1e-6
        if not (bx[0] - tol <= x <= bx[1] + tol):
            return False, 'x out of workspace'
        if not (by[0] - tol <= y <= by[1] + tol):
            return False, 'y out of workspace'
        if not (bz[0] - tol <= z <= bz[1] + tol):
            return False, 'z out of workspace'
        if self.current_pose is not None and self.sequence_max_jump > 0.0:
            p = self.current_pose.pose.position
            dist = ((x - p.x) ** 2 + (y - p.y) ** 2 + (z - p.z) ** 2) ** 0.5
            if dist > self.sequence_max_jump:
                return False, f'pose jump {dist:.2f}m exceeds max {self.sequence_max_jump:.2f}m'
        return True, ''

    def _seed_skill_motion(self):
        pose = self.current_pose
        if pose is not None:
            p = pose.pose.position
            o = pose.pose.orientation
            self._skill_pos = [p.x, p.y, p.z]
            self._skill_orient = (o.x, o.y, o.z, o.w)
        else:
            self._skill_pos = [self.bounds[0][0], 0.0, self.bounds[2][0]]
            self._skill_orient = (0.0, 0.0, 0.0, 1.0)

    # ---- motion helpers (skill worker thread only) ----
    def clamp_pose_to_workspace(self, pose):
        return [
            _clamp(pose[0], self.bounds[0][0], self.bounds[0][1]),
            _clamp(pose[1], self.bounds[1][0], self.bounds[1][1]),
            _clamp(pose[2], self.bounds[2][0], self.bounds[2][1]),
        ]

    def publish_movel_pose(self, x, y, z, qx, qy, qz, qw, duration_sec):
        x, y, z = self.clamp_pose_to_workspace([x, y, z])
        duration_sec = max(self.skill_min_duration, float(duration_sec))
        msg = MoveL()
        msg.pose.header.frame_id = self.base_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation.x = float(qx)
        msg.pose.pose.orientation.y = float(qy)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)
        sec = int(duration_sec)
        msg.time_from_start = Duration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        self.movel_pub.publish(msg)
        self._skill_pos = [x, y, z]
        self.get_logger().info(
            f'movel published pose=({x:.3f}, {y:.3f}, {z:.3f}) dur={duration_sec:.2f}')
        return [x, y, z]

    def _resolve_orientation(self, pose):
        if all(k in pose for k in ('qx', 'qy', 'qz', 'qw')):
            return (float(pose['qx']), float(pose['qy']),
                    float(pose['qz']), float(pose['qw']))
        return self._skill_orient

    def move_to_pose_dict(self, pose):
        qx, qy, qz, qw = self._resolve_orientation(pose)
        duration = float(pose.get('duration', self.skill_default_duration))
        self.publish_movel_pose(
            float(pose['x']), float(pose['y']), float(pose['z']), qx, qy, qz, qw, duration)
        return self.sleep_or_timer_wait(duration + self.skill_settle_margin)

    def move_to_named_pose(self, name, overrides=None):
        pose = dict(self.poses.get(name, {}))
        if not pose:
            raise KeyError(f'pose "{name}" not in config')
        if overrides:
            pose.update(overrides)
        return self.move_to_pose_dict(pose)

    def move_delta(self, dx, dy, dz, duration_sec):
        base = list(self._skill_pos) if self._skill_pos is not None else [0.0, 0.0, 0.0]
        target = self.clamp_pose_to_workspace([base[0] + dx, base[1] + dy, base[2] + dz])
        qx, qy, qz, qw = self._skill_orient
        self.publish_movel_pose(target[0], target[1], target[2], qx, qy, qz, qw, duration_sec)
        return self.sleep_or_timer_wait(duration_sec + self.skill_settle_margin)

    def open_gripper(self):
        self._command_gripper(1.0, force=True)
        return self.sleep_or_timer_wait(self.skill_gripper_settle)

    def close_gripper(self):
        self._command_gripper(0.0, force=True)
        return self.sleep_or_timer_wait(self.skill_gripper_settle)

    def _set_skill_phase(self, phase):
        with self._skill_lock:
            self._current_skill_phase = str(phase)
        self._maybe_publish_status()

    def _command_gripper(self, aperture, force=False):
        pos = self.gripper_closed + aperture * (self.gripper_open - self.gripper_closed)
        same = (self._last_gripper_pos is not None
                and abs(pos - self._last_gripper_pos) < 1e-6)
        if not force and same:
            return
        if not self.gripper_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                f'Gripper action {self.gripper_action_name} unavailable; skipping.')
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(pos)
        goal.command.max_effort = float(self.gripper_max_effort)
        self.gripper_client.send_goal_async(goal)
        self._last_gripper_pos = pos
        label = 'open' if aperture >= 0.5 else 'close'
        self.get_logger().info(f'gripper {label} sent (pos={pos:.4f})')

    def sleep_or_timer_wait(self, sec):
        deadline = time.monotonic() + max(0.0, float(sec))
        while time.monotonic() < deadline:
            if self._skill_stop.is_set():
                return False
            time.sleep(0.02)
        return not self._skill_stop.is_set()

    # ---- skill implementations (return True on success, False if aborted) ----
    def _skill_move_to_home(self, args):
        # Go to the home / ready pose. Optionally a different named pose via
        # {"pose": "<name>"}; unknown names fall back to 'ready'.
        name = str(args.get('pose', 'ready')).strip()
        if name not in self.poses:
            name = 'ready'
        return self.move_to_named_pose(name)

    def _skill_pick_and_handover(self, args):
        for name in ('ready', 'approach_pick', 'lower_pick'):
            if not self.move_to_named_pose(name):
                return False
        if not self.close_gripper():
            return False
        for name in ('lift', 'handover'):
            if not self.move_to_named_pose(name):
                return False
        if not self.sleep_or_timer_wait(self.skill_handover_wait):
            return False
        if not self.open_gripper():
            return False
        return self.move_to_named_pose('ready')

    def _skill_pick_pointed_object(self, args):
        pick = self._norm_to_pick_pose(args.get('target_norm', [0.5, 0.5]))
        approach = dict(pick, z=pick['z'] + self.skill_approach_offset)
        lift = dict(pick, z=pick['z'] + self.skill_lift_offset)
        if not self.move_to_pose_dict(approach):
            return False
        if not self.move_to_pose_dict(pick):
            return False
        if not self.close_gripper():
            return False
        return self.move_to_pose_dict(lift)

    def _skill_point_to_detected_object(self, args):
        target = self._target_pose_from_args(
            args, z=self.skill_pick_table_z + self.skill_approach_offset)
        return self.move_to_pose_dict(target)

    def _skill_approach_detected_object(self, args):
        target = self._target_pose_from_args(
            args, z=self.skill_pick_table_z + self.skill_approach_offset)
        return self.move_to_pose_dict(target)

    def _skill_pick_detected_object(self, args):
        pick = self._target_pose_from_args(args, z=self.skill_pick_table_z)
        approach = dict(pick, z=pick['z'] + self.skill_approach_offset)
        lift = dict(pick, z=pick['z'] + self.skill_lift_offset)
        if not self.move_to_pose_dict(approach):
            return False
        if not self.move_to_pose_dict(pick):
            return False
        if not self.close_gripper():
            return False
        return self.move_to_pose_dict(lift)

    def _skill_pick_and_place_detected_object(self, args):
        pick = self._target_pose_from_args(args, z=self.skill_pick_table_z)
        approach = dict(pick, z=pick['z'] + self.skill_approach_offset)
        lift = dict(pick, z=pick['z'] + self.skill_lift_offset)
        above_place, place = self._place_poses_from_args(args)

        self._set_skill_phase('approach_pick')
        if not self.move_to_pose_dict(approach):
            return False
        self._set_skill_phase('open_gripper_near_object')
        if not self.open_gripper():
            return False
        self._set_skill_phase('lower_to_pick')
        if not self.move_to_pose_dict(pick):
            return False
        self._set_skill_phase('grasp')
        if not self.close_gripper():
            return False
        self._set_skill_phase('lift_object')
        if not self.move_to_pose_dict(lift):
            return False
        self._set_skill_phase('move_to_place')
        if not self.move_to_pose_dict(above_place):
            return False
        self._set_skill_phase('lower_to_place')
        if not self.move_to_pose_dict(place):
            return False
        self._set_skill_phase('release')
        if not self.open_gripper():
            return False
        self._set_skill_phase('retreat_from_place')
        if not self.move_to_pose_dict(above_place):
            return False
        self._set_skill_phase('ready')
        return self.move_to_named_pose('ready')

    def _skill_push_detected_object(self, args):
        direction = str(args.get('direction', 'forward')).strip().lower()
        distance = _clamp(float(args.get('distance', 0.04)), 0.0, self.skill_push_max_distance)
        push_start = self._target_pose_from_args(args, z=self.skill_pick_table_z)
        above = dict(push_start, z=push_start['z'] + self.skill_approach_offset)
        delta = self._direction_to_delta(direction, distance)
        if not self.move_to_pose_dict(above):
            return False
        if not self.move_to_pose_dict(push_start):
            return False
        if not self.move_delta(delta[0], delta[1], delta[2], self.skill_default_duration):
            return False
        return self.move_to_pose_dict(above)

    def _skill_sort_detected_object(self, args):
        # Minimal demo fallback: pick the detected object and place it in the
        # marked zone. Future contributors can replace this with color bins.
        return self._skill_pick_and_place_detected_object(args)

    def _skill_place_to_zone(self, args):
        zone = args.get('zone', 'marked_zone')
        place = zone if zone in self.poses else 'marked_zone'
        if not self.move_to_named_pose('above_marked_zone'):
            return False
        if not self.move_to_named_pose(place):
            return False
        if not self.open_gripper():
            return False
        return self.move_to_named_pose('above_marked_zone')

    def _skill_push_object(self, args):
        if 'target_norm' in args or 'target_pose' in args:
            return self._skill_push_detected_object(args)
        direction = str(args.get('direction', 'right')).strip().lower()
        distance = _clamp(float(args.get('distance', 0.04)), 0.0, self.skill_push_max_distance)
        delta = self._direction_to_delta(direction, distance)
        start = dict(self.poses.get('push_start', {}))
        if not start:
            raise KeyError('pose "push_start" not in config')
        above = dict(start, z=start['z'] + self.skill_approach_offset)
        if not self.move_to_pose_dict(above):
            return False
        if not self.move_to_pose_dict(start):
            return False
        if not self.move_delta(delta[0], delta[1], delta[2], self.skill_default_duration):
            return False
        if not self.move_to_pose_dict(above):
            return False
        return self.move_to_named_pose('ready')

    def _norm_to_pick_pose(self, target_norm):
        u = _clamp(float(target_norm[0]), 0.0, 1.0)
        v = _clamp(float(target_norm[1]), 0.0, 1.0)
        y = self.bounds[1][0] + u * (self.bounds[1][1] - self.bounds[1][0])
        x = self.bounds[0][1] - v * (self.bounds[0][1] - self.bounds[0][0])
        return {'x': x, 'y': y, 'z': self.skill_pick_table_z,
                'duration': self.skill_default_duration}

    def _target_pose_from_args(self, args, z=None):
        pose = args.get('target_pose')
        if isinstance(pose, dict) and all(axis in pose for axis in ('x', 'y')):
            return {
                'x': float(pose['x']),
                'y': float(pose['y']),
                'z': float(pose.get('z', self.skill_pick_table_z if z is None else z)),
                'duration': float(pose.get('duration', self.skill_default_duration)),
            }
        target = self._norm_to_pick_pose(args.get('target_norm', [0.5, 0.5]))
        if z is not None:
            target['z'] = float(z)
        return target

    def _place_poses_from_args(self, args):
        pose = args.get('place_pose')
        if isinstance(pose, dict) and all(axis in pose for axis in ('x', 'y')):
            place = {
                'x': float(pose['x']),
                'y': float(pose['y']),
                'z': float(pose.get(
                    'z', self.poses.get('marked_zone', {}).get('z', self.skill_pick_table_z))),
                'duration': float(pose.get('duration', self.skill_default_duration)),
            }
            above = dict(place, z=place['z'] + self.skill_approach_offset)
            return above, place

        zone = args.get('place_zone') or args.get('zone') or 'marked_zone'
        place_name = zone if zone in self.poses else 'marked_zone'
        above_name = f'above_{place_name}'
        place = dict(self.poses.get(place_name, {}))
        if not place:
            raise KeyError(f'pose "{place_name}" not in config')
        above = dict(self.poses.get(above_name, {}))
        if not above:
            above = dict(place, z=place['z'] + self.skill_approach_offset)
        return above, place

    @staticmethod
    def _direction_to_delta(direction, distance):
        table = {
            'forward': (distance, 0.0, 0.0),
            'backward': (-distance, 0.0, 0.0),
            'left': (0.0, distance, 0.0),
            'right': (0.0, -distance, 0.0),
        }
        return table.get(direction, (0.0, -distance, 0.0))

    # ------------------------------------------------------------- lifecycle
    def shutdown(self):
        self._worker_shutdown = True
        self._skill_stop.set()
        try:
            self._skill_queue.put_nowait(None)
        except Exception:
            pass
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OmxMoveLBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
