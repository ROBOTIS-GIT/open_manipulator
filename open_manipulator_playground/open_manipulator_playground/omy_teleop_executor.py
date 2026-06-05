#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
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
# OMY teleop executor: consumes /omy/task_goal (JSON String) and drives the
# existing OMY controllers (arm_controller JointTrajectory + gripper_controller
# GripperCommand). Honors the Reachy sender lifecycle contract and is built for
# SAFETY first — every arm motion goes through a slew loop so nothing jumps.
#
# Anti-jerk model (the key safety design):
#   teleop_delta only nudges a TARGET; it is never published directly. A 20 Hz
#   slew loop moves the COMMAND toward the target by <= max_joint_step_per_tick
#   per tick, so even a huge/instant delta produces smooth, bounded motion.
#   On arm/recovery the target+command are seeded from the ACTUAL current pose,
#   so starting teleop never jumps. Named poses (home/ready) use a distance-
#   scaled duration so they glide instead of snapping.

import json
import time
from pathlib import Path

from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

from open_manipulator_playground.omy_motion_library import (
    TeleopConfig,
    apply_joint_limits,
    clamp_target_to_offset,
    compute_named_pose_joints,
    compute_teleop_delta_joints,
    joints_are_valid,
    max_abs_diff,
    named_pose_duration,
    ramp_up_factor,
    slew_step,
)
from open_manipulator_playground.omy_task_protocol import (
    classify_gripper_command,
    parse_task_goal,
    parse_teleop_delta,
)


class OmyTeleopExecutor(Node):

    def __init__(self):
        super().__init__('omy_teleop_executor')

        self.declare_parameter('config', '')
        self.declare_parameter('arm_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('gripper_action', '/gripper_controller/gripper_cmd')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_state_timeout_sec', 3.0)

        config_path = self.get_parameter('config').get_parameter_value().string_value
        arm_topic = self.get_parameter('arm_topic').get_parameter_value().string_value
        gripper_action = self.get_parameter('gripper_action').get_parameter_value().string_value
        joint_states_topic = (
            self.get_parameter('joint_states_topic').get_parameter_value().string_value)
        self.joint_state_timeout_sec = (
            self.get_parameter('joint_state_timeout_sec').get_parameter_value().double_value)

        self.cfg = self._load_config(config_path)
        self.joint_names = self.cfg.joint_names
        self.n_joints = len(self.joint_names)

        # --- session / slew state ---
        self.armed = False
        self.arm_time = None              # when we armed (for soft-start ramp-up)
        self.current_joint_positions = [0.0] * self.n_joints
        self.last_joint_state_time = None
        self.last_delta_time = None
        self.target_positions = None      # where teleop wants to end up
        self.command_positions = None     # what we publish, slewing toward target
        self.last_gripper_command = None
        self.last_gripper_command_time = 0.0

        self.arm_pub = self.create_publisher(JointTrajectory, arm_topic, 10)
        self.status_pub = self.create_publisher(String, '/omy/task_status', 10)
        self.create_subscription(String, '/omy/task_goal', self._on_task_goal, 10)
        self.create_subscription(JointState, joint_states_topic, self._on_joint_state, 10)
        self.gripper_client = ActionClient(self, GripperCommand, gripper_action)

        self.create_timer(0.5, self._watchdog_tick)
        self._slew_period = 1.0 / max(1.0, self.cfg.slew_rate_hz)
        self.create_timer(self._slew_period, self._slew_tick)

        self.get_logger().info(
            f'OmyTeleopExecutor ready (armed={self.armed}, '
            f'slew={self.cfg.slew_rate_hz}Hz, step={self.cfg.max_joint_step_per_tick}rad, '
            f'max_speed={self.cfg.max_joint_speed_rad_s}rad/s, '
            f'auto_arm={self.cfg.auto_arm_on_delta})')

    # ------------------------------------------------------------------ config
    def _load_config(self, path_str: str) -> TeleopConfig:
        if not path_str:
            self.get_logger().warn('No config parameter; using defaults')
            return TeleopConfig()
        path = Path(path_str)
        if not path.exists():
            self.get_logger().error(f'Config not found: {path}; using defaults')
            return TeleopConfig()
        try:
            with path.open('r') as f:
                raw = yaml.safe_load(f) or {}
            self.get_logger().info(f'Loaded teleop config: {path}')
            return TeleopConfig.from_yaml_dict(raw)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Config parse failed ({exc}); using defaults')
            return TeleopConfig()

    # --------------------------------------------------------------- callbacks
    def _on_joint_state(self, msg: JointState):
        if not set(self.joint_names).issubset(set(msg.name)):
            return
        was_stale = (
            self.last_joint_state_time is None
            or (time.monotonic() - self.last_joint_state_time) > self.joint_state_timeout_sec)

        for i, name in enumerate(self.joint_names):
            self.current_joint_positions[i] = msg.position[msg.name.index(name)]
        self.last_joint_state_time = time.monotonic()

        # On a stale->fresh transition the robot may have moved while we were
        # blind (e.g. a hardware component dropped /joint_states). Force re-arm
        # so the next delta re-seeds from the ACTUAL pose instead of slewing
        # from an outdated command (which would jump).
        if was_stale and self.armed:
            self._disarm_and_settle('joint_states recovered after a gap — re-arm needed')

    def _on_task_goal(self, msg: String):
        result = parse_task_goal(msg.data)
        if not result.ok:
            self._status('', 'failed', result.error)
            return
        self._dispatch(result.goal.task, result.goal.parameters)

    # ---------------------------------------------------------------- watchdog
    def _watchdog_tick(self):
        # Contract: if deltas stop for >command_timeout_sec, OMY must auto-hold
        # (stop) and disarm, not keep moving.
        if not self.armed or self.last_delta_time is None:
            return
        elapsed = time.monotonic() - self.last_delta_time
        if elapsed > self.cfg.command_timeout_sec:
            self.get_logger().warn(
                f'No delta for {elapsed:.1f}s (> {self.cfg.command_timeout_sec}s) — auto-hold')
            self._disarm_and_settle('command timeout — auto-hold')
            self._status('teleop_hold', 'hold', 'Auto-hold: delta timeout')

    # ----------------------------------------------------------------- dispatch
    def _dispatch(self, task: str, params: dict):
        handlers = {
            'ready': lambda p: self._move_named_pose('ready'),
            'home': lambda p: self._move_named_pose('home'),
            'teleop_start': lambda p: self._handle_start(),
            'teleop_delta': self._handle_delta,
            'teleop_hold': lambda p: self._handle_hold(),
            'teleop_stop': lambda p: self._handle_stop(),
            'stop': lambda p: self._handle_stop(emergency=True),
            'open_gripper': lambda p: self._send_gripper('open'),
            'close_gripper': lambda p: self._send_gripper('close'),
        }
        handler = handlers.get(task)
        if handler is None:
            self._status(task, 'failed', f'No handler for task: {task}')
            return
        try:
            handler(params)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().exception(f'Task {task} failed')
            self._status(task, 'failed', str(exc))

    # ------------------------------------------------------------- arm/disarm
    def _handle_start(self):
        # Seed from the actual pose so the first slew step starts exactly where
        # the arm is — no jump on (re)arm. Requires fresh joint states.
        if not self._joint_states_fresh():
            self._status('teleop_start', 'failed', 'No fresh /joint_states; cannot arm')
            return
        self.target_positions = list(self.current_joint_positions)
        self.command_positions = list(self.current_joint_positions)
        self.armed = True
        now = time.monotonic()
        self.arm_time = now               # start the soft-start ramp-up clock
        self.last_delta_time = now
        self._status('teleop_start', 'executing',
                     f'Armed (synced to current pose; soft-start {self.cfg.ramp_up_sec}s)')
        self.get_logger().info(
            f'ARMED — seeded to current pose, ramping up over {self.cfg.ramp_up_sec}s')

    def _handle_hold(self):
        # Contract: after hold, OMY disarms; a new teleop_start is required.
        self._disarm_and_settle('hold requested')
        self._status('teleop_hold', 'hold', 'Holding; disarmed (send teleop_start to resume)')

    def _handle_stop(self, emergency: bool = False):
        self._disarm_and_settle('stop')
        label = 'stop' if emergency else 'teleop_stop'
        self._status(label, 'stopped' if emergency else 'succeeded', 'Disarmed')
        self.get_logger().info('STOP — disarmed')

    def _disarm_and_settle(self, reason: str):
        """Disarm and freeze the target at the actual pose (only if fresh, so we
        never settle onto a stale/outdated pose that would jump later)."""
        self.armed = False
        if self._joint_states_fresh(report=False):
            self.target_positions = list(self.current_joint_positions)
            self.command_positions = list(self.current_joint_positions)
        self.get_logger().info(f'disarmed: {reason}')

    # ---------------------------------------------------------------- teleop
    def _handle_delta(self, params: dict):
        if not self.armed:
            if self.cfg.auto_arm_on_delta:
                self._handle_start()
                if not self.armed:
                    return
            else:
                # Contract: reject deltas received before teleop_start.
                self._status('teleop_delta', 'rejected',
                             'Not armed; send teleop_start first')
                return

        if not self._joint_states_fresh():
            return
        if self.target_positions is None or self.command_positions is None:
            self.target_positions = list(self.current_joint_positions)
            self.command_positions = list(self.current_joint_positions)

        ok, delta, err = parse_teleop_delta(params)
        if not ok:
            self._status('teleop_delta', 'failed', err)
            return

        self.last_delta_time = time.monotonic()

        # A delta only nudges the TARGET — never published directly. The slew
        # loop advances the command gradually, so no jump regardless of size.
        new_target = compute_teleop_delta_joints(
            self.target_positions, delta.dx, delta.dy, delta.dz, self.cfg)
        if not joints_are_valid(new_target, self.n_joints):
            self._status('teleop_delta', 'failed', 'Invalid computed target')
            return
        # Cap how far the target may lead the actual pose (bounds accumulation).
        # During soft-start the allowed offset is scaled down, so right after
        # arming the target can only sit very close to the actual pose — the arm
        # cannot lunge even if the hand is already far off-center.
        ramp = self._ramp_factor()
        offset = self.cfg.max_target_offset * ramp
        new_target = clamp_target_to_offset(
            new_target, self.current_joint_positions, offset)
        self.target_positions = apply_joint_limits(
            new_target, self.joint_names, self.cfg.joint_limits)

        self._status('teleop_delta', 'executing',
                     f'Delta accepted (slewing, ramp={ramp:.2f})')

        action = classify_gripper_command(delta.gripper)
        if action is not None:
            self._send_gripper_debounced(action)

    def _slew_tick(self):
        # The ONLY place arm trajectories are published during teleop.
        if not self.armed or self.target_positions is None or self.command_positions is None:
            return
        if not self._joint_states_fresh(report=False):
            return  # lost feedback — stop commanding new motion
        ramp = self._ramp_factor()
        speed_cap = max(1e-4, self.cfg.max_joint_speed_rad_s * ramp)
        max_step = min(
            self.cfg.max_joint_step_per_tick,
            speed_cap * self._slew_period,
        )
        new_cmd = slew_step(
            self.command_positions, self.target_positions, max_step)
        if max_abs_diff(new_cmd, self.command_positions) < 1e-6:
            return  # already at target; don't spam the controller
        move_distance = max_abs_diff(new_cmd, self.command_positions)
        self.command_positions = apply_joint_limits(
            new_cmd, self.joint_names, self.cfg.joint_limits)
        # The segment duration also enforces the speed cap. This avoids the
        # controller seeing a short time_from_start that implies a fast lunge.
        duration = max(self._slew_period * 2.0, move_distance / speed_cap)
        self._publish_arm(self.command_positions, duration)

    # ------------------------------------------------------------- named pose
    def _move_named_pose(self, pose_name: str):
        # Deliberate move. Refuse while armed so a delta stream and a pose command
        # never fight over the arm.
        if self.armed:
            self._status(pose_name, 'rejected',
                         'Disarm (teleop_stop/hold) before moving to a named pose')
            return
        if not self._joint_states_fresh():
            return
        positions = compute_named_pose_joints(pose_name, self.cfg)
        if positions is None:
            self._status(pose_name, 'failed', f'No predefined pose: {pose_name}')
            return
        if not joints_are_valid(positions, self.n_joints):
            self._status(pose_name, 'failed', f'Pose {pose_name} has wrong joint count')
            return
        target = apply_joint_limits(positions, self.joint_names, self.cfg.joint_limits)
        # Distance-scaled duration so a far pose glides instead of snapping.
        duration = named_pose_duration(target, self.current_joint_positions, self.cfg)
        self._publish_arm(target, duration)
        self.target_positions = list(target)
        self.command_positions = list(target)
        self._status(pose_name, 'executing', f'Moving to {pose_name} ({duration:.1f}s)')

    # ----------------------------------------------------------------- arm pub
    def _publish_arm(self, positions, duration_sec):
        traj = JointTrajectory()
        # Leave header.stamp at 0 ("start now"): a stamped-with-clock trajectory
        # can arrive slightly in the past and make the controller snap to the end
        # point. Zero stamp = begin immediately from the current actual position.
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(seconds=float(duration_sec)).to_msg()
        traj.points.append(pt)
        self.arm_pub.publish(traj)

    # ------------------------------------------------------------------ gripper
    def _send_gripper(self, command: str) -> bool:
        position = (self.cfg.gripper_open_position if command == 'open'
                    else self.cfg.gripper_close_position)
        return self._send_gripper_goal(command, position, self.cfg.gripper_max_effort)

    def _send_gripper_debounced(self, command: str):
        now = time.monotonic()
        if (command == self.last_gripper_command
                and (now - self.last_gripper_command_time) < self.cfg.gripper_debounce_sec):
            return
        self.last_gripper_command = command
        self.last_gripper_command_time = now
        self._send_gripper(command)

    def _send_gripper_goal(self, label, position, max_effort) -> bool:
        # NON-BLOCKING. Never wait_for_server here: it runs in the same callback
        # thread as joint_state/slew and would stall teleop. If the server isn't
        # up, skip quietly — arm teleop keeps working.
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn(
                f'Gripper server not ready; skipping {label}', throttle_duration_sec=2.0)
            return False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(
            lambda _: self._status(label, 'executing', f'Gripper {label} sent'))
        return True

    # ------------------------------------------------------------------ helpers
    def _joint_states_fresh(self, report: bool = True) -> bool:
        if self.last_joint_state_time is None:
            if report:
                self._status('', 'failed', 'No /joint_states yet; arm motion blocked')
            return False
        age = time.monotonic() - self.last_joint_state_time
        if age > self.joint_state_timeout_sec:
            if report:
                self._status('', 'failed', f'/joint_states stale ({age:.2f}s); blocked')
            return False
        return True

    def _ramp_factor(self) -> float:
        if self.arm_time is None:
            return 1.0
        return ramp_up_factor(time.monotonic() - self.arm_time, self.cfg)

    def _status(self, task, status, message):
        payload = json.dumps({'task': task, 'status': status, 'message': message},
                             ensure_ascii=False)
        self.status_pub.publish(String(data=payload))
        self.get_logger().info(payload)


def main(args=None):
    rclpy.init(args=args)
    node = OmyTeleopExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
