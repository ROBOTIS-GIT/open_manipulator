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
import math

from geometry_msgs.msg import PoseArray
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class OmxTrajectoryControllerNode(Node):
    """Unified Controller & Bridge."""

    def __init__(self):
        super().__init__('omx_trajectory_controller_node')
        # Parameters
        self.declare_parameter('trajectory_topic', '/shape_detector_node/drawing_trajectory')
        self.declare_parameter('movel_topic', '/omx_movel_controller/movel')
        self.declare_parameter('drawing_z', 0.01)
        self.declare_parameter('approach_duration', 3.0)
        self.declare_parameter('home_duration', 6.0)
        self.declare_parameter('joint5_angle', 90.0)
        self.declare_parameter('drawing_segment_dur', 0.05)
        self.declare_parameter('resample_step', 0.001)

        self.z_offset = float(self.get_parameter('drawing_z').value)
        self.hover_height = 0.08
        self.home_x = 0.124
        self.home_y = 0.0
        self.home_z = 0.081
        self.approach_dur = float(self.get_parameter('approach_duration').value)
        self.home_dur = float(self.get_parameter('home_duration').value)
        self.joint5_angle = float(self.get_parameter('joint5_angle').value)
        self.base_dur = float(self.get_parameter('drawing_segment_dur').value)
        self.resample_step = float(self.get_parameter('resample_step').value)
        movel_topic = self.get_parameter('movel_topic').value

        # Bridge State
        self.last_positions = None
        self.stable_frames = 0
        self.MAX_STABLE_FRAMES = 2
        self.is_mission_active = False
        self.is_ready = False

        # Subscriptions
        self.traj_sub = self.create_subscription(
            PoseArray,
            self.get_parameter('trajectory_topic').value,
            self.trajectory_callback,
            10
        )
        self.status_sub = self.create_subscription(
            Bool, '/drawing_status', self.status_callback, 10
        )
        self.trigger_sub = self.create_subscription(
            Bool, '/start_drawing', self.trigger_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos_profile_sensor_data
        )

        # Bridge Subscriptions (Relay)
        self.leader_traj_sub = self.create_subscription(
            JointTrajectory, '/leader/joint_trajectory', self.leader_traj_callback, 10
        )
        self.leader_gripper_sub = self.create_subscription(
            JointTrajectory,
            '/leader/gripper_controller/joint_trajectory',
            self.leader_gripper_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/drawing_status', 10)
        self.movel_pub = self.create_publisher(MoveL, movel_topic, 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10
        )
        self.joint_state_pub = self.create_publisher(JointState, '/cyclo_joint_states', 10)
        self.arm_traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        # Internal State
        self.is_executing = False
        self.points = []
        self.queued_points = []
        self.current_point_index = 0
        self.move_end_time = 0.0

        # Timers
        self.execution_timer = self.create_timer(0.02, self.execution_loop)
        self._auto_home_timer = self.create_timer(1.0, self.auto_home_once)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info('Unified OMX Controller & Bridge Started')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'drawing_z':
                self.z_offset = param.value
            elif param.name == 'drawing_segment_dur':
                self.base_dur = param.value
            elif param.name == 'resample_step':
                self.resample_step = param.value
            elif param.name == 'approach_duration':
                self.approach_dur = param.value
            elif param.name == 'home_duration':
                self.home_dur = param.value
            elif param.name == 'joint5_angle':
                self.joint5_angle = param.value
        return SetParametersResult(successful=True)

    def status_callback(self, msg):
        previous_active = self.is_mission_active
        self.is_mission_active = msg.data
        if self.is_mission_active and not previous_active:
            self.stable_frames = 0
            self.get_logger().info('Mission Started: Bridge Gate Fully Open')
        elif not self.is_mission_active and previous_active:
            self.get_logger().info('Mission Ended: Bridge Gate in Mute-on-Stable mode')

    def joint_state_callback(self, msg):
        self.joint_state_pub.publish(msg)

    def leader_traj_callback(self, msg):
        if not msg.points:
            return
        if not self.is_mission_active:
            if self.stable_frames >= self.MAX_STABLE_FRAMES:
                if self.stable_frames == self.MAX_STABLE_FRAMES:
                    self.get_logger().info('Bridge Gate Closed - Muting redundant packets.')
                return

        # Joint 5 Lock
        joint5_target = math.radians(self.joint5_angle)
        if 'joint5' in msg.joint_names:
            j5_idx = msg.joint_names.index('joint5')
            for p in msg.points:
                pos_array = list(p.positions)
                pos_array[j5_idx] = joint5_target
                p.positions = pos_array

        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        current_pos = msg.points[0].positions
        if self.last_positions is not None:
            is_moving = False
            for i in range(min(len(current_pos), len(self.last_positions))):
                if abs(current_pos[i] - self.last_positions[i]) > 0.002:
                    is_moving = True
                    break
            if not is_moving:
                self.stable_frames += 1
            else:
                self.stable_frames = 0

        if self.is_mission_active or self.stable_frames < self.MAX_STABLE_FRAMES:
            self.arm_traj_pub.publish(msg)
        self.last_positions = current_pos

    def leader_gripper_callback(self, msg):
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        self.gripper_pub.publish(msg)

    def auto_home_once(self):
        if self.last_positions is None:
            self.get_logger().info('Waiting for joint state sync (last_positions is None)...')
            return

        self._auto_home_timer.cancel()
        self.status_pub.publish(Bool(data=True))
        self.get_logger().info('Joint states synced. Moving to Home position slowly (7s)...')
        self.publish_movel(self.home_x, self.home_y, self.home_z, 7.0)
        self.is_ready = True

    def trigger_callback(self, msg):
        if msg.data and self.queued_points:
            self.start_mission(self.queued_points)
            self.queued_points = []

    def start_mission(self, points):
        self.get_logger().info(f'Starting mission with {len(points)} points')
        self.close_gripper()
        self.points = points
        self.is_executing = True
        self.current_point_index = 0
        self.move_end_time = self.get_clock().now().nanoseconds / 1e9
        self.status_pub.publish(Bool(data=True))

    def close_gripper(self):
        msg = JointTrajectory()
        msg.joint_names = ['gripper_left_joint']
        p = JointTrajectoryPoint(positions=[-0.5])
        p.time_from_start.nanosec = 500000000
        msg.points.append(p)
        self.gripper_pub.publish(msg)
        self.get_logger().info('Gripper Command: Closing (joint: gripper_left_joint)')

    def get_quaternion_from_euler(self, r, p, y):
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)
        return [
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr
        ]

    def publish_movel(self, x, y, z, duration):
        msg = MoveL()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = max(0.001, float(z))
        pan = math.atan2(y, x)
        pitch = math.radians(90)
        roll = math.radians(-90)
        q = self.get_quaternion_from_euler(roll, pitch, pan)
        msg.pose.pose.orientation.w = q[0]
        msg.pose.pose.orientation.x = q[1]
        msg.pose.pose.orientation.y = q[2]
        msg.pose.pose.orientation.z = q[3]
        msg.time_from_start.sec = int(math.floor(duration))
        msg.time_from_start.nanosec = int((duration - math.floor(duration)) * 1e9)
        self.movel_pub.publish(msg)

    def resample_path(self, points_xy):
        if len(points_xy) < 2:
            return points_xy
        dists = [0.0]
        for i in range(1, len(points_xy)):
            v = math.sqrt((points_xy[i][0] - points_xy[i - 1][0])**2 + (
                points_xy[i][1] - points_xy[i - 1][1])**2)
            dists.append(dists[-1] + v)
        if dists[-1] < self.resample_step:
            return points_xy
        res, target, j = [points_xy[0]], self.resample_step, 1
        while target < dists[-1] and j < len(points_xy):
            while j < len(points_xy) and dists[j] < target:
                j += 1
            if j >= len(points_xy):
                break
            denom = dists[j] - dists[j - 1]
            if denom > 1e-9:
                t = (target - dists[j - 1]) / denom
                nx = points_xy[j - 1][0] + t * (points_xy[j][0] - points_xy[j - 1][0])
                ny = points_xy[j - 1][1] + t * (points_xy[j][1] - points_xy[j - 1][1])
                res.append((nx, ny))
            target += self.resample_step
        res.append(points_xy[-1])
        return res

    def trajectory_callback(self, msg):
        if not msg.poses:
            return
        paths, current = [], []
        for p in msg.poses:
            if p.position.z < 0:
                if current:
                    paths.append(current)
                    current = []
            else:
                current.append(p)
        if current:
            paths.append(current)
        track = []
        for pi, path in enumerate(paths):
            raw = []
            for p in path:
                cx, cy = p.position.x, p.position.y
                r = math.sqrt(cx**2 + cy**2)
                if r < 0.08 or r > 0.32:
                    scale = min(max(r, 0.08), 0.32) / r
                    cx *= scale
                    cy *= scale
                raw.append((cx, cy))
            if len(raw) < 2:
                continue
            resampled = self.resample_path(raw)
            sx, sy = resampled[0]

            # Path Planning
            if pi == 0:
                for p_appr in [
                    {'x': sx, 'y': sy, 'z': self.hover_height, 'dur': 2.0},
                    {'x': sx, 'y': sy, 'z': self.z_offset, 'dur': self.approach_dur},
                    {'x': sx, 'y': sy, 'z': self.z_offset, 'dur': 0.5}
                ]:
                    track.append(p_appr)
            else:
                # Path Transition (Restored stabilization)
                for p_trans in [
                    {'x': sx, 'y': sy, 'z': self.hover_height, 'dur': 0.5},
                    {'x': sx, 'y': sy, 'z': self.z_offset, 'dur': self.approach_dur},
                    {'x': sx, 'y': sy, 'z': self.z_offset, 'dur': 0.2}
                ]:
                    track.append(p_trans)

            for i in range(1, len(resampled)):
                pt, dur = resampled[i], self.base_dur
                if i == 1:
                    dur = self.base_dur * 2.5

                if i < len(resampled) - 1:
                    v1 = (pt[0] - resampled[i - 1][0], pt[1] - resampled[i - 1][1])
                    v2 = (resampled[i + 1][0] - pt[0], resampled[i + 1][1] - pt[1])
                    n1 = math.sqrt(v1[0]**2 + v1[1]**2)
                    n2 = math.sqrt(v2[0]**2 + v2[1]**2)
                    if n1 > 0 and n2 > 0:
                        dot = v1[0] * v2[0] + v1[1] * v2[1]
                        ang = math.acos(max(-1.0, min(1.0, dot / (n1 * n2))))
                        if ang > math.radians(30):
                            dur = 0.18
                        elif ang > math.radians(10):
                            dur = 0.10
                        elif ang > math.radians(2):
                            dur = 0.07
                r_d = math.sqrt(pt[0]**2 + pt[1]**2)
                if r_d > 0.21:
                    dur *= 2.0
                elif r_d > 0.18:
                    dur *= 1.5
                track.append({'x': pt[0], 'y': pt[1], 'z': self.z_offset, 'dur': dur})

            lx, ly = resampled[-1]
            # Lift
            for p_lift in [
                {'x': lx, 'y': ly, 'z': self.z_offset, 'dur': 0.5},
                {'x': lx, 'y': ly, 'z': 0.08, 'dur': 2.0},
                {'x': lx, 'y': ly, 'z': 0.08, 'dur': 1.0}
            ]:
                track.append(p_lift)

        # Home Position
        for p_home in [
            {'x': self.home_x, 'y': self.home_y, 'z': self.home_z - 0.001, 'dur': self.home_dur},
            {'x': self.home_x, 'y': self.home_y, 'z': self.home_z, 'dur': 3.0}
        ]:
            track.append(p_home)
        self.get_logger().info(f'Trajectory processed: {len(track)} points')

        if not self.is_ready:
            self.get_logger().warn('Mission rejected: Robot not yet homed/synced.')
            return

        if not self.is_executing:
            self.get_logger().info('Starting mission immediately')
            self.start_mission(track)
        else:
            self.get_logger().info('Mission in progress, queuing trajectory')
            self.queued_points = track

    def execution_loop(self):
        if not self.is_executing or not self.points:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now >= self.move_end_time:
            if self.current_point_index < len(self.points):
                c = self.points[self.current_point_index]
                self.publish_movel(c['x'], c['y'], c['z'], c['dur'])
                self.move_end_time = now + c['dur']
                self.current_point_index += 1
            else:
                self.is_executing = False
                self.status_pub.publish(Bool(data=False))
                if self.queued_points:
                    self.start_mission(self.queued_points)
                    self.queued_points = []


def main(args=None):
    rclpy.init(args=args)
    node = OmxTrajectoryControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
