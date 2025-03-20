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
# Author: Sungho Woo

import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState  # Import JointState message
import math

class MoveToHome(Node):
    def __init__(self):
        super().__init__('move_to_home')
        self.publisher = self.create_publisher(JointTrajectory, '/leader/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Target joint positions (in radians)
        self.target_positions = [
            0.0,
            -math.pi / 2,
            self.angle_to_radian(152),
            self.angle_to_radian(-62),
            -math.pi / 2,
            0.0,
            0.0
        ]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'rh_r1_joint']

        self.epsilon = 0.5  # Allowable error threshold (radians)
        self.reached_target = False  # Flag to check if the target has been reached

        # Timer to trigger movement after 1 second
        self.timer = self.create_timer(1.0, self.move_to_home)

    def move_to_home(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        point.time_from_start.sec = 2  # Movement should be completed within 2 seconds

        traj.points.append(point)
        self.publisher.publish(traj)
        self.get_logger().info('Moving to Home Position')

    def joint_state_callback(self, msg):
        if set(self.joint_names).issubset(set(msg.name)):
            current_positions = [msg.position[msg.name.index(j)] for j in self.joint_names]

            # Check if all joints are within the acceptable error range
            if all(abs(curr - target) < self.epsilon for curr, target in zip(current_positions, self.target_positions)):
                if not self.reached_target:
                    self.get_logger().info("🎯 Target position reached! Shutting down node.")
                    self.reached_target = True
                    self.shutdown_node()

    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def angle_to_radian(self, angle):
        return angle * math.pi / 180

def main(args=None):
    rclpy.init(args=args)
    node = MoveToHome()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
