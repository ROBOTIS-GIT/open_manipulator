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

import math
import sys

from control_msgs.action import FollowJointTrajectory
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class MoveToHome(Node):
    """Node to move the robot arm to home position in pack or unpack mode."""

    def __init__(self):
        super().__init__('move_to_home')

        # Declare and get parameters
        self.declare_parameter('operation_mode', 'pack')
        self.operation_mode = (
            self.get_parameter('operation_mode').get_parameter_value().string_value
        )

        # Create action client for FollowJointTrajectory
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Joint names
        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'rh_r1_joint',
        ]

        # Final target positions (in radians)
        self.final_target_positions = [
            self.angle_to_radian(-90),
            0.0,
            self.angle_to_radian(150),
            self.angle_to_radian(30),
            self.angle_to_radian(180),
            0.0,
            0.0,
        ]

        self.init_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.current_positions = None
        self.current_velocities = None
        self.epsilon = 0.01  # Reduced error threshold for smoother motion
        self.reached_target = False
        self.num_points = 100  # Number of points for smooth trajectory
        self.duration = 10.0  # Reduced duration for step-by-step movement
        self.goal_handle = None
        self.last_status_time = 0.0
        self.status_interval = 1.0  # Log status every second
        self.current_step = 0  # 0: move joint3 to 0, 1: move all to final positions
        self.target_positions = None  # Will be initialized after receiving joint states

        self.get_logger().info(f'Operation mode: {self.operation_mode}')

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server available')

    def initialize_target_positions(self):
        step0_positions = self.final_target_positions.copy()
        step0_positions[2] = 0.0  # Set joint3 to 0

        if self.operation_mode == 'pack':
            # Pack operation sequence
            self.target_positions = [
                self.init_positions,
                step0_positions,
                self.final_target_positions,
            ]
        else:  # unpack
            # Unpack operation sequence
            self.target_positions = [
                step0_positions,
                self.init_positions,
            ]

        return True

    def create_smooth_trajectory(self, start_pos, end_pos):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        times = np.linspace(0, self.duration, self.num_points)

        for i in range(self.num_points):
            point = JointTrajectoryPoint()
            t = times[i]

            # Quintic polynomial coefficients
            t_norm = t / self.duration
            t_norm2 = t_norm * t_norm
            t_norm3 = t_norm2 * t_norm
            t_norm4 = t_norm3 * t_norm
            t_norm5 = t_norm4 * t_norm

            # Quintic polynomial coefficients for position
            pos_coeff = 10 * t_norm3 - 15 * t_norm4 + 6 * t_norm5

            # Velocity coefficients (derivative of position)
            vel_coeff = (30 * t_norm2 - 60 * t_norm3 + 30 * t_norm4) / self.duration

            # Acceleration coefficients (derivative of velocity)
            acc_coeff = (60 * t_norm - 180 * t_norm2 + 120 * t_norm3) / (
                self.duration * self.duration
            )

            positions = []
            velocities = []
            accelerations = []

            for j in range(len(self.joint_names)):
                pos = start_pos[j] + (end_pos[j] - start_pos[j]) * pos_coeff
                vel = (end_pos[j] - start_pos[j]) * vel_coeff
                acc = (end_pos[j] - start_pos[j]) * acc_coeff

                positions.append(pos)
                velocities.append(vel)
                accelerations.append(acc)

            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations
            point.time_from_start.sec = int(times[i])
            point.time_from_start.nanosec = int((times[i] % 1) * 1e9)

            traj.points.append(point)

        return traj

    def get_step_target_positions(self):
        return self.target_positions[self.current_step]

    def check_step_completion(self):
        target_positions = self.get_step_target_positions()
        return all(
            abs(curr - target) < self.epsilon
            for curr, target in zip(self.current_positions, target_positions)
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.actual.positions}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.goal_handle = goal_handle

    def joint_state_callback(self, msg):
        if set(self.joint_names).issubset(set(msg.name)):
            self.current_positions = [
                msg.position[msg.name.index(j)] for j in self.joint_names
            ]
            self.current_velocities = [
                msg.velocity[msg.name.index(j)] for j in self.joint_names
            ]

            # Initialize target positions if not already done
            if self.target_positions is None:
                if self.initialize_target_positions():
                    self.get_logger().info('Target positions initialized')
                else:
                    return

            # Check if current step has reached its target
            if self.goal_handle is None:
                if self.current_step < len(self.target_positions):
                    target_positions = self.get_step_target_positions()
                    self.get_logger().info(
                        f'Moving to step {self.current_step} target positions'
                    )

                    goal_msg = FollowJointTrajectory.Goal()
                    goal_msg.trajectory = self.create_smooth_trajectory(
                        self.current_positions, target_positions
                    )

                    goal_msg.path_tolerance = []
                    goal_msg.goal_tolerance = []
                    goal_msg.goal_time_tolerance.sec = 0
                    goal_msg.goal_time_tolerance.nanosec = 0

                    self.get_logger().info('Sending goal...')
                    self._send_goal_future = self.action_client.send_goal_async(
                        goal_msg, feedback_callback=self.feedback_callback
                    )
                    self._send_goal_future.add_done_callback(
                        self.goal_response_callback
                    )
                else:
                    self.get_logger().info('All steps completed!')
                    self.shutdown_node()
                    return

            # Check if current step has reached its target
            if self.check_step_completion():
                if not self.reached_target:
                    self.reached_target = True
                    self.get_logger().info(f'🎯 Step {self.current_step} completed!')
                    self.goal_handle = None
                    self.current_step += 1
                    self.reached_target = False

            # Log status periodically
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.last_status_time >= self.status_interval:
                self.last_status_time = current_time
                self.get_logger().info(f'Current positions: {self.current_positions}')
                self.get_logger().info(
                    f'Target positions: {self.get_step_target_positions()}'
                )
                self.get_logger().info(f'Current step: {self.current_step}')

    def shutdown_node(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
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
