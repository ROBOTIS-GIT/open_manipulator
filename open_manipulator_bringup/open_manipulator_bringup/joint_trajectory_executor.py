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


class JointTrajectoryExecutor(Node):
    """Node to execute a joint trajectory."""

    def __init__(self):
        super().__init__('joint_trajectory_executor')

        # Declare parameters
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('step_names', [''])  # List of step names
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('epsilon', 0.01)
        self.declare_parameter(
            'action_topic', '/arm_controller/follow_joint_trajectory'
        )
        self.declare_parameter('joint_states_topic', '/joint_states')

        # Load basic parameters
        self.joint_names = (
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        self.step_names = (
            self.get_parameter('step_names').get_parameter_value().string_array_value
        )
        self.duration = self.get_parameter('duration').value
        self.epsilon = self.get_parameter('epsilon').value
        self.action_topic = self.get_parameter('action_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value

        # Validate basic parameters
        if not self.joint_names:
            self.get_logger().error('Missing required parameter: joint_names')
            sys.exit(1)
        if not self.step_names:
            self.get_logger().error('Missing required parameter: step_names')
            sys.exit(1)

        # Declare and load step parameters
        self.positions_list = []
        for step_name in self.step_names:
            self.declare_parameter(step_name, [0.0] * len(self.joint_names))
            step_positions = (
                self.get_parameter(step_name).get_parameter_value().double_array_value
            )
            self.positions_list.append(step_positions)

        # Validate step positions
        if not self.positions_list:
            self.get_logger().error('No valid step positions found')
            sys.exit(1)

        # Validate that all position arrays have the correct length
        for i, pos in enumerate(self.positions_list):
            if len(pos) != len(self.joint_names):
                self.get_logger().error(
                    f'Position array {i} has incorrect length. '
                    f'Expected {len(self.joint_names)}, got {len(pos)}'
                )
                sys.exit(1)

        # Create action client for FollowJointTrajectory
        self.action_client = ActionClient(
            self, FollowJointTrajectory, self.action_topic
        )
        self.subscription = self.create_subscription(
            JointState, self.joint_states_topic, self.joint_state_callback, 10
        )

        self.current_positions = None
        self.current_velocities = None
        self.reached_target = False
        self.num_points = 100  # Number of points for smooth trajectory
        self.goal_handle = None
        self.last_status_time = 0.0
        self.status_interval = 1.0  # Log status every second
        self.current_step = 0

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server available')
        self.get_logger().info(f'Using action topic: {self.action_topic}')
        self.get_logger().info(f'Using joint states topic: {self.joint_states_topic}')

    def get_step_target_positions(self):
        return self.positions_list[self.current_step]

    def check_step_completion(self):
        target_positions = self.get_step_target_positions()
        return all(
            abs(curr - target) < self.epsilon
            for curr, target in zip(self.current_positions, target_positions)
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Feedback: {feedback.actual.positions}')

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

            # Check if current step has reached its target
            if self.goal_handle is None:
                if self.current_step < len(self.positions_list):
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

    def shutdown_node(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def angle_to_radian(self, angle):
        return angle * math.pi / 180

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


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
