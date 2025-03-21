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
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Sungho Woo


import sys
import termios
import tty
import select
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand  # Import GripperCommand action


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        # Publisher for arm joint control
        self.arm_publisher = self.create_publisher(
            JointTrajectory, '/leader/joint_trajectory', 10
        )

        # Action client for GripperCommand
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        # Subscriber for joint states
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Initialize joint positions for arm (6 joints) and gripper
        self.arm_joint_positions = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]  # Now supporting 6 joints
        self.arm_joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'rh_r1_joint',
        ]

        self.gripper_position = 0.0
        self.gripper_max = 1.1  # Maximum gripper open position
        self.gripper_min = 0.0  # Minimum gripper closed position

        self.joint_received = (
            False  # Flag to check if initial joint states are received
        )

        # Movement speed limits
        self.max_delta = 0.02  # Maximum increment for arm joints
        self.gripper_delta = 0.1  # Increment for gripper movement
        self.last_command_time = time.time()  # Last command execution time
        self.command_interval = 0.02  # Minimum time interval between commands (seconds)

        self.get_logger().info('Waiting for /joint_states...')
        self.rate = self.create_rate(10)  # 10Hz update rate

    def joint_state_callback(self, msg):

        if set(self.arm_joint_names).issubset(set(msg.name)):
            for i, joint in enumerate(self.arm_joint_names):
                index = msg.name.index(joint)
                self.arm_joint_positions[i] = msg.position[index]

        if 'rh_r1_joint' in msg.name:
            index = msg.name.index('rh_r1_joint')
            self.gripper_position = msg.position[index]

        self.joint_received = True
        self.get_logger().info(
            f'Received joint states: {self.arm_joint_positions}, '
            f'Gripper: {self.gripper_position}'
        )

    def get_key(self, timeout=0.01):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def send_arm_command(self):

        arm_msg = JointTrajectory()
        arm_msg.joint_names = self.arm_joint_names
        arm_point = JointTrajectoryPoint()
        arm_point.positions = self.arm_joint_positions
        arm_point.time_from_start.sec = 0
        arm_msg.points.append(arm_point)
        self.arm_publisher.publish(arm_msg)

        self.get_logger().info(f'Arm command sent: {self.arm_joint_positions}')

    def run(self):

        while not self.joint_received and rclpy.ok():
            self.get_logger().info('Waiting for initial joint states...')
            rclpy.spin_once(
                self, timeout_sec=1.0
            )  # Wait for joint_states topic to be received

        self.get_logger().info('Ready to receive keyboard input!')
        self.get_logger().info(
            'Use 1/q, 2/w, 3/e, 4/r, 5/t, 6/y for joints 1-6, '
            'o/p for gripper. Press ESC to exit.'
        )
        try:
            while rclpy.ok():
                key = self.get_key()
                current_time = time.time()

                # Ensure commands are only executed at a fixed interval
                if current_time - self.last_command_time >= self.command_interval:
                    if key == '\x1b':  # Exit when ESC is pressed
                        break
                    elif key == '1':  # Increase joint1
                        self.arm_joint_positions[0] = min(
                            self.arm_joint_positions[0] + self.max_delta, 3.14
                        )
                    elif key == 'q':  # Decrease joint1
                        self.arm_joint_positions[0] = max(
                            self.arm_joint_positions[0] - self.max_delta, -3.14
                        )
                    elif key == '2':  # Increase joint2
                        self.arm_joint_positions[1] = min(
                            self.arm_joint_positions[1] + self.max_delta, 3.14
                        )
                    elif key == 'w':  # Decrease joint2
                        self.arm_joint_positions[1] = max(
                            self.arm_joint_positions[1] - self.max_delta, -3.14
                        )
                    elif key == '3':  # Increase joint3
                        self.arm_joint_positions[2] = min(
                            self.arm_joint_positions[2] + self.max_delta, 3.14
                        )
                    elif key == 'e':  # Decrease joint3
                        self.arm_joint_positions[2] = max(
                            self.arm_joint_positions[2] - self.max_delta, -3.14
                        )
                    elif key == '4':  # Increase joint4
                        self.arm_joint_positions[3] = min(
                            self.arm_joint_positions[3] + self.max_delta, 3.14
                        )
                    elif key == 'r':  # Decrease joint4
                        self.arm_joint_positions[3] = max(
                            self.arm_joint_positions[3] - self.max_delta, -3.14
                        )
                    elif key == '5':  # Increase joint5
                        self.arm_joint_positions[4] = min(
                            self.arm_joint_positions[4] + self.max_delta, 3.14
                        )
                    elif key == 't':  # Decrease joint5
                        self.arm_joint_positions[4] = max(
                            self.arm_joint_positions[4] - self.max_delta, -3.14
                        )
                    elif key == '6':  # Increase joint6
                        self.arm_joint_positions[5] = min(
                            self.arm_joint_positions[5] + self.max_delta, 3.14
                        )
                    elif key == 'y':  # Decrease joint6
                        self.arm_joint_positions[5] = max(
                            self.arm_joint_positions[5] - self.max_delta, -3.14
                        )
                    elif key == 'o':  # Open gripper
                        self.arm_joint_positions[6] = min(
                            self.arm_joint_positions[6] + self.max_delta, 3.14
                        )
                    elif key == 'p':  # Close gripper
                        self.arm_joint_positions[6] = max(
                            self.arm_joint_positions[6] - self.max_delta, -3.14
                        )

                    # Send updated joint commands
                    self.send_arm_command()
                    self.last_command_time = current_time  # Update last command time

        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()
    node = KeyboardController()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
