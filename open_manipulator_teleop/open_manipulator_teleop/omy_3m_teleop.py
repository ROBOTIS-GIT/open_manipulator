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

import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_controller')

        # Publisher for arm joint control
        self.arm_publisher = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        # Subscriber for joint states
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.arm_joint_positions = [0.0] * 6
        self.arm_joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
        ]

        self.joint_received = False

        self.max_delta = 0.02
        self.last_command_time = time.time()
        self.command_interval = 0.02

        self.running = True  # for thread loop control

        self.get_logger().info('Waiting for /joint_states...')
        self.rate = self.create_rate(10)

    def joint_state_callback(self, msg):
        if set(self.arm_joint_names).issubset(set(msg.name)):
            for i, joint in enumerate(self.arm_joint_names):
                index = msg.name.index(joint)
                self.arm_joint_positions[i] = msg.position[index]

        self.joint_received = True
        self.get_logger().info(
            f'Received joint states: {self.arm_joint_positions}, '
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
        while not self.joint_received and rclpy.ok() and self.running:
            self.get_logger().info('Waiting for initial joint states...')
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info('Ready to receive keyboard input!')
        self.get_logger().info(
            'Use 1/q, 2/w, 3/e, 4/r, 5/t, 6/y for joints 1-6. Press ESC to exit.'
        )

        try:
            while rclpy.ok() and self.running:
                key = self.get_key()
                current_time = time.time()

                if key is None:
                    continue

                if current_time - self.last_command_time >= self.command_interval:
                    if key == '\x1b':  # ESC
                        self.running = False
                        break
                    elif key == '1':
                        new_pos = min(
                            self.arm_joint_positions[0] + self.max_delta, 3.14
                        )
                        self.arm_joint_positions[0] = new_pos
                    elif key == 'q':
                        new_pos = max(
                            self.arm_joint_positions[0] - self.max_delta, -3.14
                        )
                        self.arm_joint_positions[0] = new_pos
                    elif key == '2':
                        new_pos = min(
                            self.arm_joint_positions[1] + self.max_delta, 3.14
                        )
                        self.arm_joint_positions[1] = new_pos
                    elif key == 'w':
                        new_pos = max(
                            self.arm_joint_positions[1] - self.max_delta, -3.14
                        )
                        self.arm_joint_positions[1] = new_pos
                    elif key == '3':
                        new_pos = min(
                            self.arm_joint_positions[2] + self.max_delta, 3.14
                        )
                        self.arm_joint_positions[2] = new_pos
                    elif key == 'e':
                        new_pos = max(
                            self.arm_joint_positions[2] - self.max_delta, -3.14
                        )
                        self.arm_joint_positions[2] = new_pos
                    elif key == '4':
                        new_pos = min(
                            self.arm_joint_positions[3] + self.max_delta, 3.14
                        )
                        self.arm_joint_positions[3] = new_pos
                    elif key == 'r':
                        new_pos = max(
                            self.arm_joint_positions[3] - self.max_delta, -3.14
                        )
                        self.arm_joint_positions[3] = new_pos
                    elif key == '5':
                        new_pos = min(
                            self.arm_joint_positions[4] + self.max_delta, 3.14
                        )
                        self.arm_joint_positions[4] = new_pos
                    elif key == 't':
                        new_pos = max(
                            self.arm_joint_positions[4] - self.max_delta, -3.14
                        )
                        self.arm_joint_positions[4] = new_pos
                    elif key == '6':
                        new_pos = min(
                            self.arm_joint_positions[5] + self.max_delta, 3.14
                        )
                        self.arm_joint_positions[5] = new_pos
                    elif key == 'y':
                        new_pos = max(
                            self.arm_joint_positions[5] - self.max_delta, -3.14
                        )
                        self.arm_joint_positions[5] = new_pos

                    self.send_arm_command()
                    self.last_command_time = current_time

        except Exception as e:
            self.get_logger().error(f'Exception in run loop: {e}')


def main():
    rclpy.init()
    node = KeyboardController()

    thread = threading.Thread(target=node.run)
    thread.start()

    try:
        while thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\nCtrl+C detected. Shutting down...')
        node.running = False
        thread.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
