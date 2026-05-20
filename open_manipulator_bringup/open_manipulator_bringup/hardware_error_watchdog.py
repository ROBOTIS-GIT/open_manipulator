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

import socket
import sys
from urllib.parse import quote

from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.node import Node


TORQUE_ENABLE = 'Torque Enable'


def torque_enable_states(msg):
    """Return (resource_name, torque_enable) pairs from a DynamicJointState message."""
    states = []

    for index, interface_value in enumerate(msg.interface_values):
        try:
            torque_index = interface_value.interface_names.index(TORQUE_ENABLE)
        except ValueError:
            continue

        if torque_index >= len(interface_value.values):
            continue

        resource_name = (
            msg.joint_names[index]
            if index < len(msg.joint_names)
            else f'interface_values[{index}]'
        )
        try:
            torque_enable = int(interface_value.values[torque_index])
        except (OverflowError, ValueError):
            continue

        states.append((resource_name, torque_enable))

    return states


def torque_disabled_motors(msg):
    """Return motors whose DYNAMIXEL Torque Enable state is off."""
    return [
        (resource_name, torque_enable)
        for resource_name, torque_enable in torque_enable_states(msg)
        if torque_enable == 0
    ]


class DockerRestartClient:
    """Small Docker Engine client for restart over a mounted Unix socket."""

    def __init__(
        self,
        socket_path='/var/run/docker.sock',
        container_name='open_manipulator',
        restart_timeout_sec=5,
        request_timeout_sec=3.0,
    ):
        self.socket_path = socket_path
        self.container_name = container_name
        self.restart_timeout_sec = restart_timeout_sec
        self.request_timeout_sec = request_timeout_sec

    def restart_container(self):
        container_name = quote(self.container_name, safe='')
        path = f'/containers/{container_name}/restart?t={self.restart_timeout_sec}'
        request = (
            f'POST {path} HTTP/1.1\r\n'
            'Host: docker\r\n'
            'Content-Length: 0\r\n'
            'Connection: close\r\n'
            '\r\n'
        ).encode('ascii')

        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(self.request_timeout_sec)
            sock.connect(self.socket_path)
            sock.sendall(request)
            response = self._read_response(sock)

        status_code = self._parse_status_code(response)
        if 200 <= status_code < 300:
            return True

        raise RuntimeError(
            f'Docker restart failed with HTTP {status_code}: '
            f'{response.decode("utf-8", errors="replace")}'
        )

    @staticmethod
    def _read_response(sock):
        chunks = []
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
        return b''.join(chunks)

    @staticmethod
    def _parse_status_code(response):
        status_line = response.split(b'\r\n', 1)[0].decode('ascii', errors='replace')
        parts = status_line.split()
        if len(parts) < 2 or not parts[1].isdigit():
            raise RuntimeError(f'Invalid Docker response: {status_line}')
        return int(parts[1])


class HardwareErrorWatchdog(Node):
    """Watch DYNAMIXEL torque state and restart Docker if torque turns off."""

    def __init__(self, docker_client=None):
        super().__init__('hardware_error_watchdog')

        self.declare_parameter('dynamic_joint_states_topic', '/dynamic_joint_states')
        self.declare_parameter('docker_socket_path', '/var/run/docker.sock')
        self.declare_parameter('container_name', 'open_manipulator')
        self.declare_parameter('docker_restart_timeout_sec', 5)
        self.declare_parameter('docker_request_timeout_sec', 3.0)

        topic = self.get_parameter('dynamic_joint_states_topic').value
        self._restart_requested = False
        self._shutdown_timer = None
        self._docker_client = docker_client or DockerRestartClient(
            socket_path=self.get_parameter('docker_socket_path').value,
            container_name=self.get_parameter('container_name').value,
            restart_timeout_sec=self.get_parameter('docker_restart_timeout_sec').value,
            request_timeout_sec=self.get_parameter('docker_request_timeout_sec').value,
        )

        self.create_subscription(DynamicJointState, topic, self._status_callback, 10)
        self.get_logger().info(f'Watching {topic} for DYNAMIXEL torque-off states')

    def _status_callback(self, msg):
        if self._restart_requested:
            return

        disabled_motors = torque_disabled_motors(msg)
        if not disabled_motors:
            return

        affected = ', '.join(
            f'{resource}=Torque Enable {torque_enable}'
            for resource, torque_enable in disabled_motors
        )
        self.get_logger().error(
            f'DYNAMIXEL torque disabled ({affected}); restarting Docker container'
        )

        try:
            self._docker_client.restart_container()
        except Exception as exc:  # noqa: BLE001 - keep watchdog alive after failures.
            self.get_logger().error(f'Failed to restart Docker container: {exc}')
            return

        self._restart_requested = True
        self.get_logger().info('Docker restart requested successfully')
        self._shutdown_timer = self.create_timer(0.1, self._shutdown_after_restart)

    def _shutdown_after_restart(self):
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareErrorWatchdog()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    sys.exit(0)


if __name__ == '__main__':
    main()
