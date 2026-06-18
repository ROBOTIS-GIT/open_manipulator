#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0
#
# Relay cyclo_control joint trajectory commands to the OpenManipulator arm
# controller without depending on ros-jazzy-topic-tools.

import copy
import time

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class JointTrajectoryRelay(Node):

    def __init__(self):
        super().__init__('omx_joint_trajectory_relay')
        self.declare_parameter('input_topic', '/leader/joint_trajectory')
        self.declare_parameter('output_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('max_rate_hz', 0.0)
        self.declare_parameter('min_position_delta', 0.0)
        self.declare_parameter('output_time_from_start', -1.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.max_rate_hz = max(0.0, float(self.get_parameter('max_rate_hz').value))
        self.min_position_delta = max(0.0, float(
            self.get_parameter('min_position_delta').value))
        self.output_time_from_start = float(
            self.get_parameter('output_time_from_start').value)
        self._last_publish_time = 0.0
        self._last_positions = None

        self.publisher = self.create_publisher(JointTrajectory, self.output_topic, 10)
        self.subscription = self.create_subscription(
            JointTrajectory,
            self.input_topic,
            self._relay,
            10,
        )

        self.get_logger().info(
            f'Relaying {self.input_topic} -> {self.output_topic} '
            f'(max_rate={self.max_rate_hz:.1f}Hz, '
            f'min_delta={self.min_position_delta:.5f}rad, '
            f'time_from_start={self.output_time_from_start:.3f}s)')

    def _relay(self, msg):
        now = time.monotonic()
        if self.max_rate_hz > 0.0:
            min_period = 1.0 / self.max_rate_hz
            if now - self._last_publish_time < min_period:
                return

        positions = self._first_point_positions(msg)
        if (self.min_position_delta > 0.0
                and self._last_positions is not None
                and positions is not None
                and len(positions) == len(self._last_positions)):
            max_delta = max(abs(a - b) for a, b in zip(positions, self._last_positions))
            if max_delta < self.min_position_delta:
                return

        out = self._with_output_time_from_start(msg)
        self.publisher.publish(out)
        self._last_publish_time = now
        if positions is not None:
            self._last_positions = positions

    @staticmethod
    def _first_point_positions(msg):
        if not msg.points:
            return None
        positions = msg.points[0].positions
        if not positions:
            return None
        return list(positions)

    def _with_output_time_from_start(self, msg):
        if self.output_time_from_start < 0.0 or not msg.points:
            return msg

        out = copy.deepcopy(msg)
        sec = int(self.output_time_from_start)
        nsec = int((self.output_time_from_start - sec) * 1e9)
        duration = Duration(sec=sec, nanosec=nsec)
        for point in out.points:
            point.time_from_start = duration
        return out


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryRelay()
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
