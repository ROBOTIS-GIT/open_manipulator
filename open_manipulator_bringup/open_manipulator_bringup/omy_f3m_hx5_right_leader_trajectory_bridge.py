#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# Subscribes to L100 leader combined arm+gripper joint trajectories and
# maps leader rh_r1_joint to HX5 right finger presets (mirrored from left).

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

RIGHT_PRESET_RELEASE: List[float] = [
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
]

RIGHT_PRESET_GRASP: List[float] = [
    0.0, 1.57, -0.7, -0.7,
    0.0, 1.0, 1.5, 1.2,
    0.0, 1.0, 1.5, 1.2,
    0.0, 0.8, 1.5, 1.2,
    0.0, 0.5, 1.4, 1.2,
]

RIGHT_FINGER_NAMES: List[str] = [
    'finger_r_joint1', 'finger_r_joint2', 'finger_r_joint3', 'finger_r_joint4',
    'finger_r_joint5', 'finger_r_joint6', 'finger_r_joint7', 'finger_r_joint8',
    'finger_r_joint9', 'finger_r_joint10', 'finger_r_joint11', 'finger_r_joint12',
    'finger_r_joint13', 'finger_r_joint14', 'finger_r_joint15', 'finger_r_joint16',
    'finger_r_joint17', 'finger_r_joint18', 'finger_r_joint19', 'finger_r_joint20',
]


def normalize_value(value: float, min_old: float, max_old: float) -> float:
    range_old = max_old - min_old
    if range_old <= 0.0:
        return 0.0
    clamped = min(max_old, max(value, min_old))
    return (clamped - min_old) / range_old


def blend_presets(
    release: List[float], grasp: List[float], t_thumb: float, t_fingers: float
) -> List[float]:
    out: List[float] = [0.0] * 20
    for i in range(4):
        out[i] = t_thumb * grasp[i] + (1.0 - t_thumb) * release[i]
    for i in range(4, 20):
        out[i] = t_fingers * grasp[i] + (1.0 - t_fingers) * release[i]
    return out


class OmyF3mHx5RightLeaderTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__('omy_f3m_hx5_right_leader_trajectory_bridge')

        self.declare_parameter('leader_trajectory_topic', '/leader/joint_trajectory')
        self.declare_parameter('hand_command_topic', '/hand/hand_r_controller/joint_trajectory')
        self.declare_parameter('leader_gripper_joint_name', 'rh_r1_joint')
        self.declare_parameter('thumb_preset_threshold', 0.0)
        self.declare_parameter('rh_normalize_min', -0.1)
        self.declare_parameter('rh_normalize_max', 1.1)

        self._grip_name = self.get_parameter('leader_gripper_joint_name').value
        self._thumb_thr = self.get_parameter('thumb_preset_threshold').value
        self._rh_min = float(self.get_parameter('rh_normalize_min').value)
        self._rh_max = float(self.get_parameter('rh_normalize_max').value)

        in_topic = self.get_parameter('leader_trajectory_topic').value
        hand_topic = self.get_parameter('hand_command_topic').value

        self._hand_pub = self.create_publisher(
            JointTrajectory, hand_topic, qos_profile_sensor_data
        )

        self.create_subscription(JointTrajectory, in_topic, self._on_leader_traj, 10)

        self.get_logger().info(
            f'Bridge: sub={in_topic} hand_pub={hand_topic}'
        )

    def _on_leader_traj(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return

        if self._grip_name not in msg.joint_names:
            return
        gi = msg.joint_names.index(self._grip_name)
        last = msg.points[-1]
        if gi >= len(last.positions) or math.isnan(last.positions[gi]):
            return
        raw = float(last.positions[gi])

        t_grip = normalize_value(raw, self._rh_min, self._rh_max)
        t_thumb = normalize_value(t_grip, self._thumb_thr, 1.0)

        positions = blend_presets(
            RIGHT_PRESET_RELEASE, RIGHT_PRESET_GRASP, t_thumb, t_grip
        )

        hand_msg = JointTrajectory()
        hand_msg.header = msg.header
        hand_msg.joint_names = list(RIGHT_FINGER_NAMES)
        hp = JointTrajectoryPoint()
        hp.positions = positions
        hp.time_from_start.sec = 0
        hp.time_from_start.nanosec = 0
        hand_msg.points.append(hp)
        self._hand_pub.publish(hand_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmyF3mHx5RightLeaderTrajectoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
