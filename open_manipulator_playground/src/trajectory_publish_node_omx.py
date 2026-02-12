#!/usr/bin/env python3
# Copyright 2024 ROBOTIS CO., LTD.
# SPDX-FileCopyrightText: 2024 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
# Author: Minseo Choi

from __future__ import annotations

from typing import Dict, List, Optional

from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from .inverse_kinematics_omx import (
        IKConfig,
        rot_to_quat,
        SimpleURDFChain,
        solve_ik_dls,
    )
    from .shape_trajectories_omx import (
        circle_point,
        heart_point,
        rectangle_point,
    )
except ImportError:
    from inverse_kinematics_omx import (
        IKConfig,
        rot_to_quat,
        SimpleURDFChain,
        solve_ik_dls,
    )
    from shape_trajectories_omx import (
        circle_point,
        heart_point,
        rectangle_point,
    )


class TrajectoryPublishNodeOMX(Node):

    def __init__(self) -> None:
        super().__init__('trajectory_publish_node_omx')

        self.declare_parameter('traj_topic', '/omx_arm_controller/joint_trajectory')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('robot_description_node', '/robot_state_publisher')

        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('ee_link', 'end_effector_link')
        self.declare_parameter('ee_fixed_joint', 'end_effector_joint')
        self.declare_parameter('urdf_param', 'robot_description')

        self.declare_parameter('ee_feedback_topic', '/omx_playground/ee_feedback')
        self.declare_parameter('ee_target_topic', '/omx_playground/ee_target')
        self.declare_parameter('publish_target_marker', True)
        self.declare_parameter('publish_feedback_pose', True)

        self.declare_parameter('repeat', True)
        self.declare_parameter('interactive', False)

        self.declare_parameter('traj_mode', 'circle')
        self.declare_parameter('duration_sec', 0.10)

        self.declare_parameter('circle_center', [0.18, 0.00, 0.12])
        self.declare_parameter('circle_radius', 0.03)
        self.declare_parameter('circle_hz', 0.20)
        self.declare_parameter('circle_plane', 'yz')

        self.declare_parameter('rect_center', [0.18, 0.00, 0.12])
        self.declare_parameter('rect_width', 0.05)
        self.declare_parameter('rect_height', 0.05)
        self.declare_parameter('rect_corner_radius', 0.00)
        self.declare_parameter('rect_hz', 0.20)
        self.declare_parameter('rect_plane', 'yz')

        self.declare_parameter('heart_center', [0.18, 0.00, 0.12])
        self.declare_parameter('heart_scale', 0.03)
        self.declare_parameter('heart_hz', 0.20)
        self.declare_parameter('heart_plane', 'yz')

        self.declare_parameter('ee_quat', [0.0, 0.0, 0.0, 1.0])

        self.declare_parameter('ik_use_joint_states', True)
        self.declare_parameter('ik_position_only', True)
        self.declare_parameter('ik_max_iters', 120)
        self.declare_parameter('ik_damping', 0.05)
        self.declare_parameter('ik_step_scale', 0.8)
        self.declare_parameter('ik_tol_pos_m', 1e-3)
        self.declare_parameter('ik_tol_rot_rad', 1e-2)
        self.declare_parameter('ik_w_pos', 1.0)
        self.declare_parameter('ik_w_rot', 0.2)

        traj_topic = str(self.get_parameter('traj_topic').value)
        self.pub = self.create_publisher(JointTrajectory, traj_topic, 10)

        ee_feedback_topic = str(self.get_parameter('ee_feedback_topic').value)
        self.ee_pub = self.create_publisher(PoseStamped, ee_feedback_topic, 10)

        ee_target_topic = str(self.get_parameter('ee_target_topic').value)
        self.target_pub = self.create_publisher(PointStamped, ee_target_topic, 10)

        robot_description_node = str(self.get_parameter('robot_description_node').value)
        self.param_client = AsyncParameterClient(self, robot_description_node)

        self._last_joint_state: Optional[JointState] = None
        self._sent_once = False
        self._interactive_done = False

        self.sub = self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._on_joint_state,
            10,
        )

        self._t0 = self.get_clock().now()
        self._timer = self.create_timer(
            float(self.get_parameter('duration_sec').value),
            self._on_timer,
        )

        self._init_chain()

    def _init_chain(self) -> None:
        self._chain: Optional[SimpleURDFChain] = None
        self._joint_names: List[str] = []
        self._joint_to_idx: Dict[str, int] = {}

        urdf_param = str(self.get_parameter('urdf_param').value)
        base_link = str(self.get_parameter('base_link').value)
        ee_link = str(self.get_parameter('ee_link').value)
        ee_fixed_joint = str(self.get_parameter('ee_fixed_joint').value)

        if not self.param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('AsyncParameterClient not ready yet. Will retry later.')
            return

        future = self.param_client.get_parameters([urdf_param])
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            self.get_logger().warn('Failed to get URDF parameter. Will retry later.')
            return

        urdf_xml = future.result().values[0].string_value
        if not urdf_xml:
            self.get_logger().warn('URDF is empty. Will retry later.')
            return

        self._chain = SimpleURDFChain(
            urdf_xml=urdf_xml,
            base_link=base_link,
            ee_link=ee_link,
            ee_fixed_joint=ee_fixed_joint,
        )
        self._joint_names = list(self._chain.joint_names)
        self._joint_to_idx = {n: i for i, n in enumerate(self._joint_names)}

        traj_topic = self.get_parameter('traj_topic').value
        self.get_logger().info(f'Publishing to: {traj_topic}')
        self.get_logger().info(
            f'Joints ({len(self._joint_names)}): '
            f'{self._joint_names[0]} .. {self._joint_names[-1]}',
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg

    @staticmethod
    def _ask_float(label: str, default: float) -> float:
        while True:
            s = input(f'{label} (default={default}) : ').strip()
            if s == '':
                return float(default)
            try:
                return float(s)
            except ValueError:
                print('  -> Please input a valid number (or press Enter).')

    @staticmethod
    def _ask_int(label: str, default: int) -> int:
        while True:
            s = input(f'{label} (default={default}) : ').strip()
            if s == '':
                return int(default)
            try:
                return int(s)
            except ValueError:
                print('  -> Please input a valid integer (or press Enter).')

    @staticmethod
    def _ask_bool_yn(label: str, default: bool) -> bool:
        default_str = 'y' if default else 'n'
        while True:
            s = input(f'{label} (y/n, default={default_str}) : ').strip().lower()
            if s == '':
                return bool(default)
            if s in ('y', 'yes', '1', 'true', 't'):
                return True
            if s in ('n', 'no', '0', 'false', 'f'):
                return False
            print('  -> Please type y or n (or press Enter).')

    @staticmethod
    def _ask_xyz(
        label: str,
        default_xyz: List[float],
        hint_for_single_value: str = '',
    ) -> List[float]:
        rec = f'{default_xyz[0]},{default_xyz[1]},{default_xyz[2]}'
        while True:
            s = input(f'{label} x,y,z (recommended: {rec}) : ').strip()
            if s == '':
                return list(default_xyz)

            parts = [p.strip() for p in s.split(',') if p.strip() != '']
            if len(parts) == 1 and hint_for_single_value:
                print(f'  -> {hint_for_single_value}')
                try:
                    v = float(parts[0])
                except ValueError:
                    print('  -> Please input numbers like 0.18,0.00,0.12 (or press Enter).')
                    continue
                return [v, default_xyz[1], default_xyz[2]]

            if len(parts) != 3:
                print(
                    '  -> Expected 3 comma-separated values like 0.18,0.00,0.12 '
                    '(or press Enter).',
                )
                continue

            try:
                return [float(parts[0]), float(parts[1]), float(parts[2])]
            except ValueError:
                print('  -> Please input numbers like 0.18,0.00,0.12 (or press Enter).')

    def _interactive_update_params(self) -> None:
        default_mode = str(self.get_parameter('traj_mode').value).lower()
        mode = input(
            f'traj_mode [circle/rectangle/heart] (default={default_mode}) : ',
        ).strip().lower()
        if mode == '':
            mode = default_mode

        plane = input('plane [xy/xz/yz] (default=yz) : ').strip().lower()
        if plane == '':
            plane = 'yz'

        if mode == 'circle':
            c = self._ask_xyz(
                'center',
                list(self.get_parameter('circle_center').value),
                hint_for_single_value='Single value detected. Radius is asked next.',
            )
            r = self._ask_float('radius (m)', float(self.get_parameter('circle_radius').value))
            hz = self._ask_float('hz', float(self.get_parameter('circle_hz').value))
            self.set_parameters(
                [
                    Parameter('traj_mode', Parameter.Type.STRING, 'circle'),
                    Parameter('circle_center', Parameter.Type.DOUBLE_ARRAY, c),
                    Parameter('circle_radius', Parameter.Type.DOUBLE, r),
                    Parameter('circle_hz', Parameter.Type.DOUBLE, hz),
                    Parameter('circle_plane', Parameter.Type.STRING, plane),
                ],
            )

        elif mode == 'rectangle':
            c = self._ask_xyz(
                'center',
                list(self.get_parameter('rect_center').value),
                hint_for_single_value='Single value detected. Width/height are asked next.',
            )
            w = self._ask_float('width (m)', float(self.get_parameter('rect_width').value))
            h = self._ask_float('height (m)', float(self.get_parameter('rect_height').value))
            cr = self._ask_float(
                'corner radius (m) (0=sharp)',
                float(self.get_parameter('rect_corner_radius').value),
            )
            hz = self._ask_float('hz', float(self.get_parameter('rect_hz').value))
            self.set_parameters(
                [
                    Parameter('traj_mode', Parameter.Type.STRING, 'rectangle'),
                    Parameter('rect_center', Parameter.Type.DOUBLE_ARRAY, c),
                    Parameter('rect_width', Parameter.Type.DOUBLE, w),
                    Parameter('rect_height', Parameter.Type.DOUBLE, h),
                    Parameter('rect_corner_radius', Parameter.Type.DOUBLE, cr),
                    Parameter('rect_hz', Parameter.Type.DOUBLE, hz),
                    Parameter('rect_plane', Parameter.Type.STRING, plane),
                ],
            )

        elif mode == 'heart':
            c = self._ask_xyz(
                'center',
                list(self.get_parameter('heart_center').value),
                hint_for_single_value='Single value detected. Scale is asked next.',
            )
            s = self._ask_float('scale (m)', float(self.get_parameter('heart_scale').value))
            hz = self._ask_float('hz', float(self.get_parameter('heart_hz').value))
            self.set_parameters(
                [
                    Parameter('traj_mode', Parameter.Type.STRING, 'heart'),
                    Parameter('heart_center', Parameter.Type.DOUBLE_ARRAY, c),
                    Parameter('heart_scale', Parameter.Type.DOUBLE, s),
                    Parameter('heart_hz', Parameter.Type.DOUBLE, hz),
                    Parameter('heart_plane', Parameter.Type.STRING, plane),
                ],
            )
        else:
            print("  -> Please type 'heart' or 'circle' or 'rectangle' (or press Enter).")

        pos_only = self._ask_bool_yn(
            'IK position-only?',
            bool(self.get_parameter('ik_position_only').value),
        )
        w_rot = self._ask_float('ik_w_rot', float(self.get_parameter('ik_w_rot').value))
        tol_pos = self._ask_float('ik_tol_pos_m', float(self.get_parameter('ik_tol_pos_m').value))
        iters = self._ask_int('ik_max_iters', int(self.get_parameter('ik_max_iters').value))
        damping = self._ask_float('ik_damping', float(self.get_parameter('ik_damping').value))
        step = self._ask_float('ik_step_scale', float(self.get_parameter('ik_step_scale').value))

        self.set_parameters(
            [
                Parameter('ik_position_only', Parameter.Type.BOOL, pos_only),
                Parameter('ik_w_rot', Parameter.Type.DOUBLE, w_rot),
                Parameter('ik_tol_pos_m', Parameter.Type.DOUBLE, tol_pos),
                Parameter('ik_max_iters', Parameter.Type.INTEGER, iters),
                Parameter('ik_damping', Parameter.Type.DOUBLE, damping),
                Parameter('ik_step_scale', Parameter.Type.DOUBLE, step),
            ],
        )

    def _make_ik_config(self) -> IKConfig:
        return IKConfig(
            position_only=self.get_parameter('ik_position_only').value,
            max_iters=self.get_parameter('ik_max_iters').value,
            damping=self.get_parameter('ik_damping').value,
            step_scale=self.get_parameter('ik_step_scale').value,
            tol_pos_m=self.get_parameter('ik_tol_pos_m').value,
            tol_rot_rad=self.get_parameter('ik_tol_rot_rad').value,
            w_pos=self.get_parameter('ik_w_pos').value,
            w_rot=self.get_parameter('ik_w_rot').value,
        )

    def _get_seed_q(self) -> Optional[np.ndarray]:
        use_js = self.get_parameter('ik_use_joint_states').value
        if not use_js:
            return None
        if self._last_joint_state is None:
            return None

        q = np.zeros(len(self._joint_names), dtype=float)
        name_to_idx = self._joint_to_idx
        for n, p in zip(self._last_joint_state.name, self._last_joint_state.position):
            if n in name_to_idx:
                q[name_to_idx[n]] = float(p)
        return q

    def _on_timer(self) -> None:
        if self._chain is None:
            self._init_chain()
            return

        if self._sent_once and (not self.get_parameter('repeat').value):
            return

        if self.get_parameter('interactive').value and (not self._interactive_done):
            self._interactive_update_params()
            self._interactive_done = True

        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9

        mode = str(self.get_parameter('traj_mode').value).lower()
        target_xyz = self._compute_target_xyz(mode, t)

        target_quat = list(self.get_parameter('ee_quat').value)

        if self.get_parameter('publish_target_marker').value:
            self._publish_target_xyz(target_xyz)

        ik_cfg = self._make_ik_config()
        seed_q = self._get_seed_q()
        q_sol, err_pos, err_rot = solve_ik_dls(
            chain=self._chain,
            target_xyz=target_xyz,
            target_quat=target_quat,
            seed_q=seed_q,
            cfg=ik_cfg,
        )
        if q_sol is None:
            iters = self.get_parameter('ik_max_iters').value
            self.get_logger().warn(
                f'IK failed. target_xyz={list(np.round(target_xyz, 4))} '
                f'err_pos={err_pos:.4f}m err_rot={err_rot:.4f}rad iters={iters}',
            )
            return

        if self.get_parameter('publish_feedback_pose').value:
            ee_xyz, ee_rot = self._chain.forward(q_sol)
            quat = rot_to_quat(ee_rot)
            self._publish_feedback_pose(ee_xyz, quat)

        self._publish_trajectory(q_sol)
        self._sent_once = True

    def _compute_target_xyz(self, mode: str, t: float) -> np.ndarray:
        if mode == 'circle':
            c = np.array(self.get_parameter('circle_center').value, dtype=float)
            r = float(self.get_parameter('circle_radius').value)
            hz = float(self.get_parameter('circle_hz').value)
            plane = str(self.get_parameter('circle_plane').value).lower()
            return circle_point(t, c, r, hz, plane)

        if mode == 'rectangle':
            c = np.array(self.get_parameter('rect_center').value, dtype=float)
            w = float(self.get_parameter('rect_width').value)
            h = float(self.get_parameter('rect_height').value)
            cr = float(self.get_parameter('rect_corner_radius').value)
            hz = float(self.get_parameter('rect_hz').value)
            plane = str(self.get_parameter('rect_plane').value).lower()
            return rectangle_point(t, c, w, h, cr, hz, plane)

        c = np.array(self.get_parameter('heart_center').value, dtype=float)
        s = float(self.get_parameter('heart_scale').value)
        hz = float(self.get_parameter('heart_hz').value)
        plane = str(self.get_parameter('heart_plane').value).lower()
        return heart_point(t, c, s, hz, plane)

    def _publish_target_xyz(self, xyz: np.ndarray) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('base_link').value)
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        self.target_pub.publish(msg)

    def _publish_feedback_pose(self, xyz: np.ndarray, quat_xyzw: np.ndarray) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('base_link').value)
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.x = float(quat_xyzw[0])
        msg.pose.orientation.y = float(quat_xyzw[1])
        msg.pose.orientation.z = float(quat_xyzw[2])
        msg.pose.orientation.w = float(quat_xyzw[3])
        self.ee_pub.publish(msg)

    def _publish_trajectory(self, q: np.ndarray) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self._joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q.tolist()]
        pt.time_from_start.nanosec = int(float(self.get_parameter('duration_sec').value) * 1e9)

        msg.points.append(pt)
        self.pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPublishNodeOMX()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
