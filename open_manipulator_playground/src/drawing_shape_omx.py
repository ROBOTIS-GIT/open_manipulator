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
# SPDX-FileCopyrightText: 2024 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
#
# Author: Minseo Choi

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---- inverse_kinematics ----


def rpy_to_rot(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3, dtype=float)

    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def rot_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    tr = float(np.trace(R))
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        if float(R[0, 0]) > float(R[1, 1]) and float(R[0, 0]) > float(R[2, 2]):
            S = (
                math.sqrt(
                    1.0
                    + float(R[0, 0])
                    - float(R[1, 1])
                    - float(R[2, 2])
                )
                * 2.0
            )
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif float(R[1, 1]) > float(R[2, 2]):
            S = (
                math.sqrt(
                    1.0
                    + float(R[1, 1])
                    - float(R[0, 0])
                    - float(R[2, 2])
                )
                * 2.0
            )
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = (
                math.sqrt(
                    1.0
                    + float(R[2, 2])
                    - float(R[0, 0])
                    - float(R[1, 1])
                )
                * 2.0
            )
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / n, qy / n, qz / n, qw / n)


def skew(v: np.ndarray) -> np.ndarray:
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    return np.array(
        [
            [0.0, -vz, vy],
            [vz, 0.0, -vx],
            [-vy, vx, 0.0],
        ],
        dtype=float,
    )


def rot_log(R: np.ndarray) -> np.ndarray:
    tr = float(np.trace(R))
    cos_theta = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return np.zeros(3, dtype=float)

    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-9:
        return np.zeros(3, dtype=float)

    w_hat = (R - R.T) * (0.5 / sin_theta)
    return np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]], dtype=float) * theta


def _parse_vec3(s: str) -> np.ndarray:
    parts = [p for p in s.split() if p != '']
    if len(parts) != 3:
        return np.zeros(3, dtype=float)
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)


def _is_active_joint(jtype: str) -> bool:
    return jtype in ('revolute', 'continuous', 'prismatic')


class SimpleURDFChain:
    def __init__(
        self,
        urdf_xml: str,
        base_link: str,
        ee_link: str,
        ee_fixed_joint: Optional[str] = None,
        allow_root_fallback: bool = True,
    ) -> None:
        self.requested_base_link = base_link
        self.base_link = base_link
        self.ee_link = ee_link
        self.ee_fixed_joint = ee_fixed_joint
        self.allow_root_fallback = allow_root_fallback

        root = ET.fromstring(urdf_xml)

        joint_map: Dict[str, Dict[str, object]] = {}
        child_to_joint: Dict[str, str] = {}

        for j in root.findall('joint'):
            name = j.get('name')
            jtype = j.get('type')
            if name is None or jtype is None:
                continue

            parent_elem = j.find('parent')
            child_elem = j.find('child')
            if parent_elem is None or child_elem is None:
                continue

            parent = parent_elem.get('link')
            child = child_elem.get('link')
            if parent is None or child is None:
                continue

            origin = j.find('origin')
            if origin is not None:
                xyz = _parse_vec3(origin.get('xyz', '0 0 0'))
                rpy = _parse_vec3(origin.get('rpy', '0 0 0'))
            else:
                xyz = np.zeros(3, dtype=float)
                rpy = np.zeros(3, dtype=float)

            axis_elem = j.find('axis')
            if axis_elem is not None:
                axis = _parse_vec3(axis_elem.get('xyz', '0 0 1'))
            else:
                axis = np.array([0.0, 0.0, 1.0], dtype=float)

            joint_map[name] = {
                'name': name,
                'type': jtype,
                'parent': parent,
                'child': child,
                'xyz': xyz,
                'rpy': rpy,
                'axis': axis,
            }
            child_to_joint[child] = name

        chain_joint_names: List[str] = []
        cur = ee_link
        guard = 0
        reached_root = False

        while cur != base_link:
            guard += 1
            if guard > 512:
                raise ValueError('URDF chain search exceeded limit (cycle?)')

            if cur not in child_to_joint:
                if self.allow_root_fallback:
                    self.base_link = cur
                    reached_root = True
                    break
                raise ValueError(f'Cannot find parent joint for link: {cur}')

            jname = child_to_joint[cur]
            chain_joint_names.append(jname)
            cur = str(joint_map[jname]['parent'])

        chain_joint_names.reverse()
        self._segments: List[Dict[str, object]] = [joint_map[jn] for jn in chain_joint_names]

        self.joint_names: List[str] = [
            str(seg['name']) for seg in self._segments if _is_active_joint(str(seg['type']))
        ]

        self._ee_fixed: Optional[Dict[str, object]] = None
        if ee_fixed_joint:
            if ee_fixed_joint not in joint_map:
                raise ValueError(f'EE fixed joint not found in URDF: {ee_fixed_joint}')
            self._ee_fixed = joint_map[ee_fixed_joint]

        self.reached_root_fallback = reached_root

    def forward(self, q_active: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        q_map = self._qvec_to_map(q_active)
        R, p, _, _, _ = self._fk_and_jac_cache(q_map)
        return p, R

    def jacobian(self, q_active: np.ndarray) -> np.ndarray:
        q_map = self._qvec_to_map(q_active)
        _, p_ee, joint_origins, joint_axes_world, joint_types = self._fk_and_jac_cache(q_map)

        n = len(self.joint_names)
        J = np.zeros((6, n), dtype=float)

        for i in range(n):
            o = joint_origins[i]
            a = joint_axes_world[i]
            t = joint_types[i]

            if t in ('revolute', 'continuous'):
                J[0:3, i] = np.cross(a, (p_ee - o))
                J[3:6, i] = a
            else:
                J[0:3, i] = a
                J[3:6, i] = 0.0

        return J

    def _qvec_to_map(self, q_active: np.ndarray) -> Dict[str, float]:
        q = np.asarray(q_active, dtype=float).reshape(-1)
        if q.shape[0] != len(self.joint_names):
            raise ValueError('q size mismatch with joint_names')
        return {jn: float(q[i]) for i, jn in enumerate(self.joint_names)}

    def _fk_and_jac_cache(
        self,
        q: Dict[str, float],
    ) -> Tuple[np.ndarray, np.ndarray, List[np.ndarray], List[np.ndarray], List[str]]:
        R = np.eye(3, dtype=float)
        p = np.zeros(3, dtype=float)

        joint_origins: List[np.ndarray] = []
        joint_axes_world: List[np.ndarray] = []
        joint_types: List[str] = []

        for seg in self._segments:
            p = p + R @ np.asarray(seg['xyz'], dtype=float)
            rpy = np.asarray(seg['rpy'], dtype=float)
            R = R @ rpy_to_rot(float(rpy[0]), float(rpy[1]), float(rpy[2]))

            jtype = str(seg['type'])
            jname = str(seg['name'])
            if not _is_active_joint(jtype):
                continue

            axis = np.asarray(seg['axis'], dtype=float)
            axis = axis / (np.linalg.norm(axis) + 1e-12)
            a_world = R @ axis

            joint_origins.append(p.copy())
            joint_axes_world.append(a_world)
            joint_types.append(jtype)

            val = float(q.get(jname, 0.0))
            if jtype in ('revolute', 'continuous'):
                K = skew(axis)
                Rot = np.eye(3) + math.sin(val) * K + (1.0 - math.cos(val)) * (K @ K)
                R = R @ Rot
            else:
                p = p + R @ (axis * val)

        if self._ee_fixed is not None:
            p = p + R @ np.asarray(self._ee_fixed['xyz'], dtype=float)
            rpy = np.asarray(self._ee_fixed['rpy'], dtype=float)
            R = R @ rpy_to_rot(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        return R, p, joint_origins, joint_axes_world, joint_types


@dataclass(frozen=True)
class IKConfig:
    position_only: bool = True
    max_iters: int = 120
    damping: float = 0.05
    step_scale: float = 0.8
    tol_pos_m: float = 1e-3
    tol_rot_rad: float = 1e-2
    w_pos: float = 1.0
    w_rot: float = 0.2


def solve_ik_dls(
    chain: SimpleURDFChain,
    target_xyz: np.ndarray,
    target_quat: List[float],
    seed_q: Optional[np.ndarray],
    cfg: IKConfig,
) -> Tuple[Optional[np.ndarray], float, float]:
    tgt_p = np.asarray(target_xyz, dtype=float).reshape(3)
    tq = target_quat if len(target_quat) == 4 else [0.0, 0.0, 0.0, 1.0]
    tgt_R = quat_to_rot(float(tq[0]), float(tq[1]), float(tq[2]), float(tq[3]))

    n = len(chain.joint_names)
    if seed_q is None:
        q = np.zeros(n, dtype=float)
    else:
        q = np.asarray(seed_q, dtype=float).reshape(-1)
        if q.shape[0] != n:
            q = np.zeros(n, dtype=float)

    err_pos = float('inf')
    err_rot = float('inf')

    for _ in range(int(cfg.max_iters)):
        p_cur, R_cur = chain.forward(q)
        e_p = tgt_p - p_cur
        err_pos = float(np.linalg.norm(e_p))

        if cfg.position_only:
            e_r = np.zeros(3, dtype=float)
            err_rot = 0.0
        else:
            R_err = R_cur.T @ tgt_R
            e_r = rot_log(R_err)
            err_rot = float(np.linalg.norm(e_r))

        if err_pos < float(cfg.tol_pos_m):
            if cfg.position_only or err_rot < float(cfg.tol_rot_rad):
                return q, err_pos, err_rot

        J = chain.jacobian(q)

        if cfg.position_only:
            e = np.concatenate([float(cfg.w_pos) * e_p, np.zeros(3, dtype=float)], axis=0)
        else:
            e = np.concatenate([float(cfg.w_pos) * e_p, float(cfg.w_rot) * e_r], axis=0)

        lam = float(cfg.damping)
        H = (J @ J.T) + (lam * lam) * np.eye(6, dtype=float)

        try:
            y = np.linalg.solve(H, e)
        except np.linalg.LinAlgError:
            y = np.linalg.pinv(H) @ e

        dq = J.T @ y
        q = q + float(cfg.step_scale) * dq

    return None, err_pos, err_rot


# ---- shape_trajectories ----


def plane_embed(
    plane: str,
    cx: float,
    cy: float,
    cz: float,
    x2d: float,
    y2d: float,
) -> List[float]:
    plane = (plane or 'xy').lower()
    if plane == 'xy':
        return [cx + x2d, cy + y2d, cz]
    if plane == 'xz':
        return [cx + x2d, cy, cz + y2d]
    if plane == 'yz':
        return [cx, cy + x2d, cz + y2d]
    return [cx + x2d, cy + y2d, cz]


def circle_point(
    t: float,
    center: List[float],
    radius: float,
    hz: float,
    plane: str,
    phase_deg: float = 0.0,
) -> List[float]:
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    phase = float(phase_deg) * math.pi / 180.0
    w = 2.0 * math.pi * float(hz)
    ct = math.cos(w * t + phase)
    st = math.sin(w * t + phase)
    return plane_embed(plane, cx, cy, cz, float(radius) * ct, float(radius) * st)


def heart_point(
    t: float,
    center: List[float],
    scale: float,
    hz: float,
    plane: str,
    phase_deg: float = 0.0,
) -> List[float]:
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    phase = float(phase_deg) * math.pi / 180.0
    w = 2.0 * math.pi * float(hz)
    th = w * t + phase
    x = 16.0 * (math.sin(th) ** 3)
    y = (
        13.0 * math.cos(th)
        - 5.0 * math.cos(2.0 * th)
        - 2.0 * math.cos(3.0 * th)
        - math.cos(4.0 * th)
    )
    x = (x / 18.0) * float(scale)
    y = (y / 18.0) * float(scale)
    return plane_embed(plane, cx, cy, cz, x, y)


def rounded_rect_point(
    u: float,
    width: float,
    height: float,
    corner_r: float,
) -> Tuple[float, float]:
    u = float(u) % 1.0
    w = max(float(width), 1e-9)
    h = max(float(height), 1e-9)
    a = 0.5 * w
    b = 0.5 * h

    r = max(0.0, float(corner_r))
    r = min(r, a - 1e-9, b - 1e-9)

    if r < 1e-6:
        perim = 2.0 * w + 2.0 * h
        s = u * perim
        if s < w:
            return (-a + s, -b)
        s -= w
        if s < h:
            return (a, -b + s)
        s -= h
        if s < w:
            return (a - s, b)
        s -= w
        return (-a, b - s)

    lx = max(w - 2.0 * r, 0.0)
    ly = max(h - 2.0 * r, 0.0)
    perim = 2.0 * (lx + ly) + 2.0 * math.pi * r
    s = u * perim
    arc_l = 0.5 * math.pi * r

    if s < lx:
        return ((-a + r) + s, -b)
    s -= lx

    if s < arc_l:
        ang = (-0.5 * math.pi) + (s / r)
        ccx, ccy = (a - r), (-b + r)
        return (ccx + r * math.cos(ang), ccy + r * math.sin(ang))
    s -= arc_l

    if s < ly:
        return (a, (-b + r) + s)
    s -= ly

    if s < arc_l:
        ang = 0.0 + (s / r)
        ccx, ccy = (a - r), (b - r)
        return (ccx + r * math.cos(ang), ccy + r * math.sin(ang))
    s -= arc_l

    if s < lx:
        return ((a - r) - s, b)
    s -= lx

    if s < arc_l:
        ang = 0.5 * math.pi + (s / r)
        ccx, ccy = (-a + r), (b - r)
        return (ccx + r * math.cos(ang), ccy + r * math.sin(ang))
    s -= arc_l

    if s < ly:
        return (-a, (b - r) - s)
    s -= ly

    ang = math.pi + (s / r)
    ccx, ccy = (-a + r), (-b + r)
    return (ccx + r * math.cos(ang), ccy + r * math.sin(ang))


def rectangle_point(
    t: float,
    center: List[float],
    width: float,
    height: float,
    corner_r: float,
    hz: float,
    plane: str,
    phase_deg: float = 0.0,
) -> List[float]:
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    f = float(hz)
    phase = float(phase_deg) * math.pi / 180.0

    if f <= 1e-9:
        u = 0.0
    else:
        theta = 2.0 * math.pi * f * t + phase
        u = (theta / (2.0 * math.pi)) % 1.0

    x2d, y2d = rounded_rect_point(u, width, height, corner_r)
    return plane_embed(plane, cx, cy, cz, x2d, y2d)


# ---- trajectory_publish ----


_SHAPES = {
    'circle': (
        circle_point,
        'circle_center',
        'circle_hz',
        'circle_plane',
        ('circle_radius',),
    ),
    'rectangle': (
        rectangle_point,
        'rect_center',
        'rect_hz',
        'rect_plane',
        ('rect_width', 'rect_height', 'rect_corner_radius'),
    ),
    'heart': (
        heart_point,
        'heart_center',
        'heart_hz',
        'heart_plane',
        ('heart_scale',),
    ),
}


class TrajectoryPublishNodeOMX(Node):

    def __init__(self) -> None:
        super().__init__('trajectory_publish_node_omx')

        self._declare_params()

        self.pub = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter('traj_topic').value),
            10,
        )
        self.ee_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('ee_feedback_topic').value),
            10,
        )
        self.target_pub = self.create_publisher(
            PointStamped,
            str(self.get_parameter('ee_target_topic').value),
            10,
        )

        self.param_client = AsyncParameterClient(
            self,
            str(self.get_parameter('robot_description_node').value),
        )

        self._urdf_request_inflight = False
        self._chain: Optional[SimpleURDFChain] = None
        self._joint_names: List[str] = []
        self._joint_to_idx: Dict[str, int] = {}
        self._resolved_base_link: str = str(self.get_parameter('base_link').value)

        self._last_joint_state: Optional[JointState] = None
        self._sent_once = False
        self._interactive_done = False

        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._on_joint_state,
            10,
        )

        self._t0 = self.get_clock().now()
        self.create_timer(float(self.get_parameter('duration_sec').value), self._on_timer)

        self._init_chain()

    def _declare_params(self) -> None:
        params = [
            ('traj_topic', '/omx_arm_controller/joint_trajectory'),
            ('joint_states_topic', '/joint_states'),
            ('robot_description_node', '/robot_state_publisher'),
            ('base_link', 'base_link'),
            ('ee_link', 'end_effector_link'),
            ('ee_fixed_joint', 'end_effector_joint'),
            ('urdf_param', 'robot_description'),
            ('ee_feedback_topic', '/omx_playground/ee_feedback'),
            ('ee_target_topic', '/omx_playground/ee_target'),
            ('publish_target_marker', True),
            ('publish_feedback_pose', True),
            ('repeat', True),
            ('interactive', False),
            ('traj_mode', 'circle'),
            ('duration_sec', 0.10),
            ('circle_center', [0.18, 0.00, 0.12]),
            ('circle_radius', 0.03),
            ('circle_hz', 0.20),
            ('circle_plane', 'yz'),
            ('rect_center', [0.18, 0.00, 0.12]),
            ('rect_width', 0.05),
            ('rect_height', 0.05),
            ('rect_corner_radius', 0.00),
            ('rect_hz', 0.20),
            ('rect_plane', 'yz'),
            ('heart_center', [0.18, 0.00, 0.12]),
            ('heart_scale', 0.03),
            ('heart_hz', 0.20),
            ('heart_plane', 'yz'),
            ('ee_quat', [0.0, 0.0, 0.0, 1.0]),
            ('ik_use_joint_states', True),
            ('ik_position_only', True),
            ('ik_max_iters', 120),
            ('ik_damping', 0.05),
            ('ik_step_scale', 0.8),
            ('ik_tol_pos_m', 1e-3),
            ('ik_tol_rot_rad', 1e-2),
            ('ik_w_pos', 1.0),
            ('ik_w_rot', 0.2),
        ]
        for name, default in params:
            self.declare_parameter(name, default)

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _base_frame(self) -> str:
        return self._resolved_base_link

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg

    def _init_chain(self) -> None:
        if self._chain is not None:
            return
        if self._urdf_request_inflight:
            return

        urdf_param = str(self.get_parameter('urdf_param').value)
        base_link = str(self.get_parameter('base_link').value)
        ee_link = str(self.get_parameter('ee_link').value)
        ee_fixed_joint = str(self.get_parameter('ee_fixed_joint').value)

        if not self.param_client.wait_for_services(timeout_sec=2.0):
            self.get_logger().warn('AsyncParameterClient not ready yet. Will retry later.')
            return

        self._urdf_request_inflight = True
        future = self.param_client.get_parameters([urdf_param])

        def _on_done(fut):
            self._urdf_request_inflight = False

            try:
                result = fut.result()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'Failed to get URDF parameter: {e}. Will retry later.')
                return

            if result is None or not getattr(result, 'values', None):
                self.get_logger().warn('URDF parameter result is empty. Will retry later.')
                return

            urdf_xml = result.values[0].string_value
            if not urdf_xml:
                self.get_logger().warn('URDF is empty. Will retry later.')
                return

            try:
                chain = SimpleURDFChain(
                    urdf_xml=urdf_xml,
                    base_link=base_link,
                    ee_link=ee_link,
                    ee_fixed_joint=ee_fixed_joint,
                    allow_root_fallback=True,
                )
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'Failed to parse/build URDF chain: {e}. Will retry later.')
                return

            self._chain = chain
            self._joint_names = list(chain.joint_names)
            self._joint_to_idx = {n: i for i, n in enumerate(self._joint_names)}
            self._resolved_base_link = chain.base_link

            traj_topic = str(self.get_parameter('traj_topic').value)
            self.get_logger().info(f'Publishing to: {traj_topic}')

            if chain.reached_root_fallback:
                msg = (
                    f'base_link {chain.requested_base_link!r} was NOT found on the path to EE. '
                    f'Falling back to URDF root {chain.base_link!r}. '
                    f'(Tip: set -p base_link:={chain.base_link} to silence this.)'
                )
                self.get_logger().warn(msg)

            if self._joint_names:
                self.get_logger().info(
                    (
                        f'Joints ({len(self._joint_names)}): '
                        f'{self._joint_names[0]} .. {self._joint_names[-1]}'
                    ),
                )
            else:
                self.get_logger().warn('No active joints found in the resolved chain.')

        future.add_done_callback(_on_done)

    # ---------- interactive helpers ----------

    @staticmethod
    def _prompt(label, default, caster, err_msg='  -> Invalid input (or press Enter).'):
        while True:
            s = input(f'{label} (default={default}) : ').strip()
            if s == '':
                return default
            try:
                return caster(s)
            except ValueError:
                print(err_msg)

    @staticmethod
    def _ask_float(label, default):
        return float(
            TrajectoryPublishNodeOMX._prompt(
                label,
                float(default),
                float,
                '  -> Please input a valid number (or press Enter).',
            ),
        )

    @staticmethod
    def _ask_int(label, default):
        return int(
            TrajectoryPublishNodeOMX._prompt(
                label,
                int(default),
                int,
                '  -> Please input a valid integer (or press Enter).',
            ),
        )

    @staticmethod
    def _ask_bool_yn(label, default):
        default_str = 'y' if default else 'n'

        def parse(s):
            v = s.strip().lower()
            if v in ('y', 'yes', '1', 'true', 't'):
                return True
            if v in ('n', 'no', '0', 'false', 'f'):
                return False
            raise ValueError()

        return bool(
            TrajectoryPublishNodeOMX._prompt(
                f'{label} (y/n, default={default_str})',
                default,
                parse,
                '  -> Please type y or n (or press Enter).',
            ),
        )

    @staticmethod
    def _ask_xyz(label, default_xyz, hint_for_single_value=''):
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
                    '(or press Enter).'
                )
                continue

            try:
                return [float(parts[0]), float(parts[1]), float(parts[2])]
            except ValueError:
                print('  -> Please input numbers like 0.18,0.00,0.12 (or press Enter).')

    def _interactive_update_params(self) -> None:
        default_mode = str(self.get_parameter('traj_mode').value).lower()
        mode = input('traj_mode [circle/rectangle/heart] : ').strip().lower()
        if mode == '':
            mode = default_mode

        plane = input('plane [xy/xz/yz] (recommended: yz) : ').strip().lower()
        if plane == '':
            plane = 'yz'

        if mode == 'circle':
            c = self._ask_xyz(
                'center',
                list(self.get_parameter('circle_center').value),
                'Single value detected. Radius is asked next.',
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
                'Single value detected. Width/height are asked next.',
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
                'Single value detected. Scale is asked next.',
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
            print('  -> Please type heart or circle or rectangle (or press Enter).')

        pos_only = self._ask_bool_yn(
            'IK position-only?',
            bool(self.get_parameter('ik_position_only').value),
        )
        w_rot = self._ask_float('ik_w_rot', float(self.get_parameter('ik_w_rot').value))
        tol_pos = self._ask_float(
            'ik_tol_pos_m',
            float(self.get_parameter('ik_tol_pos_m').value),
        )
        iters = self._ask_int('ik_max_iters', int(self.get_parameter('ik_max_iters').value))
        damping = self._ask_float('ik_damping', float(self.get_parameter('ik_damping').value))
        step = self._ask_float(
            'ik_step_scale',
            float(self.get_parameter('ik_step_scale').value),
        )

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

    # ---------- main loop ----------

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
        if (not self.get_parameter('ik_use_joint_states').value) or (
            self._last_joint_state is None
        ):
            return None

        if not self._joint_names:
            return None

        q = np.zeros(len(self._joint_names), dtype=float)
        for n, p in zip(self._last_joint_state.name, self._last_joint_state.position):
            i = self._joint_to_idx.get(n)
            if i is not None:
                q[i] = float(p)
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

        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9

        mode = str(self.get_parameter('traj_mode').value).lower()
        target_xyz = self._compute_target_xyz(mode, t)
        target_quat = list(self.get_parameter('ee_quat').value)

        if self.get_parameter('publish_target_marker').value:
            self._publish_target_xyz(target_xyz)

        q_sol, err_pos, err_rot = solve_ik_dls(
            self._chain,
            target_xyz,
            target_quat,
            self._get_seed_q(),
            self._make_ik_config(),
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
            self._publish_feedback_pose(ee_xyz, rot_to_quat(ee_rot))

        self._publish_trajectory(q_sol)
        self._sent_once = True

    def _compute_target_xyz(self, mode: str, t: float) -> np.ndarray:
        fn, c_param, hz_param, plane_param, extras = _SHAPES.get(mode, _SHAPES['heart'])
        c = np.array(self.get_parameter(c_param).value, dtype=float)
        hz = float(self.get_parameter(hz_param).value)
        plane = str(self.get_parameter(plane_param).value).lower()
        extra_vals = [float(self.get_parameter(p).value) for p in extras]
        return np.array(fn(t, c, *extra_vals, hz, plane), dtype=float)

    def _publish_target_xyz(self, xyz: np.ndarray) -> None:
        msg = PointStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._base_frame()
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        self.target_pub.publish(msg)

    def _publish_feedback_pose(self, xyz: np.ndarray, quat_xyzw) -> None:
        msg = PoseStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._base_frame()
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
        msg.header.stamp = self._stamp()
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
