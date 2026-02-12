#!/usr/bin/env python3
# Copyright 2024 ROBOTIS CO., LTD.
# SPDX-FileCopyrightText: 2024 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple

import xml.etree.ElementTree as ET

import numpy as np


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
            S = math.sqrt(1.0 + float(R[0, 0]) - float(R[1, 1]) - float(R[2, 2])) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif float(R[1, 1]) > float(R[2, 2]):
            S = math.sqrt(1.0 + float(R[1, 1]) - float(R[0, 0]) - float(R[2, 2])) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + float(R[2, 2]) - float(R[0, 0]) - float(R[1, 1])) * 2.0
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
    ) -> None:
        self.base_link = base_link
        self.ee_link = ee_link
        self.ee_fixed_joint = ee_fixed_joint

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
        while cur != base_link:
            guard += 1
            if guard > 256:
                raise ValueError('URDF chain search exceeded limit (cycle?)')

            if cur not in child_to_joint:
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

    def forward(self, q_active: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        q_map = self._qvec_to_map(q_active)
        R, p, _, _, _ = self._fk_and_jac_cache(q_map)
        return p, R

    def jacobian(self, q_active: np.ndarray) -> np.ndarray:
        q_map = self._qvec_to_map(q_active)
        _, _, joint_origins, joint_axes_world, joint_types = self._fk_and_jac_cache(q_map)

        n = len(self.joint_names)
        J = np.zeros((6, n), dtype=float)

        R_ee, p_ee, _, _, _ = self._fk_and_jac_cache(q_map)
        _ = R_ee

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
            e = np.concatenate(
                [float(cfg.w_pos) * e_p, float(cfg.w_rot) * e_r],
                axis=0,
            )

        lam = float(cfg.damping)
        H = (J @ J.T) + (lam * lam) * np.eye(6, dtype=float)

        try:
            y = np.linalg.solve(H, e)
        except np.linalg.LinAlgError:
            y = np.linalg.pinv(H) @ e

        dq = J.T @ y
        q = q + float(cfg.step_scale) * dq

    return None, err_pos, err_rot
