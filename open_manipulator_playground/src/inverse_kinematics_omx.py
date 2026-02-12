#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


# =========================
# Math utils
# =========================
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
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=float,
    )


def rot_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    # Robust conversion
    tr = float(np.trace(R))
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = math.sqrt(1.0 + float(R[0, 0]) - float(R[1, 1]) - float(R[2, 2])) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
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
    # SO(3) logarithm map -> rotation vector (axis * angle)
    tr = float(np.trace(R))
    cos_theta = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return np.zeros(3, dtype=float)

    # When sin(theta) is small, numerical issues can happen
    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-9:
        # Fallback: approximate using diagonal
        return np.array([0.0, 0.0, 0.0], dtype=float)

    w_hat = (R - R.T) * (0.5 / sin_theta)
    return np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]], dtype=float) * theta


# =========================
# URDF chain (FK / Jacobian)
# =========================
class SimpleURDFChain:
    def __init__(
        self,
        urdf_xml: str,
        base_link: str,
        ee_link: str,
        active_joints: List[str],
        ee_fixed_joint: Optional[str] = None,
    ) -> None:
        self.base_link = base_link
        self.ee_link = ee_link
        self.active_joints = active_joints
        self.ee_fixed_joint = ee_fixed_joint

        root = ET.fromstring(urdf_xml)
        self.joint_map: Dict[str, dict] = {}
        for j in root.findall("joint"):
            name = j.get("name")
            jtype = j.get("type")
            parent = j.find("parent").get("link")
            child = j.find("child").get("link")

            origin = j.find("origin")
            if origin is not None:
                xyz_str = origin.get("xyz", "0 0 0")
                rpy_str = origin.get("rpy", "0 0 0")
            else:
                xyz_str = "0 0 0"
                rpy_str = "0 0 0"

            xyz = [float(x) for x in xyz_str.split()]
            rpy = [float(x) for x in rpy_str.split()]

            axis_elem = j.find("axis")
            if axis_elem is not None:
                axis = [float(x) for x in axis_elem.get("xyz", "0 0 1").split()]
            else:
                axis = [0.0, 0.0, 1.0]

            limit_elem = j.find("limit")
            if limit_elem is not None:
                lower = float(limit_elem.get("lower", "nan"))
                upper = float(limit_elem.get("upper", "nan"))
            else:
                lower = float("nan")
                upper = float("nan")

            self.joint_map[name] = dict(
                name=name,
                type=jtype,
                parent=parent,
                child=child,
                xyz=np.array(xyz, dtype=float),
                rpy=np.array(rpy, dtype=float),
                axis=np.array(axis, dtype=float),
                lower=lower,
                upper=upper,
            )

        # Build joint chain: follow active_joints order given by caller
        self.chain: List[dict] = []
        for jn in self.active_joints:
            if jn not in self.joint_map:
                raise ValueError(f"Joint '{jn}' not found in URDF")
            self.chain.append(self.joint_map[jn])

        # Optional EE fixed joint (tool transform)
        self.ee_fixed: Optional[dict] = None
        if self.ee_fixed_joint:
            if self.ee_fixed_joint not in self.joint_map:
                raise ValueError(f"EE fixed joint '{self.ee_fixed_joint}' not found in URDF")
            self.ee_fixed = self.joint_map[self.ee_fixed_joint]

    def fk(self, q: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        R = np.eye(3, dtype=float)
        p = np.zeros(3, dtype=float)

        for j in self.chain:
            # origin
            p = p + R @ j["xyz"]
            R = R @ rpy_to_rot(float(j["rpy"][0]), float(j["rpy"][1]), float(j["rpy"][2]))

            axis = j["axis"].astype(float)
            jtype = j["type"]
            val = float(q.get(j["name"], 0.0))

            if jtype in ("revolute", "continuous"):
                # Rodrigues
                a = axis / (np.linalg.norm(axis) + 1e-12)
                K = skew(a)
                Rot = np.eye(3) + math.sin(val) * K + (1 - math.cos(val)) * (K @ K)
                R = R @ Rot
            elif jtype == "prismatic":
                a = axis / (np.linalg.norm(axis) + 1e-12)
                p = p + R @ (a * val)
            else:
                # fixed etc.
                pass

        # EE fixed joint transform
        if self.ee_fixed is not None:
            p = p + R @ self.ee_fixed["xyz"]
            R = R @ rpy_to_rot(
                float(self.ee_fixed["rpy"][0]),
                float(self.ee_fixed["rpy"][1]),
                float(self.ee_fixed["rpy"][2]),
            )

        return R, p

    def jacobian(self, q: Dict[str, float]) -> np.ndarray:
        # geometric Jacobian (6 x n): [v; w]
        n = len(self.chain)
        J = np.zeros((6, n), dtype=float)

        R = np.eye(3, dtype=float)
        p = np.zeros(3, dtype=float)

        joint_origins: List[np.ndarray] = []
        joint_axes_world: List[np.ndarray] = []
        joint_types: List[str] = []

        for j in self.chain:
            # origin
            p = p + R @ j["xyz"]
            R = R @ rpy_to_rot(float(j["rpy"][0]), float(j["rpy"][1]), float(j["rpy"][2]))

            axis = j["axis"].astype(float)
            a = axis / (np.linalg.norm(axis) + 1e-12)
            a_world = R @ a

            joint_origins.append(p.copy())
            joint_axes_world.append(a_world)
            joint_types.append(j["type"])

            val = float(q.get(j["name"], 0.0))
            if j["type"] in ("revolute", "continuous"):
                K = skew(a)
                Rot = np.eye(3) + math.sin(val) * K + (1 - math.cos(val)) * (K @ K)
                R = R @ Rot
            elif j["type"] == "prismatic":
                p = p + R @ (a * val)
            else:
                pass

        # EE position
        if self.ee_fixed is not None:
            p_ee = p + R @ self.ee_fixed["xyz"]
        else:
            p_ee = p

        for i in range(n):
            o = joint_origins[i]
            a = joint_axes_world[i]
            t = joint_types[i]

            if t in ("revolute", "continuous"):
                J[0:3, i] = np.cross(a, (p_ee - o))
                J[3:6, i] = a
            elif t == "prismatic":
                J[0:3, i] = a
                J[3:6, i] = 0.0
            else:
                J[:, i] = 0.0

        return J


# =========================
# IK (DLS) solver
# =========================
@dataclass(frozen=True)
class IKConfig:
    position_only: bool = True
    max_iters: int = 200
    damping: float = 0.10
    step_scale: float = 0.40
    tol_pos_m: float = 0.008
    tol_rot_rad: float = 0.15
    w_pos: float = 1.0
    w_rot: float = 0.05


def solve_ik_dls(
    chain: SimpleURDFChain,
    joint_names: List[str],
    q_init: Dict[str, float],
    target_xyz: List[float],
    target_quat: List[float],
    cfg: IKConfig,
) -> Tuple[Optional[Dict[str, float]], Tuple[float, float]]:
    # Defensive: target quaternion
    tq = target_quat if len(target_quat) == 4 else [0.0, 0.0, 0.0, 1.0]
    target_R = quat_to_rot(float(tq[0]), float(tq[1]), float(tq[2]), float(tq[3]))
    target_p = np.array(target_xyz, dtype=float)

    q: Dict[str, float] = {jn: float(q_init.get(jn, 0.0)) for jn in joint_names}
    last_err = (math.inf, math.inf)

    for _ in range(int(cfg.max_iters)):
        R, p = chain.fk(q)
        e_p = target_p - p

        if cfg.position_only:
            e_r = np.zeros(3, dtype=float)
            err_rot = 0.0
        else:
            R_err = R.T @ target_R
            e_r = rot_log(R_err)
            err_rot = float(np.linalg.norm(e_r))

        err_pos = float(np.linalg.norm(e_p))
        last_err = (err_pos, err_rot)

        if err_pos < float(cfg.tol_pos_m) and (cfg.position_only or err_rot < float(cfg.tol_rot_rad)):
            return q, last_err

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

        step = float(cfg.step_scale)
        for i, jn in enumerate(joint_names):
            q[jn] = float(q[jn]) + step * float(dq[i])

    return None, last_err
