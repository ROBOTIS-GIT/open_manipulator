from __future__ import annotations

import math
import xml.etree.ElementTree as ET
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


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    tr = float(np.trace(R))
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
    return np.array([qx, qy, qz, qw], dtype=float)


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def rot_log(R: np.ndarray) -> np.ndarray:
    tr = float(np.trace(R))
    c = (tr - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    theta = math.acos(c)
    if theta < 1e-9:
        return np.zeros(3, dtype=float)
    w = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        dtype=float,
    ) * (0.5 / math.sin(theta))
    return w * theta


# =========================
# URDF chain for FK/Jacobian
# =========================
class SimpleURDFChain:
    def __init__(
        self,
        urdf_xml: str,
        base_link: str = "link0",
        ee_link: str = "end_effector_link",
        active_joints: Tuple[str, ...] = ("joint1", "joint2", "joint3", "joint4", "joint5"),
        ee_fixed_joint: str = "end_effector_joint",
    ):
        root = ET.fromstring(urdf_xml)
        joint_map: Dict[str, Dict[str, object]] = {}

        for j in root.findall("joint"):
            name = j.get("name")
            if not name:
                continue

            parent = j.find("parent").get("link")
            child = j.find("child").get("link")

            origin_el = j.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin_el is not None:
                if origin_el.get("xyz"):
                    xyz = [float(v) for v in origin_el.get("xyz").split()]
                if origin_el.get("rpy"):
                    rpy = [float(v) for v in origin_el.get("rpy").split()]

            axis_el = j.find("axis")
            axis = [0.0, 0.0, 0.0]
            if axis_el is not None and axis_el.get("xyz"):
                axis = [float(v) for v in axis_el.get("xyz").split()]

            joint_map[name] = {
                "type": j.get("type"),
                "parent": parent,
                "child": child,
                "xyz": np.array(xyz, dtype=float),
                "rpy": np.array(rpy, dtype=float),
                "axis": np.array(axis, dtype=float),
            }

        self.base_link = base_link
        self.ee_link = ee_link
        self.active_joints = list(active_joints)

        self.joints: List[Tuple[str, Dict[str, object]]] = []
        current_parent = base_link
        for jn in self.active_joints:
            if jn not in joint_map:
                raise RuntimeError(f"URDF missing joint: {jn}")
            jd = joint_map[jn]
            if jd["parent"] != current_parent:
                raise RuntimeError(
                    f"Chain mismatch: expected parent {current_parent} for {jn}, got {jd['parent']}"
                )
            self.joints.append((jn, jd))
            current_parent = jd["child"]

        self.ee_fixed = joint_map.get(ee_fixed_joint, None) if ee_fixed_joint else None

    def fk(self, q: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        R = np.eye(3)
        p = np.zeros(3)

        for jn, jd in self.joints:
            xyz = jd["xyz"]
            rpy = jd["rpy"]
            axis = jd["axis"]
            jtype = jd["type"]

            p = p + R @ xyz
            R = R @ rpy_to_rot(*rpy)

            ang = float(q[jn])
            if jtype in ("revolute", "continuous"):
                a = axis / (np.linalg.norm(axis) + 1e-12)
                K = skew(a)
                Rj = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)
                R = R @ Rj
            elif jtype == "prismatic":
                a = axis / (np.linalg.norm(axis) + 1e-12)
                p = p + R @ (a * ang)

        if self.ee_fixed is not None:
            p = p + R @ self.ee_fixed["xyz"]
            R = R @ rpy_to_rot(*self.ee_fixed["rpy"])

        return R, p

    def jacobian(self, q: Dict[str, float]) -> np.ndarray:
        R = np.eye(3)
        p = np.zeros(3)

        joint_origins: List[np.ndarray] = []
        joint_axes: List[np.ndarray] = []

        for jn, jd in self.joints:
            xyz = jd["xyz"]
            rpy = jd["rpy"]
            axis = jd["axis"]
            jtype = jd["type"]

            p = p + R @ xyz
            R = R @ rpy_to_rot(*rpy)

            a = axis / (np.linalg.norm(axis) + 1e-12)
            a_world = R @ a
            joint_origins.append(p.copy())
            joint_axes.append(a_world.copy())

            ang = float(q[jn])
            if jtype in ("revolute", "continuous"):
                K = skew(a)
                Rj = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)
                R = R @ Rj
            elif jtype == "prismatic":
                p = p + R @ (a * ang)

        if self.ee_fixed is not None:
            p_ee = p + R @ self.ee_fixed["xyz"]
        else:
            p_ee = p.copy()

        n = len(self.joints)
        J = np.zeros((6, n), dtype=float)
        for i in range(n):
            a = joint_axes[i]
            o = joint_origins[i]
            J[0:3, i] = np.cross(a, (p_ee - o))
            J[3:6, i] = a
        return J
