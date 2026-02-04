from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from traj_kinematics import SimpleURDFChain, quat_to_rot, rot_log


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

        JT = J.T
        H = J @ JT + (float(cfg.damping) ** 2) * np.eye(6)
        dq = JT @ np.linalg.solve(H, e)

        step = float(cfg.step_scale)
        for i, jn in enumerate(joint_names):
            q[jn] = float(q[jn] + step * dq[i])

    return None, last_err
