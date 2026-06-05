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
# Motion helpers for OMY teleoperation: config loading, Cartesian-delta -> joint
# mapping (MVP), anti-jerk slew interpolation, joint-limit clamps, and
# distance-scaled durations for named poses.

import math
from dataclasses import dataclass
from dataclasses import field
from typing import Dict
from typing import List
from typing import Optional


@dataclass
class JointLimits:
    min_pos: float
    max_pos: float


@dataclass
class TeleopConfig:
    # --- delta -> joint mapping ---
    # Reachy sends Cartesian deltas in METERS (±0.02 m clamp on its side).
    # meters_to_rad converts a 1 m Cartesian delta into joint radians for the MVP
    # mapping. Tuned small for safety; raise carefully after validating on hardware.
    meters_to_rad: float = 1.0
    max_joint_delta_per_command: float = 0.01   # clamp per single delta (rad)

    # --- trajectory publishing ---
    trajectory_duration: float = 0.2            # fallback segment duration (s)
    command_timeout_sec: float = 1.0            # auto-hold if no delta for this long

    # --- gripper ---
    gripper_debounce_sec: float = 0.8

    # --- anti-jerk slew ---
    # The arm never jumps to the target. A background loop advances the commanded
    # position toward the target by at most max_joint_step_per_tick each tick.
    # Effective max joint speed = slew_rate_hz * max_joint_step_per_tick (rad/s).
    slew_rate_hz: float = 20.0
    max_joint_speed_rad_s: float = 0.03        # hard teleop speed cap (rad/s)
    max_joint_step_per_tick: float = 0.002
    max_target_offset: float = 0.08             # target may lead actual by this (rad)

    # --- soft-start (ramp-up) ---
    # Right after arming, motion must start SLOW and ease in — otherwise a hand
    # already off-center makes the arm lunge at full speed the instant it arms.
    # For ramp_up_sec after arming, both the per-tick step and the target offset
    # are scaled by a factor that ramps 0 -> 1, so the arm creeps at first and
    # only reaches full speed once the operator has settled in.
    ramp_up_sec: float = 4.0
    ramp_up_min_factor: float = 0.02            # starting fraction at t=0 (not 0,
                                                # so it still inches toward the hand)

    # --- named poses (home/ready) ---
    # Move time is scaled by distance, bounded, so a far pose is reached smoothly
    # rather than snapping. duration = clamp(distance / named_pose_speed, min, max).
    named_pose_speed: float = 0.03              # rad/s used to derive duration
    named_pose_min_duration: float = 2.0
    named_pose_max_duration: float = 30.0

    # --- arming ---
    # If True, a teleop_delta while disarmed auto-arms (seeded from current pose).
    # The Reachy contract sends teleop_start explicitly, so default False to honor
    # "reject deltas before start". Can be enabled for clients that only stream.
    auto_arm_on_delta: bool = False

    joint_names: List[str] = field(default_factory=lambda: [
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
    ])
    joint_limits: Dict[str, JointLimits] = field(default_factory=dict)
    gripper_open_position: float = 0.0
    gripper_close_position: float = 1.14
    gripper_max_effort: float = 10.0
    predefined_poses: Dict[str, List[float]] = field(default_factory=dict)

    @classmethod
    def from_yaml_dict(cls, cfg: dict) -> 'TeleopConfig':
        teleop = cfg.get('teleop', {})
        joint_names = cfg.get('joint_names', [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ])

        joint_limits = {}
        for name, bounds in cfg.get('joint_limits', {}).items():
            if isinstance(bounds, (list, tuple)) and len(bounds) == 2:
                joint_limits[name] = JointLimits(float(bounds[0]), float(bounds[1]))

        predefined_poses = {}
        for pose_name, positions in cfg.get('predefined_poses', {}).items():
            if isinstance(positions, list):
                predefined_poses[pose_name] = [float(p) for p in positions]

        g = cfg.get('gripper', {})

        return cls(
            meters_to_rad=float(teleop.get('meters_to_rad', 1.0)),
            max_joint_delta_per_command=float(
                teleop.get('max_joint_delta_per_command', 0.01)),
            trajectory_duration=float(teleop.get('trajectory_duration', 0.2)),
            command_timeout_sec=float(teleop.get('command_timeout_sec', 1.0)),
            gripper_debounce_sec=float(teleop.get('gripper_debounce_sec', 0.8)),
            slew_rate_hz=float(teleop.get('slew_rate_hz', 20.0)),
            max_joint_speed_rad_s=float(teleop.get('max_joint_speed_rad_s', 0.03)),
            max_joint_step_per_tick=float(teleop.get('max_joint_step_per_tick', 0.002)),
            max_target_offset=float(teleop.get('max_target_offset', 0.08)),
            ramp_up_sec=float(teleop.get('ramp_up_sec', 4.0)),
            ramp_up_min_factor=float(teleop.get('ramp_up_min_factor', 0.02)),
            named_pose_speed=float(teleop.get('named_pose_speed', 0.03)),
            named_pose_min_duration=float(teleop.get('named_pose_min_duration', 2.0)),
            named_pose_max_duration=float(teleop.get('named_pose_max_duration', 30.0)),
            auto_arm_on_delta=bool(teleop.get('auto_arm_on_delta', False)),
            joint_names=joint_names,
            joint_limits=joint_limits,
            gripper_open_position=float(g.get('open_position', 0.0)),
            gripper_close_position=float(g.get('close_position', 1.14)),
            gripper_max_effort=float(g.get('max_effort', 10.0)),
            predefined_poses=predefined_poses,
        )


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def apply_joint_limits(
    positions: List[float],
    joint_names: List[str],
    joint_limits: Dict[str, JointLimits],
) -> List[float]:
    result = list(positions)
    for i, name in enumerate(joint_names):
        if name in joint_limits:
            lim = joint_limits[name]
            result[i] = clamp(result[i], lim.min_pos, lim.max_pos)
    return result


def compute_teleop_delta_joints(
    base: List[float],
    dx: float,
    dy: float,
    dz: float,
    config: TeleopConfig,
) -> List[float]:
    """
    Map a Cartesian delta (meters) onto joint-space, added to `base`.

    MVP demo mapping (NOT real IK). TODO: replace with MoveIt Servo / analytic IK
    for precise end-effector tracking. Only directional response is guaranteed.

    omy_f3m, arm extended forward (Reachy frame: dx=fwd, dy=right, dz=up):
      dy → joint1 (base yaw, lateral sweep)
      dz → joint2/joint3 (shoulder/elbow, up-down)
      dx → joint2 (reach in/out)
    """
    s = config.meters_to_rad
    md = config.max_joint_delta_per_command

    def scaled(v: float) -> float:
        return clamp(v * s, -md, md)

    out = list(base)
    out[0] += scaled(-dy)                       # +dy (right) sweeps base right
    out[1] += scaled(-dz * 0.6 + dx * 0.4)      # shoulder: down for +dz, fwd for +dx
    out[2] += scaled(dz * 0.8)                  # elbow follows dz
    out[4] += scaled(dz * 0.2)                  # wrist pitch keeps EE level-ish
    return apply_joint_limits(out, config.joint_names, config.joint_limits)


def compute_named_pose_joints(pose_name: str, config: TeleopConfig) -> Optional[List[float]]:
    return config.predefined_poses.get(pose_name)


def named_pose_duration(target: List[float], current: List[float], config: TeleopConfig) -> float:
    """Distance-scaled, bounded move time so a far pose isn't reached by snapping."""
    distance = max_abs_diff(target, current)
    speed = max(1e-3, min(config.named_pose_speed, config.max_joint_speed_rad_s))
    speed_limited_duration = distance / speed
    bounded_duration = clamp(
        speed_limited_duration,
        config.named_pose_min_duration,
        config.named_pose_max_duration,
    )
    return max(bounded_duration, speed_limited_duration)


def clamp_target_to_offset(
    target: List[float],
    reference: List[float],
    max_offset: float,
) -> List[float]:
    """Keep each target joint within ±max_offset of the actual (reference) pose,
    so accumulated deltas can never build up an unbounded lead/jump."""
    if len(target) != len(reference):
        return list(target)
    return [clamp(t, r - max_offset, r + max_offset) for t, r in zip(target, reference)]


def ramp_up_factor(elapsed_sec: float, config: TeleopConfig) -> float:
    """
    Soft-start factor that ramps from ramp_up_min_factor (at arm time) to 1.0
    (after ramp_up_sec), eased so motion starts slow and accelerates gently.

    Returns 1.0 once ramp_up_sec has elapsed (or if ramp_up_sec <= 0).
    The factor scales BOTH the per-tick slew step and the target offset, so right
    after arming the arm only creeps toward the hand and never lunges.
    """
    if config.ramp_up_sec <= 0.0:
        return 1.0
    frac = clamp(elapsed_sec / config.ramp_up_sec, 0.0, 1.0)
    # smoothstep easing (3f^2 - 2f^3): gentle at the start, smooth at the end
    eased = frac * frac * (3.0 - 2.0 * frac)
    lo = config.ramp_up_min_factor
    return lo + (1.0 - lo) * eased


def slew_step(command: List[float], target: List[float], max_step: float) -> List[float]:
    """Advance `command` one bounded step toward `target` (±max_step per joint).
    Core anti-jerk: even a far target only moves the command a little each tick."""
    if len(command) != len(target):
        return list(target)
    out = []
    for c, t in zip(command, target):
        d = clamp(t - c, -max_step, max_step)
        out.append(c + d)
    return out


def max_abs_diff(a: List[float], b: List[float]) -> float:
    if not a or not b or len(a) != len(b):
        return 0.0
    return max(abs(x - y) for x, y in zip(a, b))


def joints_are_valid(positions: List[float], expected_count: int) -> bool:
    return (
        isinstance(positions, list)
        and len(positions) == expected_count
        and all(math.isfinite(p) for p in positions)
    )
