#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
# Author: Minseo Choi

from __future__ import annotations
import math
from typing import Dict, List, Optional, Tuple
import os
import sys

import numpy as np
import rclpy
import rclpy.parameter
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(__file__))
from inverse_kinematics_omx import (
    SimpleURDFChain,
    rot_to_quat,
    IKConfig,
    solve_ik_dls,
)
from shape_trajectories_omx import circle_point, heart_point, rectangle_point


class TrajPubWithIK(Node):
    DEFAULT_PLANE_RECOMMENDED = "yz"

    def __init__(self):
        super().__init__("traj_pub")

        self.declare_parameter("interactive", False)
        self.declare_parameter("ee_xyz", [0.18, 0.00, 0.12])
        self.declare_parameter("ee_quat", [0.0, 0.0, 0.0, 1.0])

        self.declare_parameter("ik_position_only", True)
        self.declare_parameter("ik_max_iters", 200)
        self.declare_parameter("ik_damping", 0.10)
        self.declare_parameter("ik_step_scale", 0.40)
        self.declare_parameter("ik_use_joint_states", True)
        self.declare_parameter("ik_tol_pos_m", 0.008)
        self.declare_parameter("ik_tol_rot_rad", 0.15)
        self.declare_parameter("ik_w_pos", 1.0)
        self.declare_parameter("ik_w_rot", 0.05)

        self.declare_parameter("robot_description_node", "/robot_state_publisher")
        self.declare_parameter("robot_description_param", "robot_description")

        self.declare_parameter("traj_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5"])
        self.declare_parameter("duration_sec", 0.05)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("repeat", True)

        self.declare_parameter("ee_feedback_topic", "/omx_ik/ee_feedback")
        self.declare_parameter("ee_target_topic", "/omx_ik/ee_target")

        self.declare_parameter("traj_mode", "heart")

        self.declare_parameter("circle_center", [0.18, 0.00, 0.12])
        self.declare_parameter("circle_radius", 0.05)
        self.declare_parameter("circle_hz", 0.05)
        self.declare_parameter("circle_plane", "xy")
        self.declare_parameter("circle_phase_deg", 0.0)

        self.declare_parameter("heart_center", [0.18, 0.00, 0.12])
        self.declare_parameter("heart_scale", 0.07)
        self.declare_parameter("heart_hz", 0.05)
        self.declare_parameter("heart_plane", "xy")
        self.declare_parameter("heart_phase_deg", 0.0)

        self.declare_parameter("rect_center", [0.18, 0.00, 0.12])
        self.declare_parameter("rect_width", 0.17)
        self.declare_parameter("rect_height", 0.15)
        self.declare_parameter("rect_hz", 0.05)
        self.declare_parameter("rect_plane", "xy")
        self.declare_parameter("rect_phase_deg", 0.0)
        self.declare_parameter("rect_corner_radius", 0.01)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.pub = self.create_publisher(JointTrajectory, str(self.get_parameter("traj_topic").value), 10)
        self.ee_pub = self.create_publisher(PoseStamped, str(self.get_parameter("ee_feedback_topic").value), 10)
        self.target_pub = self.create_publisher(PointStamped, str(self.get_parameter("ee_target_topic").value), 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self.latest_js: Dict[str, float] = {}
        self.urdf_chain: Optional[SimpleURDFChain] = None

        self.param_client = AsyncParameterClient(self, str(self.get_parameter("robot_description_node").value))
        self._urdf_future = None

        self._t0 = self.get_clock().now()
        self._tick = 0
        self._sent_once = False
        self._interactive_done = False

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(hz, 0.1), self._tick_cb)

    def _on_joint_states(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.latest_js[name] = float(pos)

    def _publish_target(self, xyz: List[float]) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link0"
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        self.target_pub.publish(msg)

    def _publish_feedback(self, q: Dict[str, float]) -> None:
        if self.urdf_chain is None:
            return
        R, p = self.urdf_chain.fk(q)
        quat = rot_to_quat(R)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link0"
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = float(p[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self.ee_pub.publish(msg)

    def _request_urdf_if_needed(self) -> None:
        if self.urdf_chain is not None or self._urdf_future is not None:
            return
        if not self.param_client.services_are_ready():
            self.get_logger().warning("Waiting for robot_description parameter service...")
            return
        param_name = str(self.get_parameter("robot_description_param").value)
        self._urdf_future = self.param_client.get_parameters([param_name])

    def _build_chain_if_ready(self) -> bool:
        if self.urdf_chain is not None:
            return True

        self._request_urdf_if_needed()
        if self._urdf_future is None or not self._urdf_future.done():
            return False

        try:
            values = self._urdf_future.result()
        except Exception:
            self._urdf_future = None
            return False

        if values is None or len(values.values) == 0:
            self._urdf_future = None
            return False

        urdf_xml = values.values[0].string_value
        if not urdf_xml:
            self._urdf_future = None
            return False

        active = tuple(self.get_parameter("joint_names").value)
        self.urdf_chain = SimpleURDFChain(
            urdf_xml=urdf_xml,
            base_link="link0",
            ee_link="end_effector_link",
            active_joints=active,
            ee_fixed_joint="end_effector_joint",
        )
        return True

    def _current_q(self) -> Dict[str, float]:
        return {jn: float(self.latest_js.get(jn, 0.0)) for jn in self.joint_names}

    def _now_t(self) -> float:
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def _compute_target_xyz(self) -> List[float]:
        t = self._now_t()
        mode = str(self.get_parameter("traj_mode").value).lower()

        if mode == "circle":
            return circle_point(
                t=t,
                center=list(self.get_parameter("circle_center").value),
                radius=float(self.get_parameter("circle_radius").value),
                hz=float(self.get_parameter("circle_hz").value),
                plane=str(self.get_parameter("circle_plane").value),
                phase_deg=float(self.get_parameter("circle_phase_deg").value),
            )

        if mode in ("rectangle", "rect"):
            return rectangle_point(
                t=t,
                center=list(self.get_parameter("rect_center").value),
                width=float(self.get_parameter("rect_width").value),
                height=float(self.get_parameter("rect_height").value),
                hz=float(self.get_parameter("rect_hz").value),
                plane=str(self.get_parameter("rect_plane").value),
                phase_deg=float(self.get_parameter("rect_phase_deg").value),
                corner_r=float(self.get_parameter("rect_corner_radius").value),
            )

        if mode == "heart":
            return heart_point(
                t=t,
                center=list(self.get_parameter("heart_center").value),
                scale=float(self.get_parameter("heart_scale").value),
                hz=float(self.get_parameter("heart_hz").value),
                plane=str(self.get_parameter("heart_plane").value),
                phase_deg=float(self.get_parameter("heart_phase_deg").value),
            )

        return list(self.get_parameter("ee_xyz").value)

    @staticmethod
    def _ask_mode(default_mode: str) -> str:
        while True:
            s = input("trajectory (heart/circle/rectangle) : ").strip().lower()
            if not s:
                return default_mode
            if s in ("heart", "circle", "rectangle", "rect"):
                return "rectangle" if s in ("rectangle", "rect") else s
            print("  -> Please type 'heart' or 'circle' or 'rectangle' (or press Enter).")

    @staticmethod
    def _ask_plane(default_plane: str) -> str:
        while True:
            s = input(f"plane (xy/xz/yz) (recommended: {default_plane}) : ").strip().lower()
            if not s:
                return default_plane
            if s in ("xy", "xz", "yz"):
                return s
            print("  -> Please type one of: xy / xz / yz (or press Enter).")

    @staticmethod
    def _ask_float(label: str, default_val: float) -> float:
        while True:
            s = input(f"{label} (recommended: {default_val}) : ").strip()
            if not s:
                return float(default_val)
            try:
                return float(s)
            except ValueError:
                print("  -> Please input a valid number (or press Enter).")

    @staticmethod
    def _ask_int(label: str, default_val: int) -> int:
        while True:
            s = input(f"{label} (recommended: {default_val}) : ").strip()
            if not s:
                return int(default_val)
            try:
                return int(s)
            except ValueError:
                print("  -> Please input a valid integer (or press Enter).")

    @staticmethod
    def _ask_bool_yn(label: str, default_val: bool) -> bool:
        rec = "y" if default_val else "n"
        while True:
            s = input(f"{label} (y/n) (recommended: {rec}) : ").strip().lower()
            if not s:
                return bool(default_val)
            if s in ("y", "yes", "1", "true", "t"):
                return True
            if s in ("n", "no", "0", "false", "f"):
                return False
            print("  -> Please type y or n (or press Enter).")

    @staticmethod
    def _ask_xyz(label: str, default_xyz: List[float], hint_for_single_value: str = "") -> List[float]:
        rec = f"{default_xyz[0]},{default_xyz[1]},{default_xyz[2]}"
        while True:
            s = input(f"{label} x,y,z (recommended: {rec}) : ").strip()
            if not s:
                return list(default_xyz)

            parts = [p.strip() for p in s.split(",") if p.strip() != ""]
            if len(parts) == 3:
                try:
                    return [float(parts[0]), float(parts[1]), float(parts[2])]
                except ValueError:
                    print("  -> Please input numbers like 0.18,0.00,0.12 (or press Enter).")
                    continue

            if len(parts) == 1:
                try:
                    float(parts[0])
                    if hint_for_single_value:
                        print(hint_for_single_value)
                    continue
                except ValueError:
                    pass

            print("  -> Expected 3 comma-separated values like 0.18,0.00,0.12 (or press Enter).")

    def _interactive_config(self) -> None:
        default_mode = str(self.get_parameter("traj_mode").value).lower()
        mode = self._ask_mode(default_mode)
        plane_rec = self.DEFAULT_PLANE_RECOMMENDED

        if mode == "circle":
            c = self._ask_xyz(
                "center",
                list(self.get_parameter("circle_center").value),
                hint_for_single_value="Single value detected. Radius is asked next.",
            )
            r = self._ask_float("radius (m)", float(self.get_parameter("circle_radius").value))
            hz = self._ask_float("hz", float(self.get_parameter("circle_hz").value))
            plane = self._ask_plane(plane_rec)

            self.set_parameters(
                [
                    rclpy.parameter.Parameter("traj_mode", rclpy.parameter.Parameter.Type.STRING, "circle"),
                    rclpy.parameter.Parameter("circle_center", rclpy.parameter.Parameter.Type.DOUBLE_ARRAY, c),
                    rclpy.parameter.Parameter("circle_radius", rclpy.parameter.Parameter.Type.DOUBLE, r),
                    rclpy.parameter.Parameter("circle_hz", rclpy.parameter.Parameter.Type.DOUBLE, hz),
                    rclpy.parameter.Parameter("circle_plane", rclpy.parameter.Parameter.Type.STRING, plane),
                ]
            )

        elif mode == "rectangle":
            c = self._ask_xyz(
                "center",
                list(self.get_parameter("rect_center").value),
                hint_for_single_value="Single value detected. Width/height are asked next.",
            )
            w = self._ask_float("width (m)", float(self.get_parameter("rect_width").value))
            h = self._ask_float("height (m)", float(self.get_parameter("rect_height").value))
            cr = self._ask_float("corner radius (m) (0=sharp)", float(self.get_parameter("rect_corner_radius").value))
            hz = self._ask_float("hz", float(self.get_parameter("rect_hz").value))
            plane = self._ask_plane(plane_rec)

            self.set_parameters(
                [
                    rclpy.parameter.Parameter("traj_mode", rclpy.parameter.Parameter.Type.STRING, "rectangle"),
                    rclpy.parameter.Parameter("rect_center", rclpy.parameter.Parameter.Type.DOUBLE_ARRAY, c),
                    rclpy.parameter.Parameter("rect_width", rclpy.parameter.Parameter.Type.DOUBLE, w),
                    rclpy.parameter.Parameter("rect_height", rclpy.parameter.Parameter.Type.DOUBLE, h),
                    rclpy.parameter.Parameter("rect_corner_radius", rclpy.parameter.Parameter.Type.DOUBLE, cr),
                    rclpy.parameter.Parameter("rect_hz", rclpy.parameter.Parameter.Type.DOUBLE, hz),
                    rclpy.parameter.Parameter("rect_plane", rclpy.parameter.Parameter.Type.STRING, plane),
                ]
            )

        else:
            c = self._ask_xyz(
                "center",
                list(self.get_parameter("heart_center").value),
                hint_for_single_value="Single value detected. Scale is asked next.",
            )
            s = self._ask_float("scale (m)", float(self.get_parameter("heart_scale").value))
            hz = self._ask_float("hz", float(self.get_parameter("heart_hz").value))
            plane = self._ask_plane(plane_rec)

            self.set_parameters(
                [
                    rclpy.parameter.Parameter("traj_mode", rclpy.parameter.Parameter.Type.STRING, "heart"),
                    rclpy.parameter.Parameter("heart_center", rclpy.parameter.Parameter.Type.DOUBLE_ARRAY, c),
                    rclpy.parameter.Parameter("heart_scale", rclpy.parameter.Parameter.Type.DOUBLE, s),
                    rclpy.parameter.Parameter("heart_hz", rclpy.parameter.Parameter.Type.DOUBLE, hz),
                    rclpy.parameter.Parameter("heart_plane", rclpy.parameter.Parameter.Type.STRING, plane),
                ]
            )

        pos_only = self._ask_bool_yn("IK position-only?", bool(self.get_parameter("ik_position_only").value))
        w_rot = self._ask_float("ik_w_rot", float(self.get_parameter("ik_w_rot").value))
        tol_pos = self._ask_float("ik_tol_pos_m", float(self.get_parameter("ik_tol_pos_m").value))
        iters = self._ask_int("ik_max_iters", int(self.get_parameter("ik_max_iters").value))
        damping = self._ask_float("ik_damping", float(self.get_parameter("ik_damping").value))
        step = self._ask_float("ik_step_scale", float(self.get_parameter("ik_step_scale").value))

        self.set_parameters(
            [
                rclpy.parameter.Parameter("ik_position_only", rclpy.parameter.Parameter.Type.BOOL, pos_only),
                rclpy.parameter.Parameter("ik_w_rot", rclpy.parameter.Parameter.Type.DOUBLE, w_rot),
                rclpy.parameter.Parameter("ik_tol_pos_m", rclpy.parameter.Parameter.Type.DOUBLE, tol_pos),
                rclpy.parameter.Parameter("ik_max_iters", rclpy.parameter.Parameter.Type.INTEGER, iters),
                rclpy.parameter.Parameter("ik_damping", rclpy.parameter.Parameter.Type.DOUBLE, damping),
                rclpy.parameter.Parameter("ik_step_scale", rclpy.parameter.Parameter.Type.DOUBLE, step),
            ]
        )

        self._t0 = self.get_clock().now()

    def _ik_pose(
        self, target_xyz: List[float], target_quat: List[float]
    ) -> Tuple[Optional[Dict[str, float]], Tuple[float, float]]:
        if self.urdf_chain is None:
            return None, (math.inf, math.inf)

        cfg = IKConfig(
            position_only=self.get_parameter("ik_position_only").value,
            max_iters=self.get_parameter("ik_max_iters").value,
            damping=self.get_parameter("ik_damping").value,
            step_scale=self.get_parameter("ik_step_scale").value,
            tol_pos_m=self.get_parameter("ik_tol_pos_m").value,
            tol_rot_rad=self.get_parameter("ik_tol_rot_rad").value,
            w_pos=self.get_parameter("ik_w_pos").value,
            w_rot=self.get_parameter("ik_w_rot").value,
        )

        use_js = self.get_parameter("ik_use_joint_states").value
        q_init = self._current_q() if use_js else {jn: 0.0 for jn in self.joint_names}

        return solve_ik_dls(
            chain=self.urdf_chain,
            joint_names=self.joint_names,
            q_init=q_init,
            target_xyz=target_xyz,
            target_quat=target_quat,
            cfg=cfg,
        )

    def _tick_cb(self) -> None:
        if self._sent_once and (not self.get_parameter("repeat").value):
            return

        if self.get_parameter("interactive").value and (not self._interactive_done):
            self._interactive_config()
            self._interactive_done = True

        if not self._build_chain_if_ready():
            return

        target_xyz = self._compute_target_xyz()

        target_quat = list(self.get_parameter("ee_quat").value)
        if len(target_quat) != 4:
            target_quat = [0.0, 0.0, 0.0, 1.0]

        self._publish_target(target_xyz)

        q_sol, (err_pos, err_rot) = self._ik_pose(target_xyz, target_quat)
        if q_sol is None:
            iters = self.get_parameter("ik_max_iters").value
            self.get_logger().error(
                f"IK failed. target_xyz={list(np.round(target_xyz, 4))} "
                f"err_pos={err_pos:.4f}m err_rot={err_rot:.4f}rad iters={iters}"
            )
            return

        msg = JointTrajectory()
        msg.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = [float(q_sol[jn]) for jn in self.joint_names]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(float(self.get_parameter("duration_sec").value) * 1e9)
        msg.points.append(pt)
        self.pub.publish(msg)

        self._tick += 1
        if self._tick % 5 == 0:
            self._publish_feedback(q_sol)

        self._sent_once = True


def main() -> None:
    rclpy.init()
    node = TrajPubWithIK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
