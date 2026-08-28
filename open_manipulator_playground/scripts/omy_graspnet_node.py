#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
# Author: SeongjinJeong
#
# ROS2 service node wrapping graspnet-baseline capture + inference.

from dataclasses import dataclass
import math
import os
import subprocess
import sys
import threading
import time
import traceback

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import open3d as o3d
from PIL import Image
import pyrealsense2 as rs
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robotis_interfaces.msg import MoveL
from scipy import ndimage
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
from std_srvs.srv import Trigger
import tf2_ros
import torch
from visualization_msgs.msg import Marker
import yaml

# Paths
GRASPNET_ROOT = '/opt/graspnet-baseline'
PLAYGROUND_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(GRASPNET_ROOT)
sys.path.append(os.path.join(GRASPNET_ROOT, 'models'))
sys.path.append(os.path.join(GRASPNET_ROOT, 'dataset'))
sys.path.append(os.path.join(GRASPNET_ROOT, 'utils'))

# Import demo.py (back up argv first, then inject the checkpoint path it expects)
_original_argv = sys.argv[:]
sys.argv = ['demo.py', '--checkpoint_path',
            os.path.join(GRASPNET_ROOT, 'logs/log_rs/checkpoint-rs.tar')]
import demo  # noqa: E402, I100

WIDTH, HEIGHT = 1280, 720


@dataclass
class CameraConfig:
    depth_min: float = 0.14
    depth_max: float = 0.8
    edge_margin_px: int = 40
    depth_margin_m: float = 0.02
    frame_id: str = 'camera_link'
    point_cloud_publish_max_points: int = 50000
    capture_settle_sec: float = 0.3


@dataclass
class GraspFilterConfig:
    table_plane_fit_thresh_m: float = 0.003
    min_grasp_height_above_table_m: float = 0.01
    outlier_removal_neighbors: int = 20
    outlier_removal_std_ratio: float = 2.0
    min_grasp_z_m: float = -0.05
    max_grasp_z_m: float = 1.0
    max_grasp_width_m: float = None
    min_base_distance_m: float = 0.20
    max_base_distance_m: float = 0.58
    min_grasp_x_m: float = 0.0
    max_approach_tilt_deg: float = 45.0
    tilt_score_penalty: float = 0.3
    width_score_penalty: float = 0.3


@dataclass
class GripperConfig:
    open_width_m: float = 0.10
    open_joint: float = 0.0
    closed_joint: float = 1.12
    close_bias: float = 0.6
    wait_sec: float = 1.5
    resend_period_sec: float = 0.2
    yaw_offset_rad: float = math.pi / 2


@dataclass
class PickOffsetConfig:
    pregrasp_offset_m: float = 0.08
    grasp_depth_offset_m: float = 0.06
    min_grasp_execution_z_m: float = 0.007
    post_grasp_lift_m: float = 0.10


def open_camera():
    """Open the RealSense pipeline and the depth filter chain."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 5)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 5)
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    # Depth post-processing filter chain
    depth_to_disparity = rs.disparity_transform(True)
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    disparity_to_depth = rs.disparity_transform(False)
    hole_filling = rs.hole_filling_filter()
    pc = rs.pointcloud()

    def filter_depth(depth_frame):
        f = depth_to_disparity.process(depth_frame)
        f = spatial.process(f)
        f = temporal.process(f)
        f = disparity_to_depth.process(f)
        return hole_filling.process(f)

    return pipeline, align, filter_depth, pc


def grab_frame(pipeline, align, filter_depth, pc):
    """Capture a single frame. (color RGB uint8, depth uint16 mm, vertices (H,W,3) float32 m)."""
    frames = align.process(pipeline.wait_for_frames())
    depth_frame = filter_depth(frames.get_depth_frame())
    color_frame = frames.get_color_frame()

    points = pc.calculate(depth_frame)
    vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(HEIGHT, WIDTH, 3)
    color_image = np.asanyarray(color_frame.get_data())[:, :, ::-1]  # BGR -> RGB
    depth_image = np.asanyarray(depth_frame.get_data())
    return color_image, depth_image, vertices


def save_capture(out_dir, color_image, depth_image, vertices,
                 depth_min=0.14, depth_max=0.8, edge_margin_px=40, depth_margin_m=0.02):
    os.makedirs(out_dir, exist_ok=True)
    Image.fromarray(color_image).save(os.path.join(out_dir, 'color.png'))
    Image.fromarray(depth_image).save(os.path.join(out_dir, 'depth.png'))
    np.save(os.path.join(out_dir, 'points.npy'), vertices)

    # Workspace mask (depth range + edge erosion)
    z = vertices[:, :, 2]
    mask = (z > depth_min + depth_margin_m) & (z < depth_max - depth_margin_m)
    if edge_margin_px > 0:
        mask = ndimage.binary_erosion(mask, iterations=edge_margin_px)
    Image.fromarray(mask).save(os.path.join(out_dir, 'workspace_mask.png'))

    print(f'-> saved capture to {out_dir}')
    return out_dir


class OmyGraspnetNode(Node):

    def __init__(self):
        super().__init__('omy_graspnet')

        # ROS parameters
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('auto', False)
        self.declare_parameter('auto_pick_delay_sec', 0.0)
        self.declare_parameter('movel_duration_sec', 3.0)
        self.declare_parameter('gripper_close_bias', 0.6)
        self.declare_parameter('top_k', 1)
        self.declare_parameter('place_enabled', True)

        self.execute_motion = self.get_parameter('execute_motion').value
        self.auto = self.get_parameter('auto').value
        self.auto_pick_delay_sec = self.get_parameter('auto_pick_delay_sec').value
        self.movel_duration_sec = self.get_parameter('movel_duration_sec').value
        self.descent_duration_sec = max(0.5, self.movel_duration_sec - 2.0)
        self.return_home_duration_sec = max(0.5, self.movel_duration_sec - 1.0)
        self.top_k = self.get_parameter('top_k').value
        self.place_enabled = self.get_parameter('place_enabled').value

        # Grouped config values
        self.camera = CameraConfig()
        self.filters = GraspFilterConfig()
        self.gripper = GripperConfig(close_bias=self.get_parameter('gripper_close_bias').value)
        self.offsets = PickOffsetConfig()

        self.out_dir = os.path.join(GRASPNET_ROOT, 'doc/realsense_capture')
        self.auto_retry_delay_sec = 0
        self.base_frame = 'link0'

        # Load fixed poses
        with open(os.path.join(PLAYGROUND_DIR, 'config', 'omy_graspnet_poses.yaml')) as f:
            poses_config = yaml.safe_load(f)
        self.initial_translation = np.array(poses_config['initial']['position'])
        self.initial_quat = np.array(poses_config['initial']['orientation'])

        # Load the graspnet-baseline model
        demo.cfgs.checkpoint_path = os.path.join(GRASPNET_ROOT, 'logs/log_rs/checkpoint-rs.tar')
        demo.cfgs.num_point = 20000
        demo.cfgs.num_view = 300
        demo.cfgs.collision_thresh = 0.01
        demo.cfgs.voxel_size = 0.01

        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.net = demo.get_net()

        # Open3D visualization thread
        self._geom_lock = threading.Lock()
        self._pending_geoms = None
        self._stop_render = False
        self._view_initialized = False
        self.vis = None
        window_ready = threading.Event()
        self._render_thread = threading.Thread(
            target=self._render_loop, args=(window_ready,), daemon=True)
        self._render_thread.start()
        window_ready.wait(timeout=10.0)

        # Publishers / TF / action clients
        self.pose_pub = self.create_publisher(PoseStamped, 'omy_graspnet/pose', 10)
        self.info_pub = self.create_publisher(Float32MultiArray, 'omy_graspnet/grasp_info', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.movel_pub = self.create_publisher(MoveL, '/omy_movel_controller/movel', 10)
        self._gripper_callback_group = ReentrantCallbackGroup()
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd',
            callback_group=self._gripper_callback_group)
        self.marker_pub = self.create_publisher(Marker, 'omy_graspnet/gripper_marker', 10)
        cloud_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, 'omy_graspnet/point_cloud', cloud_qos)

        # Camera pipeline
        (self._camera_pipeline, self._camera_align, self._camera_filter_depth,
         self._camera_pc) = open_camera()
        self._camera_lock = threading.Lock()

        # Execute/pick state
        self._last_grasp = None
        self._last_translation_base = None
        self._last_quat_base = None
        self._last_detection_duration = None

        # Services
        self._cancel_event = threading.Event()
        self.srv = self.create_service(Trigger, 'omy_graspnet/execute', self.on_execute)
        self.pick_srv = self.create_service(Trigger, 'omy_graspnet/pick', self.on_pick)
        self._cancel_callback_group = ReentrantCallbackGroup()
        self.cancel_srv = self.create_service(
            Trigger, 'omy_graspnet/cancel', self.on_cancel,
            callback_group=self._cancel_callback_group)
        self.get_logger().info(
            'ready -- call: ros2 service call /omy_graspnet/execute std_srvs/srv/Trigger {}, '
            'then ros2 service call /omy_graspnet/pick std_srvs/srv/Trigger {} to act on it '
            '(or set -p auto:=true to run continuously; omy_graspnet/cancel stops it)')

    def _render_loop(self, window_ready):
        window_name = 'omy_graspnet'
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name=window_name, width=600, height=540)
        window_ready.set()
        while not self._stop_render:
            with self._geom_lock:
                if self._pending_geoms is not None:
                    self.vis.clear_geometries()
                    for geom in self._pending_geoms:
                        self.vis.add_geometry(geom, reset_bounding_box=not self._view_initialized)
                    if not self._view_initialized:
                        # Only set the camera viewpoint to the point cloud's center once
                        center = self._pending_geoms[0].get_center()
                        ctr = self.vis.get_view_control()
                        ctr.set_front([0, 0, -1])
                        ctr.set_up([0, -1, 0])
                        ctr.set_lookat(center)
                        ctr.set_zoom(0.6)
                        self._view_initialized = True
                    self._pending_geoms = None
                    subprocess.run(['wmctrl', '-a', window_name],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.vis.poll_events()
            self.vis.update_renderer()
            time.sleep(0.03)
        self.vis.destroy_window()

    def _get_and_process_data(self, data_dir):
        # Load the captured files
        color = np.array(Image.open(os.path.join(data_dir, 'color.png')), dtype=np.float32) / 255.0
        vertices = np.load(os.path.join(data_dir, 'points.npy'))
        workspace_mask = np.array(Image.open(os.path.join(data_dir, 'workspace_mask.png')))

        mask = workspace_mask & (vertices[:, :, 2] > 0)
        cloud_masked = vertices[mask]
        color_masked = color[mask]

        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
        cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

        # Outlier removal
        if (self.filters.outlier_removal_neighbors > 0
                and len(cloud_masked) > self.filters.outlier_removal_neighbors):
            cloud_o3d, inlier_idx = cloud_o3d.remove_statistical_outlier(
                nb_neighbors=self.filters.outlier_removal_neighbors,
                std_ratio=self.filters.outlier_removal_std_ratio)
            cloud_masked = cloud_masked[inlier_idx]
            color_masked = color_masked[inlier_idx]

        # Table plane estimation
        table_plane = None
        if len(cloud_masked) > 1000:
            try:
                table_plane, _ = cloud_o3d.segment_plane(
                    distance_threshold=self.filters.table_plane_fit_thresh_m,
                    ransac_n=3, num_iterations=1000)
            except Exception as exc:
                self.get_logger().warn(f'table plane fit failed: {exc}')

        points_for_sampling = cloud_masked
        colors_for_sampling = color_masked

        # Sample down (or up) to the network's expected input point count
        num_point = demo.cfgs.num_point
        self.get_logger().info(
            f'{len(points_for_sampling)} points for grasp sampling (target {num_point})')
        if len(points_for_sampling) >= num_point:
            idxs = np.random.choice(len(points_for_sampling), num_point, replace=False)
            points_final = points_for_sampling[idxs]
            colors_for_sampling = colors_for_sampling[idxs]
        else:
            idxs1 = np.arange(len(points_for_sampling))
            idxs2 = np.random.choice(
                len(points_for_sampling), num_point - len(points_for_sampling), replace=True)
            points_final = points_for_sampling[np.concatenate([idxs1, idxs2])].copy()
            colors_for_sampling = colors_for_sampling[np.concatenate([idxs1, idxs2])]
            points_final[len(idxs1):] += np.random.normal(
                0.0, 0.001, size=(len(idxs2), 3)).astype(np.float32)
        cloud_sampled = torch.from_numpy(
            points_final[np.newaxis].astype(np.float32)).to(self.device)

        end_points = {'point_clouds': cloud_sampled, 'cloud_colors': colors_for_sampling}
        return end_points, cloud_o3d, table_plane

    def _publish_point_cloud(self, points, colors, frame_id):
        # Pack RGB into a single float32
        points = np.asarray(points, dtype=np.float32)
        colors = (np.asarray(colors) * 255.0).astype(np.uint32)
        rgb = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
        rgb_float = rgb.view(np.float32)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_points = np.column_stack([points, rgb_float])
        msg = point_cloud2.create_cloud(header, fields, cloud_points)
        self.cloud_pub.publish(msg)

    def _publish_grasp(self, grasp):
        quat = Rotation.from_matrix(grasp.rotation_matrix).as_quat()  # (x, y, z, w)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.camera.frame_id
        (pose_msg.pose.position.x, pose_msg.pose.position.y,
         pose_msg.pose.position.z) = grasp.translation.tolist()
        (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
         pose_msg.pose.orientation.z, pose_msg.pose.orientation.w) = quat.tolist()
        self.pose_pub.publish(pose_msg)

        info_msg = Float32MultiArray()
        info_msg.data = [float(grasp.width), float(grasp.depth), float(grasp.score)]
        self.info_pub.publish(info_msg)

        self.get_logger().info(
            f'published: width={grasp.width:.4f} depth={grasp.depth:.4f} score={grasp.score:.4f}')

    def _publish_gripper_marker(self, grasp, translation_base, quat_base):
        # Move the mesh vertices back to local coordinates (marker.pose sets the position)
        mesh = grasp.to_open3d_geometry()
        verts_cam = np.asarray(mesh.vertices)
        verts_local = (grasp.rotation_matrix.T @ (verts_cam - grasp.translation).T).T
        triangles = np.asarray(mesh.triangles)

        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'omy_graspnet'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        (marker.pose.position.x, marker.pose.position.y,
         marker.pose.position.z) = translation_base.tolist()
        (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z, marker.pose.orientation.w) = quat_base.tolist()
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.9
        marker.lifetime.sec = 0

        for tri in triangles:
            for idx in tri:
                x, y, z = verts_local[idx]
                marker.points.append(Point(x=float(x), y=float(y), z=float(z)))

        self.marker_pub.publish(marker)

    def _clear_gripper_marker(self):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'omy_graspnet'
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    # optical(X-right/Y-down/Z-forward) -> body(X-forward/Y-left/Z-up)
    _R_LINK_FROM_OPTICAL = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])

    def _to_base_frame(self, translation, quat, source_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'TF lookup {source_frame}->{self.base_frame} failed: {exc}')
            return None, None

        t = tf.transform.translation
        q = tf.transform.rotation
        rot_src_to_base = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        t_src_to_base = np.array([t.x, t.y, t.z])

        translation_link = self._R_LINK_FROM_OPTICAL @ translation
        rot_grasp_link = self._R_LINK_FROM_OPTICAL @ Rotation.from_quat(quat).as_matrix()

        translation_base = rot_src_to_base @ translation_link + t_src_to_base
        rot_grasp_base = rot_src_to_base @ rot_grasp_link
        quat_base = Rotation.from_matrix(rot_grasp_base).as_quat()
        return translation_base, quat_base

    def _points_to_base_frame(self, points, source_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'TF lookup {source_frame}->{self.base_frame} failed: {exc}')
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        rot_src_to_base = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        t_src_to_base = np.array([t.x, t.y, t.z])

        points_link = points @ self._R_LINK_FROM_OPTICAL.T
        return points_link @ rot_src_to_base.T + t_src_to_base

    def _send_movel(self, translation, quat, duration_sec=None, wait=True):
        duration = self.movel_duration_sec if duration_sec is None else duration_sec
        msg = MoveL()
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.header.frame_id = self.base_frame
        (msg.pose.pose.position.x, msg.pose.pose.position.y,
         msg.pose.pose.position.z) = translation.tolist()
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = quat.tolist()
        msg.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        self.movel_pub.publish(msg)
        self.get_logger().info(f'MoveL -> {translation}')
        if wait:
            time.sleep(duration)
        return duration

    def _send_movel_and_gripper(self, translation, quat, joint_position, duration_sec=None):
        # Move the arm and drive the gripper at the same time
        duration = self._send_movel(translation, quat, duration_sec=duration_sec, wait=False)
        started = time.monotonic()
        self._send_gripper(joint_position)
        remaining = duration - (time.monotonic() - started)
        if remaining > 0:
            time.sleep(remaining)

    def _width_to_gripper_joint(self, width_m, close_bias=None):
        if close_bias is None:
            close_bias = self.gripper.close_bias
        ratio = np.clip(width_m / self.gripper.open_width_m, 0.0, 1.0)
        joint = self.gripper.closed_joint + ratio * (
            self.gripper.open_joint - self.gripper.closed_joint)
        joint += math.copysign(close_bias, self.gripper.closed_joint - self.gripper.open_joint)
        closed_bound = max(self.gripper.open_joint, self.gripper.closed_joint)
        open_bound = min(self.gripper.open_joint, self.gripper.closed_joint)
        return float(np.clip(joint, open_bound, closed_bound))

    def _send_gripper(self, joint_position):
        # Same GripperCommand action used elsewhere; a single goal often doesn't
        # take, so keep resending the same target until wait_sec elapses
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('gripper action server not available, skipping')
            return
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(joint_position)
        self.get_logger().info(f'gripper -> {joint_position:.3f}')

        deadline = time.monotonic() + self.gripper.wait_sec
        while True:
            self.gripper_client.send_goal_async(goal_msg)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(self.gripper.resend_period_sec, remaining))

    # end_effector_link -> actual gripper axis correction (fixed URDF offset)
    _R_EEF_FROM_GRIPPER = np.array([
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ])

    def _eef_rotation_for(self, rot_base):
        rot_eef = rot_base @ self._R_EEF_FROM_GRIPPER.T

        if self.gripper.yaw_offset_rad != 0.0:
            eef_yaw_correction = Rotation.from_rotvec(
                np.array([0.0, -1.0, 0.0]) * self.gripper.yaw_offset_rad).as_matrix()
            rot_eef = rot_eef @ eef_yaw_correction
        return rot_eef

    def _canonicalize_grasp_rotation(self, rot_base):
        # The parallel gripper is symmetric under a 180-degree flip about its
        # approach axis -- pick whichever equivalent needs less rotation from
        # the current pose
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, 'end_effector_link', rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'grasp canonicalization: TF lookup failed, skipping: {exc}')
            return rot_base

        q = tf.transform.rotation
        rot_current_eef = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        flip_180_x = Rotation.from_rotvec(np.array([1.0, 0.0, 0.0]) * np.pi).as_matrix()
        rot_base_flipped = rot_base @ flip_180_x

        def angle_from_current(rb):
            rot_eef = self._eef_rotation_for(rb)
            delta = Rotation.from_matrix(rot_current_eef.T @ rot_eef)
            return delta.magnitude()

        angle_normal = angle_from_current(rot_base)
        angle_flipped = angle_from_current(rot_base_flipped)

        if angle_flipped < angle_normal:
            self.get_logger().info(
                'grasp canonicalization: using 180-flipped equivalent '
                f'({np.degrees(angle_normal):.1f} -> {np.degrees(angle_flipped):.1f} '
                'deg of rotation from current pose)')
            return rot_base_flipped
        return rot_base

    def _execute_pick(self, grasp, translation_base, quat_base):
        self._pick_object(grasp, translation_base, quat_base)
        if self.place_enabled:
            self._place_object()

    def _pick_object(self, grasp, translation_base, quat_base):
        rot_base = Rotation.from_quat(quat_base).as_matrix()
        rot_base = self._canonicalize_grasp_rotation(rot_base)

        approach_axis = rot_base[:, 0]
        pregrasp_translation = translation_base - approach_axis * self.offsets.pregrasp_offset_m
        grasp_translation = translation_base + approach_axis * self.offsets.grasp_depth_offset_m
        if grasp_translation[2] < self.offsets.min_grasp_execution_z_m:
            grasp_translation[2] = self.offsets.min_grasp_execution_z_m
        lift_translation = grasp_translation.copy()
        lift_translation[2] += self.offsets.post_grasp_lift_m

        rot_eef_command = self._eef_rotation_for(rot_base)
        quat_eef_command = Rotation.from_matrix(rot_eef_command).as_quat()

        self._send_gripper(self.gripper.open_joint)
        # Close the gripper slightly while approaching the pregrasp pose
        pregrasp_gripper_joint = min(
            self._width_to_gripper_joint(grasp.width, close_bias=0.2), 0.6)
        self._send_movel_and_gripper(
            pregrasp_translation, quat_eef_command, pregrasp_gripper_joint)
        self._send_movel(
            grasp_translation, quat_eef_command, duration_sec=self.descent_duration_sec)
        self._send_gripper(self._width_to_gripper_joint(grasp.width))
        self._send_movel(
            lift_translation, quat_eef_command, duration_sec=self.descent_duration_sec)

    def _place_object(self):
        # Overridden by subclasses (e.g. omy_ai_graspnet_node.py, omy_target_graspnet_node.py)
        raise NotImplementedError

    def on_cancel(self, request, response):
        self._cancel_event.set()
        response.success = True
        response.message = 'cancel requested'
        return response

    def _cancelable_sleep(self, seconds):
        # Returns False if cancelled during the sleep
        deadline = time.monotonic() + seconds
        while True:
            if self._cancel_event.is_set():
                return False
            if time.monotonic() >= deadline:
                return True
            time.sleep(min(0.05, deadline - time.monotonic()))

    def _detect_and_select(self):
        """Capture, run inference, pick the best candidate (None x3 + reason on failure)."""
        time.sleep(self.camera.capture_settle_sec)
        start_time = time.monotonic()

        # Capture
        with self._camera_lock:
            color_image, depth_image, vertices = grab_frame(
                self._camera_pipeline, self._camera_align,
                self._camera_filter_depth, self._camera_pc)
        self._last_color_image = color_image
        self._last_vertices = vertices
        data_dir = save_capture(
            self.out_dir, color_image, depth_image, vertices,
            depth_min=self.camera.depth_min, depth_max=self.camera.depth_max,
            edge_margin_px=self.camera.edge_margin_px, depth_margin_m=self.camera.depth_margin_m)
        end_points, cloud, table_plane = self._get_and_process_data(data_dir)
        with self._geom_lock:
            self._pending_geoms = [cloud]
        points_base = self._points_to_base_frame(np.asarray(cloud.points), self.camera.frame_id)
        if points_base is not None:
            colors_base = np.asarray(cloud.colors)
            if len(points_base) > self.camera.point_cloud_publish_max_points:
                idxs = np.random.choice(
                    len(points_base), self.camera.point_cloud_publish_max_points, replace=False)
                points_base = points_base[idxs]
                colors_base = colors_base[idxs]
            self._publish_point_cloud(points_base, colors_base, self.base_frame)

        # Grasp inference
        gg = demo.get_grasps(self.net, end_points)
        if demo.cfgs.collision_thresh > 0:
            gg = demo.collision_detection(gg, np.array(cloud.points))
        if len(gg) == 0:
            self._clear_gripper_marker()
            self._last_detection_duration = time.monotonic() - start_time
            self.get_logger().info(
                f'detection took {self._last_detection_duration:.2f}s (no candidates)')
            return None, None, None, 'no valid grasp candidate found'

        gg.nms()
        gg.sort_by_score()

        # Candidate filtering
        candidates = []
        tf_fail_count = 0
        table_reject_count = 0
        height_reject_count = 0
        base_distance_reject_count = 0
        behind_reject_count = 0
        tilt_reject_count = 0
        width_reject_count = 0
        for candidate in gg[:100]:
            if (self.filters.max_grasp_width_m is not None
                    and candidate.width > self.filters.max_grasp_width_m):
                width_reject_count += 1
                continue
            if table_plane is not None:
                a, b, c, d = table_plane
                x, y, z = candidate.translation
                dist_to_table = abs(a * x + b * y + c * z + d)
                if dist_to_table < self.filters.min_grasp_height_above_table_m:
                    table_reject_count += 1
                    continue
            t, q = self._to_base_frame(
                candidate.translation, Rotation.from_matrix(candidate.rotation_matrix).as_quat(),
                self.camera.frame_id)
            if t is None:
                tf_fail_count += 1
                continue
            if not (self.filters.min_grasp_z_m <= t[2] <= self.filters.max_grasp_z_m):
                height_reject_count += 1
                self.get_logger().warn(
                    f'rejecting grasp candidate (score={candidate.score:.3f}): '
                    f'z={t[2]:.3f} outside '
                    f'[{self.filters.min_grasp_z_m}, {self.filters.max_grasp_z_m}]')
                continue
            base_dist = np.linalg.norm(t)
            if (base_dist < self.filters.min_base_distance_m
                    or base_dist > self.filters.max_base_distance_m):
                base_distance_reject_count += 1
                self.get_logger().warn(
                    f'rejecting grasp candidate (score={candidate.score:.3f}): '
                    f'{base_dist:.3f}m from base outside '
                    f'[{self.filters.min_base_distance_m}, {self.filters.max_base_distance_m}]')
                continue
            if t[0] < self.filters.min_grasp_x_m:
                behind_reject_count += 1
                self.get_logger().warn(
                    f'rejecting grasp candidate (score={candidate.score:.3f}): '
                    f'x={t[0]:.3f} behind robot (< {self.filters.min_grasp_x_m})')
                continue
            approach_axis = Rotation.from_quat(q).as_matrix()[:, 0]
            tilt_deg = np.degrees(np.arccos(np.clip(-approach_axis[2], -1.0, 1.0)))
            if tilt_deg > self.filters.max_approach_tilt_deg:
                tilt_reject_count += 1
                self.get_logger().warn(
                    f'rejecting grasp candidate (score={candidate.score:.3f}): '
                    f'tilt={tilt_deg:.1f}deg > {self.filters.max_approach_tilt_deg}deg '
                    '(too lying-down)')
                continue
            width_ratio = np.clip(candidate.width / self.gripper.open_width_m, 0.0, 1.0)
            rank_score = (candidate.score
                          - self.filters.tilt_score_penalty * (tilt_deg / 90.0)
                          - self.filters.width_score_penalty * width_ratio)
            candidates.append((rank_score, candidate, t, q, tilt_deg))

        if not candidates:
            self.get_logger().warn(
                f'0 candidates survived out of {len(gg[:100])} checked: '
                f'{width_reject_count} rejected by width, '
                f'{table_reject_count} rejected as on-table, '
                f'{tf_fail_count} TF lookup failed, {height_reject_count} rejected by height, '
                f'{base_distance_reject_count} rejected by base distance, '
                f'{behind_reject_count} rejected as behind robot, '
                f'{tilt_reject_count} rejected by tilt')
            self._clear_gripper_marker()
            self._last_detection_duration = time.monotonic() - start_time
            self.get_logger().info(
                f'detection took {self._last_detection_duration:.2f}s (no candidates)')
            return None, None, None, 'no grasp candidate passed the height/tilt sanity check'

        # Final candidate selection
        candidates.sort(key=lambda c: c[0], reverse=True)
        _, selected, translation_base, quat_base, selected_tilt_deg = candidates[0]

        with self._geom_lock:
            self._pending_geoms = [cloud] + [
                c[1].to_open3d_geometry() for c in candidates[:self.top_k]]

        self._publish_grasp(selected)
        self.get_logger().info(
            f'selected grasp: score={selected.score:.3f} tilt={selected_tilt_deg:.1f}deg '
            f'(of {len(candidates)} candidates that passed sanity checks)')
        self.get_logger().info(
            f'grasp in {self.base_frame} frame (raw, no yaw offset): '
            f'translation={translation_base} quat_xyzw={quat_base}')
        self._publish_gripper_marker(selected, translation_base, quat_base)
        self._last_detection_duration = time.monotonic() - start_time
        self.get_logger().info(f'detection took {self._last_detection_duration:.2f}s')

        return selected, translation_base, quat_base, f'best sane score={selected.score:.4f}'

    def on_execute(self, request, response):
        try:
            while True:
                self._last_grasp = None
                self._last_translation_base = None
                self._last_quat_base = None

                selected, translation_base, quat_base, message = self._detect_and_select()

                if selected is None:
                    if self.auto:
                        self.get_logger().warn(
                            f'{message} -- retrying in {self.auto_retry_delay_sec}s '
                            '(omy_graspnet/cancel to stop)')
                        if self._cancelable_sleep(self.auto_retry_delay_sec):
                            continue
                        self._cancel_event.clear()
                        self.get_logger().info('auto retry cancelled')
                    response.success = False
                    response.message = message
                    return response

                self._last_grasp = selected
                self._last_translation_base = translation_base
                self._last_quat_base = quat_base
                response.success = True
                response.message = message

                if not (self.auto and self.execute_motion):
                    return response

                self.get_logger().info(
                    f'auto: waiting {self.auto_pick_delay_sec}s before pick '
                    '(call omy_graspnet/cancel to stop)')
                if not self._cancelable_sleep(self.auto_pick_delay_sec):
                    self._cancel_event.clear()
                    self.get_logger().info('auto pick cancelled before starting')
                    return response

                self._execute_pick(selected, translation_base, quat_base)
                if not self.auto or self._cancel_event.is_set():
                    self._cancel_event.clear()
                    self.get_logger().info('auto loop stopped (cancelled)')
                    return response
        except Exception:
            self.get_logger().error('execute failed:\n' + traceback.format_exc())
            response.success = False
            response.message = 'exception during execute, see node log'
        return response

    def on_pick(self, request, response):
        if not self.execute_motion:
            response.success = False
            response.message = (
                'execute_motion is False -- restart the node with '
                '--ros-args -p execute_motion:=true to allow motion')
            return response
        if self._last_grasp is None:
            response.success = False
            response.message = 'no grasp available yet -- call omy_graspnet/execute first'
            return response

        try:
            self._execute_pick(self._last_grasp, self._last_translation_base, self._last_quat_base)
            response.success = True
            response.message = 'pick sequence complete'
        except Exception:
            self.get_logger().error('pick failed:\n' + traceback.format_exc())
            response.success = False
            response.message = 'exception during pick, see node log'
        return response

    def move_to_initial_pose(self):
        if not self.execute_motion:
            return
        deadline = time.monotonic() + 5.0
        while self.movel_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.1)
        if self.movel_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                'no subscriber on omy_movel_controller/movel after 5s, '
                'skipping move to initial pose')
            return
        self.get_logger().info('moving to initial pose on startup')
        self._send_movel(self.initial_translation, self.initial_quat)

    def destroy_node(self):
        self._stop_render = True
        self._render_thread.join(timeout=2.0)
        self._camera_pipeline.stop()
        super().destroy_node()


def main():
    rclpy.init(args=_original_argv)
    node = OmyGraspnetNode()
    node.move_to_initial_pose()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
