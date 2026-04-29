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
# Author: Daeyeol Kang

import os

import cv2
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node


class ShapeDetectorNode(Node):

    def __init__(self):
        super().__init__('shape_detector_node')

        self.declare_parameter('image_path', '')
        self.declare_parameter('trajectory_topic', '/drawing_trajectory')
        self.declare_parameter('workspace_x_min', 0.10)
        self.declare_parameter('workspace_x_max', 0.26)
        self.declare_parameter('workspace_y_min', -0.13)
        self.declare_parameter('workspace_y_max', 0.13)
        self.declare_parameter('workspace_z', 0.003)
        self.declare_parameter('smoothing_sigma', 1.0)
        self.declare_parameter('resample_num_pts', 100)

        self.image_path = self.get_parameter('image_path').value
        self.traj_topic = self.get_parameter('trajectory_topic').value
        self.x_min = self.get_parameter('workspace_x_min').value
        self.x_max = self.get_parameter('workspace_x_max').value
        self.y_min = self.get_parameter('workspace_y_min').value
        self.y_max = self.get_parameter('workspace_y_max').value
        self.z_draw = self.get_parameter('workspace_z').value
        self.sigma_base = self.get_parameter('smoothing_sigma').value
        self.resample_pts = self.get_parameter('resample_num_pts').value
        self.enable_debug = False

        # Headless protection
        if self.enable_debug and 'DISPLAY' not in os.environ:
            self.get_logger().warn(
                'enable_debug_view is True but no DISPLAY found. Disabling GUI.')
            self.enable_debug = False

        self.publisher = self.create_publisher(PoseArray, self.traj_topic, 10)

        self.get_logger().info('ShapeDetectorNode started.')

        self.points = []
        self.vis_img = None
        self.process_once()

        self.ui_timer = None
        if self.enable_debug:
            self.ui_timer = self.create_timer(0.033, self.ui_timer_callback)
        self.timer = self.create_timer(10.0, self.timer_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'image_path':
                self.image_path = param.value
                self.process_once(reset=True)
            elif param.name == 'workspace_z':
                self.z_draw = param.value
                self.process_once(reset=True)
            elif param.name == 'smoothing_sigma':
                self.sigma_base = param.value
                self.process_once(reset=True)
            elif param.name == 'resample_num_pts':
                self.resample_pts = param.value
                self.process_once(reset=True)
        return SetParametersResult(successful=True)

    def smooth_trajectory(self, points, sigma=1.0, is_closed=False):
        """[V3] Adaptive Smoothing Engine."""
        if len(points) < 3:
            return points
        pts_float = points.astype(np.float64)

        peri = cv2.arcLength(pts_float.astype(np.float32), is_closed)
        epsilon = 0.015 * peri
        approx = cv2.approxPolyDP(pts_float.astype(np.float32), epsilon, is_closed)
        vertices = approx.reshape(-1, 2)

        vertex_indices = []
        for v in vertices:
            dists = np.sum((pts_float - v)**2, axis=1)
            vertex_indices.append(int(np.argmin(dists)))
        vertex_indices = sorted(set(vertex_indices))

        def chaikin(pts):
            res = []
            for i in range(len(pts) - 1):
                p1, p2 = pts[i], pts[i + 1]
                res.append(0.75 * p1 + 0.25 * p2)
                res.append(0.25 * p1 + 0.75 * p2)
            return np.array(res) if res else pts

        def resample_path(seg, num_pts):
            """Resample segment to a fixed number of points."""
            if len(seg) < 2 or num_pts < 2:
                return seg
            diffs = np.diff(seg, axis=0)
            dists = np.sqrt(diffs[:, 0]**2 + diffs[:, 1]**2)
            cum = np.concatenate(([0], np.cumsum(dists)))
            total = cum[-1]
            if total < 1e-6:
                return seg
            targets = np.linspace(0, total, num_pts)
            rx = np.interp(targets, cum, seg[:, 0])
            ry = np.interp(targets, cum, seg[:, 1])
            return np.column_stack((rx, ry))

        if len(vertex_indices) < 2:
            smoothed = pts_float.copy()
            for _ in range(2):
                smoothed = chaikin(smoothed)
            kernel = np.array([0.1, 0.2, 0.4, 0.2, 0.1])
            pad_mode = 'wrap' if is_closed else 'edge'
            px = np.convolve(np.pad(smoothed[:, 0], (2, 2), mode=pad_mode), kernel, mode='valid')
            py = np.convolve(np.pad(smoothed[:, 1], (2, 2), mode=pad_mode), kernel, mode='valid')
            result = resample_path(np.column_stack((px, py)), self.resample_pts)
            if is_closed:
                result[-1] = result[0]
                return np.vstack([result, result[1:3]])
            return result

        # Build segments between consecutive vertices
        segments = []
        n_vi = len(vertex_indices)
        for si in range(n_vi - (0 if is_closed else 1)):
            i0 = vertex_indices[si]
            i1 = vertex_indices[(si + 1) % n_vi]
            if i1 > i0:
                seg = pts_float[i0:i1 + 1]
            else:
                seg = np.vstack([pts_float[i0:], pts_float[:i1 + 1]])
            segments.append(seg)

        # Classify and smooth each segment
        result_parts = []
        for seg in segments:
            if len(seg) < 2:
                result_parts.append(seg)
                continue
            chord = np.linalg.norm(seg[-1] - seg[0])
            arc_dists = np.sqrt(np.sum(np.diff(seg, axis=0)**2, axis=1))
            arc = np.sum(arc_dists)
            linearity = chord / arc if arc > 0 else 1.0

            if linearity > 0.98:
                result_parts.append(resample_path(np.array([seg[0], seg[-1]]), 2))
            else:
                smoothed = seg.copy()
                for _ in range(2):
                    smoothed = chaikin(smoothed)
                kernel = np.array([0.1, 0.2, 0.4, 0.2, 0.1])
                if len(smoothed) > 4:
                    px = np.convolve(
                        np.pad(smoothed[:, 0], (2, 2), mode='edge'), kernel, mode='valid')
                    py = np.convolve(
                        np.pad(smoothed[:, 1], (2, 2), mode='edge'), kernel, mode='valid')
                    smoothed = np.column_stack((px, py))
                result_parts.append(resample_path(smoothed, self.resample_pts))

        if not result_parts:
            return pts_float
        result = [result_parts[0]]
        for part in result_parts[1:]:
            if len(part) > 0 and len(result[-1]) > 0:
                if np.linalg.norm(result[-1][-1] - part[0]) < 2.0:
                    result.append(part[1:] if len(part) > 1 else part)
                else:
                    result.append(part)
        final = np.vstack([r for r in result if len(r) > 0])

        if is_closed and len(final) > 2:
            final[-1] = final[0]
            return np.vstack([final, final[1:3]])
        return final

    def skeletonize(self, img):
        # Skeletonization
        skel = np.zeros(img.shape, np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        temp_img = img.copy()
        while True:
            eroded = cv2.erode(temp_img, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(temp_img, temp)
            skel = cv2.bitwise_or(skel, temp)
            temp_img = eroded.copy()
            if cv2.countNonZero(temp_img) == 0:
                break
        return skel

    def extract_single_line(self, contour_points):
        unique_pts, seen = [], set()
        for p in contour_points.reshape(-1, 2):
            pt_tup = tuple(p)
            if pt_tup not in seen:
                seen.add(pt_tup)
                unique_pts.append(p)
        if len(unique_pts) <= 1:
            return np.array(unique_pts)
        unvisited, ordered = np.array(unique_pts[1:]), [unique_pts[0]]
        results = []
        while len(unvisited) > 0:
            start_pt, end_pt = ordered[0], ordered[-1]
            dists_start = np.sum((unvisited - start_pt)**2, axis=1)
            dists_end = np.sum((unvisited - end_pt)**2, axis=1)
            min_idx_start, min_idx_end = np.argmin(dists_start), np.argmin(dists_end)

            if dists_start[min_idx_start] < dists_end[min_idx_end]:
                min_dist, min_idx, to_start = dists_start[min_idx_start], min_idx_start, True
            else:
                min_dist, min_idx, to_start = dists_end[min_idx_end], min_idx_end, False

            if min_dist < 50.0:
                val = unvisited[min_idx]
                if to_start:
                    ordered.insert(0, val)
                else:
                    ordered.append(val)
                unvisited = np.delete(unvisited, min_idx, axis=0)
            else:
                if len(ordered) > 5:
                    results.append(np.array(ordered))
                ordered = [unvisited[0]]
                unvisited = np.delete(unvisited, 0, axis=0)

        if len(ordered) > 5:
            results.append(np.array(ordered))
        return results[0] if results else np.array([])

    def process_once(self, reset=False):
        if reset:
            self.points = []
        if not os.path.exists(self.image_path):
            return
        img_raw = cv2.imread(self.image_path)
        if img_raw is None:
            return

        h_raw, w_raw, _ = img_raw.shape
        max_dim = 800
        if h_raw > max_dim or w_raw > max_dim:
            scale_down = max_dim / max(h_raw, w_raw)
            img = cv2.resize(img_raw, (int(w_raw * scale_down), int(h_raw * scale_down)))
        else:
            img = img_raw.copy()

        h, w, _ = img.shape
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        denoised = cv2.bilateralFilter(gray, 9, 75, 75)

        thresh = cv2.adaptiveThreshold(
            denoised, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 21, 5
        )

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        all_cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        line_mask, initial_paths = np.zeros_like(thresh), []

        if hierarchy is not None:
            for i, cnt in enumerate(all_cnts):
                if hierarchy[0][i][3] != -1:
                    continue

                area, peri = cv2.contourArea(cnt), cv2.arcLength(cnt, True)
                if peri < 5 or area < 3:
                    continue

                hull_area = cv2.contourArea(cv2.convexHull(cnt))
                solidity = float(area) / hull_area if hull_area > 0 else 0

                if solidity > 0.90 and area > 1000:
                    initial_paths.append(
                        self.smooth_trajectory(
                            cnt.reshape(-1, 2), sigma=self.sigma_base, is_closed=True))
                else:
                    temp_mask = np.zeros_like(thresh)
                    cv2.drawContours(temp_mask, all_cnts, i, 255, -1)
                    line_mask = cv2.bitwise_or(line_mask, cv2.bitwise_and(temp_mask, thresh))

        skel = self.skeletonize(line_mask)
        skel_cnts, _ = cv2.findContours(skel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for scnt in skel_cnts:
            if cv2.arcLength(scnt, True) < 3:
                continue
            single_line_pts = self.extract_single_line(scnt)
            if len(single_line_pts) < 3:
                continue

            d_start_end = np.linalg.norm(single_line_pts[0] - single_line_pts[-1])
            is_closed_loop = d_start_end < 12.0
            smoothed = self.smooth_trajectory(
                single_line_pts, sigma=self.sigma_base, is_closed=is_closed_loop)
            initial_paths.append(smoothed)

        if not initial_paths:
            return
        sorted_paths, remaining = [], list(initial_paths)
        sorted_paths.append(remaining.pop(0))

        while remaining:
            last_pt = sorted_paths[-1][-1]
            best_idx, best_dist, reverse_needed = -1, float('inf'), False
            for i, path in enumerate(remaining):
                d1 = np.linalg.norm(last_pt - path[0])
                d2 = np.linalg.norm(last_pt - path[-1])
                if d1 < best_dist:
                    best_dist, best_idx, reverse_needed = d1, i, False
                if d2 < best_dist:
                    best_dist, best_idx, reverse_needed = d2, i, True
            next_path = remaining.pop(best_idx)
            if reverse_needed:
                next_path = next_path[::-1]
            sorted_paths.append(next_path)

        final_merged_paths = []
        if sorted_paths:
            current_merged = sorted_paths[0].tolist()
            for i in range(1, len(sorted_paths)):
                if np.linalg.norm(np.array(current_merged[-1]) - sorted_paths[i][0]) < 25.0:
                    current_merged.extend(sorted_paths[i].tolist())
                else:
                    final_merged_paths.append(np.array(current_merged))
                    current_merged = sorted_paths[i].tolist()
            final_merged_paths.append(np.array(current_merged))

        workspace_w, workspace_h = self.y_max - self.y_min, self.x_max - self.x_min
        scale = min(workspace_w / w, workspace_h / h) * 0.95
        offset_y = (workspace_w - (w * scale)) / 2.0
        offset_x = (workspace_h - (h * scale)) / 2.0

        self.points, vis_img = [], img.copy()
        for pi, path in enumerate(final_merged_paths):
            if pi > 0:
                self.points.append((-999.0, -999.0, -1.0))
            for pt in path:
                u, v = pt
                target_y = self.y_max - (offset_y + u * scale)
                target_x = self.x_max - (offset_x + v * scale)
                self.points.append((target_x, target_y, self.z_draw))
            cv2.polylines(vis_img, [path.astype(np.int32)], False, (0, 255, 0), 2)

        self.vis_img = vis_img
        self.timer_callback()

    def ui_timer_callback(self):
        if self.vis_img is not None:
            cv2.imshow('Adaptive Smoothing Detection', self.vis_img)
        cv2.waitKey(1)

    def timer_callback(self):
        if not self.points:
            return
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link0'
        for p in self.points:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = p[2]
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetectorNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
