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
# Picks only the object described by a free-text target. The whole scene is
# sent to Gemini to get the target's approximate location (bounding box),
# and the grasp candidate closest to that location is selected. Candidate
# selection overlaps with omy_graspnet_node.py's _detect_and_select, but is
# re-implemented here to leave that original file untouched.

import json
import os
import re
import sys
import threading
import time
import tkinter as tk

from google import genai
import numpy as np
from PIL import Image, ImageTk
import rclpy
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger
import yaml

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from omy_graspnet_node import (  # noqa: E402, I100
    _original_argv, demo, grab_frame, OmyGraspnetNode, PLAYGROUND_DIR, save_capture)


class OmyTargetGraspnetNode(OmyGraspnetNode):

    def __init__(self):
        super().__init__()

        self.classify_crop_px = 220
        self.classify_model = 'gemini-flash-lite-latest'
        # candidates farther than this from the found location don't count as a match
        self.target_match_max_dist_m = 0.05
        # free text set from the command window; picks the best candidate if empty
        self.target_object = None

        self._found_crop_save_path = os.path.join(
            PLAYGROUND_DIR, 'images', 'target_found_crop.png')
        if os.path.exists(self._found_crop_save_path):
            os.remove(self._found_crop_save_path)

        self._detecting = False  # for the command window's progress indicator

        with open(os.path.join(PLAYGROUND_DIR, 'config', 'omy_target_graspnet_poses.yaml')) as f:
            poses_config = yaml.safe_load(f)
        self.place_approach_translation = np.array(poses_config['place_approach']['position'])
        self.place_approach_quat = np.array(poses_config['place_approach']['orientation'])
        self.place_translation = np.array(poses_config['place']['position'])
        self.place_quat = np.array(poses_config['place']['orientation'])

        gemini_api_key = os.environ.get('GEMINI_API_KEY')
        self._genai_client = genai.Client(api_key=gemini_api_key) if gemini_api_key else None
        if self._genai_client is None:
            self.get_logger().warn('GEMINI_API_KEY not set -- target matching disabled')

        threading.Thread(target=self._run_command_window, daemon=True).start()

    def _place_object(self):
        self._send_movel(self.place_approach_translation, self.place_approach_quat)
        self._send_movel(
            self.place_translation, self.place_quat, duration_sec=self.descent_duration_sec)
        self._send_gripper(self.gripper.open_joint)
        self._send_movel(
            self.initial_translation, self.initial_quat,
            duration_sec=self.return_home_duration_sec)

    def _run_command_window(self):
        root = tk.Tk()
        root.title('omy_target_graspnet')

        tk.Label(root, text='Target object (leave empty for best candidate):').pack(
            padx=10, pady=(10, 0))
        target_entry = tk.Entry(root, width=30)
        target_entry.pack(padx=10, pady=(0, 5))

        status = tk.Label(root, text='Target: (none)', fg='blue')
        status.pack(padx=10, pady=(0, 5))

        def set_target():
            self.target_object = target_entry.get().strip() or None
            status.config(text=f'Target: {self.target_object or "(none)"}')

        target_entry.bind('<Return>', lambda e: set_target())
        tk.Button(root, text='Set target', command=set_target).pack(padx=10, pady=(0, 10))

        control_frame = tk.Frame(root)
        control_frame.pack(padx=10, pady=(0, 10))
        exec_status = tk.Label(root, text='', fg='blue', wraplength=350, justify='left')
        exec_status.pack(padx=10, pady=(0, 10))

        def run_service(name, handler):
            def worker():
                response = handler(Trigger.Request(), Trigger.Response())
                root.after(0, lambda: exec_status.config(text=f'{name}: {response.message}'))

            threading.Thread(target=worker, daemon=True).start()

        tk.Button(control_frame, text='Execute',
                  command=lambda: run_service('execute', self.on_execute)
                  ).pack(side=tk.LEFT, padx=5)
        tk.Button(control_frame, text='Pick',
                  command=lambda: run_service('pick', self.on_pick)).pack(side=tk.LEFT, padx=5)
        tk.Button(control_frame, text='Cancel',
                  command=lambda: run_service('cancel', self.on_cancel)).pack(side=tk.LEFT, padx=5)

        auto_var = tk.BooleanVar(value=self.auto)
        tk.Checkbutton(control_frame, text='Auto mode', variable=auto_var,
                       command=lambda: setattr(self, 'auto', auto_var.get())
                       ).pack(side=tk.LEFT, padx=5)

        # Detection progress -- dots animate while running, then show elapsed time
        timing_status = tk.Label(root, text='', fg='green')
        timing_status.pack(padx=10, pady=(0, 10))
        dots_counter = [0]

        def poll_timing():
            if self._detecting:
                dots_counter[0] += 1
                text = 'detecting' + '.' * (dots_counter[0] % 4)
            elif self._last_detection_duration is not None:
                text = f'last detection: {self._last_detection_duration:.2f}s'
            else:
                text = ''
            timing_status.config(text=text)
            root.after(300 if self._detecting else 1000, poll_timing)

        poll_timing()

        # Preview of the crop once a target is found -- only reloads when the file changes
        tk.Label(root, text='Found object:').pack(padx=10, pady=(0, 0))
        crop_label = tk.Label(root)
        crop_label.pack(padx=10, pady=(0, 10))
        last_crop_mtime = [None]

        def poll_crop_image():
            try:
                mtime = os.path.getmtime(self._found_crop_save_path)
            except OSError:
                mtime = None
            if mtime != last_crop_mtime[0]:
                last_crop_mtime[0] = mtime
                if mtime is None:
                    # File gets removed when nothing is found -- clear the preview too
                    crop_label.config(image='')
                    crop_label.image = None
                else:
                    try:
                        img = Image.open(self._found_crop_save_path).copy()
                        img.thumbnail((220, 220))
                        photo = ImageTk.PhotoImage(img)
                        crop_label.config(image=photo)
                        crop_label.image = photo
                    except Exception as exc:
                        self.get_logger().warn(f'failed to load crop preview: {exc}')
            root.after(500, poll_crop_image)

        poll_crop_image()

        root.mainloop()

    def _locate_target_pixel(self, color_image):
        # Get the target's approximate location (bounding box) from the full
        # image, then convert to a center pixel
        h, w = color_image.shape[:2]
        contents = [
            Image.fromarray(color_image),
            f'Find the region described as "{self.target_object}" in this image -- this may be '
            'a whole object (e.g. "bottle") or a specific part of one (e.g. "hammer handle", '
            '"center of the bottle"); return a tight box around exactly what was described, '
            'not the whole object if only a part was asked for. Respond with its bounding box '
            'as JSON in the form {"box_2d": [ymin, xmin, ymax, xmax]}, with values normalized '
            'to a 0-1000 scale. If it is not visible in the image, respond with exactly '
            '"not found". Answer with nothing else.']
        try:
            resp = self._genai_client.models.generate_content(
                model=self.classify_model, contents=contents)
        except Exception as exc:
            self.get_logger().warn(f'target localization failed: {exc}')
            return None
        text = resp.text.strip()
        if 'not found' in text.lower():
            return None
        box = None
        obj_match = re.search(r'\{[^{}]*\}', text)
        if obj_match is not None:
            try:
                box = json.loads(obj_match.group(0))['box_2d']
            except Exception:
                box = None
        if box is None:
            # Sometimes it answers with a bare array instead of {"box_2d": [...]}
            arr_match = re.search(r'\[[^\[\]]*\]', text)
            if arr_match is not None:
                try:
                    box = json.loads(arr_match.group(0))
                except Exception:
                    box = None
        if box is None or len(box) != 4:
            self.get_logger().warn(f'could not parse localization response: {text!r}')
            return None
        ymin, xmin, ymax, xmax = box
        row = int(np.clip((ymin + ymax) / 2 / 1000 * h, 0, h - 1))
        col = int(np.clip((xmin + xmax) / 2 / 1000 * w, 0, w - 1))
        return row, col

    def _save_found_crop(self, color_image, row, col):
        half = self.classify_crop_px // 2
        h, w = color_image.shape[:2]
        r0, r1 = max(0, row - half), min(h, row + half)
        c0, c1 = max(0, col - half), min(w, col + half)
        Image.fromarray(color_image[r0:r1, c0:c1]).save(self._found_crop_save_path)

    def _select_candidate_near_target(self, candidates, target_point):
        """
        Index into candidates of the one closest to target_point.

        Returns None if none are close enough. Subclasses can override this
        to use a different criterion (e.g. prefer near-vertical candidates).
        """
        dists = [np.linalg.norm(c[1].translation - target_point) for c in candidates]
        nearest_idx = int(np.argmin(dists))
        if dists[nearest_idx] > self.target_match_max_dist_m:
            return None
        return nearest_idx

    def _detect_and_select(self):
        self._detecting = True
        try:
            return self._detect_and_select_impl()
        finally:
            self._detecting = False

    def _detect_and_select_impl(self):
        """Capture, run inference, select target/best candidate (None x3 + reason on fail)."""
        time.sleep(self.camera.capture_settle_sec)
        start_time = time.monotonic()

        with self._camera_lock:
            color_image, depth_image, vertices = grab_frame(
                self._camera_pipeline, self._camera_align,
                self._camera_filter_depth, self._camera_pc)
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

        gg = demo.get_grasps(self.net, end_points)
        if demo.cfgs.collision_thresh > 0:
            gg = demo.collision_detection(gg, np.array(cloud.points))
        if len(gg) == 0:
            self._clear_gripper_marker()
            self._last_detection_duration = time.monotonic() - start_time
            return None, None, None, 'no valid grasp candidate found'

        gg.nms()
        gg.sort_by_score()

        # Candidate filtering (safety checks only -- per-reason counts/logging are skipped here)
        candidates = []
        for candidate in gg[:100]:
            if (self.filters.max_grasp_width_m is not None
                    and candidate.width > self.filters.max_grasp_width_m):
                continue
            if table_plane is not None:
                a, b, c, d = table_plane
                x, y, z = candidate.translation
                if abs(a * x + b * y + c * z + d) < self.filters.min_grasp_height_above_table_m:
                    continue
            t, q = self._to_base_frame(
                candidate.translation, Rotation.from_matrix(candidate.rotation_matrix).as_quat(),
                self.camera.frame_id)
            if t is None:
                continue
            if not (self.filters.min_grasp_z_m <= t[2] <= self.filters.max_grasp_z_m):
                continue
            base_dist = np.linalg.norm(t)
            if (base_dist < self.filters.min_base_distance_m
                    or base_dist > self.filters.max_base_distance_m):
                continue
            if t[0] < self.filters.min_grasp_x_m:
                continue
            approach_axis = Rotation.from_quat(q).as_matrix()[:, 0]
            tilt_deg = np.degrees(np.arccos(np.clip(-approach_axis[2], -1.0, 1.0)))
            if tilt_deg > self.filters.max_approach_tilt_deg:
                continue
            width_ratio = np.clip(candidate.width / self.gripper.open_width_m, 0.0, 1.0)
            rank_score = (candidate.score
                          - self.filters.tilt_score_penalty * (tilt_deg / 90.0)
                          - self.filters.width_score_penalty * width_ratio)
            candidates.append((rank_score, candidate, t, q, tilt_deg))

        if not candidates:
            self._clear_gripper_marker()
            self._last_detection_duration = time.monotonic() - start_time
            return None, None, None, 'no grasp candidate passed the height/tilt sanity check'

        candidates.sort(key=lambda c: c[0], reverse=True)

        # Show every detected candidate regardless of whether the target is found
        with self._geom_lock:
            self._pending_geoms = [cloud] + [
                c[1].to_open3d_geometry() for c in candidates[:self.top_k]]

        if self.target_object and self._genai_client is not None:
            pixel = self._locate_target_pixel(color_image)
            fail_message = f'target "{self.target_object}" not found in image'
            target_point = None if pixel is None else vertices[pixel[0], pixel[1]]
            if target_point is not None and not np.all(np.isfinite(target_point)):
                target_point = None
            if target_point is None:
                self._clear_gripper_marker()
                if os.path.exists(self._found_crop_save_path):
                    os.remove(self._found_crop_save_path)
                self._last_detection_duration = time.monotonic() - start_time
                return None, None, None, fail_message

            # The location was found, so keep the crop for debugging regardless of
            # whether a grasp candidate ends up nearby
            self._save_found_crop(color_image, pixel[0], pixel[1])

            nearest_idx = self._select_candidate_near_target(candidates, target_point)
            if nearest_idx is None:
                self._clear_gripper_marker()
                self._last_detection_duration = time.monotonic() - start_time
                return (None, None, None,
                        f'target "{self.target_object}" located, but no grasp candidate nearby')

            _, selected, translation_base, quat_base, selected_tilt_deg = candidates[nearest_idx]
        else:
            _, selected, translation_base, quat_base, selected_tilt_deg = candidates[0]

        self._publish_grasp(selected)
        self._publish_gripper_marker(selected, translation_base, quat_base)
        self._last_detection_duration = time.monotonic() - start_time
        self.get_logger().info(f'detection took {self._last_detection_duration:.2f}s')

        return selected, translation_base, quat_base, f'selected grasp score={selected.score:.4f}'


def main():
    rclpy.init(args=_original_argv)
    node = OmyTargetGraspnetNode()
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
