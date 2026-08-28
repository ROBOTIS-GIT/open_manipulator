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
# Adds Gemini-based object classification on top of omy_graspnet_node.py.

import os
import sys
import threading
import time
import tkinter as tk

from google import genai
import numpy as np
import rclpy
from PIL import Image, ImageTk
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
import yaml

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from omy_graspnet_node import OmyGraspnetNode, PLAYGROUND_DIR, _original_argv  # noqa: E402


class OmyAiGraspnetNode(OmyGraspnetNode):

    def __init__(self):
        super().__init__()

        self.classify_crop_px = 220
        self.classify_model = 'gemini-flash-lite-latest'
        images_dir = os.path.join(PLAYGROUND_DIR, 'images')
        self._images_dir = images_dir
        self._classify_crop_save_path = os.path.join(images_dir, 'classify_crop.png')
        if os.path.exists(self._classify_crop_save_path):
            os.remove(self._classify_crop_save_path)

        # Load place locations / sorting rules
        with open(os.path.join(PLAYGROUND_DIR, 'config', 'omy_ai_graspnet_places.yaml')) as f:
            places_config = yaml.safe_load(f)
        self._named_places = {
            place_id: (np.array(info['position']), np.array(info['orientation']))
            for place_id, info in places_config['places'].items()}
        self._slow_places = {
            place_id for place_id, info in places_config['places'].items() if info.get('slow', False)}
        self._default_place = places_config['default_place']
        self._sort_categories = [
            (cat['name'], f'{cat["name"]}_reference.png', cat['place'])
            for cat in places_config.get('sort_categories', [])]
        self._category_refs = [
            (name, self._load_reference_image(images_dir, filename), place)
            for name, filename, place in self._sort_categories]

        gemini_api_key = os.environ.get('GEMINI_API_KEY')
        self._genai_client = genai.Client(api_key=gemini_api_key) if gemini_api_key else None
        if self._genai_client is None:
            self.get_logger().warn('GEMINI_API_KEY not set -- object classification disabled')
        self._last_classification_duration = None
        self._classifying = False  # for the command window's progress indicator

        self.ai_mode = True

        self._last_label = None
        self._label_marker_pub = self.create_publisher(Marker, 'omy_ai_graspnet/label_marker', 10)
        self._clear_label_marker()

        # Command window GUI
        threading.Thread(target=self._run_command_window, daemon=True).start()

    @staticmethod
    def _load_reference_image(images_dir, filename):
        path = os.path.join(images_dir, filename)
        return Image.open(path) if os.path.exists(path) else None

    def _run_command_window(self):
        root = tk.Tk()
        root.title('omy_ai_graspnet')

        status = tk.Label(root, text=self._sort_categories_summary(), fg='blue', wraplength=400, justify='left')
        status.pack(padx=10, pady=(10, 5))

        # Per-place category list
        places_frame = tk.Frame(root)
        places_frame.pack(padx=10, pady=(0, 10))
        place_boxes = {}
        default_var = tk.IntVar(value=self._default_place)

        def refresh():
            for place_id, (box, listbox) in place_boxes.items():
                listbox.delete(0, tk.END)
                for name, _, place in self._sort_categories:
                    if place == place_id:
                        listbox.insert(tk.END, name)
                box.config(text=f'place{place_id}' + (' (else)' if place_id == self._default_place else ''))
            status.config(text=self._sort_categories_summary())

        for place_id in sorted(self._named_places.keys()):
            box = tk.LabelFrame(places_frame, text=f'place{place_id}')
            box.pack(side=tk.LEFT, padx=5, fill=tk.Y)
            listbox = tk.Listbox(box, height=5, width=14, exportselection=False)
            listbox.pack(padx=5, pady=5)
            place_boxes[place_id] = (box, listbox)

            add_entry = tk.Entry(box, width=12)
            add_entry.pack(padx=5)

            def add(pid=place_id, add_entry=add_entry):
                name = add_entry.get().strip().lower()
                if not name:
                    return
                self._set_category_place(name, pid)
                add_entry.delete(0, tk.END)
                refresh()

            add_entry.bind('<Return>', lambda e, f=add: f())
            tk.Button(box, text='Add', command=add).pack(padx=5, pady=2)

            def remove(pid=place_id, listbox=listbox):
                sel = listbox.curselection()
                if not sel:
                    return
                self._remove_category(listbox.get(sel[0]))
                refresh()

            tk.Button(box, text='Remove', command=remove).pack(padx=5, pady=(0, 2))

            def set_default(pid=place_id):
                self._default_place = pid
                refresh()

            tk.Radiobutton(box, text='else', variable=default_var, value=place_id,
                           command=set_default).pack(padx=5, pady=(0, 5))

        refresh()

        def on_clear():
            self._sort_categories = []
            self._category_refs = []
            self._default_place = 1
            default_var.set(1)
            self.get_logger().info(f'sort categories cleared: {self._sort_categories_summary()}')
            refresh()

        tk.Button(root, text='Clear Rules', command=on_clear).pack(padx=10, pady=(0, 10))

        # Execute / pick / cancel
        control_frame = tk.Frame(root)
        control_frame.pack(padx=10, pady=(0, 10))
        exec_status = tk.Label(root, text='', fg='blue', wraplength=400, justify='left')
        exec_status.pack(padx=10, pady=(0, 5))

        def run_service(name, handler):
            def worker():
                response = handler(Trigger.Request(), Trigger.Response())
                root.after(0, lambda: exec_status.config(text=f'{name}: {response.message}'))

            threading.Thread(target=worker, daemon=True).start()

        tk.Button(control_frame, text='Execute',
                  command=lambda: run_service('execute', self.on_execute)).pack(side=tk.LEFT, padx=5)
        tk.Button(control_frame, text='Pick',
                  command=lambda: run_service('pick', self.on_pick)).pack(side=tk.LEFT, padx=5)
        tk.Button(control_frame, text='Cancel',
                  command=lambda: run_service('cancel', self.on_cancel)).pack(side=tk.LEFT, padx=5)

        # Live status display
        timing_status = tk.Label(root, text='', fg='green', wraplength=400, justify='left')
        timing_status.pack(padx=10, pady=(0, 10))

        dots_counter = [0]

        def poll_timing():
            det = (f'{self._last_detection_duration:.2f}s'
                   if self._last_detection_duration is not None else '-')
            if self._classifying:
                dots_counter[0] += 1
                cls = 'classifying' + '.' * (dots_counter[0] % 4)
            else:
                cls = (f'{self._last_classification_duration:.2f}s'
                       if self._last_classification_duration is not None else '-')
            timing_status.config(
                text=f'last label: {self._last_label or "-"}  |  detection: {det}  |  classification: {cls}')
            root.after(300 if self._classifying else 1000, poll_timing)

        poll_timing()

        # Crop image preview
        tk.Label(root, text='Crop image:').pack(padx=10, pady=(0, 0))
        crop_label = tk.Label(root)
        crop_label.pack(padx=10, pady=(0, 10))
        last_crop_mtime = [None]

        def poll_crop_image():
            try:
                mtime = os.path.getmtime(self._classify_crop_save_path)
            except OSError:
                mtime = None
            if mtime is not None and mtime != last_crop_mtime[0]:
                last_crop_mtime[0] = mtime
                try:
                    img = Image.open(self._classify_crop_save_path).copy()
                    img.thumbnail((220, 220))
                    photo = ImageTk.PhotoImage(img)
                    crop_label.config(image=photo)
                    crop_label.image = photo
                except Exception as exc:
                    self.get_logger().warn(f'failed to load crop preview: {exc}')
            root.after(500, poll_crop_image)

        poll_crop_image()

        auto_var = tk.BooleanVar(value=self.auto)
        tk.Checkbutton(control_frame, text='Auto mode', variable=auto_var,
                       command=lambda: setattr(self, 'auto', auto_var.get())).pack(side=tk.LEFT, padx=5)

        ai_mode_var = tk.BooleanVar(value=self.ai_mode)
        tk.Checkbutton(control_frame, text='AI mode', variable=ai_mode_var,
                       command=lambda: setattr(self, 'ai_mode', ai_mode_var.get())).pack(side=tk.LEFT, padx=5)

        root.mainloop()

    def _sort_categories_summary(self):
        rules = ', '.join(f'{name}->place{place}' for name, _, place in self._sort_categories)
        return f'Current rules: {rules} (else -> place{self._default_place})'

    def _set_category_place(self, name, place_id):
        others = [(n, f, p) for n, f, p in self._sort_categories if n != name]
        self._sort_categories = others + [(name, f'{name}_reference.png', place_id)]
        self._category_refs = [
            (n, self._load_reference_image(self._images_dir, f), p) for n, f, p in self._sort_categories]

    def _remove_category(self, name):
        self._sort_categories = [(n, f, p) for n, f, p in self._sort_categories if n != name]
        self._category_refs = [
            (n, self._load_reference_image(self._images_dir, f), p) for n, f, p in self._sort_categories]

    def _classify_grasp_object(self, color_image, vertices, candidate):
        if self._genai_client is None:
            return None
        try:
            # Crop around the grasp point
            dists = np.linalg.norm(vertices - candidate.translation, axis=2)
            row, col = np.unravel_index(np.argmin(dists), dists.shape)
            half = self.classify_crop_px // 2
            h, w = color_image.shape[:2]
            r0, r1 = max(0, row - half), min(h, row + half)
            c0, c1 = max(0, col - half), min(w, col + half)
            crop = Image.fromarray(color_image[r0:r1, c0:c1])
            crop.save(self._classify_crop_save_path)

            # Build the prompt (few-shot reference photos + the crop)
            contents = []
            for name, ref_image, _ in self._category_refs:
                if ref_image is not None:
                    contents.append(ref_image)
                    contents.append(f'This is an example photo of "{name}".')
            contents.append(crop)
            category_names = ', '.join(f'"{name}"' for name, _, _ in self._category_refs)
            contents.append(
                f'The last photo is cropped so the object being grasped is at the center of '
                'the frame -- other objects may be partially visible near the edges due to '
                'clutter/occlusion, but only classify the object at the center; ignore the '
                'rest. Which of these categories does that centered object belong to: '
                f'{category_names}? Interpret each category broadly/loosely -- e.g. any '
                'stuffed animal (unicorn, teddy bear, etc.) counts as "doll", any small '
                'actuator/servo counts as "motor". A category name may have multiple words '
                '(e.g. a color plus a type, like "green doll") -- match it only if the object '
                'satisfies every word in the name, and answer with that exact category name in '
                'full, not just part of it. Prefer matching one of the category names over '
                'giving a more specific description, if it plausibly fits. If the photo shows '
                'no distinct object at all -- just empty table/background surface, no graspable '
                'item in view -- answer exactly "background". Otherwise, if none of the '
                'categories plausibly match, answer with a short single word (in English, no '
                'spaces) for what the object actually is. Answer with no explanation: either '
                'one of the category names above written out in full, "background", or your '
                'best single-word guess at what the object is.')
            start_time = time.monotonic()
            self._classifying = True
            try:
                resp = self._genai_client.models.generate_content(
                    model=self.classify_model, contents=contents)
            finally:
                self._classifying = False
            self._last_classification_duration = time.monotonic() - start_time
            self.get_logger().info(f'gemini classification took {self._last_classification_duration:.2f}s')
            return self._parse_classification_response(resp.text)
        except Exception as exc:
            self.get_logger().warn(f'object classification failed: {exc}')
            return None

    def _parse_classification_response(self, text):
        # Match longer category names first (e.g. prefer "green doll" over "doll")
        text_lower = text.strip().lower()
        for name in sorted((n for n, _, _ in self._category_refs), key=len, reverse=True):
            if name.lower() in text_lower:
                return name
        if 'background' in text_lower.split():
            return 'background'
        words = text.strip().split()
        return words[-1].strip('.,()!?"\'') if words else None

    def _clear_label_marker(self):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'omy_ai_graspnet'
        marker.id = 0
        marker.action = Marker.DELETE
        self._label_marker_pub.publish(marker)

    def _publish_label_marker(self, translation_base, label):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'omy_ai_graspnet'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        (marker.pose.position.x, marker.pose.position.y,
         marker.pose.position.z) = translation_base.tolist()
        marker.pose.position.z += 0.08
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.03
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 1.0, 1.0
        # rviz's TEXT_VIEW_FACING marker renders spaces with an odd amount of
        # extra width, so swap spaces for newlines to work around it
        marker.text = label.replace(' ', '\n')
        marker.lifetime.sec = 0
        self._label_marker_pub.publish(marker)

    def _detect_and_select(self):
        self._clear_label_marker()
        selected, translation_base, quat_base, message = super()._detect_and_select()
        self._last_label = None
        if not self.ai_mode:
            return selected, translation_base, quat_base, message
        if selected is not None:
            label = self._classify_grasp_object(self._last_color_image, self._last_vertices, selected)
            if label:
                self.get_logger().info(f'object classified as: {label}')
                if label.lower() == 'background':
                    self.get_logger().warn('classified as background -- rejecting, will rescan')
                    self._clear_gripper_marker()
                    return None, None, None, 'rejected background detection, rescanning'
                message += f', object={label}'
                self._last_label = label
                self._publish_label_marker(translation_base, label)
        return selected, translation_base, quat_base, message

    def _place_object(self):
        place_id = self._default_place
        if self.ai_mode and self._last_label:
            label_lower = self._last_label.lower()
            for name, _, place in sorted(self._sort_categories, key=lambda c: len(c[0]), reverse=True):
                if name.lower() in label_lower:
                    place_id = place
                    break
        translation, quat = self._named_places[place_id]
        self.get_logger().info(f'placing at place{place_id} (label={self._last_label})')

        approach_translation = translation.copy()
        approach_translation[2] += self.offsets.pregrasp_offset_m

        # slow_places approach/return more slowly
        is_slow = place_id in self._slow_places
        approach_duration = self.movel_duration_sec - 0.5 + (1.5 if is_slow else 0.0)
        return_duration = self.return_home_duration_sec + (2.0 if is_slow else 0.0)
        self._send_movel(approach_translation, quat, duration_sec=approach_duration)
        self._send_movel(translation, quat, duration_sec=self.descent_duration_sec)
        self._send_gripper(self.gripper.open_joint)
        self._send_movel(self.initial_translation, self.initial_quat, duration_sec=return_duration)


def main():
    rclpy.init(args=_original_argv)
    node = OmyAiGraspnetNode()
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
