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
# omy_graspnet_node.py에 Gemini 기반 물체 이름 분류를 추가한 버전.
# 캡처/추론/후보선정/pick/place는 전부 그대로 상속해서 재사용함.

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

        self.classify_crop_px = 220  # 선택된 grasp 지점 주변을 이 크기로 잘라서 물체 이름 분류에 씀
        self.classify_model = 'gemini-flash-lite-latest'
        # playground/images는 host에 bind-mount돼 있어서 컨테이너 밖에서 바로 확인 가능
        images_dir = os.path.join(PLAYGROUND_DIR, 'images')
        self._images_dir = images_dir
        self._classify_crop_save_path = os.path.join(images_dir, 'classify_crop.png')
        # 이전 실행에서 남은 크롭이 시작하자마자 미리보기에 뜨지 않게 지움
        if os.path.exists(self._classify_crop_save_path):
            os.remove(self._classify_crop_save_path)

        # place 위치/분류 규칙 초기값은 config/omy_ai_graspnet_places.yaml에서 읽음
        # (GUI에서 바꾼 내용은 메모리에만 반영되고 이 파일에는 다시 저장 안 됨)
        with open(os.path.join(PLAYGROUND_DIR, 'config', 'omy_ai_graspnet_places.yaml')) as f:
            places_config = yaml.safe_load(f)
        self._named_places = {
            place_id: (np.array(info['position']), np.array(info['orientation']))
            for place_id, info in places_config['places'].items()}
        # 몸체가 크게 회전해서 가야 하는 place들 -- _place_object에서 접근/복귀를 더 천천히 함
        self._slow_places = {
            place_id for place_id, info in places_config['places'].items() if info.get('slow', False)}
        self._default_place = places_config['default_place']  # 어떤 카테고리에도 안 맞으면 여기로
        self._ai_off_place = places_config['ai_off_place']  # AI mode 꺼졌을 때 항상 여기로
        # 참고사진 있으면 few-shot으로 같이 보내서 텍스트 설명보다 정확하게 판별함
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
        self._last_classification_duration = None  # 명령창 표시용

        # 꺼지면 제미나이 분류 없이 항상 ai_off_place(위 places_config, 기본 place2)로 place
        self.ai_mode = True

        self._last_label = None  # 마지막으로 분류된 라벨, _place_object에서 씀
        self._label_marker_pub = self.create_publisher(Marker, 'omy_ai_graspnet/label_marker', 10)
        self._clear_label_marker()  # rviz에 이전 실행에서 남은 라벨 마커가 계속 떠있지 않게

        # 분류 규칙을 텍스트로 입력받는 창 -- GLX/tk는 자기 스레드에서만 다뤄야 해서 별도 스레드
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

        # place별 박스 -- 각자 목록 보여주고 그 자리에서 추가/삭제
        places_frame = tk.Frame(root)
        places_frame.pack(padx=10, pady=(0, 10))
        place_boxes = {}  # place_id -> (LabelFrame, Listbox)
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

        # execute/pick은 오래 걸리고 auto면 계속 도니까, GUI가 안 멈추게 별도 스레드에서
        # 돌리고 결과는 root.after로 안전하게 위젯에 반영 (tk는 다른 스레드에서 직접 못 건드림)
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

        # on_execute는 auto면 계속 안 끝나서 리턴을 못 기다림 -- 1초마다 최신 값을
        # 그냥 그대로 읽어서 보여줌 (auto 도는 중에도 실시간으로 갱신됨)
        timing_status = tk.Label(root, text='', fg='green', wraplength=400, justify='left')
        timing_status.pack(padx=10, pady=(0, 10))

        def poll_timing():
            det = (f'{self._last_detection_duration:.2f}s'
                   if self._last_detection_duration is not None else '-')
            cls = (f'{self._last_classification_duration:.2f}s'
                   if self._last_classification_duration is not None else '-')
            timing_status.config(
                text=f'last label: {self._last_label or "-"}  |  detection: {det}  |  classification: {cls}')
            root.after(1000, poll_timing)

        poll_timing()

        # 제미나이한테 보낸 크롭 사진 미리보기 -- 파일이 바뀌었을 때만 다시 로드
        tk.Label(root, text='Crop image:').pack(padx=10, pady=(0, 0))
        crop_label = tk.Label(root)
        crop_label.pack(padx=10, pady=(0, 10))
        last_crop_mtime = [None]  # poll_crop_image 클로저에서 갱신할 상태

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
                    crop_label.image = photo  # 참조 안 붙잡으면 tk가 가비지컬렉트해서 사라짐
                except Exception as exc:
                    self.get_logger().warn(f'failed to load crop preview: {exc}')
            root.after(500, poll_crop_image)

        poll_crop_image()

        # self.auto는 on_execute가 매번 그대로 읽으니, 체크박스로 바로 바꾸면 다음
        # execute부터 반영됨 (재시작 필요 없음)
        auto_var = tk.BooleanVar(value=self.auto)
        tk.Checkbutton(control_frame, text='Auto mode', variable=auto_var,
                       command=lambda: setattr(self, 'auto', auto_var.get())).pack(side=tk.LEFT, padx=5)

        # 꺼지면 분류 없이 omy_graspnet_node 원래 동작(고정 위치 하나에 place)으로 감.
        # _detect_and_select/_place_object가 매번 self.ai_mode를 그대로 읽으니 바로 반영됨
        ai_mode_var = tk.BooleanVar(value=self.ai_mode)
        tk.Checkbutton(control_frame, text='AI mode', variable=ai_mode_var,
                       command=lambda: setattr(self, 'ai_mode', ai_mode_var.get())).pack(side=tk.LEFT, padx=5)

        root.mainloop()

    def _sort_categories_summary(self):
        rules = ', '.join(f'{name}->place{place}' for name, _, place in self._sort_categories)
        return f'Current rules: {rules} (else -> place{self._default_place})'

    def _set_category_place(self, name, place_id):
        # 이미 있는 이름이면 place만 갱신, 없으면 새로 추가
        others = [(n, f, p) for n, f, p in self._sort_categories if n != name]
        self._sort_categories = others + [(name, f'{name}_reference.png', place_id)]
        self._category_refs = [
            (n, self._load_reference_image(self._images_dir, f), p) for n, f, p in self._sort_categories]

    def _remove_category(self, name):
        self._sort_categories = [(n, f, p) for n, f, p in self._sort_categories if n != name]
        self._category_refs = [
            (n, self._load_reference_image(self._images_dir, f), p) for n, f, p in self._sort_categories]

    def _classify_grasp_object(self, color_image, vertices, candidate):
        # 위치는 이미 candidate.translation으로 알고 있으니 VLM한테는 "이게 뭔지"만 물어봄.
        # vertices[i,j]가 color_image[i,j]랑 픽셀 단위로 대응되므로(캡처 시 align 처리됨),
        # translation과 가장 가까운 점의 픽셀 좌표를 찾아 그 주변만 잘라서 보냄
        if self._genai_client is None:
            return None
        try:
            dists = np.linalg.norm(vertices - candidate.translation, axis=2)
            row, col = np.unravel_index(np.argmin(dists), dists.shape)
            half = self.classify_crop_px // 2
            h, w = color_image.shape[:2]
            r0, r1 = max(0, row - half), min(h, row + half)
            c0, c1 = max(0, col - half), min(w, col + half)
            crop = Image.fromarray(color_image[r0:r1, c0:c1])
            crop.save(self._classify_crop_save_path)  # 매번 덮어씀, host에서 바로 확인용

            # _sort_categories 목록으로 프롬프트를 자동 생성 -- 참고사진 있는 카테고리는
            # few-shot으로 같이 보내고, 없는 카테고리도 이름으로만 후보에 넣음
            contents = []
            for name, ref_image, _ in self._category_refs:
                if ref_image is not None:
                    contents.append(ref_image)
                    contents.append(f'This is an example photo of "{name}".')
            contents.append(crop)
            category_names = ', '.join(f'"{name}"' for name, _, _ in self._category_refs)
            contents.append(
                f'Which of these categories does the object in the last photo belong to: '
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
            resp = self._genai_client.models.generate_content(
                model=self.classify_model, contents=contents)
            self._last_classification_duration = time.monotonic() - start_time
            self.get_logger().info(f'gemini classification took {self._last_classification_duration:.2f}s')
            return self._parse_classification_response(resp.text)
        except Exception as exc:
            self.get_logger().warn(f'object classification failed: {exc}')
            return None

    def _parse_classification_response(self, text):
        # "한 단어로만" 지시해도 가끔 설명을 먼저 붙이고 맨 끝에 진짜 답을 냄
        # (예: "...partially obscured...). \n\ngreen doll") -- 카테고리 이름(여러 단어일 수
        # 있음)이 응답 안에 통째로 들어있는지 먼저 확인. "doll"이 "green doll"보다 먼저
        # 매칭되면 안 되니 긴 이름부터 검사
        text_lower = text.strip().lower()
        for name in sorted((n for n, _, _ in self._category_refs), key=len, reverse=True):
            if name.lower() in text_lower:
                return name
        if 'background' in text_lower.split():
            return 'background'
        # 어떤 카테고리에도 안 맞으면 자유 답변 -- 마지막 단어만 정리해서 사용
        words = text.strip().split()
        return words[-1].strip('.,()!?"\'') if words else None

    def _clear_label_marker(self):
        # 이전 실행에서 그려진 라벨 마커가 rviz에 계속 남아있지 않도록 지움
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
        marker.pose.position.z += 0.08  # 그리퍼 마커 위에 뜨게
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.03  # 텍스트 높이(m)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 1.0, 1.0
        # rviz의 TEXT_VIEW_FACING 마커는 스페이스가 있으면 단어 사이 간격을 이상하게
        # 크게 그림 -- "green doll" 같은 여러 단어 라벨은 줄바꿈으로 바꿔서 우회
        marker.text = label.replace(' ', '\n')
        marker.lifetime.sec = 0
        self._label_marker_pub.publish(marker)

    def _detect_and_select(self):
        # 새로 스캔할 때마다 이전 스캔의 라벨 마커가 남아있지 않게 먼저 지움 --
        # 이번 스캔에서 라벨이 나오면 아래에서 다시 그림
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
                    # 물체 없이 배경만 잡은 오탐 -- 집지 말고 실패 처리해서 재스캔 유도
                    self.get_logger().warn('classified as background -- rejecting, will rescan')
                    self._clear_gripper_marker()
                    return None, None, None, 'rejected background detection, rescanning'
                message += f', object={label}'
                self._last_label = label
                self._publish_label_marker(translation_base, label)
        return selected, translation_base, quat_base, message

    def _place_object(self):
        if not self.ai_mode:
            # 분류 안 하니 그냥 정해둔 place로 (config의 ai_off_place)
            place_id = self._ai_off_place
        else:
            # _sort_categories에서 라벨과 이름이 맞는 첫 카테고리의 place로. 못 받았거나
            # 어떤 카테고리에도 안 맞으면(라벨이 "other" 등) _default_place로
            place_id = self._default_place
            if self._last_label:
                label_lower = self._last_label.lower()
                # 긴 이름부터 검사 -- "doll"과 "green doll"이 둘 다 등록돼 있으면
                # 더 구체적인("green doll") 쪽을 우선해야 함
                for name, _, place in sorted(self._sort_categories, key=lambda c: len(c[0]), reverse=True):
                    if name.lower() in label_lower:
                        place_id = place
                        break
        translation, quat = self._named_places[place_id]
        self.get_logger().info(f'placing at place{place_id} (label={self._last_label})')

        approach_translation = translation.copy()
        approach_translation[2] += self.offsets.pregrasp_offset_m

        # 놓는 곳 위로 이동하는 것만 0.5초 더 빠르게, 내려가는 것/복귀는 원래 속도.
        # _slow_places(yaml의 slow: true)는 몸체가 크게 돌아가야 해서 접근은 1.5초,
        # 홈 복귀는 2.0초 더 천천히 감
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
