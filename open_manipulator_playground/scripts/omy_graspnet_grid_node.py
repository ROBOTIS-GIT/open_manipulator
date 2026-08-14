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
# 격자형 보관함에 왼쪽위부터 순서대로 채워넣는 pick&place 노드.
# omy_graspnet_node.py를 상속해서 집는 동작(캡처/추론/후보선정/pick)은 그대로
# 재사용하고, 놓는 동작만 격자 좌표 계산으로 교체함.

import os
import sys

import numpy as np
import rclpy
import yaml

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from omy_graspnet_node import OmyGraspnetNode, PLAYGROUND_DIR, _original_argv  # noqa: E402


class OmyGraspnetGridNode(OmyGraspnetNode):

    def __init__(self):
        super().__init__()

        self.filters.max_grasp_width_m = 0.06  # 다이나믹셀 크기 기준 -- 이보다 넓게 측정된 후보는 탈락

        # 격자 좌표(모서리 4개, end_effector_link 타겟)는 config/에서 읽음
        with open(os.path.join(PLAYGROUND_DIR, 'config', 'omy_graspnet_grid.yaml')) as f:
            grid_config = yaml.safe_load(f)
        self.grid_rows = grid_config['grid_rows']
        self.grid_cols = grid_config['grid_cols']
        corners = grid_config['corners']
        self.grid_top_left = np.array(corners['top_left'])
        self.grid_top_right = np.array(corners['top_right'])
        self.grid_bottom_left = np.array(corners['bottom_left'])
        self.grid_bottom_right = np.array(corners['bottom_right'])

        # 네 모서리 orientation은 거의 동일(측정 오차 수준)해서 평균 하나로 고정해서 씀
        corner_quats = np.array(grid_config['corner_orientations'])
        avg_quat = corner_quats.mean(axis=0)
        self.grid_quat = avg_quat / np.linalg.norm(avg_quat)

        self._next_cell_index = 0  # 왼쪽위(0,0)부터 오른쪽아래까지 row-major 순서

    def _grid_cell_position(self, row, col):
        # 네 모서리로 쌍선형보간 -- s: 왼쪽(0)~오른쪽(1), t: 위(0)~아래(1)
        s = col / (self.grid_cols - 1)
        t = row / (self.grid_rows - 1)
        top = self.grid_top_left + s * (self.grid_top_right - self.grid_top_left)
        bottom = self.grid_bottom_left + s * (self.grid_bottom_right - self.grid_bottom_left)
        return top + t * (bottom - top)

    def _place_object(self):
        row, col = divmod(self._next_cell_index, self.grid_cols)
        cell_translation = self._grid_cell_position(row, col)
        self.get_logger().info(
            f'placing at grid cell (row={row}, col={col}), '
            f'{self._next_cell_index + 1}/{self.grid_rows * self.grid_cols}')

        approach_translation = cell_translation.copy()
        approach_translation[2] += self.offsets.pregrasp_offset_m

        # 격자에 놓으러 갈 때만 기존보다 0.5초씩 더 여유있게 (정밀하게 놓아야 해서)
        self._send_movel(approach_translation, self.grid_quat, duration_sec=self.movel_duration_sec + 0.5)
        self._send_movel(cell_translation, self.grid_quat, duration_sec=self.descent_duration_sec + 0.5)
        self._send_gripper(self.gripper.open_joint)
        self._send_movel(
            self.initial_translation, self.initial_quat, duration_sec=self.return_home_duration_sec + 0.5)

        self._next_cell_index += 1
        if self._next_cell_index >= self.grid_rows * self.grid_cols:
            # 마지막 칸까지 채웠으면 멈추지 않고 첫 칸부터 다시 채움 (덮어쓰기)
            self.get_logger().info('grid filled completely -- starting over from the first cell')
            self._next_cell_index = 0


def main():
    rclpy.init(args=_original_argv)
    node = OmyGraspnetGridNode()
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
