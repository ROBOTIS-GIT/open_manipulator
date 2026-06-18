#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0
#
# pose_capture
# ------------
# Teach skill poses from the live controller instead of guessing them. Jog the
# end-effector to a spot (rviz interactive marker from the cyclo controller, or
# the hand teleop bridge), type a name, and this prints a ready-to-paste pose
# entry for omx_skill_poses.yaml / omy_skill_poses.yaml. It also tracks the
# min/max of everywhere you jogged to suggest a workspace box.
#
# Reuses the existing `<robot>_movel_controller/current_pose` topic; works for
# OMX and OMY by pointing current_pose_topic at the right controller.
#
#   ros2 run open_manipulator_playground pose_capture.py \
#     --ros-args -p current_pose_topic:=/omy_movel_controller/current_pose

import threading

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class PoseCapture(Node):

    def __init__(self):
        super().__init__('pose_capture')
        self.declare_parameter('current_pose_topic', '/omy_movel_controller/current_pose')
        self.declare_parameter('default_duration', 1.2)
        self.declare_parameter('include_orientation', False)
        self.topic = self.get_parameter('current_pose_topic').value
        self.default_duration = float(self.get_parameter('default_duration').value)
        self.include_orientation = bool(self.get_parameter('include_orientation').value)

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(PoseStamped, self.topic, self._on_pose, qos)

        self._latest = None          # (x, y, z, qx, qy, qz, qw)
        self._captured = {}          # name -> yaml block
        self._lo = [None, None, None]
        self._hi = [None, None, None]
        self.get_logger().info(f'pose_capture listening on {self.topic}')

    def _on_pose(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        self._latest = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        for i, v in enumerate((p.x, p.y, p.z)):
            self._lo[i] = v if self._lo[i] is None else min(self._lo[i], v)
            self._hi[i] = v if self._hi[i] is None else max(self._hi[i], v)

    def capture(self, name):
        if self._latest is None:
            return None
        x, y, z, qx, qy, qz, qw = self._latest
        lines = [f'  {name}:', f'    x: {x:.4f}', f'    y: {y:.4f}', f'    z: {z:.4f}']
        if self.include_orientation:
            lines += [f'    qx: {qx:.5f}', f'    qy: {qy:.5f}',
                      f'    qz: {qz:.5f}', f'    qw: {qw:.5f}']
        else:
            lines += [f'    # orientation (optional): qx {qx:.5f} qy {qy:.5f} '
                      f'qz {qz:.5f} qw {qw:.5f}']
        lines.append(f'    duration: {self.default_duration}')
        self._captured[name] = '\n'.join(lines)
        return self._captured[name]

    def workspace_yaml(self):
        if any(v is None for v in self._lo):
            return None
        return ('workspace:  # raw envelope of where you jogged; shrink for safety\n'
                f'  x: [{self._lo[0]:.3f}, {self._hi[0]:.3f}]\n'
                f'  y: [{self._lo[1]:.3f}, {self._hi[1]:.3f}]\n'
                f'  z: [{self._lo[2]:.3f}, {self._hi[2]:.3f}]')

    def dump(self):
        out = ['poses:']
        out += [self._captured[name] for name in self._captured]
        ws = self.workspace_yaml()
        if ws:
            out += ['', ws]
        return '\n'.join(out)


_HELP = (
    '\n'
    'Jog the arm (rviz interactive marker or hand teleop), then:\n'
    '  <name>  capture the current pose as <name> (e.g. ready, lower_pick)\n'
    '  ws      print the workspace box observed so far\n'
    '  list    print the full pose book collected so far\n'
    '  q       quit (prints the final pose book)\n'
)


def main(args=None):
    rclpy.init(args=args)
    node = PoseCapture()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    print(_HELP, flush=True)
    try:
        while True:
            cmd = input('pose_capture> ').strip()
            if cmd in ('q', 'quit', 'exit'):
                break
            if not cmd:
                continue
            if cmd == 'ws':
                print(node.workspace_yaml() or '(no pose received yet)', flush=True)
            elif cmd == 'list':
                print(node.dump(), flush=True)
            else:
                block = node.capture(cmd)
                print(block or '(no /current_pose yet - is the controller running?)', flush=True)
    except (EOFError, KeyboardInterrupt):
        pass
    print('\n# ---- final pose book (paste into your *_skill_poses.yaml) ----', flush=True)
    print(node.dump(), flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
