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
# One-shot move to a fixed joint-space pose, via arm_controller's
# FollowJointTrajectory action (joint_trajectory_controller/
# JointTrajectoryController, per hardware_controller_manager.yaml) -- not
# MoveL, since MoveL only takes a Cartesian target and this pose was
# captured directly off /joint_states.

import sys

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
JOINT_POSITIONS = [
    0.04901547986290838,
    -0.5809233180622545,
    1.3872579223693606,
    0.783876166834468,
    1.4852768978702722,
    -0.06135923151542565,
]
DURATION_SEC = 4.0


class GraspnetInitial(Node):

    def __init__(self):
        super().__init__('graspnet_initial')
        self.client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def run(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('arm_controller/follow_joint_trajectory not available')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = JOINT_POSITIONS
        point.time_from_start = Duration(
            sec=int(DURATION_SEC), nanosec=int((DURATION_SEC % 1.0) * 1e9))
        goal.trajectory.points = [point]

        self.get_logger().info(f'sending trajectory goal, {DURATION_SEC}s')
        send_goal_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'done, error_code={result.error_code}')
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def main():
    rclpy.init()
    node = GraspnetInitial()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
