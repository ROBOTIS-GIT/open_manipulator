#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class MoveToHome(Node):
    def __init__(self):
        super().__init__('move_to_home')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.move_to_home)

    def move_to_home(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.0, 1.0, 0.0]
        point.time_from_start.sec = 3

        traj.points.append(point)
        self.publisher.publish(traj)
        self.get_logger().info('Moving to Home Position')

        self.timer.cancel()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MoveToHome()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
