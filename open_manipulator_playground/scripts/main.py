#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0
#
# Single entry point for the OMX-F control side.
#
# One process runs everything this repo is responsible for:
#   - OmxMoveLBridge: HTTP receiver + teleop (delta/absolute/gripper/enable) +
#     the 5 high-level skills, converting Reachy Mini commands into
#     robotis_interfaces/msg/MoveL and control_msgs/action/GripperCommand.
#   - JointTrajectoryRelay: forwards cyclo_control's /leader/joint_trajectory to
#     the arm controller's /arm_controller/joint_trajectory.
#
# The LLM / task decision lives on Reachy Mini; this repo only does control.
#
# The relay forwards cyclo_control's ~100 Hz joint commands to the arm. It runs
# on its OWN executor thread so that the bridge's teleop timer / HTTP work can
# never delay the joint-command stream (which would make arm motion stutter),
# and vice versa. Each node stays single-threaded internally, so no extra locks.

import threading

from omx_joint_trajectory_relay import JointTrajectoryRelay
from omx_movel_bridge import OmxMoveLBridge
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    bridge = OmxMoveLBridge()
    relay = JointTrajectoryRelay()

    # Relay on its own thread/executor: isolates the 100 Hz joint stream.
    relay_executor = SingleThreadedExecutor()
    relay_executor.add_node(relay)
    relay_thread = threading.Thread(target=relay_executor.spin, daemon=True)
    relay_thread.start()

    bridge_executor = SingleThreadedExecutor()
    bridge_executor.add_node(bridge)
    bridge.get_logger().info(
        'OpenManipulator control main up: MoveL bridge (teleop+skills) + '
        'joint_trajectory relay on a dedicated thread.')

    try:
        bridge_executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        bridge.shutdown()
        relay_executor.shutdown()
        bridge_executor.shutdown()
        bridge.destroy_node()
        relay.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
