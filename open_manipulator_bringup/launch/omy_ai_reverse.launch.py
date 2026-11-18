#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
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
# Author: Sungho Woo, Woojin Wie

"""OMY teleoperation with one launch owner for every child process."""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('open_manipulator_bringup')
    # Include both descriptions in this LaunchService. Spawning another `ros2
    # launch` (especially through a shell) can lose its new children on SIGINT.
    # Follower exit handlers evaluate launch arguments later; retain their scope.
    follower = GroupAction(scoped=False, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            package, 'launch', 'omy_l100_follower_ai.launch.py'])),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            # Only the executor owned here performs the startup motion.
            'init_position': 'false',
        }.items())])
    leader = GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            package, 'launch', 'omy_f3m_leader_ai.launch.py'])),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'use_self_collision_avoidance': LaunchConfiguration('use_self_collision_avoidance'),
            'ros2_control_type': 'omy_f3m_current',
        }.items())])
    executor = Node(
        package='open_manipulator_bringup', executable='joint_trajectory_executor',
        parameters=[PathJoinSubstitution([
            package, 'config', 'omy_l100_follower_ai', 'initial_positions.yaml'])],
        output='screen')

    def after_initial_pose(event, context):
        # OnProcessExit also runs on Ctrl+C/failure. Neither may start a leader.
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg='Initial pose failed; stopping teleoperation.'),
                    EmitEvent(event=Shutdown(reason='Initial pose executor failed'))]
        return [LogInfo(msg='Initial pose completed; starting leader.'), leader]

    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('use_self_collision_avoidance', default_value='true'),
        RegisterEventHandler(OnProcessExit(target_action=executor, on_exit=after_initial_pose)),
        LogInfo(msg='Starting follower; waiting for its trajectory action server.'),
        follower,
        # The executor waits for the action server; process creation alone is
        # not interpreted as controller readiness.
        executor,
    ])
