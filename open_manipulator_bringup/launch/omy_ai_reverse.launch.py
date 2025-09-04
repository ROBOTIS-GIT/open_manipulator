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

"""Launch file for AI teleoperation of OMY F3M for leader, OMY L100 for follower."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for AI teleoperation."""
    # Step 1: Start follower launch file
    start_follower = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'open_manipulator_bringup',
            'omy_l100_follower_ai.launch.py',
        ],
        output='screen',
    )

    # Step 2: Run the joint trajectory executor for the follower
    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omy_l100_follower_ai',
        'initial_positions.yaml',
    ])

    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='screen',
    )

    # Step 3: Start leader launch file
    start_leader = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'open_manipulator_bringup',
            'omy_f3m_leader_ai.launch.py',
        ],
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        LogInfo(msg='Starting omy_l100_follower_ai.launch.py...'),
        start_follower,
        # Step 2: Ensure joint_trajectory_executor starts after start_follower
        RegisterEventHandler(
            OnProcessStart(
                target_action=start_follower,
                on_start=[
                    LogInfo(
                        msg=(
                            '✅ omy_l100_follower_ai.launch.py has fully started. '
                            'Running joint_trajectory_executor...'
                        )
                    ),
                    joint_trajectory_executor,
                ],
            )
        ),
        # Step 3: Ensure start_leader starts after joint_trajectory_executor
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_trajectory_executor,
                on_exit=[
                    LogInfo(
                        msg=(
                            '✅ joint_trajectory_executor has completed. '
                            'Starting omy_f3m_leader_ai.launch.py...'
                        )
                    ),
                    start_leader,
                ],
            )
        ),
    ])
