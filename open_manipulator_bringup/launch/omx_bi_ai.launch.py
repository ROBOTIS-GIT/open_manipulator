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

"""Launch file for bimanual AI teleoperation: 2x OMX L leader, 2x OMX F follower."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'follower_left_port_name',
            default_value='/dev/serial/by-id/'
                          'usb-ROBOTIS_OpenRB-150_6D5D4B68503059384C2E3120FF05133E-if00',
            description='Port name of the left follower arm',
        ),
        DeclareLaunchArgument(
            'follower_right_port_name',
            default_value='/dev/serial/by-id/'
                          'usb-ROBOTIS_OpenRB-150_23D2E9915157375037202020FF0F2510-if00',
            description='Port name of the right follower arm',
        ),
        DeclareLaunchArgument(
            'leader_left_port_name',
            default_value='/dev/serial/by-id/'
                          'usb-ROBOTIS_OpenRB-150_895EBFC8503059384C2E3120FF08031C-if00',
            description='Port name of the left leader arm',
        ),
        DeclareLaunchArgument(
            'leader_right_port_name',
            default_value='/dev/serial/by-id/'
                          'usb-ROBOTIS_OpenRB-150_84BB894D5157375037202020FF102815-if00',
            description='Port name of the right leader arm',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'start_rviz', default_value='false', description='Whether to execute rviz2'
        ),
    ]

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    # Step 1: Start bimanual follower
    start_follower = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'open_manipulator_bringup',
            'omx_f_follower_bi_ai.launch.py',
            ['left_port_name:=', LaunchConfiguration('follower_left_port_name')],
            ['right_port_name:=', LaunchConfiguration('follower_right_port_name')],
            ['use_mock_hardware:=', use_mock_hardware],
            ['start_rviz:=', LaunchConfiguration('start_rviz')],
        ],
        output='screen',
    )

    # Step 2: Move both follower arms to the initial pose
    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omx_f_follower_bi_ai',
        'initial_positions.yaml',
    ])

    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='screen',
    )

    # Step 3: Start bimanual leader
    start_leader = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'open_manipulator_bringup',
            'omx_l_leader_bi_ai.launch.py',
            ['left_port_name:=', LaunchConfiguration('leader_left_port_name')],
            ['right_port_name:=', LaunchConfiguration('leader_right_port_name')],
            ['use_mock_hardware:=', use_mock_hardware],
        ],
        output='screen',
    )

    return LaunchDescription(declared_arguments + [
        LogInfo(msg='Starting omx_f_follower_bi_ai.launch.py...'),
        start_follower,
        RegisterEventHandler(
            OnProcessStart(
                target_action=start_follower,
                on_start=[
                    LogInfo(msg='Follower started. Running joint_trajectory_executor...'),
                    joint_trajectory_executor,
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_trajectory_executor,
                on_exit=[
                    LogInfo(msg='Initial pose reached. Starting omx_l_leader_bi_ai.launch.py...'),
                    start_leader,
                ],
            )
        ),
    ])
