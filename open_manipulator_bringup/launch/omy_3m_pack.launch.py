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

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    # Step 1: Start follower launch file
    start_y = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'open_manipulator_bringup',
            'omy_3m.launch.py',
        ],
        output='screen',
    )

    # Step 2: Run the initialization script for the follower with pack mode
    omy_3m_pack = Node(
        package='open_manipulator_bringup',
        executable='pack_unpack_3m',
        output='screen',
        parameters=[{'operation_mode': 'pack'}],
    )

    return LaunchDescription([
        LogInfo(msg='🚀 Starting omy_3m.launch.py...'),
        start_y,
        RegisterEventHandler(
            OnProcessStart(
                target_action=start_y,
                on_start=[
                    LogInfo(
                        msg='✅ omy_3m.launch.py has fully started.'
                        'Start to pack...'
                    ),
                    omy_3m_pack,
                ],
            )
        ),
    ])
