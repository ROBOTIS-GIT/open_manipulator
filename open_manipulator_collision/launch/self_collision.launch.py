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
# Author: Sungho Woo

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'urdf',
        'omy_f3m',
        'omy_f3m.urdf.xacro'
    ])

    return LaunchDescription([

        Node(
            package='open_manipulator_collision',
            executable='self_collision_node',
            name='self_collision_node',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'base_link': 'link0',
                'tip_link': 'rh_p12_rn_base',
                'enable_marker': False
            }],
            output='screen'
        )
    ])
