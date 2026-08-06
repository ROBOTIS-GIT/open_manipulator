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

from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def robot_description(xacro_file, prefix):
    return Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf',
            'omy_l100',
            xacro_file,
        ]),
        ' ',
        'prefix:=', prefix,
        ' ',
        'use_mock_hardware:=', 'True',
    ])


def generate_launch_description():
    original_description = robot_description('omy_l100.urdf.xacro', 'original_')
    custom_description = robot_description('custom_omy_l100.urdf.xacro', 'custom_')
    custom_inverted_description = robot_description(
        'custom_inverted_omy_l100.urdf.xacro', 'custom_inverted_'
    )
    inverted_description = robot_description('inverted_omy_l100.urdf.xacro', 'inverted_')

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz',
        'compare_omy_l100.rviz',
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': original_description}],
            remappings=[
                ('robot_description', 'original/robot_description'),
                ('joint_states', 'original/joint_states'),
            ],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            remappings=[
                ('robot_description', 'original/robot_description'),
                ('joint_states', 'original/joint_states'),
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': custom_description}],
            remappings=[
                ('robot_description', 'custom/robot_description'),
                ('joint_states', 'custom/joint_states'),
            ],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            remappings=[
                ('robot_description', 'custom/robot_description'),
                ('joint_states', 'custom/joint_states'),
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': custom_inverted_description}],
            remappings=[
                ('robot_description', 'custom_inverted/robot_description'),
                ('joint_states', 'custom_inverted/joint_states'),
            ],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            remappings=[
                ('robot_description', 'custom_inverted/robot_description'),
                ('joint_states', 'custom_inverted/joint_states'),
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': inverted_description}],
            remappings=[
                ('robot_description', 'inverted/robot_description'),
                ('joint_states', 'inverted/joint_states'),
            ],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            remappings=[
                ('robot_description', 'inverted/robot_description'),
                ('joint_states', 'inverted/joint_states'),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])
