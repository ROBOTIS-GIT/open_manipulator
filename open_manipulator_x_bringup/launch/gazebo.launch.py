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
# Author: Wonho Yoon, Sungho Woo

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def is_valid_to_launch():
    # Path includes model name of Raspberry Pi series
    path = '/sys/firmware/devicetree/base/model'
    if os.path.exists(path):
        return False
    else:
        return True

def generate_launch_description():
    if not is_valid_to_launch():
        print('Can not launch fake robot in Raspberry Pi')
        return LaunchDescription([])

    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')

    world = LaunchConfiguration(
        'world',
        default=PathJoinSubstitution(
            [
                FindPackageShare('open_manipulator_x_bringup'),
                'worlds',
                'empty_world.model'
            ]
        )
    )

    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether execute rviz2'),

        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.'),

        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='Directory of gazebo world file'),

        DeclareLaunchArgument(
            'x_pose',
            default_value=pose['x'],
            description='position of open_manipulator_x'),

        DeclareLaunchArgument(
            'y_pose',
            default_value=pose['y'],
            description='position of open_manipulator_x'),

        DeclareLaunchArgument(
            'z_pose',
            default_value=pose['z'],
            description='position of open_manipulator_x'),

        DeclareLaunchArgument(
            'roll',
            default_value=pose['R'],
            description='orientation of open_manipulator_x'),

        DeclareLaunchArgument(
            'pitch',
            default_value=pose['P'],
            description='orientation of open_manipulator_x'),

        DeclareLaunchArgument(
            'yaw',
            default_value=pose['Y'],
            description='orientation of open_manipulator_x'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/base.launch.py']),
            launch_arguments={
                'start_rviz': start_rviz,
                'prefix': prefix,
                'use_sim': use_sim,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare('gazebo_ros'),
                            'launch',
                            'gazebo.launch.py'
                        ]
                    )
                ]
            ),
            launch_arguments={
                'verbose': 'false',
                'world': world,
            }.items(),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'open_manipulator_x_system',
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],
                ],
            output='screen',
        ),
    ])
