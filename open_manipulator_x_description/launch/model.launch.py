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
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

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

    prefix = LaunchConfiguration('prefix')
    use_gui = LaunchConfiguration('use_gui')

    urdf_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('open_manipulator_x_description'),
                    'urdf',
                    'open_manipulator_x_robot.urdf.xacro'
                ]
            ),
            ' ',
            'prefix:=',
            prefix,
            ' ',
            'use_fake_hardware:=',
            'False',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('open_manipulator_x_description'),
            'rviz',
            'open_manipulator_x.rviz'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names'),

        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Run joint state publisher gui node'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf_file}],
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui)),
    ])
