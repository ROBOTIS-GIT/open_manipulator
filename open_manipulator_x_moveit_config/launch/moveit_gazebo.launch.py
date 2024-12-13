#!/usr/bin/env python3
#
# Copyright 2020 ROBOTIS CO., LTD.
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
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    launch_dir = os.path.join(
        get_package_share_directory(
            'open_manipulator_x_moveit_config'), 'launch')
    bringup_launch_dir = os.path.join(
        get_package_share_directory(
            'open_manipulator_x_bringup'), 'launch')

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py'])
    )
    ld.add_action(rviz_launch)

    # move_group
    move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py']),
            launch_arguments={
                'use_sim': 'true',
            }.items(),
        )
    ld.add_action(move_group_launch)

    # gazebo_control with robot_state_publisher
    rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether execute rviz2')
    ld.add_action(rviz_arg)

    empty_world_path = PathJoinSubstitution(
        [
            FindPackageShare('open_manipulator_x_bringup'),
            'worlds',
            'empty_world.model'
        ]
    )

    gazebo_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/gazebo.launch.py']),
        launch_arguments={
            'world': empty_world_path,
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
        )
    ld.add_action(gazebo_control_launch)

    return ld
