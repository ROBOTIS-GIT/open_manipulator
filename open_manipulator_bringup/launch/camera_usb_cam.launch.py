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
# Author: Wonho Yun, Sungho Woo, Woojin Wie, Junha Cha

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'name',
            default_value='camera1',
            description='Name of the camera',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Video device path to open (e.g., /dev/video2)',
        ),
    ]

    # Launch configurations
    name = LaunchConfiguration('name')
    video_device = LaunchConfiguration('video_device')

    camera_config = PathJoinSubstitution([
        FindPackageShare('usb_cam'),
        'config',
        'params_1.yaml',
    ])

    camera_nodes = [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[
                camera_config,
                {
                    'video_device': video_device,
                },
            ],
            output='both',
            remappings=[
                ('image_raw', [name, '/image_raw']),
                ('image_raw/compressed', [name, '/image_raw/compressed']),
                ('image_raw/compressedDepth', [name, '/image_raw/compressedDepth']),
                ('image_raw/theora', [name, '/image_raw/theora']),
                ('camera_info', [name, '/camera_info']),
            ]
        )
    ]

    return LaunchDescription(declared_arguments + camera_nodes)
