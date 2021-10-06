#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Author: Ryan Shim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Parameters
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')
    baud_rate = LaunchConfiguration('baud_rate', default='1000000')
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('open_manipulator_x_controller'),
            'param',
            'open_manipulator_x_controller_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port'),

        DeclareLaunchArgument(
           'baud_rate',
           default_value=baud_rate,
           description='Set Baudrate'),

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Specifying parameter direction'),

        Node(
            package='open_manipulator_x_controller',
            executable='open_manipulator_x_controller',
            name='open_manipulator_x_controller',
            arguments=[usb_port, baud_rate],
            parameters=[param_dir],
            output='screen'),
    ])
