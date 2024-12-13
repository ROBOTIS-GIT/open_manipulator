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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    csv_file_path = os.path.join(
        get_package_share_directory('open_manipulator_x_gui'),
        'config', 'robot_joint_log.csv')

    gui_node = Node(
        package='open_manipulator_x_gui',
        executable='open_manipulator_x_gui_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'csv_path': csv_file_path}
        ]
    )

    return LaunchDescription([
        gui_node
    ])
