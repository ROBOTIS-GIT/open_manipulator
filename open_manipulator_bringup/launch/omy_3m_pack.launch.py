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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('open_manipulator_bringup')

    # Path to the omy_3m.launch.py file
    omy_3m_launch_file = os.path.join(pkg_share, 'launch', 'omy_3m.launch.py')

    # Include the omy_3m.launch.py with pack parameters
    omy_3m_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([omy_3m_launch_file]),
        launch_arguments={
            'init_position': 'true',
            'init_position_file': 'pack_positions.yaml'
        }.items()
    )

    return LaunchDescription([
        LogInfo(msg='Starting OMY packing...'),
        omy_3m_launch,
    ])
