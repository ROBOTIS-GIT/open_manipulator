# Copyright 2026 ROBOTIS CO., LTD.
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
# Author: SeongjinJeong

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # AI mode, auto mode, and place rules are all toggled live from the
    # command window, so this always launches omy_ai_graspnet_node.py with
    # motion enabled -- auto/AI mode are just adjusted from that window
    execute_motion_arg = DeclareLaunchArgument('execute_motion', default_value='true')
    auto_arg = DeclareLaunchArgument('auto', default_value='true')
    auto_pick_delay_sec_arg = DeclareLaunchArgument('auto_pick_delay_sec', default_value='0.0')
    movel_duration_sec_arg = DeclareLaunchArgument('movel_duration_sec', default_value='3.0')
    gripper_close_bias_arg = DeclareLaunchArgument('gripper_close_bias', default_value='0.6')
    top_k_arg = DeclareLaunchArgument('top_k', default_value='50')
    place_enabled_arg = DeclareLaunchArgument('place_enabled', default_value='true')

    omy_graspnet = Node(
        package='open_manipulator_playground',
        executable='omy_ai_graspnet_node.py',
        name='omy_graspnet',
        output='screen',
        parameters=[{
            'execute_motion': LaunchConfiguration('execute_motion'),
            'auto': LaunchConfiguration('auto'),
            'auto_pick_delay_sec': LaunchConfiguration('auto_pick_delay_sec'),
            'movel_duration_sec': LaunchConfiguration('movel_duration_sec'),
            'gripper_close_bias': LaunchConfiguration('gripper_close_bias'),
            'top_k': LaunchConfiguration('top_k'),
            'place_enabled': LaunchConfiguration('place_enabled'),
        }]
    )

    return LaunchDescription([
        execute_motion_arg,
        auto_arg,
        auto_pick_delay_sec_arg,
        movel_duration_sec_arg,
        gripper_close_bias_arg,
        top_k_arg,
        place_enabled_arg,
        omy_graspnet,
    ])
