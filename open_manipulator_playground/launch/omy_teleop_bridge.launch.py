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
# Launch the OMY teleop receiver: http_bridge (HTTP -> /omy/task_goal) +
# teleop_executor (/omy/task_goal -> arm/gripper controllers). Optionally include
# the existing omy_f3m bringup.
#
# Usage:
#   # receiver only (bringup already running)
#   ros2 launch open_manipulator_playground omy_teleop_bridge.launch.py
#
#   # include omy_f3m bringup too
#   ros2 launch open_manipulator_playground omy_teleop_bridge.launch.py bringup:=true
#
#   # different HTTP port (e.g. if 8000 is taken)
#   ros2 launch open_manipulator_playground omy_teleop_bridge.launch.py http_port:=8010

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared = [
        DeclareLaunchArgument('bringup', default_value='false',
                              description='Include omy_f3m bringup'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='Use mock hardware for offline testing'),
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution([
                FindPackageShare('open_manipulator_playground'),
                'config', 'omy_teleop_config.yaml']),
            description='Path to omy_teleop_config.yaml'),
        DeclareLaunchArgument('arm_topic', default_value='/arm_controller/joint_trajectory'),
        DeclareLaunchArgument('gripper_action', default_value='/gripper_controller/gripper_cmd'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('http_bridge', default_value='true',
                              description='Run HTTP->ROS bridge for Reachy'),
        DeclareLaunchArgument('http_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('http_path', default_value='/omy/task'),
    ]

    bringup = LaunchConfiguration('bringup')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    config = LaunchConfiguration('config')
    arm_topic = LaunchConfiguration('arm_topic')
    gripper_action = LaunchConfiguration('gripper_action')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    http_bridge = LaunchConfiguration('http_bridge')
    http_host = LaunchConfiguration('http_host')
    http_port = LaunchConfiguration('http_port')
    http_path = LaunchConfiguration('http_path')

    omy_f3m_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch', 'omy_f3m.launch.py'])),
        launch_arguments={'use_mock_hardware': use_mock_hardware}.items(),
        condition=IfCondition(bringup),
    )

    teleop_executor = Node(
        package='open_manipulator_playground',
        executable='omy_teleop_executor',
        name='omy_teleop_executor',
        output='screen',
        parameters=[{
            'config': config,
            'arm_topic': arm_topic,
            'gripper_action': gripper_action,
            'joint_states_topic': joint_states_topic,
        }],
    )

    http_bridge_node = Node(
        package='open_manipulator_playground',
        executable='omy_http_bridge',
        name='omy_http_bridge',
        output='screen',
        parameters=[{
            'host': http_host,
            'port': ParameterValue(http_port, value_type=int),
            'path': http_path,
        }],
        condition=IfCondition(http_bridge),
    )

    return LaunchDescription(declared + [omy_f3m_bringup, teleop_executor, http_bridge_node])
