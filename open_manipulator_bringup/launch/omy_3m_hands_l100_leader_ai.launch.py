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
# Composite launch: Real OMY 3M follower + real HX5 right hand + L100 leader.
# Gazebo dependencies are intentionally removed.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments for leader hardware.
    leader_port_name = LaunchConfiguration('leader_port_name')
    use_self_collision_avoidance = LaunchConfiguration('use_self_collision_avoidance')
    leader_use_sim = LaunchConfiguration('leader_use_sim')

    # Arguments for right hand hardware.
    hand_port_name = LaunchConfiguration('hand_port_name')
    hand_model = LaunchConfiguration('hand_model')
    hand_init_position = LaunchConfiguration('hand_init_position')
    hand_init_position_file = LaunchConfiguration('hand_init_position_file')

    # Arguments for follower arm hardware.
    follower_start_rviz = LaunchConfiguration('follower_start_rviz')
    follower_use_mock_hardware = LaunchConfiguration('follower_use_mock_hardware')
    follower_mock_sensor_commands = LaunchConfiguration('follower_mock_sensor_commands')
    follower_ros2_control_type = LaunchConfiguration('follower_ros2_control_type')
    follower_init_position_file = LaunchConfiguration('follower_init_position_file')

    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omy_3m.launch.py',
            ]),
        ]),
        launch_arguments=[
            ('start_rviz', follower_start_rviz),
            ('use_sim', 'false'),
            ('use_mock_hardware', follower_use_mock_hardware),
            ('mock_sensor_commands', follower_mock_sensor_commands),
            # Keep follower arm init as a separate executor for deterministic ordering.
            ('init_position', 'false'),
            ('ros2_control_type', follower_ros2_control_type),
        ],
    )

    hand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robotis_hand_bringup'),
                'launch',
                'hx5_d20_right.launch.py',
            ]),
        ]),
        launch_arguments=[
            ('use_sim', 'false'),
            ('port_name', hand_port_name),
            ('model', hand_model),
            ('init_position', hand_init_position),
            ('init_position_file', hand_init_position_file),
        ],
    )

    follower_trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omy_3m_hands_gazebo_l100_leader_ai',
        follower_init_position_file,
    ])

    follower_joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[follower_trajectory_params_file],
        output='screen',
    )

    leader_traj_bridge = Node(
        package='open_manipulator_bringup',
        executable='omy_3m_hands_leader_trajectory_bridge',
        parameters=[
            # OMY 3M follower arm controller is remapped to /leader/joint_trajectory in omy_3m.launch.py.
            {'arm_command_topic': '/leader/joint_trajectory'},
            # HX5 real hardware controller name from hx5_d20_right.launch.py.
            {'hand_command_topic': '/right_hand_controller/joint_trajectory'},
        ],
        output='screen',
    )

    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omy_l100_leader_ai.launch.py',
            ]),
        ]),
        launch_arguments=[
            ('port_name', leader_port_name),
            ('use_self_collision_avoidance', use_self_collision_avoidance),
            ('use_sim', leader_use_sim),
        ],
    )

    delay_leader_after_follower_init = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=follower_joint_trajectory_executor,
            on_exit=[leader_launch],
        )
    )

    declared_arguments = [
        DeclareLaunchArgument(
            'leader_port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port for OMY L100 leader hardware.',
        ),
        DeclareLaunchArgument(
            'use_self_collision_avoidance',
            default_value='true',
            description='Whether to launch self-collision detection on the leader.',
        ),
        DeclareLaunchArgument(
            'leader_use_sim',
            default_value='false',
            description='If true, pass use_sim:=true to omy_l100_leader_ai.launch.py.',
        ),
        DeclareLaunchArgument(
            'hand_port_name',
            default_value='/dev/ttyUSB1',
            description='Serial port for HX5 D20 right hand hardware.',
        ),
        DeclareLaunchArgument(
            'hand_model',
            default_value='hx5_d20_rev2',
            description='HX5 right hand model.',
        ),
        DeclareLaunchArgument(
            'hand_init_position',
            default_value='true',
            description='Whether to move right hand to initial positions on startup.',
        ),
        DeclareLaunchArgument(
            'hand_init_position_file',
            default_value='hx5_d20_right_initial_positions.yaml',
            description='Initial position file from robotis_hand_bringup/config.',
        ),
        DeclareLaunchArgument(
            'follower_start_rviz',
            default_value='false',
            description='Whether to run RViz from the follower arm launch.',
        ),
        DeclareLaunchArgument(
            'follower_use_mock_hardware',
            default_value='false',
            description='Use mock hardware for the follower arm.',
        ),
        DeclareLaunchArgument(
            'follower_mock_sensor_commands',
            default_value='false',
            description='Enable mock sensor commands for the follower arm.',
        ),
        DeclareLaunchArgument(
            'follower_ros2_control_type',
            default_value='omy_3m_position',
            description='ros2_control type used by follower arm.',
        ),
        DeclareLaunchArgument(
            'follower_init_position_file',
            default_value='initial_positions.yaml',
            description='Initial pose file from open_manipulator_bringup/config/omy_3m_hands_gazebo_l100_leader_ai.',
        ),
    ]

    return LaunchDescription(
        declared_arguments
        + [
            delay_leader_after_follower_init,
            follower_launch,
            hand_launch,
            follower_joint_trajectory_executor,
            leader_traj_bridge,
        ]
    )
