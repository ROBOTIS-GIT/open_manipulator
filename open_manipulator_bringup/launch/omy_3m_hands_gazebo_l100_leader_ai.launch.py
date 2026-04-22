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
# Author: Wonho Yoon, Sungho Woo, Woojin Wie
#
# Composite launch: Gazebo OMY 3M + hands (follower arm mirrors L100 leader joint trajectories).

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    open_manipulator_description_path = os.path.join(
        get_package_share_directory('open_manipulator_description')
    )

    open_manipulator_bringup_path = os.path.join(
        get_package_share_directory('open_manipulator_bringup')
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(open_manipulator_bringup_path, 'worlds'),
            ':' + str(Path(open_manipulator_description_path).parent.resolve()),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 1', ' -r'])
        ],
    )

    xacro_file = os.path.join(
        open_manipulator_description_path,
        'urdf',
        'omy_3m',
        'omy_3m_hands.urdf.xacro',
    )

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params, {'use_sim_time': True}],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            robot_desc,
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0',
            '-R',
            '0.0',
            '-P',
            '0.0',
            '-Y',
            '0.0',
            '-name',
            'omy_3m_hands',
            '-allow_renaming',
            'true',
            '-use_sim',
            'true',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-ros-args',
            '-r /arm_controller/joint_trajectory:=/omy_3m_follower_arm/joint_trajectory',
        ],
        output='screen',
        parameters=[params],
    )

    hx5_right_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hx5_right_hand_controller'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omy_3m_hands_gazebo_l100_leader_ai',
        'initial_positions.yaml',
    ])

    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[trajectory_params_file, {'use_sim_time': True}],
        output='screen',
    )

    leader_traj_bridge = Node(
        package='open_manipulator_bringup',
        executable='omy_3m_hands_leader_trajectory_bridge',
        parameters=[{'use_sim_time': True}],
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
            ('port_name', LaunchConfiguration('port_name')),
            ('use_self_collision_avoidance', LaunchConfiguration('use_self_collision_avoidance')),
            ('use_sim', LaunchConfiguration('leader_use_sim')),
        ],
    )

    delay_executor_after_gazebo_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hx5_right_hand_controller_spawner,
            on_exit=[joint_trajectory_executor, leader_traj_bridge],
        )
    )

    delay_leader_after_init_poses = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_executor,
            on_exit=[leader_launch],
        )
    )

    declared_arguments = [
        DeclareLaunchArgument(
            'world', default_value='empty_world', description='Gz sim World'
        ),
        DeclareLaunchArgument(
            'port_name',
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
            description='If true, pass use_sim:=true to omy_l100_leader_ai (simulated leader).',
        ),
    ]

    return LaunchDescription(
        declared_arguments
        + [
            delay_executor_after_gazebo_controllers,
            delay_leader_after_init_poses,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=arm_controller_spawner,
                    on_exit=[hx5_right_hand_controller_spawner],
                )
            ),
            bridge,
            gazebo_resource_path,
            gazebo,
            node_robot_state_publisher,
            gz_spawn_entity,
        ]
    )
