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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument('start_rviz', default_value='true', description='Whether to execute rviz2'),
        DeclareLaunchArgument('prefix', default_value='""', description='Prefix of the joint and link names'),
        DeclareLaunchArgument('use_sim', default_value='false', description='Start robot in Gazebo simulation.'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Use fake hardware mirroring command.'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false', description='Enable fake sensor commands.'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0', description='Port name for hardware connection.'),
        DeclareLaunchArgument('run_init_position', default_value='true', description='Run init_position.py after launch')
    ]

    # Launch configurations
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    run_init_position = LaunchConfiguration('run_init_position')

    # Generate URDF file using xacro
    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('open_manipulator_description'), 'urdf', 'om_y', 'open_manipulator_y.urdf.xacro']),
        ' ',
        'prefix:=', prefix,
        ' ',
        'use_sim:=', use_sim,
        ' ',
        'use_fake_hardware:=', use_fake_hardware,
        ' ',
        'fake_sensor_commands:=', fake_sensor_commands,
        ' ',
        'port_name:=', port_name,
    ])

    # Paths for configuration files
    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'), 'config', 'om_y_follower', 'hardware_controller_manager.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'), 'rviz', 'open_manipulator.rviz'
    ])

    # Define nodes
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
        remappings=[
            ('/arm_controller/joint_trajectory', '/leader/joint_trajectory')
        ]
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim}],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    # Event handlers to ensure order of execution
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )

    delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner]
        )
    )

    delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner]
        )
    )

    #Timer action to run init_position.py after launch
    init_position_timer = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'open_manipulator_bringup', 'follower_launcher_node.py'],
                output='screen',
                condition=IfCondition(run_init_position)
            )
        ]
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_arm_controller_spawner_after_joint_state_broadcaster_spawner,
            # delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner,
            # init_position_timer
        ]
    )
