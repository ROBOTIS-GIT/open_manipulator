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
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether execute rviz2'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Enable fake command interfaces for sensors used for simple simulations. \
            Used only if "use_fake_hardware" parameter is true.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='The port name to connect to hardware.'
        )
    )

    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    port_name = LaunchConfiguration('port_name')

    urdf_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('open_manipulator_x_description'),
                    'urdf',
                    'open_manipulator_x_robot.urdf.xacro'
                ]
            ),
            ' ',
            'prefix:=',
            prefix,
            ' ',
            'use_sim:=',
            use_sim,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'fake_sensor_commands:=',
            fake_sensor_commands,
            ' ',
            'port_name:=',
            port_name,
        ]
    )

    controller_manager_config = PathJoinSubstitution(
        [
            FindPackageShare('open_manipulator_x_bringup'),
            'config',
            'hardware_controller_manager.yaml',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('open_manipulator_x_bringup'),
            'rviz',
            'open_manipulator_x.rviz'
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': urdf_file},
            controller_manager_config
        ],
        output="both",
        condition=UnlessCondition(use_sim))

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
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = \
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        )

    delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner = \
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[gripper_controller_spawner],
            )
        )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
