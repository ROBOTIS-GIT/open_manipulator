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
# Gravity-compensation bringup for the ceiling-mounted inverted_omy_l100
# robot (link lengths unchanged, unlike custom_inverted_omy_l100). Runs the
# arm hand-backdrivable (effort control) and mirrors it live in rviz.

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
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'robot_ns',
            default_value='unit1',
            description=(
                'ROS namespace this unit runs under (controller_manager, '
                'joint_state_broadcaster, /robot_description, rviz2, etc.). '
                'Must be unique per unit when running multiple OMY-L100s at '
                'the same time, or their nodes/topics/services collide.'
            ),
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            description='Enable mock sensor commands.',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Port name for hardware connection.',
        ),
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='omy_l100_current',
            description='Type of ros2_control',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    robot_ns = LaunchConfiguration('robot_ns')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    ros2_control_type = LaunchConfiguration('ros2_control_type')

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf',
            'omy_l100',
            'inverted_omy_l100.urdf.xacro',
        ]),
        ' ',
        'prefix:=',
        prefix,
        ' ',
        'use_sim:=',
        use_sim,
        ' ',
        'use_mock_hardware:=',
        use_mock_hardware,
        ' ',
        'mock_sensor_commands:=',
        mock_sensor_commands,
        ' ',
        'port_name:=',
        port_name,
        ' ',
        'ros2_control_type:=',
        ros2_control_type,
    ])

    # Reuse the follower_ai gravity controller config (joint names are
    # unchanged, only the base orientation was flipped)
    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omy_l100_follower_ai',
        'gravity_controller_manager.yaml',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz',
        'open_manipulator.rviz',
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot_ns,
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns,
        arguments=[
            'gravity_compensation_controller',
            'spring_actuator_controller',
            'joint_state_broadcaster',
        ],
        output='both',
        parameters=[{'robot_description': urdf_file}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_ns,
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim}],
        # tf2_ros::TransformBroadcaster publishes to the absolute "/tf" and
        # "/tf_static" topics regardless of node namespace, so multiple units
        # sharing identical frame_ids (no xacro prefix) would otherwise
        # collide on one global TF tree. Remap explicitly, same pattern as
        # /robot_description below.
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='both',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=robot_ns,
        arguments=['-d', rviz_config_file],
        # open_manipulator.rviz hardcodes absolute "/robot_description", "/tf",
        # and "/tf_static" topics (leading slash), so pushing a namespace alone
        # won't redirect them — they must be remapped explicitly to land on
        # this unit's own publishers.
        remappings=[
            ('/robot_description', 'robot_description'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='both',
        condition=IfCondition(start_rviz),
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner, on_exit=[rviz_node]
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
        ]
    )
