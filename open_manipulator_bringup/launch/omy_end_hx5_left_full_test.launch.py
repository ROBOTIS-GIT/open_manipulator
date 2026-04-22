#!/usr/bin/env python3
"""
Full integration test: PC -> U2D2 -> OMY END (ID 210) -> HX5 (ID 140)
with all 20 finger joints + 5 pressure sensors via SyncTable relay.

omy_end_hx5.model has no Indirect Address entries, forcing the plugin to use
direct BulkRead/BulkWrite on contiguous Table Sync Read/Write Data areas.

Usage:
  ros2 launch open_manipulator_bringup omy_end_hx5_left_full_test.launch.py
  ros2 launch open_manipulator_bringup omy_end_hx5_left_full_test.launch.py port_name:=/dev/ttyAMA4

After launch, verify with:
  ros2 topic echo /dynamic_joint_states
  ros2 topic echo /joint_states
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port for OMY END (U2D2).',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware (no physical device needed).',
        ),
    ]

    port_name = LaunchConfiguration('port_name')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    urdf = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_description'),
                'urdf', 'omy_f3m',
                'omy_end_hx5_left_standalone.urdf.xacro',
            ]),
            ' port_name:=', port_name,
            ' use_mock_hardware:=', use_mock_hardware,
        ]),
        value_type=str,
    )

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omy_end_hx5_left_full_test',
        'hardware_controller_manager.yaml',
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf}],
        output='both',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_manager_config],
        output='both',
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='both',
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            control_node,
            controller_spawner,
        ]
    )
