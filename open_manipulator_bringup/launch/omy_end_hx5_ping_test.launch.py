#!/usr/bin/env python3
"""
Minimal ping test: PC -> U2D2 -> OMY END (ID 210) -> HX5 controller (ID 140).

Verifies that OMY END is reachable and can relay SyncTable data from HX5.

Usage:
  ros2 launch open_manipulator_bringup omy_end_hx5_ping_test.launch.py
  ros2 launch open_manipulator_bringup omy_end_hx5_ping_test.launch.py port_name:=/dev/ttyAMA4

After launch, verify with:
  # 1) Check OMY END is alive (Button Status)
  ros2 service call /dynamixel_hardware_interface/get_dxl_data \
      dynamixel_interfaces/srv/GetDataFromDxl \
      "{id: 210, item_name: 'Button Status', timeout_sec: 2.0}"

  # 2) Read HX5 data relayed via SyncTable (first 6 bytes at OMY END addr 72)
  ros2 service call /dynamixel_hardware_interface/get_dxl_data \
      dynamixel_interfaces/srv/GetDataFromDxl \
      "{id: 210, item_name: 'Table Sync Read Data', timeout_sec: 2.0}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
                'omy_end_hx5_ping_test.urdf.xacro',
            ]),
            ' port_name:=', port_name,
            ' use_mock_hardware:=', use_mock_hardware,
        ]),
        value_type=str,
    )

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omy_end_hx5_ping_test',
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

    enable_synctable_process = ExecuteProcess(
        name='enable_synctable',
        cmd=[
            'bash', '-c',
            'sleep 3 && '
            'echo "[enable_synctable] Activating SyncTable Enable..." && '
            'ros2 service call /dynamixel_hardware_interface/set_dxl_data '
            'dynamixel_interfaces/srv/SetDataToDxl '
            '"{id: 210, item_name: \'SyncTable Enable\', item_data: 1}" && '
            'sleep 1 && '
            'echo "[enable_synctable] Activating SyncTable Enable HX5..." && '
            'ros2 service call /dynamixel_hardware_interface/set_dxl_data '
            'dynamixel_interfaces/srv/SetDataToDxl '
            '"{id: 210, item_name: \'SyncTable Enable HX5\', item_data: 1}" && '
            'echo "[enable_synctable] Done."',
        ],
        output='both',
    )

    delay_enable_synctable = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner,
            on_exit=[enable_synctable_process],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            control_node,
            controller_spawner,
            delay_enable_synctable,
        ]
    )
