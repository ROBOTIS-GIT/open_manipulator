# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

LEADER_LEFT_PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_84BB894D5157375037202020FF102815-if00'
LEADER_RIGHT_PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_895EBFC8503059384C2E3120FF08031C-if00'


def validate_mount(value, name):
    """Reject invalid transforms before starting the hardware process."""
    try:
        numbers = [float(part) for part in value.split()]
    except ValueError as error:
        raise ValueError(f'{name} must contain three finite numbers') from error
    if len(numbers) != 3 or not all(math.isfinite(number) for number in numbers):
        raise ValueError(f'{name} must contain three finite numbers')
    return ' '.join(str(number) for number in numbers)


def validate_ports(left_port, right_port, use_mock_hardware):
    if not use_mock_hardware:
        if not left_port or not right_port:
            raise ValueError('Real hardware requires non-empty left_port and right_port')
        if os.path.realpath(left_port) == os.path.realpath(right_port):
            raise ValueError('left_port and right_port must identify different USB devices')


def launch_setup(context):
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context)
    mappings = {'use_mock_hardware': use_mock_hardware}
    for name in ('left_xyz', 'left_rpy', 'right_xyz', 'right_rpy'):
        mappings[name] = validate_mount(LaunchConfiguration(name).perform(context), name)

    left_port = LaunchConfiguration('left_port').perform(context).strip()
    right_port = LaunchConfiguration('right_port').perform(context).strip()
    validate_ports(left_port, right_port, use_mock_hardware == 'true')
    mappings['left_port'] = left_port or '/dev/omx_leader_left'
    mappings['right_port'] = right_port or '/dev/omx_leader_right'

    description_share = get_package_share_directory('open_manipulator_description')
    bringup_share = get_package_share_directory('open_manipulator_bringup')
    description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'omx_dual', 'omx_dual_leader.urdf.xacro'),
        mappings=mappings,
    ).toxml()

    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='leader',
        parameters=[{'robot_description': ParameterValue(description, value_type=str),
                     'frame_prefix': 'leader/'}],
        output='screen',
    )
    controller_config = 'leader_controller_manager.yaml'
    manager_remappings = [('robot_description', '/leader/robot_description'),
                          ('/collision_flag', '/leader/collision_flag')]
    manager = Node(
        package='controller_manager', executable='ros2_control_node',
        namespace='leader',
        parameters=[os.path.join(
            bringup_share, 'config', 'omx_dual', controller_config)],
        remappings=manager_remappings,
        output='screen',
    )
    broadcaster = Node(
        package='controller_manager', executable='spawner',
        namespace='leader',
        arguments=['joint_state_broadcaster', '-c', '/leader/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )
    controller_names = [
        'left_trigger_position_controller', 'right_trigger_position_controller',
        'left_joint_trajectory_command_broadcaster', 'right_joint_trajectory_command_broadcaster',
    ]
    controllers = Node(
        package='controller_manager', executable='spawner',
        namespace='leader',
        arguments=[
            *controller_names,
            '-c', '/leader/controller_manager', '--controller-manager-timeout', '30',
            '--activate-as-group',
        ],
        output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        namespace='leader',
        arguments=['-d', os.path.join(description_share, 'rviz', 'omx_dual_leader.rviz')],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        output='screen',
    )
    # Stock OMX leader trigger preload. Arm joints remain state-only.
    initializers = [
        ExecuteProcess(
            name=f'{side}_trigger_preload',
            cmd=['ros2', 'topic', 'pub', '--once',
                 f'/leader/{side}_trigger_position_controller/commands',
                 'std_msgs/msg/Float64MultiArray', 'data: [-0.7]'],
            condition=IfCondition(LaunchConfiguration('trigger_preload')),
            output='screen',
        )
        for side in ('left', 'right')
    ]

    def after_broadcaster(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Joint state broadcaster failed'))]
        return [controllers]

    def after_controllers(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Dual-arm controller startup failed'))]
        return [rviz, *initializers]

    return [
        RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=[
            EmitEvent(event=Shutdown(reason='Controller manager exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=after_broadcaster)),
        RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=after_controllers)),
        state_publisher, manager, broadcaster,
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_mock_hardware',
                              default_value='true',
                              choices=['true', 'false'],
                              description='Set true for a mock preview without serial access'),
        DeclareLaunchArgument('start_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('left_port',
                              default_value=LEADER_LEFT_PORT,
                              description='Persistent serial device path for the left arm'),
        DeclareLaunchArgument('right_port',
                              default_value=LEADER_RIGHT_PORT,
                              description='Persistent serial device path for the right arm'),
        DeclareLaunchArgument('left_xyz', default_value='0 0.184 0',
                              description='Left link0 position in workcell_base, metres'),
        DeclareLaunchArgument('left_rpy', default_value='0 0 0',
                              description='Left link0 roll pitch yaw, radians'),
        DeclareLaunchArgument('right_xyz', default_value='0 -0.184 0',
                              description='Right link0 position in workcell_base, metres'),
        DeclareLaunchArgument('right_rpy', default_value='0 0 0',
                              description='Right link0 roll pitch yaw, radians'),
    ]
    arguments += [DeclareLaunchArgument(
        'trigger_preload', default_value='true', choices=['true', 'false'],
        description='Apply the stock -0.7 rad trigger command after activation')]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
