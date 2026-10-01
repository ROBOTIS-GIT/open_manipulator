# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

FOLLOWER_LEFT_PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_6D5D4B68503059384C2E3120FF05133E-if00'
FOLLOWER_RIGHT_PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_23D2E9915157375037202020FF0F2510-if00'


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
    teleop = LaunchConfiguration('teleop').perform(context) == 'true'
    mappings = {'use_mock_hardware': use_mock_hardware}
    for name in ('left_xyz', 'left_rpy', 'right_xyz', 'right_rpy'):
        mappings[name] = validate_mount(LaunchConfiguration(name).perform(context), name)

    left_port = LaunchConfiguration('left_port').perform(context).strip()
    right_port = LaunchConfiguration('right_port').perform(context).strip()
    validate_ports(left_port, right_port, use_mock_hardware == 'true')
    mappings['left_port'] = left_port or '/dev/omx_follower_left'
    mappings['right_port'] = right_port or '/dev/omx_follower_right'

    description_share = get_package_share_directory('open_manipulator_description')
    bringup_share = get_package_share_directory('open_manipulator_bringup')
    description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'omx_dual', 'omx_dual_follower.urdf.xacro'),
        mappings=mappings,
    ).toxml()

    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='follower',
        parameters=[{'robot_description': ParameterValue(description, value_type=str),
                     'frame_prefix': 'follower/'}],
        output='screen',
    )
    controller_config = 'follower_teleop_controller_manager.yaml' if teleop else 'follower_controller_manager.yaml'
    manager_remappings = [('robot_description', '/follower/robot_description'),
                          ('/collision_flag', '/follower/collision_flag')]
    if teleop:
        manager_remappings += [
            (f'/follower/{side}_arm_controller/joint_trajectory',
             f'/leader/{side}_joint_trajectory_command_broadcaster/joint_trajectory')
            for side in ('left', 'right')
        ]
    manager = Node(
        package='controller_manager', executable='ros2_control_node',
        namespace='follower',
        parameters=[os.path.join(
            bringup_share, 'config', 'omx_dual', controller_config)],
        remappings=manager_remappings,
        output='screen',
    )
    broadcaster = Node(
        package='controller_manager', executable='spawner',
        namespace='follower',
        arguments=['joint_state_broadcaster', '-c', '/follower/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )
    controller_names = ['left_arm_controller', 'right_arm_controller']
    if not teleop:
        controller_names += ['left_gripper_controller', 'right_gripper_controller']
    controllers = Node(
        name='follower_controller_spawner',
        package='controller_manager', executable='spawner',
        namespace='follower',
        arguments=[
            *controller_names,
            '-c', '/follower/controller_manager', '--controller-manager-timeout', '30',
            '--activate-as-group',
        ],
        output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        namespace='follower',
        arguments=['-d', os.path.join(description_share, 'rviz', 'omx_dual_follower.rviz')],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        output='screen',
    )
    initial_positions_file = os.path.join(
        bringup_share, 'config', 'omx_dual',
        LaunchConfiguration('init_position_file').perform(context),
    )
    initializers = [
        Node(
            package='open_manipulator_bringup', executable='joint_trajectory_executor',
            name=f'{side}_init_position', namespace='follower',
            parameters=[initial_positions_file, {'wait_for_result': True}],
            condition=IfCondition(LaunchConfiguration('init_position')),
            output='screen',
        )
        for side in ('left', 'right')
    ]

    def after_broadcaster(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Joint state broadcaster failed'))]
        return [controllers]

    def after_initializer(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Follower initial pose failed'))]
        return []

    def after_controllers(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Dual-arm controller startup failed'))]
        return [rviz, *initializers]

    return [
        *[RegisterEventHandler(OnProcessExit(target_action=node, on_exit=after_initializer))
          for node in initializers],
        RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=[
            EmitEvent(event=Shutdown(reason='Controller manager exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=after_broadcaster)),
        RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=after_controllers)),
        state_publisher, manager, broadcaster,
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_mock_hardware',
                              default_value='false',
                              choices=['true', 'false'],
                              description='Set true for a mock preview without serial access'),
        DeclareLaunchArgument('start_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('left_port',
                              default_value=FOLLOWER_LEFT_PORT,
                              description='Persistent serial device path for the left arm'),
        DeclareLaunchArgument('right_port',
                              default_value=FOLLOWER_RIGHT_PORT,
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
    arguments += [
        DeclareLaunchArgument('teleop', default_value='false', choices=['true', 'false'],
                              description='Use six-joint arm controllers connected to leader streams'),
        DeclareLaunchArgument('init_position', default_value='true', choices=['true', 'false'],
                              description='Move both arms through the stock OMX initial poses'),
        DeclareLaunchArgument('init_position_file', default_value='follower_initial_positions.yaml',
                              description='Initial pose YAML in config/omx_dual, or an absolute path'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
