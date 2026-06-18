#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0
#
# One-shot launch for OMX-F hand teleop:
# bringup + cyclo MoveL controller + local trajectory relay + HTTP/WebSocket bridge.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    http_port = LaunchConfiguration('http_port')
    enable_websocket = LaunchConfiguration('enable_websocket')
    publish_rate = LaunchConfiguration('publish_rate')
    publish_epsilon = LaunchConfiguration('publish_epsilon')
    time_from_start = LaunchConfiguration('time_from_start')
    stale_timeout = LaunchConfiguration('stale_timeout')
    control_mode = LaunchConfiguration('control_mode')
    auto_enable_on_input = LaunchConfiguration('auto_enable_on_input')
    delta_policy = LaunchConfiguration('delta_policy')
    delta_deadband = LaunchConfiguration('delta_deadband')
    absolute_smoothing_alpha = LaunchConfiguration('absolute_smoothing_alpha')
    absolute_max_step = LaunchConfiguration('absolute_max_step')
    absolute_min_confidence = LaunchConfiguration('absolute_min_confidence')
    absolute_input_max_step = LaunchConfiguration('absolute_input_max_step')
    absolute_input_reacquire_after = LaunchConfiguration('absolute_input_reacquire_after')
    absolute_input_reacquire_step = LaunchConfiguration('absolute_input_reacquire_step')
    interpolation_enabled = LaunchConfiguration('interpolation_enabled')
    interpolation_time_constant = LaunchConfiguration('interpolation_time_constant')
    interpolation_max_speed = LaunchConfiguration('interpolation_max_speed')
    interpolation_arrival_epsilon = LaunchConfiguration('interpolation_arrival_epsilon')
    input_deadzone = LaunchConfiguration('input_deadzone')
    relay_max_rate = LaunchConfiguration('relay_max_rate')
    relay_min_position_delta = LaunchConfiguration('relay_min_position_delta')
    relay_time_from_start = LaunchConfiguration('relay_time_from_start')
    scale_x = LaunchConfiguration('scale_x')
    scale_y = LaunchConfiguration('scale_y')
    scale_z = LaunchConfiguration('scale_z')
    x_min = LaunchConfiguration('x_min')
    x_max = LaunchConfiguration('x_max')
    y_min = LaunchConfiguration('y_min')
    y_max = LaunchConfiguration('y_max')
    z_min = LaunchConfiguration('z_min')
    z_max = LaunchConfiguration('z_max')
    port_name = LaunchConfiguration('port_name')
    start_rviz = LaunchConfiguration('start_rviz')
    cyclo_delay = LaunchConfiguration('cyclo_delay')
    bridge_delay = LaunchConfiguration('bridge_delay')
    skill_pose_config = LaunchConfiguration('skill_pose_config')
    cyclo_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_playground'),
        'config',
        'omx_movel_stable_config.yaml',
    ])

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omx_f.launch.py',
            ])
        ]),
        launch_arguments={
            'port_name': port_name,
            'start_rviz': start_rviz,
            'init_position': 'false',
        }.items(),
    )

    cyclo_movel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cyclo_motion_controller_ros'),
                'launch',
                'omx_controller.launch.py',
            ])
        ]),
        launch_arguments={
            'controller_type': 'movel',
            'config_file': cyclo_config,
        }.items(),
    )

    control = Node(
        package='open_manipulator_playground',
        executable='main.py',
        output='screen',
        parameters=[{
            'http_port': http_port,
            'enable_websocket': ParameterValue(enable_websocket, value_type=bool),
            'publish_rate': ParameterValue(publish_rate, value_type=float),
            'publish_epsilon': ParameterValue(publish_epsilon, value_type=float),
            'time_from_start': ParameterValue(time_from_start, value_type=float),
            'stale_timeout': ParameterValue(stale_timeout, value_type=float),
            'control_mode': control_mode,
            'auto_enable_on_input': ParameterValue(auto_enable_on_input, value_type=bool),
            'delta_policy': delta_policy,
            'delta_deadband': ParameterValue(delta_deadband, value_type=float),
            'absolute_smoothing_alpha': ParameterValue(
                absolute_smoothing_alpha, value_type=float),
            'absolute_max_step': ParameterValue(absolute_max_step, value_type=float),
            'absolute_min_confidence': ParameterValue(
                absolute_min_confidence, value_type=float),
            'absolute_input_max_step': ParameterValue(
                absolute_input_max_step, value_type=float),
            'absolute_input_reacquire_after': ParameterValue(
                absolute_input_reacquire_after, value_type=float),
            'absolute_input_reacquire_step': ParameterValue(
                absolute_input_reacquire_step, value_type=float),
            'interpolation_enabled': ParameterValue(
                interpolation_enabled, value_type=bool),
            'interpolation_time_constant': ParameterValue(
                interpolation_time_constant, value_type=float),
            'interpolation_max_speed': ParameterValue(
                interpolation_max_speed, value_type=float),
            'interpolation_arrival_epsilon': ParameterValue(
                interpolation_arrival_epsilon, value_type=float),
            'input_deadzone': ParameterValue(input_deadzone, value_type=float),
            'max_rate_hz': ParameterValue(relay_max_rate, value_type=float),
            'min_position_delta': ParameterValue(relay_min_position_delta, value_type=float),
            'output_time_from_start': ParameterValue(relay_time_from_start, value_type=float),
            'scale_x': ParameterValue(scale_x, value_type=float),
            'scale_y': ParameterValue(scale_y, value_type=float),
            'scale_z': ParameterValue(scale_z, value_type=float),
            'x_min': ParameterValue(x_min, value_type=float),
            'x_max': ParameterValue(x_max, value_type=float),
            'y_min': ParameterValue(y_min, value_type=float),
            'y_max': ParameterValue(y_max, value_type=float),
            'z_min': ParameterValue(z_min, value_type=float),
            'z_max': ParameterValue(z_max, value_type=float),
            'skill_pose_config': skill_pose_config,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='18001'),
        DeclareLaunchArgument('enable_websocket', default_value='true'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('publish_epsilon', default_value='0.0015'),
        DeclareLaunchArgument('time_from_start', default_value='0.20'),
        DeclareLaunchArgument('stale_timeout', default_value='0.35'),
        DeclareLaunchArgument('control_mode', default_value='absolute'),
        DeclareLaunchArgument('auto_enable_on_input', default_value='true'),
        DeclareLaunchArgument('delta_policy', default_value='latest'),
        DeclareLaunchArgument('delta_deadband', default_value='0.0015'),
        DeclareLaunchArgument('absolute_smoothing_alpha', default_value='0.30'),
        DeclareLaunchArgument('absolute_max_step', default_value='0.008'),
        DeclareLaunchArgument('absolute_min_confidence', default_value='0.0'),
        DeclareLaunchArgument('absolute_input_max_step', default_value='0.025'),
        DeclareLaunchArgument('absolute_input_reacquire_after', default_value='0.25'),
        DeclareLaunchArgument('absolute_input_reacquire_step', default_value='0.008'),
        DeclareLaunchArgument('interpolation_enabled', default_value='true'),
        DeclareLaunchArgument('interpolation_time_constant', default_value='0.25'),
        DeclareLaunchArgument('interpolation_max_speed', default_value='0.10'),
        DeclareLaunchArgument('interpolation_arrival_epsilon', default_value='0.0015'),
        DeclareLaunchArgument('input_deadzone', default_value='0.015'),
        DeclareLaunchArgument('relay_max_rate', default_value='40.0'),
        DeclareLaunchArgument('relay_min_position_delta', default_value='0.0010'),
        DeclareLaunchArgument('relay_time_from_start', default_value='0.06'),
        DeclareLaunchArgument('scale_x', default_value='1.4'),
        DeclareLaunchArgument('scale_y', default_value='1.4'),
        DeclareLaunchArgument('scale_z', default_value='1.2'),
        DeclareLaunchArgument('x_min', default_value='0.08'),
        DeclareLaunchArgument('x_max', default_value='0.28'),
        DeclareLaunchArgument('y_min', default_value='-0.16'),
        DeclareLaunchArgument('y_max', default_value='0.16'),
        DeclareLaunchArgument('z_min', default_value='0.05'),
        DeclareLaunchArgument('z_max', default_value='0.28'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'skill_pose_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('open_manipulator_playground'),
                'config',
                'omx_skill_poses.yaml',
            ]),
            description='YAML pose book for high-level skills.',
        ),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('cyclo_delay', default_value='20.0'),
        DeclareLaunchArgument('bridge_delay', default_value='5.0'),
        bringup,
        TimerAction(period=cyclo_delay, actions=[cyclo_movel]),
        TimerAction(period=bridge_delay, actions=[control]),
    ])
