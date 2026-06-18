#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0
#
# One-shot launch for OMY-F3M hand teleop.

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
    scale_x = LaunchConfiguration('scale_x')
    scale_y = LaunchConfiguration('scale_y')
    scale_z = LaunchConfiguration('scale_z')
    input_axis_x = LaunchConfiguration('input_axis_x')
    input_axis_y = LaunchConfiguration('input_axis_y')
    input_axis_z = LaunchConfiguration('input_axis_z')
    input_gain_x = LaunchConfiguration('input_gain_x')
    input_gain_y = LaunchConfiguration('input_gain_y')
    input_gain_z = LaunchConfiguration('input_gain_z')
    input_invert_x = LaunchConfiguration('input_invert_x')
    input_invert_y = LaunchConfiguration('input_invert_y')
    input_invert_z = LaunchConfiguration('input_invert_z')
    x_min = LaunchConfiguration('x_min')
    x_max = LaunchConfiguration('x_max')
    y_min = LaunchConfiguration('y_min')
    y_max = LaunchConfiguration('y_max')
    z_min = LaunchConfiguration('z_min')
    z_max = LaunchConfiguration('z_max')
    start_rviz = LaunchConfiguration('start_rviz')
    cyclo_delay = LaunchConfiguration('cyclo_delay')
    bridge_delay = LaunchConfiguration('bridge_delay')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    init_position = LaunchConfiguration('init_position')
    init_position_file = LaunchConfiguration('init_position_file')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    skill_pose_config = LaunchConfiguration('skill_pose_config')
    gripper_open_position = LaunchConfiguration('gripper_open_position')
    gripper_closed_position = LaunchConfiguration('gripper_closed_position')
    gripper_max_effort = LaunchConfiguration('gripper_max_effort')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omy_f3m.launch.py',
            ])
        ]),
        launch_arguments={
            'start_rviz': start_rviz,
            'init_position': init_position,
            'init_position_file': init_position_file,
            'ros2_control_type': ros2_control_type,
            'use_mock_hardware': use_mock_hardware,
            'mock_sensor_commands': mock_sensor_commands,
        }.items(),
    )

    cyclo_movel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cyclo_motion_controller_ros'),
                'launch',
                'omy_controller.launch.py',
            ])
        ]),
        launch_arguments={
            'controller_type': 'movel',
        }.items(),
    )

    control = Node(
        package='open_manipulator_playground',
        executable='main.py',
        output='screen',
        parameters=[{
            'movel_topic': '/omy_movel_controller/movel',
            'current_pose_topic': '/omy_movel_controller/current_pose',
            'gripper_action': '/gripper_controller/gripper_cmd',
            # Inverted gripper vs OMX (open: 0.0, closed: 1.1).
            'gripper_open_position': ParameterValue(
                gripper_open_position, value_type=float),
            'gripper_closed_position': ParameterValue(
                gripper_closed_position, value_type=float),
            'gripper_max_effort': ParameterValue(gripper_max_effort, value_type=float),
            'input_topic': '/leader/joint_trajectory',
            'output_topic': '/arm_controller/joint_trajectory',
            'http_port': ParameterValue(http_port, value_type=int),
            'enable_websocket': ParameterValue(enable_websocket, value_type=bool),
            'publish_rate': ParameterValue(publish_rate, value_type=float),
            'time_from_start': ParameterValue(time_from_start, value_type=float),
            'stale_timeout': ParameterValue(stale_timeout, value_type=float),
            'control_mode': ParameterValue(control_mode, value_type=str),
            'auto_enable_on_input': ParameterValue(auto_enable_on_input, value_type=bool),
            'delta_policy': ParameterValue(delta_policy, value_type=str),
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
            'scale_x': ParameterValue(scale_x, value_type=float),
            'scale_y': ParameterValue(scale_y, value_type=float),
            'scale_z': ParameterValue(scale_z, value_type=float),
            # Remap axes due to 90 deg base frame rotation.
            'input_axis_x': ParameterValue(input_axis_x, value_type=str),
            'input_axis_y': ParameterValue(input_axis_y, value_type=str),
            'input_axis_z': ParameterValue(input_axis_z, value_type=str),
            'input_gain_x': ParameterValue(input_gain_x, value_type=float),
            'input_gain_y': ParameterValue(input_gain_y, value_type=float),
            'input_gain_z': ParameterValue(input_gain_z, value_type=float),
            'input_invert_x': ParameterValue(input_invert_x, value_type=bool),
            'input_invert_y': ParameterValue(input_invert_y, value_type=bool),
            'input_invert_z': ParameterValue(input_invert_z, value_type=bool),
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
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        DeclareLaunchArgument('time_from_start', default_value='0.08'),
        DeclareLaunchArgument('stale_timeout', default_value='0.35'),
        DeclareLaunchArgument('control_mode', default_value='absolute'),
        # Wait for explicit enable from client.
        DeclareLaunchArgument('auto_enable_on_input', default_value='false'),
        DeclareLaunchArgument('delta_policy', default_value='latest'),
        DeclareLaunchArgument('delta_deadband', default_value='0.0002'),
        DeclareLaunchArgument('absolute_smoothing_alpha', default_value='0.65'),
        DeclareLaunchArgument('absolute_max_step', default_value='0.02'),
        DeclareLaunchArgument('absolute_min_confidence', default_value='0.0'),
        # Larger step and tracking speed for larger reach.
        DeclareLaunchArgument('absolute_input_max_step', default_value='0.035'),
        DeclareLaunchArgument('absolute_input_reacquire_after', default_value='0.25'),
        DeclareLaunchArgument('absolute_input_reacquire_step', default_value='0.012'),
        DeclareLaunchArgument('interpolation_enabled', default_value='true'),
        DeclareLaunchArgument('interpolation_time_constant', default_value='0.12'),
        DeclareLaunchArgument('interpolation_max_speed', default_value='0.24'),
        DeclareLaunchArgument('interpolation_arrival_epsilon', default_value='0.0005'),
        DeclareLaunchArgument('scale_x', default_value='1.4'),
        DeclareLaunchArgument('scale_y', default_value='1.4'),
        DeclareLaunchArgument('scale_z', default_value='1.2'),
        # Reachy hand axis to OMY robot axis remap.
        # robot x <- hand y, robot y <- hand x (flipped), robot z <- hand z.
        DeclareLaunchArgument('input_axis_x', default_value='y'),
        DeclareLaunchArgument('input_axis_y', default_value='x'),
        DeclareLaunchArgument('input_axis_z', default_value='z'),
        DeclareLaunchArgument('input_gain_x', default_value='2.5'),
        DeclareLaunchArgument('input_gain_y', default_value='2.0'),
        DeclareLaunchArgument('input_gain_z', default_value='1.5'),
        DeclareLaunchArgument('input_invert_x', default_value='false'),
        DeclareLaunchArgument('input_invert_y', default_value='true'),
        DeclareLaunchArgument('input_invert_z', default_value='false'),
        # Absolute teleop workspace box in base frame.
        DeclareLaunchArgument('x_min', default_value='-0.28'),
        DeclareLaunchArgument('x_max', default_value='0.28'),
        DeclareLaunchArgument('y_min', default_value='-0.44'),
        DeclareLaunchArgument('y_max', default_value='-0.32'),
        DeclareLaunchArgument('z_min', default_value='0.30'),
        DeclareLaunchArgument('z_max', default_value='0.60'),
        # Gripper convention.
        DeclareLaunchArgument('gripper_open_position', default_value='0.0'),
        DeclareLaunchArgument('gripper_closed_position', default_value='1.1'),
        DeclareLaunchArgument('gripper_max_effort', default_value='10.0'),
        # Keep live hardware pose and seed from it.
        DeclareLaunchArgument('init_position', default_value='false'),
        DeclareLaunchArgument('init_position_file', default_value='initial_positions.yaml'),
        DeclareLaunchArgument('ros2_control_type', default_value='omy_f3m_position'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument(
            'skill_pose_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('open_manipulator_playground'),
                'config',
                'omy_skill_poses.yaml',
            ]),
            description='YAML pose book for high-level skills (OMY-F3M).',
        ),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('cyclo_delay', default_value='20.0'),
        DeclareLaunchArgument('bridge_delay', default_value='5.0'),
        bringup,
        TimerAction(period=cyclo_delay, actions=[cyclo_movel]),
        TimerAction(period=bridge_delay, actions=[control]),
    ])
