# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Bring up both dual-arm roles with isolated launch configurations."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FOLLOWER_LEFT_PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_6D5D4B68503059384C2E3120FF05133E-if00'
FOLLOWER_RIGHT_PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_23D2E9915157375037202020FF0F2510-if00'
LEADER_LEFT_PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_228BDD7B503059384C2E3120FF0A2B19-if00'
LEADER_RIGHT_PORT = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_895EBFC8503059384C2E3120FF08031C-if00'


def validate_devices(context):
    """Check all real buses before either child starts opening devices."""
    devices = {}
    for role in ('follower', 'leader'):
        if LaunchConfiguration(f'{role}_use_mock_hardware').perform(context) == 'true':
            continue
        for side in ('left', 'right'):
            name = f'{role}_{side}_port'
            port = LaunchConfiguration(name).perform(context).strip()
            if not port:
                raise ValueError(f'{name} must not be empty for real hardware')
            path = os.path.realpath(port)
            if path in devices:
                raise ValueError(f'{name} and {devices[path]} must identify different USB devices')
            devices[path] = name
    return []


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_mock_hardware', default_value='true',
                              choices=['true', 'false'],
                              description='Common hardware mode for both roles'),
        DeclareLaunchArgument('start_rviz', default_value='true', choices=['true', 'false'],
                              description='Open an RViz window for each role'),
        DeclareLaunchArgument('init_position', default_value='true', choices=['true', 'false'],
                              description='Move follower arms through the initial poses'),
        DeclareLaunchArgument('init_position_file', default_value='follower_initial_positions.yaml'),
        DeclareLaunchArgument('trigger_preload', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('teleop', default_value='true', choices=['true', 'false'],
                              description='Connect each leader arm and trigger to its follower'),
    ]
    includes = []
    launch_dir = os.path.join(get_package_share_directory('open_manipulator_bringup'), 'launch')
    for role, left_port, right_port in (
        ('follower', FOLLOWER_LEFT_PORT, FOLLOWER_RIGHT_PORT),
        ('leader', LEADER_LEFT_PORT, LEADER_RIGHT_PORT),
    ):
        child_arguments = {}
        for name, default in (
            ('use_mock_hardware', LaunchConfiguration('use_mock_hardware')),
            ('start_rviz', LaunchConfiguration('start_rviz')),
            ('left_port', left_port), ('right_port', right_port),
            ('left_xyz', '0 0.184 0'), ('right_xyz', '0 -0.184 0'),
            ('left_rpy', '0 0 0'), ('right_rpy', '0 0 0'),
        ):
            argument_name = f'{role}_{name}'
            options = {'choices': ['true', 'false']} if name in (
                'use_mock_hardware', 'start_rviz') else {}
            arguments.append(DeclareLaunchArgument(argument_name, default_value=default, **options))
            child_arguments[name] = LaunchConfiguration(argument_name)
        for name in (('init_position', 'init_position_file', 'teleop') if role == 'follower'
                     else ('trigger_preload',)):
            child_arguments[name] = LaunchConfiguration(name)
        includes.append(GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, f'omx_dual_{role}.launch.py')),
                launch_arguments=child_arguments.items(),
            ),
        ]))
    completed_initializers = set()
    leader_started = False

    def start_leader_when_follower_ready(event, context):
        nonlocal leader_started
        if context.is_shutdown or leader_started or event.returncode != 0:
            return []
        if not isinstance(event.action, Node) or event.action.expanded_node_namespace != '/follower':
            return []
        if LaunchConfiguration('init_position').perform(context) == 'true':
            if event.action.node_name not in ('/follower/left_init_position', '/follower/right_init_position'):
                return []
            completed_initializers.add(event.action.node_name)
            if len(completed_initializers) != 2:
                return []
        elif event.action.node_name != '/follower/follower_controller_spawner':
            return []
        leader_started = True
        return [includes[1]]

    return LaunchDescription(arguments + [
        OpaqueFunction(function=validate_devices),
        RegisterEventHandler(OnProcessExit(on_exit=start_leader_when_follower_ready)),
        includes[0],
    ])
