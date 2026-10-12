#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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

"""Launch the bimanual OMX cameras: two USB cameras and one RealSense."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def usb_camera(context, name, frame_id):
    # usb_cam cannot open symlinks, so resolve /dev/v4l/by-path/... to /dev/videoN.
    device = os.path.realpath(LaunchConfiguration(f'{name}_device').perform(context))
    if not os.path.exists(device):
        return [LogInfo(msg=(
            f'[{name}] {device} not found, skipping. '
            'Check the USB port with: ls -l /dev/v4l/by-path/'
        ))]
    return [
        LogInfo(msg=f'[{name}] using {device}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'camera_usb_cam.launch.py',
            ])),
            launch_arguments={
                'name': name, 'video_device': device, 'frame_id': frame_id,
            }.items(),
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'camera1_device',
            default_value='/dev/v4l/by-path/pci-0000:00:14.0-usb-0:9:1.0-video-index0',
            description='Video device of camera1 (left wrist), fixed to USB port 9',
        ),
        DeclareLaunchArgument(
            'camera2_device',
            default_value='/dev/v4l/by-path/pci-0000:00:14.0-usb-0:12:1.0-video-index0',
            description='Video device of camera2 (right wrist), fixed to USB port 12',
        ),
        DeclareLaunchArgument(
            'use_realsense',
            default_value='true',
            description='Whether to launch the RealSense camera',
        ),
        DeclareLaunchArgument(
            'realsense_serial_no',
            default_value='_944622074739',
            description='RealSense serial number, prefixed with an underscore',
        ),
        DeclareLaunchArgument(
            'realsense_color_profile',
            default_value='640,480,30',
            description='RealSense color stream profile (width,height,fps)',
        ),
        DeclareLaunchArgument(
            'realsense_enable_depth',
            default_value='false',
            description='Whether to enable the RealSense depth stream',
        ),
    ]

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('open_manipulator_bringup'),
            'launch',
            'camera_realsense.launch.py',
        ])),
        launch_arguments={
            'serial_no1': LaunchConfiguration('realsense_serial_no'),
            'rgb_camera.color_profile1': LaunchConfiguration('realsense_color_profile'),
            'enable_depth1': LaunchConfiguration('realsense_enable_depth'),
            # Camera frames come from the omx_f_bi URDF (camera_link = body center).
            'publish_tf1': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_realsense')),
    )

    # usb_cam probes every /dev/video* on startup, so starting both at once races.
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=usb_camera, kwargs={
            'name': 'camera1', 'frame_id': 'left_wrist_camera_optical_frame'}),
        TimerAction(
            period=2.0,
            actions=[OpaqueFunction(function=usb_camera, kwargs={
                'name': 'camera2', 'frame_id': 'right_wrist_camera_optical_frame'})],
        ),
        realsense,
    ])
