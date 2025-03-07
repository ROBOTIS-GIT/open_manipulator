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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  start_rviz = LaunchConfiguration('start_rviz')
  prefix = LaunchConfiguration('prefix')
  use_fake_hardware = LaunchConfiguration('use_fake_hardware')
  use_sim = LaunchConfiguration('use_sim')
  port_name = LaunchConfiguration('port_name')
  run_init_position = LaunchConfiguration('run_init_position')  # 추가

  return LaunchDescription([
    DeclareLaunchArgument(
      'start_rviz',
      default_value='false',
      description='Whether to execute rviz2'),

    DeclareLaunchArgument(
      'prefix',
      default_value='""',
      description='Prefix of the joint and link names'),

    DeclareLaunchArgument(
      'use_fake_hardware',
      default_value='false',
      description='Start robot with fake hardware mirroring command to its states.'),

    DeclareLaunchArgument(
      'use_sim',
      default_value='false',
      description='Start robot in Gazebo simulation.'),

    DeclareLaunchArgument(
      'port_name',
      default_value='/dev/ttyUSB0',
      description='The port name to connect to hardware.'),

    DeclareLaunchArgument(
      'run_init_position',
      default_value='true',  # 기본적으로 실행
      description='Run init_position.py after launch'),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/base.launch.py']),
      launch_arguments={
        'start_rviz': start_rviz,
        'prefix': prefix,
        'use_fake_hardware': use_fake_hardware,
        'use_sim': use_sim,
        'port_name': port_name,
      }.items(),
    ),

    # 1초 후 init_position.py 실행
    TimerAction(
      period=1.0,  # 1초 대기 후 실행
      actions=[
        ExecuteProcess(
          cmd=['ros2', 'run', 'open_manipulator_bringup', 'init_position.py'],
          output='screen',
          condition=IfCondition(run_init_position)  # 실행 여부 설정 가능
        )
      ]
    ),
  ])
