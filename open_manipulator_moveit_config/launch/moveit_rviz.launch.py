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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def get_robot_model():
    robot_model = os.getenv('ROBOT_MODEL', 'om_x')
    if robot_model not in ['om_x', 'om_y', 'om_y_follower', 'om_y_leader']:
        raise ValueError(f'Invalid ROBOT_MODEL: {robot_model}')
    return robot_model


def generate_launch_description():
    robot_model = get_robot_model()

    if robot_model == 'om_x':
        urdf_file = 'open_manipulator_x'
        urdf_folder = 'om_x'
    elif robot_model == 'om_y':
        urdf_file = 'open_manipulator_y'
        urdf_folder = 'om_y'
    elif robot_model == 'om_y_follower':
        urdf_file = 'open_manipulator_y_follower'
        urdf_folder = 'om_y_follower'
    elif robot_model == 'om_y_leader':
        urdf_file = 'open_manipulator_y_leader'
        urdf_folder = 'om_y_leader'
    else:
        raise ValueError(f'Invalid ROBOT_MODEL: {robot_model}')

    rviz_config = os.path.join(
        get_package_share_directory('open_manipulator_moveit_config'),
        'config',
        urdf_folder,
        'moveit.rviz',
    )

    # Robot description (URDF)
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('open_manipulator_description'),
            'urdf',
            urdf_folder,
            f'{urdf_file}.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot description Semantic (SRDF)
    robot_description_semantic_path = os.path.join(
        get_package_share_directory('open_manipulator_moveit_config'),
        'config',
        urdf_folder,
        f'{urdf_file}.srdf',
    )
    with open(robot_description_semantic_path, 'r') as file:
        robot_description_semantic_config = file.read()

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    # Planning Functionality (OMPL)
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml_path = os.path.join(
        get_package_share_directory('open_manipulator_moveit_config'),
        'config',
        urdf_folder,
        'ompl_planning.yaml',
    )
    with open(ompl_planning_yaml_path, 'r') as file:
        ompl_planning_yaml = yaml.safe_load(file)
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory('open_manipulator_moveit_config'),
        'config',
        urdf_folder,
        'kinematics.yaml',
    )
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_yaml = yaml.safe_load(file)

    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Joint Limits yaml
    joint_limits_yaml_path = os.path.join(
        get_package_share_directory('open_manipulator_moveit_config'),
        'config',
        urdf_folder,
        'joint_limits.yaml',
    )
    with open(joint_limits_yaml_path, 'r') as file:
        joint_limits_yaml = yaml.safe_load(file)
    robot_description_joint_limits = {'robot_description_planning': joint_limits_yaml}

    # Warehouse Database Config
    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': '/home/robotis/warehouse_db.sqlite',
        'port': 33829,
        'scene_name': '',
        'queries_regex': '.*',
    }

    ld = LaunchDescription()

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_joint_limits,
            warehouse_ros_config,
        ],
    )

    ld.add_action(rviz_node)

    return ld
