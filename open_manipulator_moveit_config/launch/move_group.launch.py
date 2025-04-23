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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('open_manipulator_description'),
            'urdf',
            urdf_folder,
            f'{urdf_file}.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot description Semantic config
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

    # kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory('open_manipulator_moveit_config'),
        'config',
        urdf_folder,
        'kinematics.yaml',
    )
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_yaml = yaml.safe_load(file)

    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # joint_limits yaml
    joint_limits_yaml_path = os.path.join(
        get_package_share_directory('open_manipulator_moveit_config'),
        'config',
        urdf_folder,
        'joint_limits.yaml',
    )
    with open(joint_limits_yaml_path, 'r') as file:
        joint_limits_yaml = yaml.safe_load(file)
    robot_description_joint_limits = {'robot_description_planning': joint_limits_yaml}

    # Planning Functionality
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

    # Trajectory Execution
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # Moveit Controllers
    moveit_simple_controllers_yaml_path = os.path.join(
        get_package_share_directory('open_manipulator_moveit_config'),
        'config',
        urdf_folder,
        'moveit_controllers.yaml',
    )
    with open(moveit_simple_controllers_yaml_path, 'r') as file:
        moveit_simple_controllers_yaml = yaml.safe_load(file)

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': (
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        ),
    }

    # Planning Scene Monitor Parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }

    ld = LaunchDescription()
    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim', default_value='true', description='Start robot in Gazebo simulation.'
    )
    ld.add_action(declare_use_sim)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_joint_limits,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim},
        ],
    )

    ld.add_action(move_group_node)

    return ld
