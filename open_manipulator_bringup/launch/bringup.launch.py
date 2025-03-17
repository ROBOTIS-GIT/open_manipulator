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
# Authors: Hye-jong KIM, Sungho Woo

import os
import yaml
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("open_manipulator_description"),
            "urdf",
            "om_y",
            "open_manipulator_y.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot description Semantic config
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("open_manipulator_moveit_config"),
        "config",
        "om_y",
        "open_manipulator_y.srdf",
    )
    with open(robot_description_semantic_path, "r") as file:
        robot_description_semantic_config = file.read()

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_moveit_config"),
        "config",
        "om_y",
        "kinematics.yaml",
    )
    with open(kinematics_yaml_path, "r") as file:
        kinematics_yaml = yaml.safe_load(file)

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # joint_limits yaml
    joint_limits_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_moveit_config"),
        "config",
        "om_y",
        "joint_limits.yaml",
    )

    with open(joint_limits_yaml_path, "r") as file:
        joint_limits_yaml = yaml.safe_load(file)

    robot_description_joint_limits = {"robot_description_planning": joint_limits_yaml}

    ompl_planning_pipeline_config = {
        "planning_plugins": ["ompl_interface/OMPLPlanner",],
        "request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision",
        ],
        "response_adapters": [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ],
        "start_state_max_bounds_error": 0.1,
    }

    ompl_planning_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_moveit_config"),
        "config",
        "om_y",
        "ompl_planning.yaml",
    )
    with open(ompl_planning_yaml_path, "r") as file:
        ompl_planning_yaml = yaml.safe_load(file)
    ompl_planning_pipeline_config.update(ompl_planning_yaml)

    # Trajectory Execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    # Moveit Controllers
    moveit_simple_controllers_yaml_path = os.path.join(
      get_package_share_directory("open_manipulator_moveit_config"),
      "config",
      "om_y",
      "moveit_controllers.yaml",
    )
    with open(moveit_simple_controllers_yaml_path, "r") as file:
        moveit_simple_controllers_yaml = yaml.safe_load(file)

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Planning Scene Monitor Parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True
    }

    ld = LaunchDescription()
    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Start robot in Gazebo simulation.')
    ld.add_action(declare_use_sim)

    planning_parameters = {
        "plan_request_params":{
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnectkConfigDefault",
            "max_velocity_scaling_factor": 0.1,
            "max_acceleration_scaling_factor": 0.1,
            "planning_time": 1.0
        }
    }

    ompl_planning_pipeline_config.update(planning_parameters)

    move_group_node = Node(
        package="follower_launcher",
        executable="follower_launcher_node",
        output="screen",
        parameters=[
            {"planning_pipelines" : {'pipeline_names': ["ompl"]}},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_joint_limits,
            {"ompl": ompl_planning_pipeline_config},
            planning_parameters,
            trajectory_execution,
            moveit_controllers,
        ],
    )

    ld.add_action(move_group_node)

    return ld