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
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Rviz config save file
    rviz_config = os.path.join(
        get_package_share_directory("open_manipulator_x_moveit_config"),
        "config",
        "moveit.rviz"
    )

    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("open_manipulator_x_description"),
            "urdf",
            "open_manipulator_x_robot.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot description Semantic config
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("open_manipulator_x_moveit_config"),
        "config",
        "open_manipulator_x.srdf",
    )
    with open(robot_description_semantic_path, "r") as file:
        robot_description_semantic_config = file.read()

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization \
            default_planner_request_adapters/FixWorkspaceBounds \
             default_planner_request_adapters/FixStartStateBounds \
            default_planner_request_adapters/FixStartStateCollision \
            default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_x_moveit_config"),
        "config",
        "ompl_planning.yaml",
    )
    with open(ompl_planning_yaml_path, "r") as file:
        ompl_planning_yaml = yaml.safe_load(file)
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_x_moveit_config"),
        "config",
        "kinematics.yaml",
    )
    with open(kinematics_yaml_path, "r") as file:
        kinematics_yaml = yaml.safe_load(file)

    ld = LaunchDescription()

    warehouse_ros_config = {
        # For warehouse_ros_sqlite
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": "/home/robotis/warehouse_db.sqlite", # change to your path
        "port": 33829,
        "scene_name": "",  # If scene name is empty, all scenes will be used
        "queries_regex": ".*",
    }

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            warehouse_ros_config,
        ]
    )

    ld.add_action(rviz_node)

    return ld
