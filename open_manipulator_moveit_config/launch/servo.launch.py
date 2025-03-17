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
import yaml
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_robot_model():
    robot_model = os.getenv("ROBOT_MODEL", "om_x")
    if robot_model not in ["om_x", "om_y","om_y_follower"]:
        raise ValueError(f"Invalid ROBOT_MODEL: {robot_model}")
    return robot_model

def generate_launch_description():

    robot_model = get_robot_model()

    if robot_model == "om_x":
        urdf_file = "open_manipulator_x"
        urdf_folder = "om_x"
    else:  # "om_y_follower"
        urdf_file = "open_manipulator_y"
        urdf_folder = "om_y"

    ld = LaunchDescription()

    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Start robot in Gazebo simulation.')
    ld.add_action(declare_use_sim)

    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("open_manipulator_description"),
            "urdf",
            urdf_folder,
            f"{urdf_file}.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot description Semantic config
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("open_manipulator_moveit_config"),
        "config",
        urdf_folder,
        f"{urdf_file}.srdf",
    )
    try:
        with open(robot_description_semantic_path, "r") as file:
            robot_description_semantic_config = file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_moveit_config"),
        "config",
        urdf_folder,
        "kinematics.yaml",
    )
    with open(kinematics_yaml_path, "r") as file:
        kinematics_yaml = yaml.safe_load(file)

    kinematics_params = {"robot_description_kinematics": kinematics_yaml}

    # Get parameters for the Servo node
    servo_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_moveit_config"),
        "config",
        urdf_folder,
        "moveit_servo.yaml",
    )
    try:
        with open(servo_yaml_path, "r") as file:
            servo_params = {"moveit_servo": yaml.safe_load(file)}
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

    # joint_limits yaml
    joint_limits_yaml_path = os.path.join(
        get_package_share_directory("open_manipulator_moveit_config"),
        "config",
        urdf_folder,
        "joint_limits.yaml",
    )
    with open(joint_limits_yaml_path, "r") as file:
        joint_limits_yaml = yaml.safe_load(file)
    robot_description_joint_limits = {"robot_description_planning": joint_limits_yaml}


    # Launch as much as possible in components
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            {'use_gazebo':use_sim},
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_params,
            robot_description_joint_limits,
        ]
    )
    ld.add_action(servo_node)

    return ld
