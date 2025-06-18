// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Sungho Woo, Wonho Yun, Woojin Wie

#include "open_manipulator_playground/omy_f3m_hello_moveit.h"

#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);

  // Create a shared pointer for the node and enable automatic parameter declaration
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a logger for logging messages
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface for the "arm" planning group
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Define the target pose for the robot arm
  auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.x = 0.0;  // Orientation (quaternion x)
      msg.orientation.y = 0.0;  // Orientation (quaternion y)
      msg.orientation.z = 0.0;  // Orientation (quaternion z)
      msg.orientation.w = 1.0;  // Orientation (quaternion w)
      msg.position.x = -0.062;   // Position in x
      msg.position.y = -0.468;  // Position in y
      msg.position.z = 0.599;   // Position in z
      return msg;
    }();

  // Set the target pose for the arm
  move_group_interface.setPoseTarget(target_pose);

  // Set tolerances for goal position and orientation
  move_group_interface.setGoalPositionTolerance(0.01);
  move_group_interface.setGoalOrientationTolerance(0.01);

  // Plan the motion for the arm to reach the target pose
  auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

  // If planning succeeds, execute the planned motion
  if (success) {
    move_group_interface.execute(plan);
    std::this_thread::sleep_for(std::chrono::seconds(2));
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the arm!");
  }

  // Create the MoveIt MoveGroup Interface for the "gripper" planning group
  auto gripper_interface = MoveGroupInterface(node, "gripper");

  // Set the "close" position for the gripper and move it
  gripper_interface.setNamedTarget("close");
  if (gripper_interface.move()) {
    RCLCPP_INFO(logger, "Gripper closed successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds
  } else {
    RCLCPP_ERROR(logger, "Failed to close the gripper");
  }

  // Move the arm back to the "home" position
  move_group_interface.setNamedTarget("home");
  if (move_group_interface.move()) {
    RCLCPP_INFO(logger, "Arm moved back to home position");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds

    // Open the gripper
    gripper_interface.setNamedTarget("open");
    if (gripper_interface.move()) {
      RCLCPP_INFO(logger, "Gripper opened successfully");  // Log success
    } else {
      RCLCPP_ERROR(logger, "Failed to open the gripper");
    }

  } else {
    RCLCPP_ERROR(logger, "Failed to move the arm back to home position");
  }

  // Shut down the ROS2 node
  rclcpp::shutdown();
  return 0;
}
