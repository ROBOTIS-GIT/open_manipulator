/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

#ifndef OPEN_MANIPULATOR_X_CONTROLLER_MOVEIT_HPP
#define OPEN_MANIPULATOR_X_CONTROLLER_MOVEIT_HPP

#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <memory>
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/ExecuteTrajectoryActionGoal.h"
#include "moveit_msgs/MoveGroupActionGoal.h"

namespace open_manipulator_x_controller_moveit
{
class OpenManipulatorXControllerMoveit : public rclcpp::Node
{
 public:
  OpenManipulatorXControllerMoveit(std::string usb_port, std::string baud_rate);
  ~OpenManipulatorXControllerMoveit();

  void moveit_timer(double present_time);

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void init_publisher();
  void init_subscriber();
  void init_server();

 private:
  /*****************************************************************************
  ** Parameters
  *****************************************************************************/
  bool moveit_plan_state_;
  moveit::planning_interface::MoveGroupInterface* move_group_;
  trajectory_msgs::msg::JointTrajectory joint_trajectory_;
  double moveit_sampling_time_;
  bool moveit_plan_only_;

  /*****************************************************************************
  ** ROS Publishers, Callback Functions and Relevant Functions
  *****************************************************************************/
  rclcpp::Publisher<????>::SharedPtr moveit_update_start_state_pub_;

  /*****************************************************************************
  ** ROS Subscribers and Callback Functions
  *****************************************************************************/
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_planned_path_sub_;
  rclcpp::Subscription<moveit_msgs::msg::MoveGroupActionGoal>::SharedPtr move_group_goal_sub_;
  rclcpp::Subscription<moveit_msgs::msg::ExecuteTrajectoryActionGoal>::SharedPtr execute_trajectory_goal_sub_;

  void display_planned_path_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);
  void move_group_goal_callback(const moveit_msgs::msg::MoveGroupActionGoal::SharedPtr msg);
  void execute_trajectory_goal_callback(const moveit_msgs::msg::ExecuteTrajectoryActionGoal::SharedPtr msg);

  /*****************************************************************************
  ** ROS Servers and Callback Functions
  *****************************************************************************/
  rclcpp::Service<open_manipulator_msgs::srv::GetJointPosition>::SharedPtr get_joint_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::GetKinematicsPose>::SharedPtr get_kinematics_pose_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr set_joint_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr set_kinematics_pose_server_;

  void get_joint_position_msg_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Response> res);
  void get_kinematics_pose_msg_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Response> res);
  void set_joint_position_msg_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void set_kinematics_pose_msg_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);  
};
}  // namespace open_manipulator_x_controller_moveit
#endif //OPEN_MANIPULATOR_X_CONTROLLER_MOVEIT_HPP
