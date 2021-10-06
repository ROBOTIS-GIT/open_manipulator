// Copyright 2019 ROBOTIS CO., LTD.
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
// Author: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na


#ifndef OPEN_MANIPULATOR_X_CONTROLLER__OPEN_MANIPULATOR_X_CONTROLLER_HPP_
#define OPEN_MANIPULATOR_X_CONTROLLER__OPEN_MANIPULATOR_X_CONTROLLER_HPP_

#include <chrono>
#include <cstdio>
#include <memory>
#include <unistd.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"
#include "open_manipulator_msgs/srv/set_drawing_trajectory.hpp"
#include "open_manipulator_msgs/srv/set_actuator_state.hpp"
#include "open_manipulator_msgs/srv/get_joint_position.hpp"
#include "open_manipulator_msgs/srv/get_kinematics_pose.hpp"
#include "open_manipulator_msgs/msg/open_manipulator_state.hpp"
#include "open_manipulator_x_libs/open_manipulator_x.hpp"

namespace open_manipulator_x_controller
{
class OpenManipulatorXController : public rclcpp::Node
{
 public:
  OpenManipulatorXController(std::string usb_port, std::string baud_rate);
  virtual ~OpenManipulatorXController();

 private:  
  /*****************************************************************************
  ** Parameters
  *****************************************************************************/
  bool sim_;
  double control_period_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  // Robotis_manipulator related 
  OpenManipulatorX open_manipulator_x_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void init_parameters();
  void init_publisher();
  void init_subscriber();
  void init_server();

  /*****************************************************************************
  ** ROS timers
  *****************************************************************************/
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  void process_callback(); 
  void publish_callback();  
  void process(double time);

  /*****************************************************************************
  ** ROS Publishers, Callback Functions and Relevant Functions
  *****************************************************************************/
  rclcpp::Publisher<open_manipulator_msgs::msg::OpenManipulatorState>::SharedPtr open_manipulator_x_states_pub_;
  std::vector<rclcpp::Publisher<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr> open_manipulator_x_kinematics_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr open_manipulator_x_joint_states_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> gazebo_goal_joint_position_pub_;

  void publish_open_manipulator_x_states();
  void publish_kinematics_pose();
  void publish_joint_states();
  void publish_gazebo_command();

  /*****************************************************************************
  ** ROS Subscribers and Callback Functions
  *****************************************************************************/
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr open_manipulator_x_option_sub_;

  void open_manipulator_x_option_callback(const std_msgs::msg::String::SharedPtr msg);

  /*****************************************************************************
  ** ROS Servers and Callback Functions
  *****************************************************************************/
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_pose_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_orientation_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_position_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_orientation_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_from_present_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_position_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_orientation_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetActuatorState>::SharedPtr set_actuator_state_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetDrawingTrajectory>::SharedPtr goal_drawing_trajectory_server_;

  void goal_joint_space_path_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void goal_joint_space_path_to_kinematics_pose_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_joint_space_path_to_kinematics_position_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_joint_space_path_to_kinematics_orientation_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_position_only_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_orientation_only_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_joint_space_path_from_present_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void goal_task_space_path_from_present_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_from_present_position_only_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_task_space_path_from_present_orientation_only_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);
  void goal_tool_control_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);
  void set_actuator_state_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Response> res);
  void goal_drawing_trajectory_callback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Request> req,
    const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Response> res);
};
}  // namespace open_manipulator_x_controller
#endif  // OPEN_MANIPULATOR_X_CONTROLLER__OPEN_MANIPULATOR_X_CONTROLLER_HPP_
