/*******************************************************************************
* Copyright 2024 ROBOTIS CO., LTD.
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

/* Authors: Ryan Shim, Sungho Woo */

#include "open_manipulator_x_gui/qnode.hpp"

namespace open_manipulator_x_gui {

QNode::QNode(int argc, char** argv)
  : QThread(),
    init_argc(argc),
    init_argv(argv)
{
}

QNode::~QNode()
{
  if (spinner_thread_.joinable()) {
      rclcpp::shutdown();
      spinner_thread_.join();
  }
}

bool QNode::init()
{
  rclcpp::init(init_argc, init_argv);

  std::string node_name = "open_manipulator_x_gui";
  node_ = std::make_shared<rclcpp::Node>(node_name);

  if (!rclcpp::ok()) {
      return false;
  }

  node_->declare_parameter<std::string>("csv_path", "robot_joint_log.csv");
  node_->get_parameter("csv_path", csv_file_path_);
  RCLCPP_INFO(node_->get_logger(), "CSV Path set to: %s", csv_file_path_.c_str());

  std::string planning_group_name = "arm";
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_name);

  std::string planning_group_name2 = "gripper";
  move_group2_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_name2);

  if (!move_group_ || !move_group2_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize move groups");
    return false;
  }
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);

  spinner_thread_ = std::thread([this]() { executor_->spin(); });

  torque_client_ = node_->create_client<std_srvs::srv::SetBool>("dynamixel_hardware_interface/set_dxl_torque");

  return true;
}

void QNode::run()
{
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    updateRobotState();
    loop_rate.sleep();
  }
  RCLCPP_INFO(node_->get_logger(), "ROS shutdown, proceeding to close the GUI.");
  Q_EMIT rosShutdown();
}

void QNode::updateRobotState()
{
  std::vector<double> jointValues = move_group_->getCurrentJointValues();
  std::vector<double> jointValues2 = move_group2_->getCurrentJointValues();
  std::vector<double> temp_angle;

  if (jointValues.size() >= 4) {
    temp_angle.push_back(jointValues.at(0));
    temp_angle.push_back(jointValues.at(1));
    temp_angle.push_back(jointValues.at(2));
    temp_angle.push_back(jointValues.at(3));
  } else {
    RCLCPP_ERROR(node_->get_logger(), "jointValues does not have enough elements");
    return;
  }

  if (jointValues2.size() >= 1) {
    temp_angle.push_back(jointValues2.at(0));
  } else {
    RCLCPP_ERROR(node_->get_logger(), "jointValues2 does not have enough elements");
    return;
  }

  present_joint_angle_ = temp_angle;

  geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
  std::vector<double> temp_position;
  temp_position.push_back(current_pose.position.x);
  temp_position.push_back(current_pose.position.y);
  temp_position.push_back(current_pose.position.z);
  temp_position.push_back(current_pose.orientation.x);
  temp_position.push_back(current_pose.orientation.y);
  temp_position.push_back(current_pose.orientation.z);
  temp_position.push_back(current_pose.orientation.w);

  present_kinematics_position_ = temp_position;
}

std::string QNode::getCSVPath() const {
  return csv_file_path_;
}

std::vector<double> QNode::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> QNode::getPresentKinematicsPosition()
{
  return present_kinematics_position_;
}

bool QNode::setJointSpacePath(std::vector<double> joint_angle)
{
  const moveit::core::JointModelGroup* joint_model_group = move_group_->getCurrentState()->getJointModelGroup("arm");

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = joint_angle.at(0);  // radians
  joint_group_positions[1] = joint_angle.at(1);  // radians
  joint_group_positions[2] = joint_angle.at(2);  // radians
  joint_group_positions[3] = joint_angle.at(3);  // radians
  move_group_->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success)
    return false;

  move_group_->move();
  while (!isMotionComplete()){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return true;
}

bool QNode::isMotionComplete()
{
  if (isStopRequested()) {
    RCLCPP_INFO(node_->get_logger(), "Motion stopped by user.");
    return true;
  }

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

  const moveit::core::JointModelGroup* joint_model_group = move_group_->getCurrentState()->getJointModelGroup("arm");
  std::vector<double> current_joint_positions;
  current_state->copyJointGroupPositions(joint_model_group, current_joint_positions);

  std::vector<double> target_joint_positions;
  move_group_->getJointValueTarget(target_joint_positions);

  const double tolerance = 0.03;
  for (size_t i = 0; i < current_joint_positions.size(); ++i)
  {
    if (std::abs(current_joint_positions[i] - target_joint_positions[i]) > tolerance)
    {
      return false;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Motion completed");
  return true;
}

bool QNode::setTaskSpacePath(
  std::vector<double> kinematics_pose,
  bool position_only,
  double position_tol,
  double orientation_tol)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = kinematics_pose.at(0);
  target_pose.position.y = kinematics_pose.at(1);
  target_pose.position.z = kinematics_pose.at(2);
  target_pose.orientation.x = kinematics_pose.at(3);
  target_pose.orientation.y = kinematics_pose.at(4);
  target_pose.orientation.z = kinematics_pose.at(5);
  target_pose.orientation.w = kinematics_pose.at(6);

  if (position_only) {
    move_group_->setPositionTarget(
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z);
      move_group_->setGoalPositionTolerance(position_tol);
      move_group_->setGoalOrientationTolerance(orientation_tol);
  } else {
    move_group_->setGoalPositionTolerance(position_tol);
    move_group_->setGoalOrientationTolerance(orientation_tol);
    move_group_->setPoseTarget(target_pose);
  }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success)
    return false;

  move_group_->move();

  return true;
}

bool QNode::setToolControl(std::vector<double> joint_angle)
{
    const moveit::core::JointModelGroup* joint_model_group = move_group2_->getCurrentState()->getJointModelGroup("gripper");

    moveit::core::RobotStatePtr current_state = move_group2_->getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = joint_angle.at(0);  // radians
    move_group2_->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group2_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
      return false;

    std::atomic<bool> move_completed(false);

    std::thread move_thread([&]() {
      move_group2_->move();
      move_completed.store(true);
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));

    if (!move_completed.load()) {
      move_group2_->stop();
      move_thread.join();
      return true;
    }
    move_thread.join();
    return true;
}

void QNode::stopMotion()
{
  stop_requested_ = true;
  if (move_group_) {
    move_group_->stop();
  }
  if (move_group2_) {
    move_group2_->stop();
  }
  RCLCPP_INFO(node_->get_logger(), "Robot motion stopped.");
}

bool QNode::isStopRequested()
{
  return stop_requested_;
}

void QNode::resetStopRequest()
{
  stop_requested_ = false;
}

bool QNode::sendTorqueSrv(bool checked)
{
  if (!torque_client_->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Torque service not available.");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = checked;

  auto future = torque_client_->async_send_request(request);
  RCLCPP_INFO(node_->get_logger(), "Torque service succeeded: %s", checked ? "ON" : "OFF");
  return true;
}

}  // namespace open_manipulator_x_gui
