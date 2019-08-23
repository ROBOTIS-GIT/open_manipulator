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

#include "open_manipulator_x_controller/open_manipulator_x_controller.hpp"

using namespace open_manipulator_x_controller;
using namespace std::placeholders;

OpenManipulatorXControllerMoveit::OpenManipulatorXControllerMoveit(std::string usb_port, std::string baud_rate)
: moveit_plan_state_(false),
  moveit_plan_only_(true),
  moveit_sampling_time_(0.050)
{
  /************************************************************
  ** Get Parameters
  ************************************************************/
  this->get_parameter_or("moveit_sample_duration", moveit_sampling_time_, 0.050);
  std::string planning_group_name;
  this->get_parameter_or("planning_group_name", planning_group_name, "arm");

  move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);
  log::info("Ready to control " + planning_group_name + " group");

  /************************************************************
  ** Initialise ROS Publishers, Subscribers and Servers
  ************************************************************/
  init_publisher();
  init_subscriber();
  init_server();  
}

OpenManipulatorXControllerMoveit::~OpenManipulatorXControllerMoveit() {}

/********************************************************************************
** Init Functions
********************************************************************************/
void OpenManipulatorXControllerMoveit::init_publisher()
{
  moveit_update_start_state_pub_ = node_handle_.advertise<std_msgs::Empty>("rviz/moveit/update_start_state", 10);
}

void OpenManipulatorXControllerMoveit::init_subscriber()
{
  display_planned_path_sub_ = node_handle_.subscribe("/move_group/display_planned_path", 100,
                                                      &OpenManipulatorXControllerMoveit::displayPlannedPathCallback, this);
  move_group_goal_sub_ = node_handle_.subscribe("/move_group/goal", 100,
                                                      &OpenManipulatorXControllerMoveit::moveGroupGoalCallback, this);
  execute_trajectory_goal_sub_ = node_handle_.subscribe("/execute_trajectory/goal", 100,
                                                      &OpenManipulatorXControllerMoveit::executeTrajGoalCallback, this);
}

void OpenManipulatorXControllerMoveit::init_server()
{
  get_joint_position_server_  = priv_node_handle_.advertiseService("moveit/get_joint_position", &OpenManipulatorXControllerMoveit::get_joint_position_msg_callback, this);
  get_kinematics_pose_server_ = priv_node_handle_.advertiseService("moveit/get_kinematics_pose", &OpenManipulatorXControllerMoveit::get_kinematics_pose_msg_callback, this);
  set_joint_position_server_  = priv_node_handle_.advertiseService("moveit/set_joint_position", &OpenManipulatorXControllerMoveit::set_joint_position_msg_callback, this);
  set_kinematics_pose_server_ = priv_node_handle_.advertiseService("moveit/set_kinematics_pose", &OpenManipulatorXControllerMoveit::set_kinematics_pose_msg_callback, this);
}

/*****************************************************************************
** Callback Functions for ROS Subscribers
*****************************************************************************/
void OpenManipulatorXControllerMoveit::displayPlannedPathCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
  trajectory_msgs::JointTrajectory joint_trajectory_planned = msg->trajectory[0].joint_trajectory;
  joint_trajectory_ = joint_trajectory_planned;

  if(moveit_plan_only_ == false)
  {
    log::println("[INFO] [OpenManipulator Controller] Execute Moveit planned path", "GREEN");
    moveit_plan_state_ = true;
  }
  else
    log::println("[INFO] [OpenManipulator Controller] Get Moveit planned path", "GREEN");
}

void OpenManipulatorXControllerMoveit::moveGroupGoalCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg)
{
  log::println("[INFO] [OpenManipulator Controller] Get Moveit plnning option", "GREEN");
  moveit_plan_only_ = msg->goal.planning_options.plan_only; // click "plan & execute" or "plan" button

}
void OpenManipulatorXControllerMoveit::executeTrajGoalCallback(const moveit_msgs::ExecuteTrajectoryActionGoal::ConstPtr &msg)
{
  log::println("[INFO] [OpenManipulator Controller] Execute Moveit planned path", "GREEN");
  moveit_plan_state_ = true;
}

/*****************************************************************************
** Callback Functions for ROS Servers
*****************************************************************************/
void OpenManipulatorXControllerMoveit::get_joint_position_msg_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::GetJointPosition::Response> res)
{
  rclcpp::AsyncSpinner spinner(1);
  spinner.start();

  const std::vector<std::string> &joint_names = move_group_->getJointNames();
  std::vector<double> joint_values = move_group_->getCurrentJointValues();

  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    res.joint_position.joint_name.push_back(joint_names[i]);
    res.joint_position.position.push_back(joint_values[i]);
  }

  spinner.stop();
  return;
}

void OpenManipulatorXControllerMoveit::get_kinematics_pose_msg_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::GetKinematicsPose::Response> res)
{
  rclcpp::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

  res->header                     = current_pose.header;
  res->kinematics_pose.pose       = current_pose.pose;

  spinner.stop();
  return;
}

void OpenManipulatorXControllerMoveit::set_joint_position_msg_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  open_manipulator_msgs::msg::JointPosition msg = req->joint_position;
  res->is_planned = calcPlannedPath(req->planning_group, msg);

  return;
}

void OpenManipulatorXControllerMoveit::set_kinematics_pose_msg_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request>  req,
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  open_manipulator_msgs::msg::KinematicsPose msg = req->kinematics_pose;
  res->is_planned = calc_planned_path(req->planning_group, msg);

  return;
}

bool OpenManipulatorXControllerMoveit::calc_planned_path(const std::string planning_group, open_manipulator_msgs::msg::KinematicsPose msg)
{
  // rclcpp::AsyncSpinner spinner(1);
  // spinner.start();

  bool is_planned = false;
  geometry_msgs::msg::Pose target_pose = msg.pose;

  // move_group_->setPoseTarget(target_pose);

  // move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  // move_group_->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  // move_group_->setGoalTolerance(msg.tolerance);

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (open_manipulator_x_.getMovingState() == false)
  {
    // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // if (success)
    // {
    //   is_planned = true;
    // }
    // else
    {
      log::warn("Failed to Plan (task space goal)");
      is_planned = false;
    }
  }
  else
  {
    log::warn("Robot is Moving");
    is_planned = false;
  }

  // spinner.stop();
  return is_planned;
}

bool OpenManipulatorXControllerMoveit::calc_planned_path(const std::string planning_group, open_manipulator_msgs::msg::JointPosition msg)
{
  // rclcpp::AsyncSpinner spinner(1);
  // spinner.start();

  bool is_planned = false;

  // const robot_state::JointModelGroup *joint_model_group = move_group_->getCurrentState()->getJointModelGroup(planning_group);

  // moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (uint8_t index = 0; index < msg.position.size(); index++)
  {
    // joint_group_positions[index] = msg.position[index];
  }

  // move_group_->setJointValueTarget(joint_group_positions);

  // move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  // move_group_->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (open_manipulator_x_.getMovingState() == false)
  {
    // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // if (success)
    // {
    //   is_planned = true;
    // }
    // else
    {
      log::warn("Failed to Plan (joint space goal)");
      is_planned = false;
    }
  }
  else
  {
    log::warn("Robot is moving");
    is_planned = false;
  }

  // spinner.stop();
  return is_planned;
}

/********************************************************************************
** Functions related to processCallback 
********************************************************************************/
void OpenManipulatorXControllerMoveit::moveitTimer(double present_time)
{
  static double priv_time = 0.0f;
  static uint32_t step_cnt = 0;

  if (moveit_plan_state_ == true)
  {
    double path_time = present_time - priv_time;
    if (path_time > moveit_sampling_time_)
    {
      JointWaypoint target;
      uint32_t all_time_steps = joint_trajectory_.points.size();

      for(uint8_t i = 0; i < joint_trajectory_.points[step_cnt].positions.size(); i++)
      {
        JointValue temp;
        temp.position = joint_trajectory_.points[step_cnt].positions.at(i);
        temp.velocity = joint_trajectory_.points[step_cnt].velocities.at(i);
        temp.acceleration = joint_trajectory_.points[step_cnt].accelerations.at(i);
        target.push_back(temp);
      }
      open_manipulator_x_.makeJointTrajectory(target, path_time);

      step_cnt++;
      priv_time = present_time;

      if (step_cnt >= all_time_steps)
      {
        step_cnt = 0;
        moveit_plan_state_ = false;
        if (moveit_update_start_state_pub_.getNumSubscribers() == 0)
        {
          log::warn("Could not update the start state! Enable External Communications at the Moveit Plugin");
        }
        std_msgs::Empty msg;
        moveit_update_start_state_pub_->publish(msg);
      }
    }
  }
  else
  {
    priv_time = present_time;
  }
}
