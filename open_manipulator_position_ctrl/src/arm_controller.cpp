/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Taehun Lim (Darby) */

#include "open_manipulator_position_ctrl/arm_controller.h"

using namespace open_manipulator;

ArmController::ArmController()
    :priv_nh_("~"),
     using_gazebo_(false),
     robot_name_(""),
     init_position_(false),
     joint_num_(4),
     is_moving_(false)
{
  // Init parameter
  nh_.getParam("gazebo", using_gazebo_);
  nh_.getParam("robot_name", robot_name_);
  priv_nh_.getParam("init_position", init_position_);

  joint_num_ = JOINT_NUM;

  planned_path_info_.waypoints = 10;
  planned_path_info_.planned_path_positions = Eigen::MatrixXd::Zero(planned_path_info_.waypoints, joint_num_);

  move_group = new moveit::planning_interface::MoveGroupInterface("arm");

  initPublisher(using_gazebo_);
  initSubscriber(using_gazebo_);

  initServer();

  if (init_position_ == true)
    initJointPosition();
}

ArmController::~ArmController()
{
  ros::shutdown();
  return;
}

void ArmController::initJointPosition()
{
  open_manipulator_msgs::JointPosition msg;

  msg.joint_name.push_back("joint1");
  msg.joint_name.push_back("joint2");
  msg.joint_name.push_back("joint3");
  msg.joint_name.push_back("joint4");

  msg.position.push_back(0.0);
  msg.position.push_back(-1.5707);
  msg.position.push_back(1.37);
  msg.position.push_back(0.2258);

  msg.max_velocity_scaling_factor = 0.2;
  msg.max_accelerations_scaling_factor = 0.5;

  calcPlannedPath(msg);
}

void ArmController::initPublisher(bool using_gazebo)
{
  if (using_gazebo)
  {
    ROS_INFO("SET Gazebo Simulation Mode(Joint)");

    std::string joint_name[joint_num_] = {"joint1", "joint2", "joint3", "joint4"};

    for (uint8_t index = 0; index < joint_num_; index++)
    {
      if (robot_name_ == "open_manipulator")
      {
        gazebo_goal_joint_position_pub_[index]
          = nh_.advertise<std_msgs::Float64>(robot_name_ + "/" + joint_name[index] + "_position/command", 10);
      }
      else
      {
        gazebo_goal_joint_position_pub_[index]
          = nh_.advertise<std_msgs::Float64>(joint_name[index] + "_position/command", 10);
      }
    }
  }
  else
  {
    goal_joint_position_pub_ = nh_.advertise<sensor_msgs::JointState>(robot_name_ + "/goal_joint_position", 10);
  }

  arm_state_pub_ = nh_.advertise<open_manipulator_msgs::State>(robot_name_ + "/arm_state", 10);
}

void ArmController::initSubscriber(bool using_gazebo)
{
  display_planned_path_sub_ = nh_.subscribe("/move_group/display_planned_path", 10,
                                            &ArmController::displayPlannedPathMsgCallback, this);
}

void ArmController::initServer()
{
  get_joint_position_server_  = nh_.advertiseService(robot_name_ + "/get_joint_position", &ArmController::getJointPositionMsgCallback, this);
  get_kinematics_pose_server_ = nh_.advertiseService(robot_name_ + "/get_kinematics_pose", &ArmController::getKinematicsPoseMsgCallback, this);
  set_joint_position_server_  = nh_.advertiseService(robot_name_ + "/set_joint_position", &ArmController::setJointPositionMsgCallback, this);
  set_kinematics_pose_server_ = nh_.advertiseService(robot_name_ + "/set_kinematics_pose", &ArmController::setKinematicsPoseMsgCallback, this);
}

bool ArmController::getJointPositionMsgCallback(open_manipulator_msgs::GetJointPosition::Request &req,
                                                open_manipulator_msgs::GetJointPosition::Response &res)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const std::vector<std::string> &joint_names = move_group->getJointNames();
  std::vector<double> joint_values = move_group->getCurrentJointValues();

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("%s: %f", joint_names[i].c_str(), joint_values[i]);

    res.joint_position.joint_name.push_back(joint_names[i]);
    res.joint_position.position.push_back(joint_values[i]);
  }

  spinner.stop();
}

bool ArmController::getKinematicsPoseMsgCallback(open_manipulator_msgs::GetKinematicsPose::Request &req,
                                                 open_manipulator_msgs::GetKinematicsPose::Response &res)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const std::string &pose_reference_frame = move_group->getPoseReferenceFrame();
  ROS_INFO("Pose Reference Frame = %s", pose_reference_frame.c_str());

  geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();

  res.header                     = current_pose.header;
  res.kinematics_pose.group_name = "arm";
  res.kinematics_pose.pose       = current_pose.pose;

  spinner.stop();
}

bool ArmController::setJointPositionMsgCallback(open_manipulator_msgs::SetJointPosition::Request &req,
                                                open_manipulator_msgs::SetJointPosition::Response &res)
{
  open_manipulator_msgs::JointPosition msg = req.joint_position;
  res.isPlanned = calcPlannedPath(msg);
}

bool ArmController::setKinematicsPoseMsgCallback(open_manipulator_msgs::SetKinematicsPose::Request &req,
                                                 open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  open_manipulator_msgs::KinematicsPose msg = req.kinematics_pose;
  res.isPlanned = calcPlannedPath(msg);
}

bool ArmController::calcPlannedPath(open_manipulator_msgs::KinematicsPose msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool isPlanned = false;
  geometry_msgs::Pose target_pose = msg.pose;

  move_group->setPoseTarget(target_pose);

  move_group->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  move_group->setGoalTolerance(msg.tolerance);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (is_moving_ == false)
  {
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
      isPlanned = true;
    }
    else
    {
      ROS_WARN("Planning (task space goal) is FAILED");
      isPlanned = false;
    }
  }
  else
  {
    ROS_WARN("ROBOT IS WORKING");
    isPlanned = false;
  }

  spinner.stop();

  return isPlanned;
}

bool ArmController::calcPlannedPath(open_manipulator_msgs::JointPosition msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool isPlanned = false;

  const robot_state::JointModelGroup *joint_model_group = move_group->getCurrentState()->getJointModelGroup("arm");

  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (uint8_t index = 0; index < joint_num_; index++)
  {
    if (msg.joint_name[index] == ("joint" + std::to_string((index+1))))
    {
      joint_group_positions[index] = msg.position[index];
    }
  }

  move_group->setJointValueTarget(joint_group_positions);

  move_group->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (is_moving_ == false)
  {
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
      isPlanned = true;
    }
    else
    {
      ROS_WARN("Planning (joint space goal) is FAILED");
      isPlanned = false;
    }
  }
  else
  {
    ROS_WARN("ROBOT IS WORKING");
    isPlanned = false;
  }

  spinner.stop();

  return isPlanned;
}

void ArmController::displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
  // Can't find 'grip'
  if (msg->trajectory[0].joint_trajectory.joint_names[0].find("grip") == std::string::npos)
  {
    ROS_INFO("Get ARM Planned Path");
    uint8_t joint_num = joint_num_;

    planned_path_info_.waypoints = msg->trajectory[0].joint_trajectory.points.size();

    planned_path_info_.planned_path_positions.resize(planned_path_info_.waypoints, joint_num);

    for (uint16_t point_num = 0; point_num < planned_path_info_.waypoints; point_num++)
    {
      for (uint8_t num = 0; num < joint_num; num++)
      {
        float joint_position = msg->trajectory[0].joint_trajectory.points[point_num].positions[num];

        planned_path_info_.planned_path_positions.coeffRef(point_num , num) = joint_position;
      }
    }

    all_time_steps_ = planned_path_info_.waypoints - 1;

    ros::WallDuration sleep_time(0.5);
    sleep_time.sleep();

    is_moving_  = true;    
  }
}

void ArmController::process(void)
{
  static uint16_t step_cnt = 0;
  std_msgs::Float64 gazebo_goal_joint_position;
  sensor_msgs::JointState goal_joint_position;
  open_manipulator_msgs::State state;

  if (is_moving_)
  {
    if (using_gazebo_)
    {
      for (uint8_t num = 0; num < joint_num_; num++)
      {
        gazebo_goal_joint_position.data = planned_path_info_.planned_path_positions(step_cnt, num);
        gazebo_goal_joint_position_pub_[num].publish(gazebo_goal_joint_position);
      }
    }
    else
    {
      for (uint8_t num = 0; num < joint_num_; num++)
      {
        goal_joint_position.position.push_back(planned_path_info_.planned_path_positions(step_cnt, num));
      }

      goal_joint_position_pub_.publish(goal_joint_position);
    }

    if (step_cnt >= all_time_steps_)
    {
      is_moving_ = false;
      step_cnt   = 0;

      ROS_INFO("Complete Execution");
    }
    else
    {
      step_cnt++;
    }

    state.robot = state.IS_MOVING;
    arm_state_pub_.publish(state);
  }
  else
  {
    state.robot = state.STOPPED;
    arm_state_pub_.publish(state);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_controller_for_OpenManipulator");

  ros::WallDuration sleep_time(3.0);
  sleep_time.sleep();

  ArmController controller;

  ros::Rate loop_rate(ITERATION_FREQUENCY);

  while (ros::ok())
  {
    controller.process();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
