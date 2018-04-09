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

#include "open_manipulator_position_ctrl/joint_controller.h"

using namespace open_manipulator;

JointController::JointController()
    :using_gazebo_(false),
     robot_name_(""),
     joint_num_(4),
     first_dxl_id_(1),
     is_moving_(false)
{
  // Init parameter
  nh_.getParam("gazebo", using_gazebo_);
  nh_.getParam("robot_name", robot_name_);
  nh_.getParam("first_dxl_id", first_dxl_id_);
  nh_.getParam("joint_num", joint_num_);

  for (uint8_t num = 0; num < joint_num_; num++)
  {
    Joint joint;

    joint.name   = "joint" + std::to_string(num+1);
    joint.dxl_id = num + first_dxl_id_;

    joint_.push_back(joint);
  }

  planned_path_info_.waypoints = 10;
  planned_path_info_.planned_path_positions = Eigen::MatrixXd::Zero(planned_path_info_.waypoints, joint_num_);

  move_group = new moveit::planning_interface::MoveGroupInterface("arm");
  robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");

  initPublisher(using_gazebo_);
  initSubscriber(using_gazebo_);

  initServer();

  if (robot_name_ == "open_manipulator_chain_with_tb3")
    initJointPose();
}

JointController::~JointController()
{
  ros::shutdown();
  return;
}

void JointController::initJointPose()
{
  open_manipulator_msgs::JointPose joint_group_positions;

  joint_group_positions.joint_name.push_back("joint1");
  joint_group_positions.joint_name.push_back("joint2");
  joint_group_positions.joint_name.push_back("joint3");
  joint_group_positions.joint_name.push_back("joint4");

  joint_group_positions.position.push_back(0.0);
  joint_group_positions.position.push_back(-1.5707);
  joint_group_positions.position.push_back(1.37);
  joint_group_positions.position.push_back(0.2258);

  target_joint_pose_pub_.publish(joint_group_positions);
}

void JointController::initPublisher(bool using_gazebo)
{
  if (using_gazebo)
  {
    ROS_INFO("SET Gazebo Simulation Mode(Joint)");

    if (robot_name_.find("tb3") != std::string::npos)
    {
      // open_manipulator chain with tb3
      for (uint8_t index = 0; index < joint_num_; index++)
      {
        gazebo_goal_joint_position_pub_[index]
          = nh_.advertise<std_msgs::Float64>(joint_[index].name + "_position/command", 10);
      }
    }
    else
    {
      for (uint8_t index = 0; index < joint_num_; index++)
      {
        gazebo_goal_joint_position_pub_[index]
          = nh_.advertise<std_msgs::Float64>(robot_name_ + "/" + joint_[index].name + "_position/command", 10);
      }
    }
  }

  target_joint_pose_pub_ = nh_.advertise<open_manipulator_msgs::JointPose>("/robotis/" + robot_name_ + "/joint_pose", 10);
}

void JointController::initSubscriber(bool using_gazebo)
{
  if (using_gazebo)
  {
    gazebo_present_joint_position_sub_ = nh_.subscribe("/joint_states", 10,
                                                       &JointController::gazeboPresentJointPositionMsgCallback, this);
  }

  target_joint_pose_sub_ = nh_.subscribe("/robotis/" + robot_name_ + "/joint_pose", 10,
                                         &JointController::targetJointPoseMsgCallback, this);

  target_kinematics_pose_sub_ = nh_.subscribe("/robotis/" + robot_name_ + "/kinematics_pose", 10,
                                         &JointController::targetKinematicsPoseMsgCallback, this);

  display_planned_path_sub_ = nh_.subscribe("/move_group/display_planned_path", 10,
                                            &JointController::displayPlannedPathMsgCallback, this);
}

void JointController::initServer()
{
  get_joint_pose_server_ = nh_.advertiseService("get_joint_pose", &JointController::getjointPositionMsgCallback, this);
}

void JointController::gazeboPresentJointPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  //TODO
}

bool JointController::getjointPositionMsgCallback(open_manipulator_msgs::GetJointPose::Request &req,
                                                  open_manipulator_msgs::GetJointPose::Response &res)
{
  robot_model::RobotModelPtr kinematic_model = robot_model_loader->getModel();
  ROS_WARN("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
//  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("arm");

  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  std::vector<double> joint_values;
//  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
//  for (std::size_t i = 0; i < joint_names.size(); ++i)
//  {
//    ROS_WARN("%s: %f", joint_names[i].c_str(), joint_values[i]);
//  }

//  std::vector<double> joint_values = move_group->getCurrentJointValues();

  // moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

  // for (std::size_t i = 0; i < joint_names.size(); ++i)
  // {
  //   double *position = current_state->getJointPositions("joint1");
  //   joint_values.push_back(*position);
  // }
//  joint_values.push_back(current_state->getJointPositions("joint2"));
//  joint_values.push_back(current_state->getJointPositions("joint3"));
//  joint_values.push_back(current_state->getJointPositions("joint4"));

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_WARN("%s: %f", joint_names[i].c_str(), joint_values.at(i));
  }
}

void JointController::targetJointPoseMsgCallback(const open_manipulator_msgs::JointPose::ConstPtr &msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const robot_state::JointModelGroup *joint_model_group = move_group->getCurrentState()->getJointModelGroup("arm");

  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (uint8_t index = 0; index < joint_num_; index++)
  {
    if (msg->joint_name[index] == ("joint" + std::to_string((index+1))))
    {
      joint_group_positions[index] = msg->position[index];
    }
  }

  move_group->setJointValueTarget(joint_group_positions);

  // move_group->setMaxVelocityScalingFactor(0.1);
  // move_group->setMaxAccelerationScalingFactor(0.01);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (is_moving_ == false)
  {
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) move_group->move();
    else ROS_WARN("Planning (joint space goal) is FAILED");
  }
  else
    ROS_WARN("ROBOT IS WORKING");

  spinner.stop();
}

void JointController::targetKinematicsPoseMsgCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
//  ros::AsyncSpinner spinner(1);
//  spinner.start();

//  geometry_msgs::Pose target_pose;
//  target_pose.orientation.w =  0.0;
//  target_pose.position.x    =  0.28;
//  target_pose.position.y    = -0.7;
//  target_pose.position.z    =  1.0;
//  move_group->setPoseTarget(target_pose);

//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//  ROS_INFO("Planning (Cartesian space goal) %s", success ? "SUCCESS" : "FAILED");

//  spinner.stop();
}

void JointController::displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
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

void JointController::process(void)
{
  static uint16_t step_cnt = 0;
  std_msgs::Float64 goal_joint_position;

  if (is_moving_)
  {
    if (using_gazebo_)
    {
      for (uint8_t num = 0; num < joint_num_; num++)
      {
        goal_joint_position.data = planned_path_info_.planned_path_positions(step_cnt, num);
        gazebo_goal_joint_position_pub_[num].publish(goal_joint_position);
      }
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
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_controller_for_OpenManipulator");

  ros::WallDuration sleep_time(3.0);
  sleep_time.sleep();

  JointController controller;

  ros::Rate loop_rate(ITERATION_FREQUENCY);

  while (ros::ok())
  {
    controller.process();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
