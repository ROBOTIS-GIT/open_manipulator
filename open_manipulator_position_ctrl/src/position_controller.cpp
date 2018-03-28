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

#include "open_manipulator_position_ctrl/position_controller.h"

using namespace open_manipulator;

PositionController::PositionController()
    :using_gazebo_(false),
     robot_name_(""),
     joint_num_(4),
     palm_num_(2),
     first_dxl_id_(1),
     is_moving_(false)
{
  // Init parameter
  nh_.getParam("gazebo", using_gazebo_);
  nh_.getParam("robot_name", robot_name_);
  nh_.getParam("first_dxl_id", first_dxl_id_);
  nh_.getParam("joint_num", joint_num_);
  nh_.getParam("gripper_dxl_id", gripper_dxl_id_);

  for (uint8_t num = 0; num < joint_num_; num++)
  {
    Joint joint;

    joint.name   = "joint" + std::to_string(num+1);
    joint.dxl_id = num + first_dxl_id_;

    joint_.push_back(joint);
  }

  gripper_.name = "gripper";
  gripper_.dxl_id = gripper_dxl_id_;

  planned_path_info_.group = ARM;
  planned_path_info_.waypoints = 10;
  planned_path_info_.planned_path_positions = Eigen::MatrixXd::Zero(planned_path_info_.waypoints, joint_num_);

  move_group[ARM] = new moveit::planning_interface::MoveGroupInterface("arm");
  move_group[GRIPPER] = new moveit::planning_interface::MoveGroupInterface("gripper");

  initPublisher(using_gazebo_);
  initSubscriber(using_gazebo_);
}

PositionController::~PositionController()
{
  ros::shutdown();
  return;
}

void PositionController::initPublisher(bool using_gazebo)
{
  if (using_gazebo)
  {
    ROS_INFO("SET Gazebo Simulation Mode");

    if (robot_name_.find("tb3") != std::string::npos)
    {
      for (uint8_t index = 0; index < joint_num_; index++)
      {
        gazebo_goal_joint_position_pub_[index]
          = nh_.advertise<std_msgs::Float64>("/" + joint_[index].name + "_position/command", 10);
      }
    }
    else
    {
      for (uint8_t index = 0; index < joint_num_; index++)
      {
        gazebo_goal_joint_position_pub_[index]
          = nh_.advertise<std_msgs::Float64>("/" + robot_name_ + "/" + joint_[index].name + "_position/command", 10);
      }
    }

    gazebo_gripper_position_pub_[LEFT_PALM]  = nh_.advertise<std_msgs::Float64>("/grip_joint_position/command", 10);
    gazebo_gripper_position_pub_[RIGHT_PALM] = nh_.advertise<std_msgs::Float64>("/grip_joint_sub_position/command", 10);
  }

  gripper_onoff_pub_ = nh_.advertise<open_manipulator_msgs::JointPose>("/robotis/" + robot_name_ + "/gripper_pose", 10);
}

void PositionController::initSubscriber(bool using_gazebo)
{
  if (using_gazebo)
  {
    gazebo_present_joint_position_sub_ = nh_.subscribe("/joint_states", 10,
                                                       &PositionController::gazeboPresentJointPositionMsgCallback, this);
  }

  target_joint_pose_sub_ = nh_.subscribe("/robotis/" + robot_name_ + "/joint_pose", 10,
                                         &PositionController::targetJointPoseMsgCallback, this);

  target_kinematics_pose_sub_ = nh_.subscribe("/robotis/" + robot_name_ + "/kinematics_pose", 10,
                                         &PositionController::targetKinematicsPoseMsgCallback, this);

  gripper_pose_sub_ = nh_.subscribe("/robotis/" + robot_name_ + "/gripper_pose", 10,
                                    &PositionController::targetGripperPoseMsgCallback, this);

  if (robot_name_.find("tb3") != std::string::npos)
  {
    gripper_onoff_sub_ = nh_.subscribe("/robotis/" + robot_name_ + "/gripper", 10,
                                       &PositionController::gripperPositionMsgCallback, this);
  }
  else
  {
    gripper_onoff_sub_ = nh_.subscribe("/robotis/open_manipulator/gripper", 10,
                                       &PositionController::gripperPositionMsgCallback, this);
  }

  display_planned_path_sub_ = nh_.subscribe("/move_group/display_planned_path", 10,
                                            &PositionController::displayPlannedPathMsgCallback, this);
}

void PositionController::gazeboPresentJointPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
//  for (auto index = 0; index < JOINT_NUM + GRIP_NUM; index++)
//    present_joint_position_(index) = msg->position.at(index);
}

void PositionController::targetGripperPoseMsgCallback(const open_manipulator_msgs::JointPose::ConstPtr &msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const robot_state::JointModelGroup *joint_model_group = move_group[GRIPPER]->getCurrentState()->getJointModelGroup("gripper");

  moveit::core::RobotStatePtr current_state = move_group[GRIPPER]->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (uint8_t index = 0; index < palm_num_; index++)
  {
    joint_group_positions[index] = msg->position[index];
    joint_group_positions[index] = msg->position[index];
  }

  move_group[GRIPPER]->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group[GRIPPER]->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Planning (joint space goal) %s", success ? "SUCCESS" : "FAILED");

  spinner.stop();

  planned_path_info_.group = GRIPPER;
}

void PositionController::gripperPositionMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  open_manipulator_msgs::JointPose grip_pose;

  if (msg->data == "grip_on")
  {
    grip_pose.position.push_back(0.025); // mm
    grip_pose.position.push_back(0.025); // mm

    gripper_onoff_pub_.publish(grip_pose);
  }
  else if (msg->data == "grip_off")
  {
    grip_pose.position.push_back(0.010); // mm
    grip_pose.position.push_back(0.010); // mm

    gripper_onoff_pub_.publish(grip_pose);
  }
  else
  {
    ROS_ERROR("If you want to grip or release something, publish 'grip_on' or 'grip_off'");
  }
}


void PositionController::targetJointPoseMsgCallback(const open_manipulator_msgs::JointPose::ConstPtr &msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const robot_state::JointModelGroup *joint_model_group = move_group[ARM]->getCurrentState()->getJointModelGroup("arm");

  moveit::core::RobotStatePtr current_state = move_group[ARM]->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//  for (uint8_t index = 0; index < JOINT_NUM; index++)
//  {
//    if (msg->joint_name[index] == ("joint" + std::to_string((index+1))))
//    {
//      joint_group_positions[index] = msg->position[index];

//      ROS_WARN("%lf", joint_group_positions[index]);
//    }
//  }

  joint_group_positions[0] =  0.0;                  // radians
  joint_group_positions[1] = -70.0 * (M_PI/180.0);  // radians
  joint_group_positions[2] =  30.0 * (M_PI/180.0);  // radians
  joint_group_positions[3] =  40.0 * (M_PI/180.0);  // radians

  move_group[ARM]->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group[ARM]->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Planning (joint space goal) %s", success ? "SUCCESS" : "FAILED");

  spinner.stop();

  planned_path_info_.group = ARM;
}

void PositionController::targetKinematicsPoseMsgCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
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

  planned_path_info_.group = ARM;
}

void PositionController::displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
  ROS_INFO("Get %s Planned Path", planned_path_info_.group == ARM ? "ARM" : "GRIPPER");
  uint8_t joint_num = 0;

  if      (planned_path_info_.group == ARM)     joint_num = joint_num_;
  else if (planned_path_info_.group == GRIPPER) joint_num = palm_num_;
  else                                          joint_num = joint_num_;

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

  ROS_INFO("Execute");

  is_moving_  = true;
}

void PositionController::process(void)
{
  static uint16_t step_cnt = 0;
  std_msgs::Float64 goal_joint_position;

  if (is_moving_)
  {
    if (using_gazebo_)
    {
      if (planned_path_info_.group == ARM)
      {
        for (uint8_t num = 0; num < joint_num_; num++)
        {
          goal_joint_position.data = planned_path_info_.planned_path_positions(step_cnt, num);
          gazebo_goal_joint_position_pub_[num].publish(goal_joint_position);
        }
      }
      else if (planned_path_info_.group == GRIPPER)
      {
        goal_joint_position.data = planned_path_info_.planned_path_positions(step_cnt, 0);
        gazebo_gripper_position_pub_[LEFT_PALM].publish(goal_joint_position);

        goal_joint_position.data = planned_path_info_.planned_path_positions(step_cnt, 1);
        gazebo_gripper_position_pub_[RIGHT_PALM].publish(goal_joint_position);
      }
    }

    if (step_cnt >= all_time_steps_)
    {
      is_moving_ = false;
      step_cnt   = 0;
      planned_path_info_.group = ARM; // Because of RViz

      ROS_INFO("End Trajectory");
    }
    else
    {
      step_cnt++;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_controller_for_OpenManipulator");

  ros::WallDuration sleep_time(10.0);
  sleep_time.sleep();

  PositionController controller;

  ros::Rate loop_rate(ITERATION_FREQUENCY);

  while (ros::ok())
  {
    controller.process();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
