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

#include "open_manipulator_position_ctrl/pick_place_controller.h"

using namespace open_manipulator;

PickAndPlaceController::PickAndPlaceController()
    :using_gazebo_(false),
     robot_name_(""),
     is_moving_(false)
{
  // Init parameter
  nh_.getParam("gazebo", using_gazebo_);
  nh_.getParam("robot_name", robot_name_);

  joint_[0].name = "joint1";
  joint_[1].name = "joint2";
  joint_[2].name = "joint3";
  joint_[3].name = "joint4";

  gripper_.name = "gripper";

//  present_joint_position_ = Eigen::VectorXd::Zero(JOINT_NUM+GRIP_NUM);
//  goal_joint_position_    = Eigen::VectorXd::Zero(JOINT_NUM);

  planned_path_info_.waypoints = 10;
  planned_path_info_.time_from_start = ros::Duration(10.0);
  planned_path_info_.planned_path_positions = Eigen::MatrixXd::Zero(planned_path_info_.waypoints, JOINT_NUM);

  planning_group_ = "arm";

  move_group = new moveit::planning_interface::MoveGroupInterface(planning_group_);

  initPublisher(using_gazebo_);
  initSubscriber(using_gazebo_);
}

PickAndPlaceController::~PickAndPlaceController()
{
  ros::shutdown();
  return;
}

void PickAndPlaceController::initPublisher(bool using_gazebo)
{
  if (using_gazebo)
  {
    ROS_INFO("SET Gazebo Simulation Mode");

    for (uint8_t index = 0; index < JOINT_NUM; index)
    {
      gazebo_goal_joint_position_pub_[index]
        = nh_.advertise<std_msgs::Float64>("/" + robot_name_ + "/" + joint_[index].name + "_position/command", 10);
    }

    gazebo_gripper_position_pub_[0]  = nh_.advertise<std_msgs::Float64>("/" + robot_name_ + "/grip_joint_position/command", 10);
    gazebo_gripper_position_pub_[1]  = nh_.advertise<std_msgs::Float64>("/" + robot_name_ + "/grip_joint_sub_position/command", 10);
  }
}

void PickAndPlaceController::initSubscriber(bool using_gazebo)
{
  if (using_gazebo)
  {
    gazebo_present_joint_position_sub_ = nh_.subscribe("/" + robot_name_ + "/joint_states", 10,
                                                       &PickAndPlaceController::gazeboPresentJointPositionMsgCallback, this);
  }

  target_joint_pose_sub_ = nh_.subscribe("/" + robot_name_ + "/target_pose", 10,
                                         &PickAndPlaceController::targetJointPoseMsgCallback, this);

  target_kinematics_pose_sub_ = nh_.subscribe("/" + robot_name_ + "/kinematics_pose", 10,
                                         &PickAndPlaceController::targetKinematicsPoseMsgCallback, this);


  display_planned_path_sub_ = nh_.subscribe("/move_group/display_planned_path", 10,
                                            &PickAndPlaceController::displayPlannedPathMsgCallback, this);
}

void PickAndPlaceController::gazeboPresentJointPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
//  for (auto index = 0; index < JOINT_NUM + GRIP_NUM; index++)
//    present_joint_position_(index) = msg->position.at(index);
}

void PickAndPlaceController::targetJointPoseMsgCallback(const open_manipulator_msgs::JointPose::ConstPtr &msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const robot_state::JointModelGroup *joint_model_group = move_group->getCurrentState()->getJointModelGroup(planning_group_);

  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

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

  move_group->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Planning (joint space goal) %s", success ? "SUCCESS" : "FAILED");

  spinner.stop();
}

void PickAndPlaceController::targetKinematicsPoseMsgCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::Pose target_pose;
  target_pose.orientation.w =  0.0;
  target_pose.position.x    =  0.28;
  target_pose.position.y    = -0.7;
  target_pose.position.z    =  1.0;
  move_group->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Planning (Cartesian space goal) %s", success ? "SUCCESS" : "FAILED");

  spinner.stop();
}

void PickAndPlaceController::displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
  trajectory_msg_ = *msg;

  trajectory_generate_thread_ = new boost::thread(boost::bind(&PickAndPlaceController::moveItTrajectoryGenerateThread, this));
  delete trajectory_generate_thread_;
}

void PickAndPlaceController::moveItTrajectoryGenerateThread()
{
  for (uint16_t tra_index = 0; tra_index < trajectory_msg_.trajectory.size(); tra_index++)
  {
    planned_path_info_.waypoints = trajectory_msg_.trajectory[tra_index].joint_trajectory.points.size();

    planned_path_info_.planned_path_positions.resize(planned_path_info_.waypoints, JOINT_NUM);

    for (uint16_t point_index = 0; point_index < planned_path_info_.waypoints; point_index++)
    {
      planned_path_info_.time_from_start = trajectory_msg_.trajectory[tra_index].joint_trajectory.points[point_index].time_from_start;

      for (uint16_t joint_index = 0; joint_index < trajectory_msg_.trajectory[tra_index].joint_trajectory.joint_names.size(); joint_index++)
      {
        float joint_position = trajectory_msg_.trajectory[tra_index].joint_trajectory.points[point_index].positions[joint_index];

        planned_path_info_.planned_path_positions.coeffRef(point_index , joint_index) = joint_position;
      }
    }
  }

  move_time_ = planned_path_info_.time_from_start.toSec();

  all_time_steps_ = planned_path_info_.waypoints;

  ros::WallDuration sleep_time(0.5);
  sleep_time.sleep();

  goal_trajectory_.resize(planned_path_info_.waypoints, JOINT_NUM);
  goal_trajectory_ = planned_path_info_.planned_path_positions;
  ROS_INFO("Get Joint Trajectory");

  is_moving_  = true;

  ROS_INFO("Send Motion Trajectory");
}

void PickAndPlaceController::process(void)
{
  static uint16_t step_cnt = 0;
  sensor_msgs::JointState goal_joint_state;
  std_msgs::Float64 goal_joint_position;

  if (is_moving_)
  {
    if (using_gazebo_)
    {
      for (uint8_t index = 0; index < JOINT_NUM; index++)
      {
        goal_joint_state.name.push_back(joint_[index].name);
        goal_joint_state.position.push_back(goal_trajectory_(step_cnt, index));

        goal_joint_position.data = goal_joint_state.position.at(index);
        gazebo_goal_joint_position_pub_[index].publish(goal_joint_position);
      }
    }

    if (step_cnt >= all_time_steps_)
    {
      is_moving_ = false;
      step_cnt  = 0;

      ROS_INFO("End Trajectory");
    }
    else
      step_cnt++;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_and_place_controller_for_OpenManipulator");
  PickAndPlaceController controller;

  ros::Rate loop_rate(ITERATION_FREQUENCY);

  while (ros::ok())
  {
    controller.process();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
