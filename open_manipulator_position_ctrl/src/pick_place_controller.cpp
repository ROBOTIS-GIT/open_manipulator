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

  joint_id_["joint1"] = 0;
  joint_id_["joint2"] = 1;
  joint_id_["joint3"] = 2;
  joint_id_["joint4"] = 3;

  present_joint_position_ = Eigen::VectorXd::Zero(JOINT_NUM+GRIP_NUM);

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
    for (auto it = joint_id_.begin(); it != joint_id_.end(); it++)
    {
      std::string joint_name = it->first;
      gazebo_goal_joint_position_pub_[joint_id_[joint_name]]
        = nh_.advertise<std_msgs::Float64>("/" + robot_name_ + "/" + joint_name + "_position/command", 10);
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
}

void PickAndPlaceController::gazeboPresentJointPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  for (auto index = 0; index < JOINT_NUM + GRIP_NUM; index++)
    present_joint_position_(index) = msg->position.at(index);
}

void PickAndPlaceController::process(void)
{
  if (is_moving_)
  {
    for (int index = 0; index < JOINT_NUM; index++)
    {
      std_msgs::Float64 joint_position;
      joint_position.data = present_joint_position_(index);
      gazebo_goal_joint_position_pub_[index].publish(joint_position);

      ROS_INFO("joint_position[%d] = %lf", index, joint_position.data);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_and_place_controller_for_OpenManipulator");

//  PickAndPlaceController controller;

//  ros::Rate loop_rate(ITERATION_FREQUENCY);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  const robot_state::JointModelGroup *joint_model_group =
   move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] =  0.0;  // radians
  joint_group_positions[1] = -70.0 * (M_PI/180.0);  // radians
  joint_group_positions[2] =  30.0 * (M_PI/180.0);  // radians
  joint_group_positions[3] =  40.0 * (M_PI/180.0);  // radians
  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group.move();

//  controller.is_moving_ = true;

//  while (ros::ok())
//  {
//    controller.process();

//    ros::spinOnce();
//    loop_rate.sleep();
//  }

//  return 0;

//  geometry_msgs::Pose target_pose1;
//  target_pose1.orientation.w = 0.0;
//  target_pose1.position.x = 0.28;
//  target_pose1.position.y = -0.7;
//  target_pose1.position.z = 1.0;
//  move_group.setPoseTarget(target_pose1);

//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  ros::shutdown();

  return 0;
}
