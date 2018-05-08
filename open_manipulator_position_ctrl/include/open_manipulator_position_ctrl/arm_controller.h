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

#ifndef OPEN_MANIPULATOR_ARM_CONTROLLER_H
#define OPEN_MANIPULATOR_ARM_CONTROLLER_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <vector>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include "open_manipulator_msgs/State.h"

#include "open_manipulator_msgs/GetJointPosition.h"
#include "open_manipulator_msgs/GetKinematicsPose.h"

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include <eigen3/Eigen/Eigen>

namespace open_manipulator
{
#define ITERATION_FREQUENCY 25 //Hz
#define JOINT_NUM 4

typedef struct
{
  uint8_t group;
  uint16_t waypoints;                                  // planned number of via-points
  Eigen::MatrixXd planned_path_positions;              // planned position trajectory
} PlannedPathInfo;

class ArmController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  // ROS Parameters
  bool using_gazebo_;
  std::string robot_name_;
  int joint_num_;
  bool init_position_;

  // ROS Publisher
  ros::Publisher gazebo_goal_joint_position_pub_[10];
  ros::Publisher goal_joint_position_pub_;
  ros::Publisher arm_state_pub_;

  // ROS Subscribers
  ros::Subscriber display_planned_path_sub_;

  // ROS Service Server
  ros::ServiceServer get_joint_position_server_;
  ros::ServiceServer get_kinematics_pose_server_;
  ros::ServiceServer set_joint_position_server_;
  ros::ServiceServer set_kinematics_pose_server_;

  // ROS Service Client

  // MoveIt! interface
  moveit::planning_interface::MoveGroupInterface *move_group;
  PlannedPathInfo planned_path_info_;

  // Process state variables
  bool     is_moving_;
  uint16_t all_time_steps_;

 public:
  ArmController();
  virtual ~ArmController();

  void process(void);

 private:
  void initPublisher(bool using_gazebo);
  void initSubscriber(bool using_gazebo);

  void initServer();

  void initJointPosition();

  bool calcPlannedPath(open_manipulator_msgs::JointPosition msg);
  bool calcPlannedPath(open_manipulator_msgs::KinematicsPose msg);

  void displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);

  bool setJointPositionMsgCallback(open_manipulator_msgs::SetJointPosition::Request &req,
                                   open_manipulator_msgs::SetJointPosition::Response &res);

  bool setKinematicsPoseMsgCallback(open_manipulator_msgs::SetKinematicsPose::Request &req,
                                    open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool getJointPositionMsgCallback(open_manipulator_msgs::GetJointPosition::Request &req,
                                   open_manipulator_msgs::GetJointPosition::Response &res);

  bool getKinematicsPoseMsgCallback(open_manipulator_msgs::GetKinematicsPose::Request &req,
                                    open_manipulator_msgs::GetKinematicsPose::Response &res);
};
}

#endif /*OPEN_MANIPULATOR_ARM_CONTROLLER_H*/
