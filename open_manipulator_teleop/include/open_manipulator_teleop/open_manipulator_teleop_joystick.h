/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_TELEOP_JOYSTICK_H_
#define OPEN_MANIPULATOR_TELEOP_JOYSTICK_H_

#include <termios.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#define NUM_OF_JOINT 4
#define DELTA 0.01
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5

class OpenManipulatorTeleop
{
 public:
  OpenManipulatorTeleop();
  ~OpenManipulatorTeleop();

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initClient();

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;
  ros::Subscriber joy_command_sub_;

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);

  /*****************************************************************************
  ** ROS Clients and Callback Functions
  *****************************************************************************/
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_tool_control_client_;

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);
  void setGoal(const char *str);
};

#endif // OPEN_MANIPULATOR_TELEOP_JOYSTICK_H_
