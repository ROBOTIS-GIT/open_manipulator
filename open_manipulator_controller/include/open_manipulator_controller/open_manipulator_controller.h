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

#ifndef OPEN_MANIPULATOR_CONTROLLER_H
#define OPEN_MANIPULATOR_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>
#include <unistd.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include "open_manipulator_libs/Chain.h"

#define ACTUATOR_CONTROL_TIME 0.010 // ms
#define ITERATION_FREQUENCY   100   // hz 10ms
#define ACTUATOR_CONTROL_TIME_MSEC 10 // ms

namespace open_manipulator_controller
{

class OM_CONTROLLER
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  ros::ServiceServer goal_joint_space_path_server_;
  ros::ServiceServer goal_task_space_path_server_;
  ros::ServiceServer goal_joint_space_path_to_present_server_;
  ros::ServiceServer goal_task_space_path_to_present_server_;
  ros::ServiceServer goal_tool_control_server_;

  ros::Publisher chain_kinematics_pose_pub_;
  ros::Publisher chain_joint_states_pub_;
  ros::Publisher chain_joint_states_to_gazebo_pub_[NUM_OF_JOINT];
  ros::Publisher chain_gripper_states_to_gazebo_pub_[2];

  std::string robot_name_;

  pthread_t timer_thread_;

  CHAIN chain_;

  bool tool_ctrl_flag_;
  bool using_platform_;
  double tool_position_;
  bool timer_thread_flag_;

 public:

  OM_CONTROLLER();
  ~OM_CONTROLLER();

  void initPublisher();
  void initSubscriber();

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

  bool goalJointSpacePathCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                  open_manipulator_msgs::SetJointPosition::Response &res);
  bool goalTaskSpacePathCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                  open_manipulator_msgs::SetKinematicsPose::Response &res);
  bool goalJointSpacePathToPresentCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                           open_manipulator_msgs::SetJointPosition::Response &res);
  bool goalTaskSpacePathToPresentCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                          open_manipulator_msgs::SetKinematicsPose::Response &res);
  bool goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                               open_manipulator_msgs::SetJointPosition::Response &res);

  void setTimerThread();
  static void *timerThread(void *param);

  void process(double time);

  void publishKinematicsPose();
  void publishJointStates();

};
}

#endif //OPEN_MANIPULATOR_CONTROLLER_H
