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

#ifndef OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H
#define OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <sensor_msgs/JointState.h>
#include <open_manipulator_msgs/JointPose.h>

namespace dynamixel
{
#define ITERATION_FREQUENCY  (25)

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher present_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber goal_states_sub_;

  // ROS Service Server

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[16];
  uint8_t dxl_cnt_;

 public:
  DynamixelController();
  ~DynamixelController();
  bool control_loop();

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void readDynamixelState();

  void goalPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg);
};
}

#endif //OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H
