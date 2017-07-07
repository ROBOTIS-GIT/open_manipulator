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

/* Authors: Taehoon Lim (Darby) */

#ifndef OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H
#define OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>
#include <sensor_msgs/JointState.h>
#include <open_manipulator_msgs/JointPose.h>

namespace open_manipulator_dynamixel_controller
{
#define MOTOR                (0)
#define MAX_DXL_NUM          (5)
#define ITERATION_FREQUENCY  (25)

typedef struct
{
  std::vector<uint8_t>  torque;
  std::vector<uint32_t> pos;
}WriteValue;

typedef struct
{
  std::vector<uint32_t> pos;
}ReadValue;

class DynamixelController
{
 public:
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher present_dynamixel_position_pub_;

  // ROS Topic Subscriber
  ros::Subscriber goal_dynamixel_position_sub_;

  // ROS Service Server

  // ROS Service Client

  // Dynamixel Parameters
  std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info_;
  dynamixel_multi_driver::DynamixelMultiDriver *multi_driver_;

  WriteValue *writeValue_;
  ReadValue  *readValue_;

 public:
  DynamixelController();
  ~DynamixelController();
  bool control_loop();

  bool loadDynamixel();
  bool initDynamixelStatePublisher();
  bool initDynamixelStateSubscriber();

  bool setTorque(bool onoff);
  bool setPosition(uint32_t* pos);

  bool readDynamixelState();

  uint32_t convertRadian2Value(float radian);
  float convertValue2Radian(int32_t value);

  void goalPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg);
};
}

#endif //OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H
