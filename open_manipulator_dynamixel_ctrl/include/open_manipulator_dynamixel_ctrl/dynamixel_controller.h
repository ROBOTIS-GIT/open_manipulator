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
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <sensor_msgs/JointState.h>
#include <open_manipulator_msgs/JointPose.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace open_manipulator_dynamixel_controller
{
#define MAX_JOINT_NUM        (4)
#define ITERATION_FREQUENCY  (25)

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
  bool is_debug_;

  // ROS Publisher
  ros::Publisher present_joint_position_pub_;
  ros::Publisher present_gripper_position_pub_;

  // ROS Subscriber
  ros::Subscriber goal_joint_position_sub_;
  ros::Subscriber goal_gripper_position_sub_;

  // Dynamixel Parameters
  std::string device_name_;
  std::string motor_model_;
  float protocol_version_;
  int baud_rate_;

  // Joint states
  int joint_id_[MAX_JOINT_NUM];
  int gripper_id_;

  // Parameters
  std::map<int, dynamixel_tool::DynamixelTool *> dynamixel_;
  std::map<std::string, std::vector<int64_t> *> read_data_;

  dynamixel::GroupSyncWrite *jointTorqueSyncWrite_;
  dynamixel::GroupSyncWrite *jointGoalPositionSyncWrite_;
  dynamixel::GroupSyncWrite *gripperTorqueSyncWrite_;
  dynamixel::GroupSyncWrite *gripperGoalPositionSyncWrite_;
  dynamixel::GroupSyncRead  *jointPresentPositionSyncRead_;
  dynamixel::GroupSyncRead  *gripperPresentPositionSyncRead_;

 public:
  DynamixelController();
  ~DynamixelController();
  bool subscribePosition(void);

  bool initDynamixelController(void);
  bool shutdownDynamixelController(void);

  bool initMotor(std::string motor_model, uint8_t motor_id, float protocol_version);

  bool moveJoints(int64_t joint_position[MAX_JOINT_NUM]);
  bool moveGripper(int64_t gripper_position);
  bool jointTorque(bool onoff);
  bool gripperTorque(bool onoff);
  bool jointPresentPosition(void);
  bool gripperPresentPosition(void);

  int64_t convertRadian2Value(double radian);
  double convertValue2Radian(int32_t value);

  void goalJointPosition(const sensor_msgs::JointState::ConstPtr &msg);
  void goalGripperPosition(const sensor_msgs::JointState::ConstPtr &msg);
};
}

#endif //OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H
