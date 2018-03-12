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

#include "open_manipulator_dynamixel_ctrl/dynamixel_controller.h"

using namespace dynamixel;

DynamixelController::DynamixelController()
    :node_handle_("")
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 1000000);

  uint8_t scan_range        = node_handle_.param<int>("scan_range", 100);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);
  dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range);

  dxl_wb_->addSyncWrite("Goal_Position");
  dxl_wb_->addSyncRead("Present_Position");
  dxl_wb_->addSyncRead("Present_Velocity");

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->jointMode(dxl_id_[index]);

  initPublisher();
  initSubscriber();

  ROS_INFO("open_manipulator_dynamixel_controller : Init OK!");
}

DynamixelController::~DynamixelController()
{
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void DynamixelController::initPublisher()
{
  present_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/robotis/dynamixel/present_states", 10);
}

void DynamixelController::initSubscriber()
{
  goal_states_sub_  = node_handle_.subscribe("/robotis/dynamixel/goal_states", 10, &DynamixelController::goalPositionMsgCallback, this);
}

void DynamixelController::readDynamixelState(void)
{
  int32_t* get_present_position = dxl_wb_->syncRead("Present_Position");
  int32_t present_position[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_position[index] = get_present_position[index];

  int32_t* get_present_velocity = dxl_wb_->syncRead("Present_Velocity");
  int32_t present_velocity[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_velocity[index] = get_present_velocity[index];

  sensor_msgs::JointState dynamixel_;
  dynamixel_.header.stamp = ros::Time::now();

  for (int index = 0; index < dxl_cnt_; index++)
  {
    std::stringstream id_num;
    id_num << "id_" << (int)(dxl_id_[index]);

    dynamixel_.name.push_back(id_num.str());

    dynamixel_.position.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]));
    dynamixel_.velocity.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]));
  }
  present_states_pub_.publish(dynamixel_);
}

void DynamixelController::goalPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  uint8_t index = dxl_cnt_;
  int32_t goal_position[index] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
  {
    goal_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], msg->position.at(index));

    ROS_INFO("goal_joint_position[%d] : %lf", dxl_id_[index], msg->position.at(index));
  }

  dxl_wb_->syncWrite("Goal_Position", goal_position);
}

bool DynamixelController::control_loop()
{
  // Read & Publish Dynamixel position
  readDynamixelState();
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_dynamixel_controller");
  DynamixelController dynamixel_controller;
  ros::Rate loop_rate(ITERATION_FREQUENCY);

  while (ros::ok())
  {
    dynamixel_controller.control_loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
