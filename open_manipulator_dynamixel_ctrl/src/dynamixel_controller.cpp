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

#include "open_manipulator_dynamixel_ctrl/dynamixel_controller.h"

using namespace open_manipulator_dynamixel_controller;

DynamixelController::DynamixelController()
    :nh_(""),
     nh_priv_("~")
{
  if (!loadDynamixel())
    ROS_ERROR("Cant' Load Dynamixel, Please check Parameter");

  if (!multi_driver_->initSyncWrite())
    ROS_ERROR("Init SyncWrite Failed!");

  if (!multi_driver_->initSyncRead())
    ROS_ERROR("Init SyncRead Failed!");

  writeValue_ = new WriteValue;
  readValue_  = new ReadValue;

  setTorque(true);

  initDynamixelStatePublisher();

  initDynamixelStateSubscriber();

  ROS_INFO("open_manipulator_dynamixel_controller : Init OK!");
}

DynamixelController::~DynamixelController()
{
  setTorque(false);

  ros::shutdown();
}

bool DynamixelController::loadDynamixel()
{
  bool ret = false;

  for (int id = 1; id <= MAX_DXL_NUM; id++)
  {
    dynamixel_driver::DynamixelInfo *dynamixel= new dynamixel_driver::DynamixelInfo;

    dynamixel->lode_info.device_name      = nh_.param<std::string>("device_name", "/dev/ttyUSB0");
    dynamixel->lode_info.baud_rate        = nh_.param<int>("baud_rate", 1000000);
    dynamixel->lode_info.protocol_version = nh_.param<float>("protocol_version", 2.0);

    dynamixel->model_id                   = id;

    dynamixel_info_.push_back(dynamixel);
  }

  multi_driver_ = new dynamixel_multi_driver::DynamixelMultiDriver(dynamixel_info_[MOTOR]->lode_info.device_name,
                                                                   dynamixel_info_[MOTOR]->lode_info.baud_rate,
                                                                   dynamixel_info_[MOTOR]->lode_info.protocol_version);

  ret =  multi_driver_->loadDynamixel(dynamixel_info_);

 return ret;
}

bool DynamixelController::setTorque(bool onoff)
{
  writeValue_->torque.clear();
  for (int id = 1; id <= MAX_DXL_NUM; id++)
  {
    writeValue_->torque.push_back(onoff);
  }

  if (!multi_driver_->syncWriteTorque(writeValue_->torque))
  {
    ROS_ERROR("SyncWrite Torque Failed!");
    return false;
  }

  return true;
}

bool DynamixelController::setPosition(uint32_t* pos)
{
  writeValue_->pos.clear();

  for (int id = 1; id <= MAX_DXL_NUM; id++)
  {
    writeValue_->pos.push_back(pos[id-1]);
  }

  if (!multi_driver_->syncWritePosition(writeValue_->pos))
  {
    ROS_ERROR("SyncWrite Position Failed!");
    return false;
  }

  return true;
}

bool DynamixelController::initDynamixelStatePublisher()
{
  present_dynamixel_position_pub_   = nh_.advertise<sensor_msgs::JointState>("/robotis/dynamixel/present_states", 10);
}

bool DynamixelController::initDynamixelStateSubscriber()
{
  goal_dynamixel_position_sub_   = nh_.subscribe("/robotis/dynamixel/goal_states", 10,
                                          &DynamixelController::goalPositionMsgCallback, this);
}

bool DynamixelController::readDynamixelState(void)
{
  readValue_->pos.clear();
  if (!multi_driver_->syncReadPosition(readValue_->pos))
  {
    ROS_ERROR("Sync Read Failed!");
    return true;
  }

  sensor_msgs::JointState dynamixel_position;

  for (int id = 1; id <= MAX_DXL_NUM; id++)
  {
    std::stringstream id_num;
    id_num << "id_" << id;

    dynamixel_position.name.push_back(id_num.str());
    dynamixel_position.position.push_back(convertValue2Radian((int32_t)readValue_->pos.at(id-1)));
  }
  present_dynamixel_position_pub_.publish(dynamixel_position);
}

void DynamixelController::goalPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  uint32_t goal_position[MAX_DXL_NUM] = {0, };

  for (int id = 1; id <= MAX_DXL_NUM; id++)
  {
    goal_position[id-1] = convertRadian2Value(msg->position.at(id-1));

    ROS_INFO("goal_joint_position[%d] : %lf", id, msg->position.at(id-1));
  }

  setPosition(goal_position);
}

uint32_t DynamixelController::convertRadian2Value(float radian)
{
  uint32_t value = 0;

  if (radian > 0)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_ <= multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
      return multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_;

    value = (radian * (multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) / multi_driver_->multi_dynamixel_[MOTOR]->max_radian_)
                + multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_ >= multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
      return multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_;

    value = (radian * (multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) / multi_driver_->multi_dynamixel_[MOTOR]->min_radian_)
                + multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_;
  }
  else
    value = multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_;

//  if (value > multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_)
//    return multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_;
//  else if (value < multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_)
//    return multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_;

  return value;
}

float DynamixelController::convertValue2Radian(int32_t value)
{
  float radian = 0.0;

  if (value > multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->max_radian_ <= 0)
      return multi_driver_->multi_dynamixel_[MOTOR]->max_radian_;

    radian = (float) (value - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) * multi_driver_->multi_dynamixel_[MOTOR]->max_radian_
               / (float) (multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_);
  }
  else if (value < multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->min_radian_ >= 0)
      return multi_driver_->multi_dynamixel_[MOTOR]->min_radian_;

    radian = (float) (value - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) * multi_driver_->multi_dynamixel_[MOTOR]->min_radian_
               / (float) (multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_);
  }

//  if (radian > dynamixel_[PAN_TILT_MOTOR]->max_radian_)
//    return dynamixel_[PAN_TILT_MOTOR]->max_radian_;
//  else if (radian < dynamixel_[PAN_TILT_MOTOR]->min_radian_)
//    return dynamixel_[PAN_TILT_MOTOR]->min_radian_;

  return radian;
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
