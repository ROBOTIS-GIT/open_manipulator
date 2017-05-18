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
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(""),
     baud_rate_(0),
     motor_model_(""),
     protocol_version_(0.0)
{
  // Init parameter
  nh_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name", device_name_);
  nh_priv_.getParam("baud_rate", baud_rate_);
  nh_priv_.getParam("motor_model", motor_model_);
  nh_priv_.getParam("protocol_version", protocol_version_);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // ROS Publisher
  present_dynamixel_position_pub_   = nh_.advertise<sensor_msgs::JointState>("/robotis/dynamixel/present_states", 10);

  // ROS Subscriber
  goal_dynamixel_position_sub_   = nh_.subscribe("/robotis/dynamixel/goal_states", 10,
                                          &DynamixelController::goalPositionMsgCallback, this);

  // Open port
  if (portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the port(%s)!", device_name_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the port!");
    ROS_ASSERT(shutdownDynamixelController());
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud_rate_))
  {
    ROS_INFO("Succeeded to change the baudrate(%d)\n!", portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
    ROS_ASSERT(shutdownDynamixelController());
  }

  // Init target name
  ROS_ASSERT(initDynamixelController());
}

DynamixelController::~DynamixelController()
{
  ROS_ASSERT(shutdownDynamixelController());
}

bool DynamixelController::initDynamixelController(void)
{
  // Init dynamixel
  nh_priv_.getParam("dynamixel_1/id", id_[0]);
  initMotor(motor_model_, id_[0], protocol_version_);

  nh_priv_.getParam("dynamixel_2/id", id_[1]);
  initMotor(motor_model_, id_[1], protocol_version_);

  nh_priv_.getParam("dynamixel_3/id", id_[2]);
  initMotor(motor_model_, id_[2], protocol_version_);

  nh_priv_.getParam("dynamixel_4/id", id_[3]);
  initMotor(motor_model_, id_[3], protocol_version_);

  nh_priv_.getParam("dynamixel_5/id", id_[4]);
  initMotor(motor_model_, id_[4], protocol_version_);

  // Init SyncWrite
  dynamixel_[id_[0]]->item_ = dynamixel_[id_[0]]->ctrl_table_["torque_enable"];
  dynamixelTorqueSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_,
                                                            packetHandler_,
                                                            dynamixel_[id_[0]]->item_->address,
                                                            dynamixel_[id_[0]]->item_->data_length);

  dynamixel_[id_[0]]->item_ = dynamixel_[id_[0]]->ctrl_table_["goal_position"];
  dynamixelGoalPositionSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_,
                                                                  packetHandler_,
                                                                  dynamixel_[id_[0]]->item_->address,
                                                                  dynamixel_[id_[0]]->item_->data_length);
  // Init SyncRead
  dynamixel_[id_[0]]->item_ = dynamixel_[id_[0]]->ctrl_table_["present_position"];
  dynamixelPresentPositionSyncRead_ = new dynamixel::GroupSyncRead(portHandler_,
                                                                   packetHandler_,
                                                                   dynamixel_[id_[0]]->item_->address,
                                                                   dynamixel_[id_[0]]->item_->data_length);

  for (int id_index = id_[0]; id_index <= MAX_DXL_NUM; id_index++)
  {
    if (dynamixelPresentPositionSyncRead_->addParam(id_index) != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", id_index);
      return 0;
    }
  }

  ROS_INFO("open_manipulator_dynamixel_controller : Init OK!");
  return true;
}

bool DynamixelController::shutdownDynamixelController(void)
{
  setTorque(false);
  portHandler_->closePort();
  ros::shutdown();
  return true;
}

bool DynamixelController::initMotor(std::string motor_model, uint8_t motor_id, float protocol_version)
{
  dynamixel_tool::DynamixelTool *dynamixel_motor = new dynamixel_tool::DynamixelTool(motor_id, motor_model, protocol_version);
  dynamixel_[motor_id] = dynamixel_motor;
}

bool DynamixelController::dynamixelControl(int64_t dynamixel_position[MAX_DXL_NUM])
{
  bool dynamixel_addparam_result;
  int8_t dynamixel_comm_result;

  int8_t index = 0;

  while(index < MAX_DXL_NUM)
  {
    dynamixel_addparam_result = dynamixelGoalPositionSyncWrite_->addParam(id_[index], (uint8_t*)&dynamixel_position[index]);

    if (dynamixel_addparam_result != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", id_[index]);
      return false;
    }

    index++;
  }

  dynamixel_comm_result = dynamixelGoalPositionSyncWrite_->txPacket();
  if (dynamixel_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    return false;
  }

  dynamixelGoalPositionSyncWrite_->clearParam();
  return true;
}

bool DynamixelController::setTorque(bool onoff)
{
  bool dynamixel_addparam_result;
  int8_t dynamixel_comm_result;

  int8_t index = 0;

  while (index < MAX_DXL_NUM)
  {
    dynamixel_addparam_result = dynamixelTorqueSyncWrite_->addParam(id_[index], (uint8_t*)&onoff);

    if (dynamixel_addparam_result != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", id_[index]);
      return 0;
    }

    index++;
  }

  dynamixel_comm_result = dynamixelTorqueSyncWrite_->txPacket();
  if (dynamixel_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    return 0;
  }

  dynamixelTorqueSyncWrite_->clearParam();
  return true;
}

bool DynamixelController::dynamixelPresentPosition(void)
{
  bool dynamixel_getdata_result;
  int8_t dynamixel_comm_result;

  sensor_msgs::JointState dynamixel_position;
  int32_t present_dynamixel_position;

  dynamixel_comm_result = dynamixelPresentPositionSyncRead_->txRxPacket();
  if (dynamixel_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    return 0;
  }

  for (int id_index = id_[0]; id_index <= MAX_DXL_NUM; id_index++)
  {
    dynamixel_[id_index]->item_ = dynamixel_[id_index]->ctrl_table_["present_position"];
    dynamixel_getdata_result = dynamixelPresentPositionSyncRead_->isAvailable(id_index, dynamixel_[id_index]->item_->address, dynamixel_[id_index]->item_->data_length);
    if (dynamixel_getdata_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", id_index);
      return 0;
    }

    std::stringstream id_num;
    id_num << "id_" << id_index;
    present_dynamixel_position = dynamixelPresentPositionSyncRead_->getData(id_index,
                                                                            dynamixel_[id_index]->item_->address,
                                                                            dynamixel_[id_index]->item_->data_length);

    dynamixel_position.name.push_back(id_num.str());
    dynamixel_position.position.push_back(convertValue2Radian(present_dynamixel_position));
  }

  present_dynamixel_position_pub_.publish(dynamixel_position);
}

bool DynamixelController::subscribePosition(void)
{
  // Read & Publish Dynamixel position
  dynamixelPresentPosition();
}

void DynamixelController::goalPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  int64_t goal_position[MAX_DXL_NUM] = {0,};

  for (int id_num = id_[0]; id_num <= MAX_DXL_NUM; id_num++)
  {
    goal_position[id_num-1] = convertRadian2Value(msg->position.at(id_num-1));

    ROS_INFO("goal_joint_position[%d] : %lf", id_num, msg->position.at(id_num-1));
  }

  dynamixelControl(goal_position);
}

int64_t DynamixelController::convertRadian2Value(double radian)
{
  int64_t value = 0;
  if (radian > 0)
  {
    if (dynamixel_[id_[0]]->value_of_max_radian_position_ <= dynamixel_[id_[0]]->value_of_0_radian_position_)
      return dynamixel_[id_[0]]->value_of_max_radian_position_;

    value = (radian * (dynamixel_[id_[0]]->value_of_max_radian_position_ - dynamixel_[id_[0]]->value_of_0_radian_position_) / dynamixel_[id_[0]]->max_radian_)
                + dynamixel_[id_[0]]->value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (dynamixel_[id_[0]]->value_of_min_radian_position_ >= dynamixel_[id_[0]]->value_of_0_radian_position_)
      return dynamixel_[id_[0]]->value_of_min_radian_position_;

    value = (radian * (dynamixel_[id_[0]]->value_of_min_radian_position_ - dynamixel_[id_[0]]->value_of_0_radian_position_) / dynamixel_[id_[0]]->min_radian_)
                + dynamixel_[id_[0]]->value_of_0_radian_position_;
  }
  else
    value = dynamixel_[id_[0]]->value_of_0_radian_position_;

  if (value > dynamixel_[id_[0]]->value_of_max_radian_position_)
    return dynamixel_[id_[0]]->value_of_max_radian_position_;
  else if (value < dynamixel_[id_[0]]->value_of_min_radian_position_)
    return dynamixel_[id_[0]]->value_of_min_radian_position_;

  return value;
}

double DynamixelController::convertValue2Radian(int32_t value)
{
  double radian = 0.0;
  if (value > dynamixel_[id_[0]]->value_of_0_radian_position_)
  {
    if (dynamixel_[id_[0]]->max_radian_ <= 0)
      return dynamixel_[id_[0]]->max_radian_;

    radian = (double) (value - dynamixel_[id_[0]]->value_of_0_radian_position_) * dynamixel_[id_[0]]->max_radian_
               / (double) (dynamixel_[id_[0]]->value_of_max_radian_position_ - dynamixel_[id_[0]]->value_of_0_radian_position_);
  }
  else if (value < dynamixel_[id_[0]]->value_of_0_radian_position_)
  {
    if (dynamixel_[id_[0]]->min_radian_ >= 0)
      return dynamixel_[id_[0]]->min_radian_;

    radian = (double) (value - dynamixel_[id_[0]]->value_of_0_radian_position_) * dynamixel_[id_[0]]->min_radian_
               / (double) (dynamixel_[id_[0]]->value_of_min_radian_position_ - dynamixel_[id_[0]]->value_of_0_radian_position_);
  }

  if (radian > dynamixel_[id_[0]]->max_radian_)
    return dynamixel_[id_[0]]->max_radian_;
  else if (radian < dynamixel_[id_[0]]->min_radian_)
    return dynamixel_[id_[0]]->min_radian_;

  return radian;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_dynamixel_controller");
  DynamixelController dynamixel_controller;
  ros::Rate loop_rate(ITERATION_FREQUENCY);

  dynamixel_controller.setTorque(true);

  while (ros::ok())
  {
    dynamixel_controller.subscribePosition();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
