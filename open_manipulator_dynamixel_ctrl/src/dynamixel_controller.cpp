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
  present_joint_position_pub_   = nh_.advertise<sensor_msgs::JointState>("/robotis/dynamixel/present_joint_states", 10);
  present_gripper_position_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/dynamixel/present_gripper_states", 10);

  // ROS Subscriber
  goal_joint_position_sub_   = nh_.subscribe("/robotis/dynamixel/goal_joint_states", 10,
                                          &DynamixelController::goalJointPosition, this);
  goal_gripper_position_sub_ = nh_.subscribe("/robotis/dynamixel/goal_gripper_states", 10,
                                          &DynamixelController::goalGripperPosition, this);

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
  nh_priv_.getParam("joint1/id", joint_id_[0]);
  initMotor(motor_model_, joint_id_[0], protocol_version_);

  nh_priv_.getParam("joint2/id", joint_id_[1]);
  initMotor(motor_model_, joint_id_[1], protocol_version_);

  nh_priv_.getParam("joint3/id", joint_id_[2]);
  initMotor(motor_model_, joint_id_[2], protocol_version_);

  nh_priv_.getParam("joint4/id", joint_id_[3]);
  initMotor(motor_model_, joint_id_[3], protocol_version_);

  nh_priv_.getParam("gripper/id", gripper_id_);
  initMotor(motor_model_, gripper_id_, protocol_version_);

  // Init SyncWrite
  dynamixel_[joint_id_[0]]->item_ = dynamixel_[joint_id_[0]]->ctrl_table_["torque_enable"];
  jointTorqueSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_,
                                                        packetHandler_,
                                                        dynamixel_[joint_id_[0]]->item_->address,
                                                        dynamixel_[joint_id_[0]]->item_->data_length);

  dynamixel_[joint_id_[0]]->item_ = dynamixel_[joint_id_[0]]->ctrl_table_["goal_position"];
  jointGoalPositionSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_,
                                                          packetHandler_,
                                                          dynamixel_[joint_id_[0]]->item_->address,
                                                          dynamixel_[joint_id_[0]]->item_->data_length);

  dynamixel_[gripper_id_]->item_ = dynamixel_[gripper_id_]->ctrl_table_["torque_enable"];
  gripperTorqueSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_,
                                                          packetHandler_,
                                                          dynamixel_[gripper_id_]->item_->address,
                                                          dynamixel_[gripper_id_]->item_->data_length);

  dynamixel_[gripper_id_]->item_ = dynamixel_[gripper_id_]->ctrl_table_["goal_position"];
  gripperGoalPositionSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_,
                                                            packetHandler_,
                                                            dynamixel_[gripper_id_]->item_->address,
                                                            dynamixel_[gripper_id_]->item_->data_length);

  // Init SyncRead
  dynamixel_[joint_id_[0]]->item_ = dynamixel_[joint_id_[0]]->ctrl_table_["present_position"];
  jointPresentPositionSyncRead_ = new dynamixel::GroupSyncRead(portHandler_,
                                                        packetHandler_,
                                                        dynamixel_[joint_id_[0]]->item_->address,
                                                        dynamixel_[joint_id_[0]]->item_->data_length);

  for (int joint_index = joint_id_[0]; joint_index <= MAX_JOINT_NUM; joint_index++)
  {
    if (jointPresentPositionSyncRead_->addParam(joint_index) != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", joint_index);
      return 0;
    }
  }

  dynamixel_[gripper_id_]->item_ = dynamixel_[gripper_id_]->ctrl_table_["present_position"];
  gripperPresentPositionSyncRead_ = new dynamixel::GroupSyncRead(portHandler_,
                                                          packetHandler_,
                                                          dynamixel_[gripper_id_]->item_->address,
                                                          dynamixel_[gripper_id_]->item_->data_length);

  if (gripperPresentPositionSyncRead_->addParam(gripper_id_) != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", gripper_id_);
    return 0;
  }

  ROS_INFO("open_manipulator_dynamixel_controller : Init OK!");
  return true;
}

bool DynamixelController::shutdownDynamixelController(void)
{
  //jointTorque(false);
  //gripperTorque(false);
  portHandler_->closePort();
  ros::shutdown();
  return true;
}

bool DynamixelController::initMotor(std::string motor_model, uint8_t motor_id, float protocol_version)
{
  dynamixel_tool::DynamixelTool *dynamixel_motor = new dynamixel_tool::DynamixelTool(motor_id, motor_model, protocol_version);
  dynamixel_[motor_id] = dynamixel_motor;
}

bool DynamixelController::moveJoints(int64_t joint_position[MAX_JOINT_NUM])
{
  bool dynamixel_addparam_result;
  int8_t dynamixel_comm_result;

  int8_t index = 0;

  while(index < MAX_JOINT_NUM)
  {
    dynamixel_addparam_result = jointGoalPositionSyncWrite_->addParam(joint_id_[index], (uint8_t*)&joint_position[index]);

    if (dynamixel_addparam_result != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", joint_id_[index]);
      return false;
    }

    index++;
  }

  dynamixel_comm_result = jointGoalPositionSyncWrite_->txPacket();
  if (dynamixel_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    return false;
  }

  jointGoalPositionSyncWrite_->clearParam();
  return true;
}

bool DynamixelController::moveGripper(int64_t gripper_position)
{
  bool dynamixel_addparam_result;
  int8_t dynamixel_comm_result;


  dynamixel_addparam_result = gripperGoalPositionSyncWrite_->addParam(gripper_id_, (uint8_t*)&gripper_position);

  if (dynamixel_addparam_result != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", gripper_id_);
    return false;
  }

  dynamixel_comm_result = gripperGoalPositionSyncWrite_->txPacket();
  if (dynamixel_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    return false;
  }

  gripperGoalPositionSyncWrite_->clearParam();
  return true;
}

bool DynamixelController::jointTorque(bool onoff)
{
  bool dynamixel_addparam_result;
  int8_t dynamixel_comm_result;

  int8_t index = 0;

  while (index < MAX_JOINT_NUM)
  {
    dynamixel_addparam_result = jointTorqueSyncWrite_->addParam(joint_id_[index], (uint8_t*)&onoff);

    if (dynamixel_addparam_result != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", joint_id_[index]);
      return 0;
    }

    index++;
  }

  dynamixel_comm_result = jointTorqueSyncWrite_->txPacket();
  if (dynamixel_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    return 0;
  }

  jointTorqueSyncWrite_->clearParam();
  return true;
}

bool DynamixelController::gripperTorque(bool onoff)
{
  bool dynamixel_addparam_result;
  int8_t dynamixel_comm_result;

  dynamixel_addparam_result = gripperTorqueSyncWrite_->addParam(gripper_id_, (uint8_t*)&onoff);

  if (dynamixel_addparam_result != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", gripper_id_);
    return 0;
  }

  dynamixel_comm_result = gripperTorqueSyncWrite_->txPacket();
  if (dynamixel_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    return 0;
  }

  gripperTorqueSyncWrite_->clearParam();
  return true;
}

bool DynamixelController::jointPresentPosition(void)
{
  bool dxl_getdata_result;
  int8_t dynamixel_comm_result;

  sensor_msgs::JointState joint_position;
  int32_t present_joint_position;

  dynamixel_comm_result = jointPresentPositionSyncRead_->txRxPacket();
  if (dynamixel_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    return 0;
  }

  for (int joint_index = joint_id_[0]; joint_index <= MAX_JOINT_NUM; joint_index++)
  {
    dynamixel_[joint_index]->item_ = dynamixel_[joint_index]->ctrl_table_["present_position"];
    dxl_getdata_result = jointPresentPositionSyncRead_->isAvailable(joint_index, dynamixel_[joint_index]->item_->address, dynamixel_[joint_index]->item_->data_length);
    if (dxl_getdata_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", joint_index);
      return 0;
    }

    std::stringstream joint_num;
    joint_num << "joint" << joint_index;
    present_joint_position = jointPresentPositionSyncRead_->getData(joint_index,
                                                             dynamixel_[joint_index]->item_->address,
                                                             dynamixel_[joint_index]->item_->data_length);

    joint_position.name.push_back(joint_num.str());
    joint_position.position.push_back(convertValue2Radian(present_joint_position));
  }

  present_joint_position_pub_.publish(joint_position);
}

bool DynamixelController::gripperPresentPosition(void)
{
  bool dxl_getdata_result;
  int8_t dynamixel_comm_result;

  sensor_msgs::JointState gripper_position;
  int32_t gripper_joint_position;

  dynamixel_comm_result = gripperPresentPositionSyncRead_->txRxPacket();
  if (dynamixel_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    return 0;
  }

  dynamixel_[gripper_id_]->item_ = dynamixel_[gripper_id_]->ctrl_table_["present_position"];
  dxl_getdata_result = gripperPresentPositionSyncRead_->isAvailable(gripper_id_, dynamixel_[gripper_id_]->item_->address, dynamixel_[gripper_id_]->item_->data_length);
  if (dxl_getdata_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", gripper_id_);
    return 0;
  }

  gripper_joint_position = gripperPresentPositionSyncRead_->getData(gripper_id_, dynamixel_[gripper_id_]->item_->address, dynamixel_[gripper_id_]->item_->data_length);

  gripper_position.name.push_back("grip_joint");
  gripper_position.position.push_back(convertValue2Radian(gripper_joint_position));

  present_gripper_position_pub_.publish(gripper_position);
}

bool DynamixelController::subscribePosition(void)
{
  // Read & Publish Dynamixel position
  jointPresentPosition();
  gripperPresentPosition();
}

void DynamixelController::goalJointPosition(const sensor_msgs::JointState::ConstPtr &msg)
{
  int64_t goal_joint_position[MAX_JOINT_NUM] = {0,};

  for (int joint_num = joint_id_[0]; joint_num <= MAX_JOINT_NUM; joint_num++)
  {
    goal_joint_position[joint_num-1] = convertRadian2Value(msg->position.at(joint_num-1));

    ROS_INFO("goal_joint_position[%d] : %lf", joint_num-1, msg->position.at(joint_num-1));
  }

  moveJoints(goal_joint_position);
}

void DynamixelController::goalGripperPosition(const sensor_msgs::JointState::ConstPtr &msg)
{
  int64_t goal_gripper_position = 0;

  goal_gripper_position = convertRadian2Value(msg->position.at(0));

  moveGripper(goal_gripper_position);
}

int64_t DynamixelController::convertRadian2Value(double radian)
{
  int64_t value = 0;
  if (radian > 0)
  {
    if (dynamixel_[joint_id_[0]]->value_of_max_radian_position_ <= dynamixel_[joint_id_[0]]->value_of_0_radian_position_)
      return dynamixel_[joint_id_[0]]->value_of_max_radian_position_;

    value = (radian * (dynamixel_[joint_id_[0]]->value_of_max_radian_position_ - dynamixel_[joint_id_[0]]->value_of_0_radian_position_) / dynamixel_[joint_id_[0]]->max_radian_)
                + dynamixel_[joint_id_[0]]->value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (dynamixel_[joint_id_[0]]->value_of_min_radian_position_ >= dynamixel_[joint_id_[0]]->value_of_0_radian_position_)
      return dynamixel_[joint_id_[0]]->value_of_min_radian_position_;

    value = (radian * (dynamixel_[joint_id_[0]]->value_of_min_radian_position_ - dynamixel_[joint_id_[0]]->value_of_0_radian_position_) / dynamixel_[joint_id_[0]]->min_radian_)
                + dynamixel_[joint_id_[0]]->value_of_0_radian_position_;
  }
  else
    value = dynamixel_[joint_id_[0]]->value_of_0_radian_position_;

  if (value > dynamixel_[joint_id_[0]]->value_of_max_radian_position_)
    return dynamixel_[joint_id_[0]]->value_of_max_radian_position_;
  else if (value < dynamixel_[joint_id_[0]]->value_of_min_radian_position_)
    return dynamixel_[joint_id_[0]]->value_of_min_radian_position_;

  return value;
}

double DynamixelController::convertValue2Radian(int32_t value)
{
  double radian = 0.0;
  if (value > dynamixel_[joint_id_[0]]->value_of_0_radian_position_)
  {
    if (dynamixel_[joint_id_[0]]->max_radian_ <= 0)
      return dynamixel_[joint_id_[0]]->max_radian_;

    radian = (double) (value - dynamixel_[joint_id_[0]]->value_of_0_radian_position_) * dynamixel_[joint_id_[0]]->max_radian_
               / (double) (dynamixel_[joint_id_[0]]->value_of_max_radian_position_ - dynamixel_[joint_id_[0]]->value_of_0_radian_position_);
  }
  else if (value < dynamixel_[joint_id_[0]]->value_of_0_radian_position_)
  {
    if (dynamixel_[joint_id_[0]]->min_radian_ >= 0)
      return dynamixel_[joint_id_[0]]->min_radian_;

    radian = (double) (value - dynamixel_[joint_id_[0]]->value_of_0_radian_position_) * dynamixel_[joint_id_[0]]->min_radian_
               / (double) (dynamixel_[joint_id_[0]]->value_of_min_radian_position_ - dynamixel_[joint_id_[0]]->value_of_0_radian_position_);
  }

  if (radian > dynamixel_[joint_id_[0]]->max_radian_)
    return dynamixel_[joint_id_[0]]->max_radian_;
  else if (radian < dynamixel_[joint_id_[0]]->min_radian_)
    return dynamixel_[joint_id_[0]]->min_radian_;

  return radian;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_dynamixel_controller");
  DynamixelController dynamixel_controller;
  ros::Rate loop_rate(ITERATION_FREQUENCY);

  dynamixel_controller.jointTorque(true);
  dynamixel_controller.gripperTorque(true);

  while (ros::ok())
  {
    dynamixel_controller.subscribePosition();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
