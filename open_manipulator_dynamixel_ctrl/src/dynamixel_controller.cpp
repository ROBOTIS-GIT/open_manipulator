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

double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

DynamixelController::DynamixelController()
    :node_handle_(""),
     priv_node_handle_("~")
{
  robot_name_   = priv_node_handle_.param<std::string>("robot_name", "open_manipulator");

  std::string device_name   = priv_node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = priv_node_handle_.param<int>("baud_rate", 1000000);
  protocol_version_         = priv_node_handle_.param<float>("protocol_version", 2.0);

  joint_mode_   = priv_node_handle_.param<std::string>("joint_controller", "position_mode");

  joint_id_.push_back(priv_node_handle_.param<int>("joint1_id", 1));
  joint_id_.push_back(priv_node_handle_.param<int>("joint2_id", 2));
  joint_id_.push_back(priv_node_handle_.param<int>("joint3_id", 3));
  joint_id_.push_back(priv_node_handle_.param<int>("joint4_id", 4));

  gripper_mode_ = priv_node_handle_.param<std::string>("gripper_controller", "current_mode");

  gripper_id_.push_back(priv_node_handle_.param<int>("gripper_id", 5));

  joint_controller_   = new DynamixelWorkbench;
  gripper_controller_ = new DynamixelWorkbench;

  joint_controller_->begin(device_name.c_str(), dxl_baud_rate);
  gripper_controller_->begin(device_name.c_str(), dxl_baud_rate);

  getDynamixelInst();

  initPublisher();
  initSubscriber();

  ROS_INFO("open_manipulator_dynamixel_controller : Init OK!");
}

DynamixelController::~DynamixelController()
{
  for (uint8_t num = 0; num < JOINT_NUM; num++)
    joint_controller_->itemWrite(joint_id_.at(num), "Torque_Enable", false);

  gripper_controller_->itemWrite(gripper_id_.at(0), "Torque_Enable", false);

  ros::shutdown();
}

void DynamixelController::initPublisher()
{
  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/joint_states", 10);
}

void DynamixelController::initSubscriber()
{
  goal_joint_states_sub_    = node_handle_.subscribe(robot_name_ + "/goal_joint_position", 10, &DynamixelController::goalJointPositionCallback, this);
  goal_gripper_states_sub_  = node_handle_.subscribe(robot_name_ + "/goal_gripper_position", 10, &DynamixelController::goalGripperPositionCallback, this);
}

void DynamixelController::getDynamixelInst()
{
  uint16_t get_model_number;
  for (uint8_t index = 0; index < JOINT_NUM; index++)
  {

    if (joint_controller_->ping(joint_id_.at(index), &get_model_number) != true)
    {
      ROS_ERROR("Not found Joints, Please check id and baud rate");

      ros::shutdown();
      return;
    }
  }

  if (gripper_controller_->ping(gripper_id_.at(0), &get_model_number) != true)
  {
    ROS_ERROR("Not found Grippers, Please check id and baud rate");

    ros::shutdown();
    return;
  }

  setOperatingMode();
  setSyncFunction();
}

void DynamixelController::setOperatingMode()
{
  if (joint_mode_ == "position_mode")
  {
    for (uint8_t num = 0; num < JOINT_NUM; num++)
      joint_controller_->jointMode(joint_id_.at(num));
  }
  else if (joint_mode_ == "current_mode")
  {
    for (uint8_t num = 0; num < JOINT_NUM; num++)
      joint_controller_->currentMode(joint_id_.at(num));
  }
  else
  {
    for (uint8_t num = 0; num < JOINT_NUM; num++)
      joint_controller_->jointMode(joint_id_.at(num));
  }

  if (gripper_mode_ == "position_mode")
    gripper_controller_->jointMode(gripper_id_.at(0));
  else if (gripper_mode_ == "current_mode" && protocol_version_ == 2.0)
    gripper_controller_->currentMode(gripper_id_.at(0), 30);
  else
    gripper_controller_->jointMode(gripper_id_.at(0));
}

void DynamixelController::setSyncFunction()
{
  joint_controller_->addSyncWrite("Goal_Position");

  if (protocol_version_ == 2.0)
  {
    joint_controller_->addSyncRead("Present_Position");
    joint_controller_->addSyncRead("Present_Velocity");
  }
}

void DynamixelController::readPosition(double *value)
{
  int32_t* get_joint_present_position = NULL;

  if (protocol_version_ == 2.0)
    get_joint_present_position = joint_controller_->syncRead("Present_Position");
  else if (protocol_version_ == 1.0)
  {
    for (int index = 0; index < JOINT_NUM; index++)
      get_joint_present_position[index] = joint_controller_->itemRead(joint_id_.at(index), "Present_Position");
  }

  int32_t get_gripper_present_position = gripper_controller_->itemRead(gripper_id_.at(0), "Present_Position");
  int32_t present_position[DXL_NUM] = {0, };

  for (int index = 0; index < JOINT_NUM; index++)
    present_position[index] = get_joint_present_position[index];

  present_position[DXL_NUM-1] = get_gripper_present_position;

  for (int index = 0; index < JOINT_NUM; index++)
  {
    value[index] = joint_controller_->convertValue2Radian(joint_id_.at(index), present_position[index]);
  }

  value[DXL_NUM-1] = gripper_controller_->convertValue2Radian(gripper_id_.at(0), present_position[DXL_NUM-1]);
}

void DynamixelController::readVelocity(double *value)
{
  int32_t* get_joint_present_velocity = NULL;

  if (protocol_version_ == 2.0)
    get_joint_present_velocity = joint_controller_->syncRead("Present_Velocity");
  else if (protocol_version_ == 1.0)
  {
    for (int index = 0; index < JOINT_NUM; index++)
      get_joint_present_velocity[index] = joint_controller_->itemRead(joint_id_.at(index), "Present_Velocity");
  }

  int32_t get_gripper_present_velocity = gripper_controller_->itemRead(gripper_id_.at(0), "Present_Velocity");
  int32_t present_velocity[DXL_NUM] = {0, };

  for (int index = 0; index < JOINT_NUM; index++)
    present_velocity[index] = get_joint_present_velocity[index];

  present_velocity[DXL_NUM-1] = get_gripper_present_velocity;

  for (int index = 0; index < JOINT_NUM; index++)
  {
    value[index] = joint_controller_->convertValue2Velocity(joint_id_.at(index), present_velocity[index]);
  }

  value[DXL_NUM-1] = gripper_controller_->convertValue2Velocity(gripper_id_.at(0), present_velocity[DXL_NUM-1]);
}

void DynamixelController::updateJointStates()
{
  sensor_msgs::JointState joint_state;

  float joint_states_pos[JOINT_NUM + PALM_NUM] = {0.0, };
  float joint_states_vel[JOINT_NUM + PALM_NUM] = {0.0, };
  float joint_states_eff[JOINT_NUM + PALM_NUM] = {0.0, };

  double get_joint_position[JOINT_NUM + GRIPPER_NUM] = {0.0, };
  double get_joint_velocity[JOINT_NUM + GRIPPER_NUM] = {0.0, };

  readPosition(get_joint_position);
  readVelocity(get_joint_velocity);

  joint_state.header.frame_id = "world";
  joint_state.header.stamp    = ros::Time::now();

  joint_state.name.push_back("joint1");
  joint_state.name.push_back("joint2");
  joint_state.name.push_back("joint3");
  joint_state.name.push_back("joint4");
  joint_state.name.push_back("grip_joint");
  joint_state.name.push_back("grip_joint_sub");

  joint_states_pos[0] = get_joint_position[0];
  joint_states_pos[1] = get_joint_position[1];
  joint_states_pos[2] = get_joint_position[2];
  joint_states_pos[3] = get_joint_position[3];
  joint_states_pos[4] = mapd(get_joint_position[4], 0.90, -0.80, -0.01, 0.01);
  joint_states_pos[5] = joint_states_pos[4];

  joint_states_vel[0] = get_joint_velocity[0];
  joint_states_vel[1] = get_joint_velocity[1];
  joint_states_vel[2] = get_joint_velocity[2];
  joint_states_vel[3] = get_joint_velocity[3];
  joint_states_vel[4] = get_joint_velocity[4];
  joint_states_vel[5] = joint_states_vel[4];

  for (int index = 0; index < JOINT_NUM + PALM_NUM; index++)
  {
    joint_state.position.push_back(joint_states_pos[index]);
    joint_state.velocity.push_back(joint_states_vel[index]);
    joint_state.effort.push_back(joint_states_eff[index]);
  }

  joint_states_pub_.publish(joint_state);
}

void DynamixelController::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double goal_joint_position[JOINT_NUM] = {0.0, 0.0, 0.0, 0.0};

  for (int index = 0; index < JOINT_NUM; index++)
    goal_joint_position[index] = msg->position.at(index);

  int32_t goal_position[JOINT_NUM] = {0, };

  for (int index = 0; index < JOINT_NUM; index++)
  {
    goal_position[index] = joint_controller_->convertRadian2Value(joint_id_.at(index), goal_joint_position[index]);
  }

  joint_controller_->syncWrite("Goal_Position", goal_position);
}

void DynamixelController::goalGripperPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double goal_gripper_position = msg->position[0];
  goal_gripper_position = mapd(goal_gripper_position, -0.01, 0.01, 0.90, -0.80);

  gripper_controller_->itemWrite(gripper_id_.at(0), "Goal_Position", gripper_controller_->convertRadian2Value(gripper_id_.at(0), goal_gripper_position));
}

bool DynamixelController::control_loop()
{
  // Read & Publish Dynamixel position
  updateJointStates();
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
