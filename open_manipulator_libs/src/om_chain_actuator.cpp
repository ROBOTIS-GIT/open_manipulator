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

#include "open_manipulator_libs/om_chain_actuator.h"

using namespace OM_CHAIN_ACTUATOR;


void JointDynamixel::init(std::string dxl_device_name, std::string dxl_baud_rate)
{
  joint_id_.push_back(11);
  joint_id_.push_back(12);
  joint_id_.push_back(13);
  joint_id_.push_back(14);

  joint_controller_   = new DynamixelWorkbench;

  joint_controller_->begin(dxl_device_name.c_str(), std::stoi(dxl_baud_rate));

  uint16_t get_model_number;
  for (uint8_t index = 0; index < JOINT_NUM; index++)
  {
    if (joint_controller_->ping(joint_id_.at(index), &get_model_number) != true)
    {
      return;
    }
  }
  return;
}

void JointDynamixel::setPositionControlMode()
{
  for (uint8_t num = 0; num < JOINT_NUM; num++)
    joint_controller_->jointMode(joint_id_.at(num));

  joint_controller_->addSyncWrite("Goal_Position");
  joint_controller_->addSyncRead("Present_Position");
  joint_controller_->addSyncRead("Present_Velocity");
}

void JointDynamixel::setTorqueEnable()
{
  for (uint8_t num = 0; num < JOINT_NUM; num++)
    joint_controller_->itemWrite(joint_id_.at(num), "Torque_Enable", true);
}
void JointDynamixel::setTorqueDisable()
{
  for (uint8_t num = 0; num < JOINT_NUM; num++)
    joint_controller_->itemWrite(joint_id_.at(num), "Torque_Enable", false);
}

bool JointDynamixel::goalAllJointAngle(std::vector<double> radian_vector)
{

  int32_t goal_position[JOINT_NUM] = {0, };
  for (int index = 0; index < JOINT_NUM; index++)
    goal_position[index] = joint_controller_->convertRadian2Value(joint_id_.at(index), radian_vector.at(index));

  joint_controller_->syncWrite("Goal_Position", goal_position);

  return true;
}

bool JointDynamixel::goalMultipleJointAngle(std::vector<uint8_t> id, std::vector<double> radian_vector)
{
  for (int index = 0; index < id.size(); index++)
  {
    int32_t goal_position = joint_controller_->convertRadian2Value(id.at(index), radian_vector.at(index));
    joint_controller_->itemWrite(id.at(index),"Goal_Position", goal_position);
  }
  return true;
}

bool JointDynamixel::goalJointAngle(uint8_t actuator_id, double radian)
{
  int32_t goal_position = joint_controller_->convertRadian2Value(actuator_id, radian);
  joint_controller_->itemWrite(actuator_id,"Goal_Position", goal_position);
  return true;
}
std::vector<double> JointDynamixel::receiveAllJointAngle()
{
  std::vector<double> value;

  int32_t* get_joint_present_position = NULL;
  get_joint_present_position = joint_controller_->syncRead("Present_Position");

  for (int index = 0; index < JOINT_NUM; index++)
    value.push_back(joint_controller_->convertValue2Radian(joint_id_.at(index), get_joint_present_position[index]));

  return value;
}

std::vector<double> JointDynamixel::receiveAllJointVelocity()
{
  std::vector<double> value_vel;
  int32_t* get_joint_present_velocity = NULL;
  get_joint_present_velocity = joint_controller_->syncRead("Present_Velocity");

  for (int index = 0; index < JOINT_NUM; index++)
    value_vel.push_back(joint_controller_->convertValue2Velocity(joint_id_.at(index), get_joint_present_velocity[index]));

  return value_vel;
}


void JointDynamixel::initActuator(const void *arg)
{
  std::string *get_arg_ = (std::string *)arg;
  init(get_arg_[0], get_arg_[1]);
}
void JointDynamixel::setActuatorControlMode()
{
  setPositionControlMode();
}

void JointDynamixel::Enable()
{
  setTorqueEnable();
}
void JointDynamixel::Disable()
{
  setTorqueDisable();
}

bool JointDynamixel::sendAllActuatorAngle(std::vector<double> radian_vector)
{
  return goalAllJointAngle(radian_vector);
}
bool JointDynamixel::sendMultipleActuatorAngle(std::vector<uint8_t> id, std::vector<double> radian_vector)
{
  return goalMultipleJointAngle(id, radian_vector);
}
bool JointDynamixel::sendActuatorAngle(uint8_t actuator_id, double radian)
{
  return goalJointAngle(actuator_id, radian);
}
std::vector<double> JointDynamixel::receiveAllActuatorAngle()
{
  return receiveAllJointAngle();
}
