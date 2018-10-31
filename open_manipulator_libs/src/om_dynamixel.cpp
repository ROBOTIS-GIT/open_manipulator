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

#include "open_manipulator_libs/om_dynamixel.h"

using namespace OM_DYNAMIXEL;


void JointDynamixel::init(std::vector<uint8_t> actuator_id, const void *arg)
{
  dynamixel_num_=actuator_id.size();
  std::string *get_arg_ = (std::string *)arg;
  iniialize(actuator_id ,get_arg_[0], get_arg_[1]);
}

void JointDynamixel::setMode(std::vector<uint8_t> actuator_id, const void *arg)
{
  std::string *get_arg_ = (std::string *)arg;

  if(get_arg_[0]=="position_mode" || get_arg_[0]=="current_mode")
    setOperatingMode(actuator_id, get_arg_[0]);
  else
    writeProfileValue(actuator_id, get_arg_[0], std::stoi(get_arg_[1]));
}

std::vector<uint8_t> JointDynamixel::getId()
{
  return dynamixel_id_;
}

void JointDynamixel::enable()
{
  writeTorqueEnable(dynamixel_id_, true);
}

void JointDynamixel::disable()
{
  writeTorqueEnable(dynamixel_id_, false);
}

bool JointDynamixel::sendJointActuatorValue(uint8_t actuator_id, ROBOTIS_MANIPULATOR::Actuator value)
{
  std::vector<uint8_t> id_vector;
  std::vector<double> radian_vector;
  std::vector<double> velocity_vector;
  id_vector.push_back(actuator_id);
  radian_vector.push_back(value.value);
  velocity_vector.push_back(value.velocity);

  writeGoalPosition(id_vector, radian_vector);
  writeGoalVelocity(id_vector, velocity_vector);
}

bool JointDynamixel::sendMultipleJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ROBOTIS_MANIPULATOR::Actuator> value_vector)
{
  std::vector<double> radian_vector;
  std::vector<double> velocity_vector;

  for(int index = 0; index < value_vector.size(); index++)
  {
    radian_vector.push_back(value_vector.at(index).value);
    velocity_vector.push_back(value_vector.at(index).velocity);
  }

  writeGoalPosition(actuator_id, radian_vector);
  writeGoalVelocity(actuator_id, velocity_vector);
}

ROBOTIS_MANIPULATOR::Actuator JointDynamixel::receiveJointActuatorValue(uint8_t actuator_id)
{
  std::vector<double> value;
  std::vector<double> velocity;
  ROBOTIS_MANIPULATOR::Actuator result;

  value = receiveAllDynamixelAngle();
  velocity = receiveAllDynamixelVelocity();

  for(int index = 0; index < dynamixel_num_; index++)
  {
    if(dynamixel_id_.at(index) == actuator_id)
    {
      result.value = value.at(index);
      result.velocity = velocity.at(index);
    }
  }
  return result;
}

std::vector<ROBOTIS_MANIPULATOR::Actuator> JointDynamixel::receiveMultipleJointActuatorValue(std::vector<uint8_t> actuator_id)
{
  std::vector<double> value;
  std::vector<double> velocity;
  ROBOTIS_MANIPULATOR::Actuator result;
  std::vector<ROBOTIS_MANIPULATOR::Actuator> result_vector;

  value = receiveAllDynamixelAngle();
  velocity = receiveAllDynamixelVelocity();

  for(int index = 0; index < dynamixel_num_; index++)
  {
    for(int index2 = 0; index2 < actuator_id.size(); index2++)
    {
      if(dynamixel_id_.at(index) == actuator_id.at(index2))
      {
        result.value = value.at(index);
        result.velocity = velocity.at(index);

        result_vector.push_back(result);
      }
    }
  }
  return result_vector;
}

//////////////////////////////////////////////////////////////////////////

void JointDynamixel::iniialize(std::vector<uint8_t> actuator_id, std::string dxl_device_name, std::string dxl_baud_rate)
{
  dynamixel_id_ = actuator_id;
  dynamixel_controller_   = new DynamixelWorkbench;

  dynamixel_controller_->begin(dxl_device_name.c_str(), std::stoi(dxl_baud_rate));

  uint16_t get_model_number;
  for (uint8_t index = 0; index < dynamixel_num_; index++)
  {
    if (dynamixel_controller_->ping(dynamixel_id_.at(index), &get_model_number) != true)
    {
      return;
    }
  }
  return;
}

void JointDynamixel::setOperatingMode(std::vector<uint8_t> actuator_id, std::string dynamixel_mode)
{
  if (dynamixel_mode == "position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
      dynamixel_controller_->jointMode(actuator_id.at(num));
  }
  else if (dynamixel_mode == "current_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
      dynamixel_controller_->currentMode(actuator_id.at(num));
  }
  else
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
      dynamixel_controller_->jointMode(actuator_id.at(num));
  }

  dynamixel_controller_->addSyncWrite("Goal_Position");

  dynamixel_controller_->addSyncRead("Present_Position");
  dynamixel_controller_->addSyncRead("Present_Velocity");
}

void JointDynamixel::writeProfileValue(std::vector<uint8_t> actuator_id, std::string profile_mode, int8_t value)
{
  const char * char_profile_mode = profile_mode.c_str();
  for (uint8_t num = 0; num < actuator_id.size(); num++)
    dynamixel_controller_->itemWrite(actuator_id.at(num), char_profile_mode, value);
}

void JointDynamixel::writeTorqueEnable(std::vector<uint8_t> actuator_id, int8_t value)
{
  for (uint8_t num = 0; num < actuator_id.size(); num++)
    dynamixel_controller_->itemWrite(actuator_id.at(num), "Torque_Enable", value);
}

void JointDynamixel::writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector)
{
  for(int index = 0; index < dynamixel_id_.size(); index++)
  {
    for(int index2 = 0; index2 < actuator_id.size(); index2++)
    {
      if(dynamixel_id_.at(index) == actuator_id.at(index2))
        goal_position_[index] = dynamixel_controller_->convertRadian2Value(dynamixel_id_.at(index), radian_vector.at(index2));
    }
  }
  dynamixel_controller_->syncWrite("Goal_Position", goal_position_);
}

void JointDynamixel::writeGoalVelocity(std::vector<uint8_t> actuator_id, std::vector<double> velocity_vector)
{
  for(int index = 0; index < dynamixel_id_.size(); index++)
  {
    for(int index2 = 0; index2 < actuator_id.size(); index2++)
    {
      if(dynamixel_id_.at(index) == actuator_id.at(index2))
        goal_velocity_[index] = dynamixel_controller_->convertRadian2Value(dynamixel_id_.at(index), velocity_vector.at(index2));
    }
  }
//  dynamixel_controller_->syncWrite("Goal_Velocity", goal_position_);
}

std::vector<double> JointDynamixel::receiveAllDynamixelAngle()
{
  std::vector<double> value;

  int32_t* get_joint_present_position = NULL;
  get_joint_present_position = dynamixel_controller_->syncRead("Present_Position");

  for (int index = 0; index < dynamixel_id_.size(); index++)
    value.push_back(dynamixel_controller_->convertValue2Radian(dynamixel_id_.at(index), get_joint_present_position[index]));

  return value;
}

std::vector<double> JointDynamixel::receiveAllDynamixelVelocity()
{
  std::vector<double> value_vel;
  int32_t* get_joint_present_velocity = NULL;
  get_joint_present_velocity = dynamixel_controller_->syncRead("Present_Velocity");

  for (int index = 0; index < dynamixel_id_.size(); index++)
    value_vel.push_back(dynamixel_controller_->convertValue2Velocity(dynamixel_id_.at(index), get_joint_present_velocity[index]));

  return value_vel;
}















//////////////////////////////////////tool actuator

void GripperDynamixel::init(uint8_t actuator_id, const void *arg)
{
  std::string *get_arg_ = (std::string *)arg;
  iniialize(actuator_id ,get_arg_[0], get_arg_[1]);
}

void GripperDynamixel::setMode(uint8_t actuator_id, const void *arg)
{
  std::string *get_arg_ = (std::string *)arg;

  if(get_arg_[0]=="position_mode" || get_arg_[0]=="current_mode")
    setOperatingMode(actuator_id, get_arg_[0]);
  else
    writeProfileValue(actuator_id, get_arg_[0], std::stoi(get_arg_[1]));
}

uint8_t GripperDynamixel::getId()
{
  return dynamixel_id_;
}

void GripperDynamixel::enable()
{
  writeTorqueEnable(dynamixel_id_, true);
}

void GripperDynamixel::disable()
{
  writeTorqueEnable(dynamixel_id_, false);
}

bool GripperDynamixel::sendJointActuatorValue(uint8_t actuator_id, ROBOTIS_MANIPULATOR::Actuator value)
{
  double radian;
  double velocity;

  writeGoalPosition(actuator_id, value.value);
  writeGoalVelocity(actuator_id, value.velocity);
}


ROBOTIS_MANIPULATOR::Actuator GripperDynamixel::receiveJointActuatorValue(uint8_t actuator_id)
{
  ROBOTIS_MANIPULATOR::Actuator result;

  result.value = receiveDynamixelAngle();
  result.velocity = receiveDynamixelVelocity();

  return result;
}

//////////////////////////////////////////////////////////////////////////

void GripperDynamixel::iniialize(uint8_t actuator_id, std::string dxl_device_name, std::string dxl_baud_rate)
{
  dynamixel_id_ = actuator_id;
  dynamixel_controller_   = new DynamixelWorkbench;

  dynamixel_controller_->begin(dxl_device_name.c_str(), std::stoi(dxl_baud_rate));

  uint16_t get_model_number;
  for (uint8_t index = 0; index < dynamixel_num_; index++)
  {
    if (dynamixel_controller_->ping(dynamixel_id_, &get_model_number) != true)
    {
      return;
    }
  }
  return;
}

void GripperDynamixel::setOperatingMode(uint8_t actuator_id, std::string dynamixel_mode)
{
  if (dynamixel_mode == "position_mode")
  {
    dynamixel_controller_->jointMode(actuator_id);
  }
  else if (dynamixel_mode == "current_mode")
  {
    dynamixel_controller_->currentMode(actuator_id);
  }
  else
  {
    dynamixel_controller_->jointMode(actuator_id);
  }

  dynamixel_controller_->addSyncWrite("Goal_Position");

  dynamixel_controller_->addSyncRead("Present_Position");
  dynamixel_controller_->addSyncRead("Present_Velocity");
}

void GripperDynamixel::writeProfileValue(uint8_t actuator_id, std::string profile_mode, int8_t value)
{
  const char * char_profile_mode = profile_mode.c_str();
  dynamixel_controller_->itemWrite(actuator_id, char_profile_mode, value);
}

void GripperDynamixel::writeTorqueEnable(uint8_t actuator_id, int8_t value)
{
    dynamixel_controller_->itemWrite(actuator_id, "Torque_Enable", value);
}

void GripperDynamixel::writeGoalPosition(uint8_t actuator_id, double radian)
{
  int32_t goal_position[1];
  goal_position[0] = dynamixel_controller_->convertRadian2Value(dynamixel_id_, radian);

  dynamixel_controller_->syncWrite("Goal_Position", goal_position);
}

void GripperDynamixel::writeGoalVelocity(uint8_t actuator_id, double velocity)
{
  int32_t goal_velocity_[1];
  goal_velocity_[0] = dynamixel_controller_->convertRadian2Value(dynamixel_id_, velocity);

  dynamixel_controller_->syncWrite("Goal_Velocity", goal_velocity_);
}

double GripperDynamixel::receiveDynamixelAngle()
{
  int32_t* get_joint_present_position = NULL;
  get_joint_present_position = dynamixel_controller_->syncRead("Present_Position");

  return dynamixel_controller_->convertValue2Radian(dynamixel_id_, get_joint_present_position[0]);
}

double GripperDynamixel::receiveDynamixelVelocity()
{
  int32_t* get_joint_present_velocity = NULL;
  get_joint_present_velocity = dynamixel_controller_->syncRead("Present_Velocity");

  return dynamixel_controller_->convertValue2Velocity(dynamixel_id_, get_joint_present_velocity[0]);
}

