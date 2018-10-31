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

#include "open_manipulator_libs/om_link_dynamixel.h"

using namespace RM_DYNAMIXEL;


void Dynamixel::init(std::vector<uint8_t> actuator_id, const void *arg)
{
  dynamixel_num_=actuator_id.size();
  std::string *get_arg_ = (std::string *)arg;
  iniialize(actuator_id ,get_arg_[0], get_arg_[1]);
}

void Dynamixel::setMode(std::vector<uint8_t> actuator_id, const void *arg)
{
  std::string *get_arg_ = (std::string *)arg;

  if(get_arg_[0]=="position_mode" || get_arg_[0]=="current_mode")
    setOperatingMode(actuator_id, get_arg_[0]);
  else
    writeProfileValue(actuator_id, get_arg_[0], std::stoi(get_arg_[1]));
}

std::vector<uint8_t> Dynamixel::getId()
{
  return dynamixel_id_;
}

void Dynamixel::enable()
{
  writeTorqueEnable(dynamixel_id_, true);
}

void Dynamixel::disable()
{
  writeTorqueEnable(dynamixel_id_, false);
}

bool Dynamixel::sendJointActuatorValue(uint8_t actuator_id, ROBOTIS_MANIPULATOR::Actuator value)
{

}
bool Dynamixel::sendMultipleJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ROBOTIS_MANIPULATOR::Actuator> value_vector)
{

}
ROBOTIS_MANIPULATOR::Actuator Dynamixel::receiveJointActuatorValue(uint8_t actuator_id)
{

}
std::vector<ROBOTIS_MANIPULATOR::Actuator> Dynamixel::receiveMultipleJointActuatorValue(std::vector<uint8_t> actuator_id)
{

}



//////////////////////////////////////////////////////////////////////////

void Dynamixel::iniialize(std::vector<uint8_t> actuator_id, std::string dxl_device_name, std::string dxl_baud_rate)
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

void Dynamixel::setOperatingMode(std::vector<uint8_t> actuator_id, std::string dynamixel_mode)
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

void Dynamixel::writeProfileValue(std::vector<uint8_t> actuator_id, std::string profile_mode, int8_t value)
{
  const char * char_profile_mode = profile_mode.c_str();
  for (uint8_t num = 0; num < actuator_id.size(); num++)
    dynamixel_controller_->itemWrite(actuator_id.at(num), char_profile_mode, value);
}

void Dynamixel::writeTorqueEnable(std::vector<uint8_t> actuator_id, int8_t value)
{
  for (uint8_t num = 0; num < actuator_id.size(); num++)
    dynamixel_controller_->itemWrite(actuator_id.at(num), "Torque_Enable", value);
}

void Dynamixel::writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector)
{
  int32_t goal_position[dynamixel_num_] = {0, };




  for (int index = 0; index < JOINT_NUM; index++)
    goal_position[index] = dynamixel_controller_->convertRadian2Value(dynamixel_id_.at(index), radian_vector.at(index));
  dynamixel_controller_->syncWrite("Goal_Position", goal_position);

  return true;
}








///////////////////////////////////////////////


bool Dynamixel::goalAllJointAngle(std::vector<double> radian_vector)
{
  int32_t goal_position[JOINT_NUM] = {0, };
  for (int index = 0; index < JOINT_NUM; index++)
    goal_position[index] = dynamixel_controller_->convertRadian2Value(dynamixel_id_.at(index), radian_vector.at(index));

  dynamixel_controller_->syncWrite("Goal_Position", goal_position);

  return true;
}

bool Dynamixel::goalMultipleJointAngle(std::vector<uint8_t> id, std::vector<double> radian_vector)
{
  for (int index = 0; index < id.size(); index++)
  {
    int32_t goal_position = dynamixel_controller_->convertRadian2Value(id.at(index), radian_vector.at(index));
    dynamixel_controller_->itemWrite(id.at(index),"Goal_Position", goal_position);
  }
  return true;
}

bool Dynamixel::goalJointAngle(uint8_t actuator_id, double radian)
{
  int32_t goal_position = dynamixel_controller_->convertRadian2Value(actuator_id, radian);
  dynamixel_controller_->itemWrite(actuator_id,"Goal_Position", goal_position);
  return true;
}
std::vector<double> Dynamixel::receiveAllJointAngle()
{
  std::vector<double> value;

  int32_t* get_joint_present_position = NULL;
  get_joint_present_position = dynamixel_controller_->syncRead("Present_Position");

  for (int index = 0; index < JOINT_NUM; index++)
    value.push_back(dynamixel_controller_->convertValue2Radian(dynamixel_id_.at(index), get_joint_present_position[index]));

  return value;
}

std::vector<double> Dynamixel::receiveAllJointVelocity()
{
  std::vector<double> value_vel;
  int32_t* get_joint_present_velocity = NULL;
  get_joint_present_velocity = dynamixel_controller_->syncRead("Present_Velocity");

  for (int index = 0; index < JOINT_NUM; index++)
    value_vel.push_back(dynamixel_controller_->convertValue2Velocity(dynamixel_id_.at(index), get_joint_present_velocity[index]));

  return value_vel;
}


void Dynamixel::initActuator(const void *arg)
{
  std::string *get_arg_ = (std::string *)arg;
  init(get_arg_[0], get_arg_[1]);
}
void Dynamixel::setActuatorControlMode()
{
  setPositionControlMode();
}

void Dynamixel::Enable()
{
  setTorqueEnable();
}
void Dynamixel::Disable()
{
  setTorqueDisable();
}

bool Dynamixel::sendAllActuatorAngle(std::vector<double> radian_vector)
{
  return goalAllJointAngle(radian_vector);
}
bool Dynamixel::sendMultipleActuatorAngle(std::vector<uint8_t> id, std::vector<double> radian_vector)
{
  return goalMultipleJointAngle(id, radian_vector);
}
bool Dynamixel::sendActuatorAngle(uint8_t actuator_id, double radian)
{
  return goalJointAngle(actuator_id, radian);
}
std::vector<double> Dynamixel::receiveAllActuatorAngle()
{
  return receiveAllJointAngle();
}
