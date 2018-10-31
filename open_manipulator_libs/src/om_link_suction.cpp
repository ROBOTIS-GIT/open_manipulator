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

#include "open_manipulator_libs/om_link_suction.h"

using namespace OM_CHAIN_TOOL;

void ToolDynamixel::init(std::string dxl_device_name, std::string dxl_baud_rate)
{
  gripper_id_.push_back(15);

  gripper_controller_ = new DynamixelWorkbench;
  gripper_controller_->begin(dxl_device_name.c_str(), std::stoi(dxl_baud_rate));

  uint16_t get_model_number;
  for (uint8_t index = 0; index < TOOL_NUM; index++)
  {
    if (gripper_controller_->ping(gripper_id_.at(index), &get_model_number) != true)
    {
      return;
    }
  }
  return;
}

void ToolDynamixel::setPositionControlMode()
{
  for (uint8_t num = 0; num < TOOL_NUM; num++)
    gripper_controller_->currentMode(gripper_id_.at(0), 200);

  gripper_controller_->itemWrite(gripper_id_.at(0),"Profile_Velocity", 100);
  gripper_controller_->itemWrite(gripper_id_.at(0),"Profile_Acceleration", 10);
}

void ToolDynamixel::setTorqueEnable()
{
  for (uint8_t num = 0; num < TOOL_NUM; num++)
    gripper_controller_->itemWrite(gripper_id_.at(num), "Torque_Enable", true);
}
void ToolDynamixel::setTorqueDisable()
{
  for (uint8_t num = 0; num < TOOL_NUM; num++)
    gripper_controller_->itemWrite(gripper_id_.at(num), "Torque_Enable", false);
}

bool ToolDynamixel::goalAllJointAngle(std::vector<double> radian_vector)
{
  int32_t goal_position[TOOL_NUM] = {0, };
  for (int index = 0; index < TOOL_NUM; index++)
  {
    goal_position[index] = gripper_controller_->convertRadian2Value(gripper_id_.at(index), radian_vector.at(index));
  }

  gripper_controller_->syncWrite("Goal_Position", goal_position);

  return true;
}

bool ToolDynamixel::goalMultipleJointAngle(std::vector<uint8_t> id, std::vector<double> radian_vector)
{
  for (int index = 0; index < id.size(); index++)
  {
    int32_t goal_position = gripper_controller_->convertRadian2Value(id.at(index), radian_vector.at(index));
    gripper_controller_->itemWrite(id.at(index),"Goal_Position", goal_position);
  }
  return true;
}

bool ToolDynamixel::goalJointAngle(uint8_t actuator_id, double radian)
{
  int32_t goal_position = gripper_controller_->convertRadian2Value(actuator_id, radian);
  gripper_controller_->itemWrite(actuator_id,"Goal_Position", goal_position);
  return true;
}
std::vector<double> ToolDynamixel::receiveAllJointAngle()
{
  std::vector<double> value;

  int32_t* get_joint_present_position = NULL;
  get_joint_present_position = gripper_controller_->syncRead("Present_Position");

  for (int index = 0; index < TOOL_NUM; index++)
  {
    value.push_back(gripper_controller_->convertValue2Radian(gripper_id_.at(index), get_joint_present_position[index]));
  }

  return value;
}


void ToolDynamixel::initActuator(const void *arg)
{
  std::string *get_arg_ = (std::string *)arg;
  init(get_arg_[0], get_arg_[1]);
}
void ToolDynamixel::setActuatorControlMode()
{
  setPositionControlMode();
}

void ToolDynamixel::Enable()
{
  setTorqueEnable();
}
void ToolDynamixel::Disable()
{
  setTorqueDisable();
}

bool ToolDynamixel::sendAllActuatorAngle(std::vector<double> radian_vector)
{
  return goalAllJointAngle(radian_vector);
}
bool ToolDynamixel::sendMultipleActuatorAngle(std::vector<uint8_t> id, std::vector<double> radian_vector)
{
  return goalMultipleJointAngle(id, radian_vector);
}
bool ToolDynamixel::sendActuatorAngle(uint8_t actuator_id, double radian)
{
  return goalJointAngle(actuator_id, radian);
}
bool ToolDynamixel::sendActuatorSignal(uint8_t actuator_id, bool onoff)
{}
std::vector<double> ToolDynamixel::receiveAllActuatorAngle()
{
  return receiveAllJointAngle();
}
