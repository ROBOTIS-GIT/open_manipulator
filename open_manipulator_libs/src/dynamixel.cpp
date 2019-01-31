/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "../include/open_manipulator_libs/dynamixel.h"

using namespace dynamixel;
using namespace robotis_manipulator;

/*****************************************************************************
** Joint Dynamixel Control Functions
*****************************************************************************/
void JointDynamixel::init(std::vector<uint8_t> actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  bool result = JointDynamixel::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

  if (result == false)
    return;
}

void JointDynamixel::setMode(std::vector<uint8_t> actuator_id, const void *arg)
{
  bool result = false;
  // const char* log = NULL;

  STRING *get_arg_ = (STRING *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = JointDynamixel::setOperatingMode(actuator_id, get_arg_[0]);
    if (result == false)
      return;

    result = JointDynamixel::setSDKHandler(actuator_id.at(0));
    if (result == false)
      return;
  }
  else
  {
    result = JointDynamixel::writeProfileValue(actuator_id, get_arg_[0], std::atoi(get_arg_[1].c_str()));
    if (result == false)
      return;
  }
  return;
}

std::vector<uint8_t> JointDynamixel::getId()
{
  return dynamixel_.id;
}

void JointDynamixel::enable()
{
  const char* log = NULL;
  bool result = false;
  
  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOn(dynamixel_.id.at(index), &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  enabled_state_ = true;
}

void JointDynamixel::disable()
{
  const char* log = NULL;
  bool result = false;
  
  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOff(dynamixel_.id.at(index), &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  enabled_state_ = false;
}

bool JointDynamixel::sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector)
{
  bool result = false;

  std::vector<double> radian_vector;
  for(uint32_t index = 0; index < value_vector.size(); index++)
  {
    radian_vector.push_back(value_vector.at(index).position);
  }
  result = JointDynamixel::writeGoalPosition(actuator_id, radian_vector);
  if (result == false)
    return false;

  return true;
}

std::vector<robotis_manipulator::ActuatorValue> JointDynamixel::receiveJointActuatorValue(std::vector<uint8_t> actuator_id)
{
  return JointDynamixel::receiveAllDynamixelValue(actuator_id);
}


/*****************************************************************************
** Functions called in Joint Dynamixel Control Functions
*****************************************************************************/
bool JointDynamixel::initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
  bool result = false;
  const char* log = NULL;

  STRING return_delay_time_st = "Return_Delay_Time";
  const char * return_delay_time_char = return_delay_time_st.c_str();

  dynamixel_.id = actuator_id;
  dynamixel_.num = actuator_id.size();

  dynamixel_workbench_ = new DynamixelWorkbench;

  result = dynamixel_workbench_->init(dxl_device_name.c_str(), std::atoi(dxl_baud_rate.c_str()), &log);
  if (result == false)
  {
    log::error(log);
  }    

  uint16_t get_model_number;
  for (uint8_t index = 0; index < dynamixel_.num; index++)
  {
    uint8_t id = dynamixel_.id.at(index);
    result = dynamixel_workbench_->ping(id, &get_model_number, &log);

    if (result == false)
    {
      log::error(log);
      log::error("Please check your Dynamixel ID");
    }
    else
    {
      char str[100];
      sprintf(str, "Joint Dynamixel ID : %d, Model Name : %s", id, dynamixel_workbench_->getModelName(id));
      log::println(str);

      result = dynamixel_workbench_->setVelocityBasedProfile(id, &log);
      if(result == false)
      {
        log::error(log);
        log::error("Please check your Dynamixel firmware version (v38~)");
      }

      result = dynamixel_workbench_->writeRegister(id, return_delay_time_char, 0, &log);
      if (result == false)
      {
        log::error(log);
        log::error("Please check your Dynamixel firmware version");
      }
    }
  }
  return true;
}

bool JointDynamixel::setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode)
{
  const char* log = NULL;
  bool result = false;

  const uint32_t velocity = 0;
  const uint32_t acceleration = 0;
  const uint32_t current = 0;

  if (dynamixel_mode == "position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, acceleration, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->currentBasedPositionMode(actuator_id.at(num), current, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }
  else
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, acceleration, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }

  return true;
}

bool JointDynamixel::setSDKHandler(uint8_t actuator_id)
{
  bool result = false;
  const char* log = NULL;

  result = dynamixel_workbench_->addSyncWriteHandler(actuator_id, "Goal_Position", &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->addSyncReadHandler(ADDR_PRESENT_CURRENT_2, 
                                                    (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2), 
                                                    &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

bool JointDynamixel::writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value)
{
  const char* log = NULL;
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();

  for (uint8_t num = 0; num < actuator_id.size(); num++)
  {
    result = dynamixel_workbench_->writeRegister(actuator_id.at(num), char_profile_mode, value, &log);
    if (result == false)
    {
      log::error(log);
    }
  }

  return true;
}

bool JointDynamixel::writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector)
{
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[actuator_id.size()];
  int32_t goal_position[actuator_id.size()];

  for (uint8_t index = 0; index < actuator_id.size(); index++)
  {
    id_array[index] = actuator_id.at(index);
    goal_position[index] = dynamixel_workbench_->convertRadian2Value(actuator_id.at(index), radian_vector.at(index));
  }

  result = dynamixel_workbench_->syncWrite(SYNC_WRITE_HANDLER, id_array, actuator_id.size(), goal_position, 1, &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

std::vector<robotis_manipulator::ActuatorValue> JointDynamixel::receiveAllDynamixelValue(std::vector<uint8_t> actuator_id)
{
  bool result = false;
  const char* log = NULL;

  std::vector<robotis_manipulator::ActuatorValue> all_actuator;

  uint8_t id_array[actuator_id.size()];
  for (uint8_t index = 0; index < actuator_id.size(); index++)
    id_array[index] = actuator_id.at(index);

  int32_t get_current[actuator_id.size()];
  int32_t get_velocity[actuator_id.size()];
  int32_t get_position[actuator_id.size()];

  result = dynamixel_workbench_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                          id_array,
                                          actuator_id.size(),
                                          &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                actuator_id.size(),
                                                ADDR_PRESENT_CURRENT_2,
                                                LENGTH_PRESENT_CURRENT_2,
                                                get_current,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                 id_array,
                                                 actuator_id.size(),
                                                ADDR_PRESENT_VELOCITY_2,
                                                LENGTH_PRESENT_VELOCITY_2,
                                                get_velocity,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                 id_array,
                                                 actuator_id.size(),
                                                ADDR_PRESENT_POSITION_2,
                                                LENGTH_PRESENT_POSITION_2,
                                                get_position,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  for (uint8_t index = 0; index < actuator_id.size(); index++)
  {
    robotis_manipulator::ActuatorValue actuator;
    actuator.effort = dynamixel_workbench_->convertValue2Current(get_current[index]);
    actuator.velocity = dynamixel_workbench_->convertValue2Velocity(actuator_id.at(index), get_velocity[index]);
    actuator.position = dynamixel_workbench_->convertValue2Radian(actuator_id.at(index), get_position[index]);

    all_actuator.push_back(actuator);
  }

  return all_actuator;
}


/*****************************************************************************
** Joint Dynamixel Profile Control Functions
*****************************************************************************/
JointDynamixelProfileControl::JointDynamixelProfileControl(float control_loop_time)
{
  control_loop_time_ = control_loop_time;
}

void JointDynamixelProfileControl::init(std::vector<uint8_t> actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  bool result = JointDynamixelProfileControl::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

  if (result == false)
    return;
}

void JointDynamixelProfileControl::setMode(std::vector<uint8_t> actuator_id, const void *arg)
{
  bool result = false;
  // const char* log = NULL;

  STRING *get_arg_ = (STRING *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = JointDynamixelProfileControl::setOperatingMode(actuator_id, get_arg_[0]);
    if (result == false)
      return;

    result = JointDynamixelProfileControl::setSDKHandler(actuator_id.at(0));
    if (result == false)
      return;
  }
  else
  {
    result = JointDynamixelProfileControl::writeProfileValue(actuator_id, get_arg_[0], std::atoi(get_arg_[1].c_str()));
    if (result == false)
      return;
  }
  return;
}

std::vector<uint8_t> JointDynamixelProfileControl::getId()
{
  return dynamixel_.id;
}

void JointDynamixelProfileControl::enable()
{
  const char* log = NULL;
  bool result = false;

  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOn(dynamixel_.id.at(index), &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  enabled_state_ = true;
}

void JointDynamixelProfileControl::disable()
{
  const char* log = NULL;
  bool result = false;

  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOff(dynamixel_.id.at(index), &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  enabled_state_ = false;
}

bool JointDynamixelProfileControl::sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector)
{
  bool result = false;

  result = JointDynamixelProfileControl::writeGoalProfilingControlValue(actuator_id, value_vector);
  if (result == false)
    return false;

  return true;
}

std::vector<robotis_manipulator::ActuatorValue> JointDynamixelProfileControl::receiveJointActuatorValue(std::vector<uint8_t> actuator_id)
{
  return JointDynamixelProfileControl::receiveAllDynamixelValue(actuator_id);
}


/*****************************************************************************
** Functions called in Joint Dynamixel Profile Control Functions
*****************************************************************************/
bool JointDynamixelProfileControl::initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
  bool result = false;
  const char* log = NULL;

  STRING return_delay_time_st = "Return_Delay_Time";
  const char * return_delay_time_char = return_delay_time_st.c_str();

  dynamixel_.id = actuator_id;
  dynamixel_.num = actuator_id.size();

  dynamixel_workbench_ = new DynamixelWorkbench;

  result = dynamixel_workbench_->init(dxl_device_name.c_str(), std::atoi(dxl_baud_rate.c_str()), &log);
  if (result == false)
  {
    log::error(log);
  }

  uint16_t get_model_number;
  for (uint8_t index = 0; index < dynamixel_.num; index++)
  {
    uint8_t id = dynamixel_.id.at(index);
    result = dynamixel_workbench_->ping(id, &get_model_number, &log);

    if (result == false)
    {
      log::error(log);
      log::error("Please check your Dynamixel ID");
    }
    else
    {
      char str[100];
      sprintf(str, "Joint Dynamixel ID : %d, Model Name : %s", id, dynamixel_workbench_->getModelName(id));
      log::println(str);

      result = dynamixel_workbench_->setTimeBasedProfile(id, &log);
      if(result == false)
      {
        log::error(log);
        log::error("Please check your Dynamixel firmware version (v38~)");
      }

      result = dynamixel_workbench_->writeRegister(id, return_delay_time_char, 0, &log);
      if (result == false)
      {
        log::error(log);
        log::error("Please check your Dynamixel firmware version");
      }
    }
  }
  return true;
}

bool JointDynamixelProfileControl::setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode)
{
  const char* log = NULL;
  bool result = false;

  const uint32_t velocity = uint32_t(control_loop_time_*1000) * 3;
  const uint32_t acceleration = uint32_t(control_loop_time_*1000);
  const uint32_t current = 0;

  if (dynamixel_mode == "position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, acceleration, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->currentBasedPositionMode(actuator_id.at(num), current, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }
  else
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, acceleration, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }

  return true;
}

bool JointDynamixelProfileControl::setSDKHandler(uint8_t actuator_id)
{
  bool result = false;
  const char* log = NULL;

  result = dynamixel_workbench_->addSyncWriteHandler(actuator_id, "Goal_Position", &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->addSyncReadHandler(ADDR_PRESENT_CURRENT_2,
                                                    (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2),
                                                    &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

bool JointDynamixelProfileControl::writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value)
{
  const char* log = NULL;
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();

  for (uint8_t num = 0; num < actuator_id.size(); num++)
  {
    result = dynamixel_workbench_->writeRegister(actuator_id.at(num), char_profile_mode, value, &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  return true;
}

bool JointDynamixelProfileControl::writeGoalProfilingControlValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector)
{
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[actuator_id.size()];
  int32_t goal_value[actuator_id.size()];

  //add tarajectory eq.
  for(uint8_t index = 0; index < actuator_id.size(); index++)
  {
    float result_position;
    float time_control = control_loop_time_;       //ms

    if(previous_goal_value_.find(actuator_id.at(index)) == previous_goal_value_.end())
    {
      previous_goal_value_.insert(std::make_pair(actuator_id.at(index), value_vector.at(index)));
    }

    result_position = value_vector.at(index).position + 3*(value_vector.at(index).velocity * (time_control))/2;

    id_array[index] = actuator_id.at(index);
    goal_value[index] = dynamixel_workbench_->convertRadian2Value(actuator_id.at(index), result_position);

    previous_goal_value_[actuator_id.at(index)] = value_vector.at(index);
  }

  result = dynamixel_workbench_->syncWrite(SYNC_WRITE_HANDLER, id_array, actuator_id.size(), goal_value, 1, &log);
  if (result == false)
  {
    log::error(log);
  }
  return true;
}

std::vector<robotis_manipulator::ActuatorValue> JointDynamixelProfileControl::receiveAllDynamixelValue(std::vector<uint8_t> actuator_id)
{
  bool result = false;
  const char* log = NULL;

  std::vector<robotis_manipulator::ActuatorValue> all_actuator;

  uint8_t id_array[actuator_id.size()];
  for (uint8_t index = 0; index < actuator_id.size(); index++)
    id_array[index] = actuator_id.at(index);

  int32_t get_current[actuator_id.size()];
  int32_t get_velocity[actuator_id.size()];
  int32_t get_position[actuator_id.size()];

  result = dynamixel_workbench_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                          id_array,
                                          actuator_id.size(),
                                          &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                actuator_id.size(),
                                                ADDR_PRESENT_CURRENT_2,
                                                LENGTH_PRESENT_CURRENT_2,
                                                get_current,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                 id_array,
                                                 actuator_id.size(),
                                                ADDR_PRESENT_VELOCITY_2,
                                                LENGTH_PRESENT_VELOCITY_2,
                                                get_velocity,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                 id_array,
                                                 actuator_id.size(),
                                                ADDR_PRESENT_POSITION_2,
                                                LENGTH_PRESENT_POSITION_2,
                                                get_position,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  for (uint8_t index = 0; index < actuator_id.size(); index++)
  {
    robotis_manipulator::ActuatorValue actuator;
    actuator.effort = dynamixel_workbench_->convertValue2Current(get_current[index]);
    actuator.velocity = dynamixel_workbench_->convertValue2Velocity(actuator_id.at(index), get_velocity[index]);
    actuator.position = dynamixel_workbench_->convertValue2Radian(actuator_id.at(index), get_position[index]);

    all_actuator.push_back(actuator);
  }

  return all_actuator;
}


/*****************************************************************************
** Tool Dynamixel Control Functions
*****************************************************************************/
void GripperDynamixel::init(uint8_t actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  bool result = GripperDynamixel::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

  if (result == false)
    return;
}

void GripperDynamixel::setMode(const void *arg)
{
  bool result = false;
// const char* log = NULL;

  STRING *get_arg_ = (STRING *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = GripperDynamixel::setOperatingMode(get_arg_[0]);
    if (result == false)
      return;
  }
  else
  {
    result = GripperDynamixel::writeProfileValue(get_arg_[0], std::atoi(get_arg_[1].c_str()));
    if (result == false)
      return;
  }

  result = GripperDynamixel::setSDKHandler();
  if (result == false)
    return;
}

uint8_t GripperDynamixel::getId()
{
  return dynamixel_.id.at(0);
}

void GripperDynamixel::enable()
{
  const char* log = NULL;
  bool result = false;
  
  result = dynamixel_workbench_->torqueOn(dynamixel_.id.at(0), &log);
  if (result == false)
  {
    log::error(log);
  }
  enabled_state_ = true;
}

void GripperDynamixel::disable()
{
  const char* log = NULL;
  bool result = false;
  
  result = dynamixel_workbench_->torqueOff(dynamixel_.id.at(0), &log);
  if (result == false)
  {
    log::error(log);
  }
  enabled_state_ = false;
}

bool GripperDynamixel::sendToolActuatorValue(robotis_manipulator::ActuatorValue value)
{
  return GripperDynamixel::writeGoalPosition(value.position);
}

robotis_manipulator::ActuatorValue GripperDynamixel::receiveToolActuatorValue()
{
  robotis_manipulator::ActuatorValue result;
  result.position = GripperDynamixel::receiveDynamixelValue();
  result.velocity = 0.0;
  result.acceleration = 0.0;
  result.effort = 0.0;
  return result;
}


/*****************************************************************************
** Functions called in Tool Dynamixel Profile Control Functions
*****************************************************************************/
bool GripperDynamixel::initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
  const char* log = NULL;
  bool result = false;

  STRING return_delay_time_st = "Return_Delay_Time";
  const char * return_delay_time_char = return_delay_time_st.c_str();

  dynamixel_.id.push_back(actuator_id);
  dynamixel_.num = 1;

  dynamixel_workbench_ = new DynamixelWorkbench;

  result = dynamixel_workbench_->init(dxl_device_name.c_str(), std::atoi(dxl_baud_rate.c_str()), &log);
  if (result == false)
  {
    log::error(log);
  }

  uint16_t get_model_number;
  result = dynamixel_workbench_->ping(dynamixel_.id.at(0), &get_model_number, &log);
  if (result == false)
  {
    log::error(log);
    log::error("Please check your Dynamixel ID");
  }
  else
  {
    char str[100];
    sprintf(str, "Gripper Dynamixel ID : %d, Model Name :", dynamixel_.id.at(0));
    strcat(str, dynamixel_workbench_->getModelName(dynamixel_.id.at(0)));
    log::println(str);

    result = dynamixel_workbench_->setVelocityBasedProfile(dynamixel_.id.at(0), &log);
    if(result == false)
    {
      log::error(log);
      log::error("Please check your Dynamixel firmware version (v38~)");
    }

    result = dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), return_delay_time_char, 0, &log);
    if (result == false)
    {
      log::error(log);
      log::error("Please check your Dynamixel firmware version");
    }
  }

  return true;
}

bool GripperDynamixel::setOperatingMode(STRING dynamixel_mode)
{
  const char* log = NULL;
  bool result = false;

  const uint32_t velocity = 0;
  const uint32_t acceleration = 0;
  const uint32_t current = 200;

  if (dynamixel_mode == "position_mode")
  {
    result = dynamixel_workbench_->jointMode(dynamixel_.id.at(0), velocity, acceleration, &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    result = dynamixel_workbench_->currentBasedPositionMode(dynamixel_.id.at(0), current, &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  else
  {
    result = dynamixel_workbench_->jointMode(dynamixel_.id.at(0), velocity, acceleration, &log);
    if (result == false)
    {
      log::error(log);
    }
  }

  return true;
}

bool GripperDynamixel::writeProfileValue(STRING profile_mode, uint32_t value)
{
  const char* log = NULL;
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();

  result = dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), char_profile_mode, value, &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

bool GripperDynamixel::setSDKHandler()
{
  bool result = false;
  const char* log = NULL;

  result = dynamixel_workbench_->addSyncWriteHandler(dynamixel_.id.at(0), "Goal_Position", &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->addSyncReadHandler(dynamixel_.id.at(0),
                                                    "Present_Position", 
                                                    &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

bool GripperDynamixel::writeGoalPosition(double radian)
{
  bool result = false;
  const char* log = NULL;

  int32_t goal_position = 0;

  goal_position = dynamixel_workbench_->convertRadian2Value(dynamixel_.id.at(0), radian);

  result = dynamixel_workbench_->syncWrite(SYNC_WRITE_HANDLER, &goal_position, &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

double GripperDynamixel::receiveDynamixelValue()
{
  bool result = false;
  const char* log = NULL;

  int32_t get_value = 0;
  uint8_t id_array[1] = {dynamixel_.id.at(0)};

  result = dynamixel_workbench_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, 
                                          id_array,
                                          (uint8_t)1,
                                          &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, 
                                            id_array,
                                            (uint8_t)1,
                                            &get_value, 
                                            &log);
  if (result == false)
  {
    log::error(log);
  } 

  return dynamixel_workbench_->convertValue2Radian(dynamixel_.id.at(0), get_value);
}
