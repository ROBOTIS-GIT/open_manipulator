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

#ifndef OM_DYNAMIXEL_H_
#define OM_DYNAMIXEL_H_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <robotis_manipulator/robotis_manipulator.h>
#include <robotis_manipulator/robotis_manipulator_common.h>


#include <iostream>

namespace OM_DYNAMIXEL
{

#define SYNC_WRITE_GOAL_POSITION 0
#define SYNC_READ_PRESENT_POSITION 0
#define SYNC_READ_PRESENT_VELOCITY 1

class JointDynamixel : public ROBOTIS_MANIPULATOR::JointActuator
{
private:
  int8_t dynamixel_num_;
  DynamixelWorkbench *dynamixel_controller_;
  std::vector<uint8_t> dynamixel_id_;

  uint32_t goal_position_[20] = {0, }; //need update workbench
  uint32_t goal_velocity_[20] = {0, };

public:
  JointDynamixel() {}
  virtual ~JointDynamixel() {}

  virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
  std::vector<uint8_t> getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendJointActuatorValue(uint8_t actuator_id, ROBOTIS_MANIPULATOR::Actuator value);
  virtual bool sendMultipleJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ROBOTIS_MANIPULATOR::Actuator> value_vector);
  virtual ROBOTIS_MANIPULATOR::Actuator receiveJointActuatorValue(uint8_t actuator_id);
  virtual std::vector<ROBOTIS_MANIPULATOR::Actuator> receiveMultipleJointActuatorValue(std::vector<uint8_t> actuator_id);

////////////////////////////////////////////////////////////////

  void iniialize(std::vector<uint8_t> actuator_id, std::string dxl_device_name, std::string dxl_baud_rate);
  void setOperatingMode(std::vector<uint8_t> actuator_id, std::string dynamixel_mode = "position_mode");
  void writeProfileValue(std::vector<uint8_t> actuator_id, std::string profile_mode, uint8_t value);
  void writeTorqueEnable(std::vector<uint8_t> actuator_id, uint8_t value);
  void writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector);
  void writeGoalVelocity(std::vector<uint8_t> actuator_id, std::vector<double> velocity_vector);
  std::vector<double> receiveAllDynamixelAngle();
  std::vector<double> receiveAllDynamixelVelocity();
};

class GripperDynamixel : public ROBOTIS_MANIPULATOR::ToolActuator
{
private:
  DynamixelWorkbench *dynamixel_controller_;
  uint8_t dynamixel_id_;

public:
  GripperDynamixel() {}
  virtual ~GripperDynamixel() {}

  virtual void init(uint8_t actuator_id, const void *arg);
  virtual void setMode(const void *arg);
  virtual uint8_t getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendToolActuatorValue(uint8_t actuator_id, double value);
  virtual double receiveToolActuatorValue(uint8_t actuator_id);

////////////////////////////////////////////////////////////////

  void iniialize(uint8_t actuator_id, std::string dxl_device_name, std::string dxl_baud_rate);
  void setOperatingMode(uint8_t actuator_id, std::string dynamixel_mode = "position_mode");
  void writeProfileValue(uint8_t actuator_id, std::string profile_mode, uint8_t value);
  void writeTorqueEnable(uint8_t actuator_id, uint8_t value);
  void writeGoalPosition(uint8_t actuator_id, double radian);
  void writeGoalVelocity(uint8_t actuator_id, double velocity);
  double receiveDynamixelAngle();
  double receiveDynamixelVelocity();
};

} // namespace RM_DYNAMIXEL
#endif // OM_DYNAMIXEL_H_




