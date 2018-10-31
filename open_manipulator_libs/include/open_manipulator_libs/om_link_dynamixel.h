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

#ifndef OM_LINK_DYNAMIXEL_H_
#define OM_LINK_DYNAMIXEL_H_

#include <robotis_manipulator/robotis_manipulator.h>
#include <robotis_manipulator/robotis_manipulator_common.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <iostream>

namespace RM_DYNAMIXEL
{

class Dynamixel : public ROBOTIS_MANIPULATOR::JointActuator
{
private:
  int8_t dynamixel_num_;
  DynamixelWorkbench *dynamixel_controller_;
  std::vector<uint8_t> dynamixel_id_;

public:
  Dynamixel() {}
  virtual ~Dynamixel() {}

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
  void writeProfileValue(std::vector<uint8_t> actuator_id, std::string profile_mode, int8_t value);
  void writeTorqueEnable(std::vector<uint8_t> actuator_id, int8_t value);




////////////////////////////////////////////////////////////////////

  void setPositionControlMode();
  void setTorqueEnable();
  void setTorqueDisable();
  bool goalAllJointAngle(std::vector<double> radian_vector);
  bool goalMultipleJointAngle(std::vector<uint8_t> id, std::vector<double> radian_vector);
  bool goalJointAngle(uint8_t actuator_id, double radian);
  std::vector<double> receiveAllJointAngle();
  std::vector<double> receiveAllJointVelocity();

  void initActuator(const void *arg);
  void setActuatorControlMode();

  void Enable();
  void Disable();

  bool sendAllActuatorAngle(std::vector<double> radian_vector);
  bool sendMultipleActuatorAngle(std::vector<uint8_t> id, std::vector<double> radian_vector);
  bool sendActuatorAngle(uint8_t actuator_id, double radian);
  std::vector<double> receiveAllActuatorAngle();
};


} // namespace RM_DYNAMIXEL
#endif // OM_CHAIN_ACTUATOR_H_




