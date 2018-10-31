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

#ifndef OM_CHAIN_TOOL_H_
#define OM_CHAIN_TOOL_H_

#include "robotis_manipulator/robotis_manipulator.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define TOOL_NUM 1
#define DEVICE_NAME "/dev/ttyACM0"


namespace OM_CHAIN_TOOL
{

class ToolDynamixel : public ROBOTIS_MANIPULATOR::Actuator
{
private:
  DynamixelWorkbench *gripper_controller_;
  std::vector<uint8_t> gripper_id_;

public:
  ToolDynamixel() {}
  virtual ~ToolDynamixel() {}

  void init(std::string dxl_device_name, std::string dxl_baud_rate);
  void setPositionControlMode();
  void setTorqueEnable();
  void setTorqueDisable();
  bool goalAllJointAngle(std::vector<double> radian_vector);
  bool goalMultipleJointAngle(std::vector<uint8_t> id, std::vector<double> radian_vector);
  bool goalJointAngle(uint8_t actuator_id, double radian);
  std::vector<double> receiveAllJointAngle();

  void initActuator(const void *arg);
  void setActuatorControlMode();

  void Enable();
  void Disable();

  bool sendAllActuatorAngle(std::vector<double> radian_vector);
  bool sendMultipleActuatorAngle(std::vector<uint8_t> id, std::vector<double> radian_vector);
  bool sendActuatorAngle(uint8_t actuator_id, double radian);
  bool sendActuatorSignal(uint8_t actuator_id, bool onoff);
  std::vector<double> receiveAllActuatorAngle();
};


} // namespace OM_CHAIN_TOOL
#endif // OM_CHAIN_TOOL_H_




