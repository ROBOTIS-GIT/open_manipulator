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

#ifndef OM_DYNAMIXEL_H_
#define OM_DYNAMIXEL_H_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <robotis_manipulator/robotis_manipulator.h>
#include <robotis_manipulator/robotis_manipulator_common.h>

// #include <iostream>
#include <cstdio>

namespace OM_DYNAMIXEL
{

#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

#define ADDR_PRESENT_CURRENT_2 126
#define ADDR_PRESENT_VELOCITY_2 128
#define ADDR_PRESENT_POSITION_2 132

#define LENGTH_PRESENT_CURRENT_2 2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4

// #define ADDR_PRESENT_CURRENT_1 = 40;
// #define ADDR_PRESENT_VELOCITY_1 = 38;
// #define ADDR_PRESENT_POSITION_1 = 36;

// #define LENGTH_PRESENT_CURRENT_1 = 2;
// #define LENGTH_PRESENT_VELOCITY_1 = 2;
// #define LENGTH_PRESENT_POSITION_1 = 2;

typedef struct
{
  std::vector<uint8_t> id;
  uint8_t num;
} Joint;

class JointDynamixel : public ROBOTIS_MANIPULATOR::JointActuator
{
 private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;

 public:
  JointDynamixel() {}
  virtual ~JointDynamixel() {}

  virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
  virtual std::vector<uint8_t> getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ROBOTIS_MANIPULATOR::Actuator> value_vector);
  virtual std::vector<ROBOTIS_MANIPULATOR::Actuator> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);

////////////////////////////////////////////////////////////////

  bool initialize(std::vector<uint8_t> actuator_id, std::string dxl_device_name, std::string dxl_baud_rate);
  bool setOperatingMode(std::vector<uint8_t> actuator_id, std::string dynamixel_mode = "position_mode");
  bool setSDKHandler(uint8_t actuator_id);
  bool writeProfileValue(std::vector<uint8_t> actuator_id, std::string profile_mode, uint32_t value);
  bool writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector);
  std::vector<ROBOTIS_MANIPULATOR::Actuator> receiveAllDynamixelValue(std::vector<uint8_t> actuator_id);
};

class GripperDynamixel : public ROBOTIS_MANIPULATOR::ToolActuator
{
 private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;

 public:
  GripperDynamixel() {}
  virtual ~GripperDynamixel() {}

  virtual void init(uint8_t actuator_id, const void *arg);
  virtual void setMode(const void *arg);
  virtual uint8_t getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendToolActuatorValue(double value);
  virtual double receiveToolActuatorValue();

////////////////////////////////////////////////////////////////

  bool initialize(uint8_t actuator_id, std::string dxl_device_name, std::string dxl_baud_rate);
  bool setOperatingMode(std::string dynamixel_mode = "position_mode");
  bool writeProfileValue(std::string profile_mode, uint32_t value);
  bool setSDKHandler();
  bool writeGoalPosition(double radian);
  double receiveDynamixelValue();
};

} // namespace RM_DYNAMIXEL
#endif // OM_DYNAMIXEL_H_




