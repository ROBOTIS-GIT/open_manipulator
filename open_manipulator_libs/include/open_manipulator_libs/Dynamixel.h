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

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
  #include <DynamixelWorkbench.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
  #include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#endif

namespace DYNAMIXEL
{

#define SYNC_WRITE_HANDLER 0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

//#define CONTROL_LOOP_TIME 10;    //ms

// Protocol 2.0
#define ADDR_PRESENT_CURRENT_2 126
#define ADDR_PRESENT_VELOCITY_2 128
#define ADDR_PRESENT_POSITION_2 132
#define ADDR_VELOCITY_TRAJECTORY_2 136
#define ADDR_POSITION_TRAJECTORY_2 140
#define ADDR_PROFILE_ACCELERATION_2 108
#define ADDR_PROFILE_VELOCITY_2 112
#define ADDR_GOAL_POSITION_2 116


#define LENGTH_PRESENT_CURRENT_2 2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4
#define LENGTH_VELOCITY_TRAJECTORY_2 4
#define LENGTH_POSITION_TRAJECTORY_2 4
#define LENGTH_PROFILE_ACCELERATION_2 4
#define LENGTH_PROFILE_VELOCITY_2 4
#define LENGTH_GOAL_POSITION_2 4


// Protocol 1.0
#define ADDR_PRESENT_CURRENT_1 = 40;
#define ADDR_PRESENT_VELOCITY_1 = 38;
#define ADDR_PRESENT_POSITION_1 = 36;

#define LENGTH_PRESENT_CURRENT_1 = 2;
#define LENGTH_PRESENT_VELOCITY_1 = 2;
#define LENGTH_PRESENT_POSITION_1 = 2;

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
  JointDynamixel(){}
  virtual ~JointDynamixel(){}

  virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
  virtual std::vector<uint8_t> getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ROBOTIS_MANIPULATOR::Actuator> value_vector);
  virtual std::vector<ROBOTIS_MANIPULATOR::Actuator> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);

////////////////////////////////////////////////////////////////

  bool initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
  bool setSDKHandler(uint8_t actuator_id);
  bool writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value);
  bool writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector);
  std::vector<ROBOTIS_MANIPULATOR::Actuator> receiveAllDynamixelValue(std::vector<uint8_t> actuator_id);
};

class JointDynamixelProfileControl : public ROBOTIS_MANIPULATOR::JointActuator
{
 private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;
  float control_loop_time_;       //ms
  std::map<uint8_t, ROBOTIS_MANIPULATOR::Actuator> previous_goal_value_;

 public:
  JointDynamixelProfileControl(float control_loop_time = 0.010);
  virtual ~JointDynamixelProfileControl(){}

  virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
  virtual std::vector<uint8_t> getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ROBOTIS_MANIPULATOR::Actuator> value_vector);
  virtual std::vector<ROBOTIS_MANIPULATOR::Actuator> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);

////////////////////////////////////////////////////////////////

  bool initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
  bool setSDKHandler(uint8_t actuator_id);
  bool writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value);
  bool writeGoalProfilingControlValue(std::vector<uint8_t> actuator_id, std::vector<ROBOTIS_MANIPULATOR::Actuator> value_vector);
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

  virtual bool sendToolActuatorValue(ROBOTIS_MANIPULATOR::Actuator value);
  virtual ROBOTIS_MANIPULATOR::Actuator receiveToolActuatorValue();

////////////////////////////////////////////////////////////////

  bool initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(STRING dynamixel_mode = "position_mode");
  bool writeProfileValue(STRING profile_mode, uint32_t value);
  bool setSDKHandler();
  bool writeGoalPosition(double radian);
  double receiveDynamixelValue();
};

} // namespace DYNAMIXEL
#endif // DYNAMIXEL_H_




