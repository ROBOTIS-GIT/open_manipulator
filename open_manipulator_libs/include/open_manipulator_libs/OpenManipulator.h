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

#ifndef OPEN_MANIPULTOR_H_
#define OPEN_MANIPULTOR_H_

#include "Dynamixel.h"
#include "CustomTrajectory.h"
#include "Kinematics.h"

#define NUM_OF_JOINT 4
#define DXL_SIZE 5

#define DRAWING_LINE "custom_trajectory_line"
#define DRAWING_CIRCLE "custom_trajectory_circle"
#define DRAWING_RHOMBUS "custom_trajectory_rhombus"
#define DRAWING_HEART "custom_trajectory_heart"

#define JOINT_DYNAMIXEL "joint_dxl"
#define TOOL_DYNAMIXEL "tool_dxl"

#define CONTROL_TIME 0.010 //s

#define X_AXIS RM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS RM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS RM_MATH::makeVector3(0.0, 0.0, 1.0)


class OPEN_MANIPULATOR : public ROBOTIS_MANIPULATOR::RobotisManipulator
{
private:
  ROBOTIS_MANIPULATOR::Kinematics *kinematics_;
  ROBOTIS_MANIPULATOR::JointActuator *actuator_;
  ROBOTIS_MANIPULATOR::ToolActuator *tool_;

  CUSTOM_TRAJECTORY::Line line_;
  CUSTOM_TRAJECTORY::Circle circle_;
  CUSTOM_TRAJECTORY::Rhombus rhombus_;
  CUSTOM_TRAJECTORY::Heart heart_;

  std::vector<uint8_t> jointDxlId;
 public:
  OPEN_MANIPULATOR();
  virtual ~OPEN_MANIPULATOR();

  void initManipulator(bool using_platform, STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000");
  void communicationProcessToActuator(JointWayPoint goal_joint_value, JointWayPoint goal_tool_value);
  void calculationProcess(double tick_time, JointWayPoint* goal_joint_value, JointWayPoint *goal_tool_value);
};

#endif // OPEN_MANIPULTOR_H_
