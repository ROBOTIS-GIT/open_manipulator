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

#ifndef OM_CHAIN_CONFIG_HPP_
#define OM_CHAIN_CONFIG_HPP_

#include "om_chain_actuator.h"
#include "om_chain_drawing.h"
#include "om_chain_kinematics.h"
#include "om_chain_tool.h"

#define DXL_BAUD_RATE 1000000

#define WORLD 0
#define COMP1 1
#define COMP2 2
#define COMP3 3
#define COMP4 4
#define TOOL 5

#define X_AXIS RM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS RM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS RM_MATH::makeVector3(0.0, 0.0, 1.0)

#define NUM_OF_JOINT 4

#define DRAWING_LINE 0
#define DRAWING_CIRCLE 1
#define DRAWING_RHOMBUS 2
#define DRAWING_HEART 3

#define JOINT_DYNAMIXEL 0
#define TOOL_DYNAMIXEL 1

using namespace ROBOTIS_MANIPULATOR;

class OM_CHAIN : public RobotisManipulator
{
private:
  std::vector<double> present_joint_angle;
  std::vector<double> present_gripper_angle;

  ROBOTIS_MANIPULATOR::Kinematics *kinematics_;
  ROBOTIS_MANIPULATOR::Actuator *actuator_;
  ROBOTIS_MANIPULATOR::Actuator *tool_;

  OM_CHAIN_DRAWING::Line line_;
  OM_CHAIN_DRAWING::Circle circle_;
  OM_CHAIN_DRAWING::Rhombus rhombus_;
  OM_CHAIN_DRAWING::Heart heart_;

  bool platform_;
  bool processing_;

 public:
  OM_CHAIN();
  virtual ~OM_CHAIN();

  void initManipulator();
  void chainProcess(double present_time);

};
#endif // OM_CHAIN_CONFIG_HPP_




