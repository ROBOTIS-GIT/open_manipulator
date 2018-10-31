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


#include "open_manipulator_libs/om_chain.h"

OM_CHAIN::OM_CHAIN()
{
  present_joint_angle.resize(4);
  present_gripper_angle.resize(1);
}
OM_CHAIN::~OM_CHAIN()
{
}

void OM_CHAIN::initManipulator()
{
  addWorld(WORLD,
           COMP1);

  addComponent(COMP1,
               WORLD,
               COMP2,
               RM_MATH::makeVector3(-0.278, 0.0, 0.017),
               RM_MATH::convertRPYToQuaternion(0.0, 0.0, 0.0),
               Z_AXIS,
               11);

  addComponent(COMP2,
               COMP1,
               COMP3,
               RM_MATH::makeVector3(0.0, 0.0, 0.058),
               RM_MATH::convertRPYToQuaternion(0.0, 0.0, 0.0),
               Y_AXIS,
               12);

  addComponent(COMP3,
               COMP2,
               COMP4,
               RM_MATH::makeVector3(0.024, 0.0, 0.128),
               RM_MATH::convertRPYToQuaternion(0.0, 0.0, 0.0),
               Y_AXIS,
               13);

  addComponent(COMP4,
               COMP3,
               TOOL,
               RM_MATH::makeVector3(0.124, 0.0, 0.0),
               RM_MATH::convertRPYToQuaternion(0.0, 0.0, 0.0),
               Y_AXIS,
               14);

  addTool(TOOL,
          COMP4,
          RM_MATH::makeVector3(0.130, 0.0, 0.0),
          RM_MATH::convertRPYToQuaternion(0.0, 0.0, 0.0),
          15,
          1.0f); // Change unit from `meter` to `radian`

  // kinematics init.
  kinematics_ = new OM_CHAIN_KINEMATICS::Chain();
  addKinematics(kinematics_);

  // joint actuator init.
  actuator_ = new OM_DYNAMIXEL::JointDynamixel();
  addJointActuator(JOINT_DYNAMIXEL, actuator_);

  std::string dxl_comm_arg[2] = {"/dev/ttyUSB0", "1000000"};
  void *p_dxl_comm_arg = &dxl_comm_arg;

  std::vector<uint8_t> jointDxlId;
  jointDxlId.push_back(11);
  jointDxlId.push_back(12);
  jointDxlId.push_back(13);
  jointDxlId.push_back(14);

  JointActuatorInit(JOINT_DYNAMIXEL, jointDxlId, p_dxl_comm_arg);

  std::string joint_dxl_mode_arg = "position_mode";
  void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
  JointActuatorSetMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

  // tool actuator init.
  tool_ = new OM_DYNAMIXEL::GripperDynamixel();
  addToolActuator(TOOL_DYNAMIXEL, tool_);

  uint8_t gripperDxlId = 15;
  toolActuatorInit(TOOL_DYNAMIXEL, gripperDxlId, p_dxl_comm_arg);

  std::string gripper_dxl_mode_arg = "curemt_mode";
  void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
  toolActuatorSetMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

  allActuatorEnable();
  // drawing path
  addDrawingTrajectory(DRAWING_LINE, &line_);
  addDrawingTrajectory(DRAWING_CIRCLE, &circle_);
  addDrawingTrajectory(DRAWING_RHOMBUS, &rhombus_);
  addDrawingTrajectory(DRAWING_HEART, &heart_);

  setAllJointActuatorValue(receiveAllJointActuatorValue(jointDxlId));

  //initTrajectory(getAllActiveJointAngle());
  setControlTime(ACTUATOR_CONTROL_TIME);
}

void OM_CHAIN::chainProcess(double present_time)
{
  trajectoryControllerLoop(present_time);
}
