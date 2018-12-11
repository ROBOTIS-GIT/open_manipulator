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

#include "../include/open_manipulator_libs/OpenManipulator.h"

OPEN_MANIPULATOR::OPEN_MANIPULATOR()
{}
OPEN_MANIPULATOR::~OPEN_MANIPULATOR()
{}

void OPEN_MANIPULATOR::initManipulator(bool using_platform, STRING usb_port, STRING baud_rate)
{
  platform_ = using_platform;
  ////////// manipulator parameter initialization

  addWorld("world",   // world name
           "joint1"); // child name

  addJoint("joint1", // my name
           "world",  // parent name
           "joint2", // child name
           RM_MATH::makeVector3(0.012, 0.0, 0.017), // relative position
           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS, // axis of rotation
           11,     // actuator id
           M_PI,   // max joint limit (3.14 rad)
           -M_PI); // min joint limit (-3.14 rad)


  addJoint("joint2", // my name
           "joint1", // parent name
           "joint3", // child name
           RM_MATH::makeVector3(0.0, 0.0, 0.058), // relative position
           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS, // axis of rotation
           12,     // actuator id
           M_PI_2,   // max joint limit (1.67 rad)
           -2.05); // min joint limit (-2.05 rad)

  addJoint("joint3", // my name
           "joint2", // parent name
           "joint4", // child name
           RM_MATH::makeVector3(0.024, 0.0, 0.128), // relative position
           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS, // axis of rotation
           13,     // actuator id
           1.53,      // max joint limit (1.53 rad)
           -M_PI_2); // min joint limit (-1.67 rad)

  addJoint("joint4", // my name
           "joint3", // parent name
           "gripper",   // child name
           RM_MATH::makeVector3(0.124, 0.0, 0.0), // relative position
           RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS, // axis of rotation
           14,     // actuator id
           2.0,    // max joint limit (2.0 rad)
           -1.8);  // min joint limit (-1.8 rad)

  addTool("gripper",   // my name
          "joint4", // parent name
          RM_MATH::makeVector3(0.130, 0.0, 0.0), // relative position
          RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
          15,     // actuator id
          0.010,  // max gripper limit (0.01 m)
          -0.010, // min gripper limit (-0.01 m)
          -0.015); // Change unit from `meter` to `radian`

  ////////// kinematics init.
  kinematics_ = new KINEMATICS::Chain();
  addKinematics(kinematics_);
  STRING inverse_option[2] = {"inverse_solver", "chain_custum_inverse_kinematics"};
//  STRING inverse_option[2] = {"inverse_solver", "sr_inverse"};
//  STRING inverse_option[2] = {"inverse_solver", "position_only_inverse"};
//  STRING inverse_option[2] = {"inverse_solver", "normal_inverse"};
  void *inverse_option_arg = &inverse_option;
  kinematicsSetOption(inverse_option_arg);

  if(platform_)
  {
    ////////// joint actuator init.
    actuator_ = new DYNAMIXEL::JointDynamixel();
    // communication setting argument
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg;

    // set joint actuator id
    jointDxlId.push_back(11);
    jointDxlId.push_back(12);
    jointDxlId.push_back(13);
    jointDxlId.push_back(14);

    addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

    // set joint actuator parameter
    STRING joint_dxl_opt_arg[2] = {"Return_Delay_Time", "0"};
    void *p_joint_dxl_opt_arg = &joint_dxl_opt_arg;
    jointActuatorSetMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_opt_arg);

    // set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    jointActuatorSetMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

    ////////// tool actuator init.
    tool_ = new DYNAMIXEL::GripperDynamixel();

    uint8_t gripperDxlId = 15;
    addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

    // set gripper actuator parameter
    STRING gripper_dxl_opt_arg[2] = {"Return_Delay_Time", "0"};
    void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
    toolActuatorSetMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    // set gripper actuator control mode
    STRING gripper_dxl_mode_arg = "current_based_position_mode";
    void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
    toolActuatorSetMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

    gripper_dxl_opt_arg[0] = "Profile_Acceleration";
    gripper_dxl_opt_arg[1] = "20";
    toolActuatorSetMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    gripper_dxl_opt_arg[0] = "Profile_Velocity";
    gripper_dxl_opt_arg[1] = "200";
    toolActuatorSetMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    // all actuator enable
    allActuatorEnable();
    receiveAllJointActuatorValue();
    receiveAllToolActuatorValue();
  }
  ////////// drawing path
  addDrawingTrajectory(DRAWING_LINE, &line_);
  addDrawingTrajectory(DRAWING_CIRCLE, &circle_);
  addDrawingTrajectory(DRAWING_RHOMBUS, &rhombus_);
  addDrawingTrajectory(DRAWING_HEART, &heart_);

  ////////// manipulator trajectory & control time initialization
  setTrajectoryControlTime(CONTROL_TIME);
}

void OPEN_MANIPULATOR::openManipulatorProcess(double present_time)
{
  std::vector<WayPoint> goal_value  = getJointGoalValueFromTrajectory(present_time);
  std::vector<double> tool_value    = getToolGoalValue();

  if(platform_)
  {
    receiveAllJointActuatorValue();
    receiveAllToolActuatorValue();
    if(goal_value.size() != 0) sendAllJointActuatorValue(goal_value);
    if(tool_value.size() != 0) sendAllToolActuatorValue(tool_value);
  }
  else // visualization
  {
    if(goal_value.size() != 0) setAllActiveJointWayPoint(goal_value);
    if(tool_value.size() != 0) setAllToolValue(tool_value);
  }
  forwardKinematics();
}

bool OPEN_MANIPULATOR::getPlatformFlag()
{
  return platform_;
}

