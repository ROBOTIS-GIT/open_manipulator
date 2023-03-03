// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na

#include "../include/open_manipulator_x_libs/open_manipulator_x.hpp"

OpenManipulatorX::OpenManipulatorX() {}

OpenManipulatorX::~OpenManipulatorX()
{
  delete kinematics_;
  delete actuator_;
  delete tool_;
  for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
    delete custom_trajectory_[index];
}

void OpenManipulatorX::init_open_manipulator_x(bool sim, STRING usb_port, STRING baud_rate, float control_loop_time, std::vector<uint8_t> dxl_id)
{
  /*****************************************************************************
    ** Initialize Manipulator Parameter
    *****************************************************************************/
    addWorld("world",   // world name
             "joint1"); // child name

    addJoint("joint1",  // my name
             "world",   // parent name
             "joint2",  // child name
             math::vector3(0.012, 0.0, 0.017),                // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Z_AXIS,    // axis of rotation
             dxl_id[0], // actuator id
             M_PI,      // max joint limit (3.14 rad)
             -M_PI,     // min joint limit (-3.14 rad)
             1.0,       // coefficient
             1.0483260e-01,                                                        // mass
             math::inertiaMatrix(1.0781918e-04, 0.0,           0.0,
                                                1.0355255e-04, -1.8062416e-06,
                                                               1.7644210e-05),     // inertial tensor
             math::vector3(0.0, 5.6914372e-04, 0.018 + 2.6565513e-02)              // COM
             );

    addJoint("joint2",  // my name
             "joint1",  // parent name
             "joint3",  // child name
             math::vector3(0.0, 0.0, 0.0595),                // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
             dxl_id[1], // actuator id
             M_PI_2,    // max joint limit (1.67 rad)
             -2.05,     // min joint limit (-2.05 rad)
             1.0,       // coefficient
             1.4234630e-01,                                                        // mass
             math::inertiaMatrix(1.8365231e-03, -8.2177190e-07, -1.6490470e-04,
                                                1.8562153e-03, -7.6370887e-06,
                                                                5.4940213e-05),    // inertial tensor
             math::vector3(9.1617228e-03, 4.1915210e-04, 1.0599936e-01)            // COM
             );

    addJoint("joint3",  // my name
             "joint2",  // parent name
             "joint4",  // child name
             math::vector3(0.024, 0.0, 0.128),               // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
             dxl_id[2], // actuator id
             1.53,      // max joint limit (1.53 rad)
             -M_PI_2,   // min joint limit (-1.67 rad)
             1.0,       // coefficient
             1.3467049e-01,                                                        // mass
             math::inertiaMatrix(2.4835638e-05, -6.7882502e-06, -2.7331036e-09,
                                                1.3502276e-03, 0.0,
                                                               1.3589608e-03),     // inertial tensor
             math::vector3(9.3290225e-02, 4.4304274e-04, 3.6312773e-07)            // COM
             );

    addJoint("joint4",  // my name
             "joint3",  // parent name
             "gripper", // child name
             math::vector3(0.124, 0.0, 0.0),                 // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
             dxl_id[3], // actuator id
             2.0,       // max joint limit (2.0 rad)
             -1.8,      // min joint limit (-1.8 rad)
             1.0,       // coefficient
             1.6512361e-01,                                                        // mass
             math::inertiaMatrix(9.9192159e-05, 7.1970402e-09, -6.4649200e-05,
                                                4.7615217e-04, -2.2085254e-07,
                                                                4.7876798e-04),    // inertial tensor
             math::vector3(4.4206755e-02, 3.6839985e-07, 8.9142216e-03)            // COM
             );

    addTool("gripper",  // my name
            "joint4",   // parent name
            math::vector3(0.126, 0.0, 0.0),                 // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            dxl_id[4],  // actuator id
            0.010,      // max gripper limit (0.01 m)
            -0.010,     // min gripper limit (-0.01 m)
            -0.015,     // Change unit from `meter` to `radian`
            3.6700241e-02 * 2,                                                    // mass
            math::inertiaMatrix(1.3343511e-05, 6.5936965e-06, -1.2045858e-09,
                                               3.1948952e-05, 1.8419228e-10,
                                                              2.7456763e-05),     // inertial tensor
            math::vector3(1.3928529e-02, -7.3239535e-03, 4.5486219e-07)          // COM
            );
          
  /*****************************************************************************
  ** Initialize Kinematics 
  *****************************************************************************/
  kinematics_ = new kinematics::SolverCustomizedforOMChain();
//  kinematics_ = new kinematics::SolverUsingCRAndSRPositionOnlyJacobian();
  addKinematics(kinematics_);

  if(!sim)
  {
    /*****************************************************************************
    ** Initialize Joint Actuator
    *****************************************************************************/
    // actuator_ = new dynamixel::JointDynamixel();
    actuator_ = new dynamixel::JointDynamixelProfileControl(control_loop_time);
    
    // Set communication arguments
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg;

    // Set joint actuator id
    std::vector<uint8_t> jointDxlId;
    jointDxlId.push_back(dxl_id[0]);
    jointDxlId.push_back(dxl_id[1]);
    jointDxlId.push_back(dxl_id[2]);
    jointDxlId.push_back(dxl_id[3]);
    addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

    // Set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

    /*****************************************************************************
    ** Initialize Tool Actuator
    *****************************************************************************/
    tool_ = new dynamixel::GripperDynamixel();

    uint8_t gripperDxlId = dxl_id[4];
    addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

    // Set gripper actuator control mode
    STRING gripper_dxl_mode_arg = "current_based_position_mode";
    void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

    // Set gripper actuator parameter
    STRING gripper_dxl_opt_arg[2];
    void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
    gripper_dxl_opt_arg[0] = "Profile_Acceleration";
    gripper_dxl_opt_arg[1] = "20";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    gripper_dxl_opt_arg[0] = "Profile_Velocity";
    gripper_dxl_opt_arg[1] = "200";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    // Enable All Actuators 
    enableAllActuator();

    // Receive current angles from all actuators 
    receiveAllJointActuatorValue();
    receiveAllToolActuatorValue();
  }

  /*****************************************************************************
  ** Initialize Custom Trajectory
  *****************************************************************************/
  custom_trajectory_[0] = new custom_trajectory::Line();
  custom_trajectory_[1] = new custom_trajectory::Circle();
  custom_trajectory_[2] = new custom_trajectory::Rhombus();
  custom_trajectory_[3] = new custom_trajectory::Heart();

  addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);
}

void OpenManipulatorX::process_open_manipulator_x(double present_time)
{
  // Planning (ik)
  JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);
  JointWaypoint goal_tool_value  = getToolGoalValue();

  // Control (motor)
  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();
  if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);
  if(goal_tool_value.size() != 0) sendAllToolActuatorValue(goal_tool_value);

  // Perception (fk)
  solveForwardKinematics();
}
