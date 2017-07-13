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

/* Authors: Darby Lim */

#ifndef OPEN_MANIPULATOR_SCARA_CONFIG_H_
#define OPEN_MANIPULATOR_SCARA_CONFIG_H_

#include "OpenManipulator.h"

#define CONTROL_RATE        8000
#define SERIAL_RATE         57600
#define BAUE_RATE           1000000

#define PROTOCOL_VERSION    2.0

#define JOINT_NUM           3
#define GRIP_NUM            1
#define LINK_NUM            5

#define BASE    0
#define JOINT1  1
#define JOINT2  2
#define JOINT3  3
#define END     4

const float grip_on  = 0.0;
const float grip_off = -1.0;

float mov_time             = 2.5;
const float control_period = 0.008;

bool moving        = false;
bool comm          = false;

String cmd[5];

float joint_pos[LINK_NUM];
float link_angle[LINK_NUM];
float motor_angle[LINK_NUM];

Eigen::MatrixXf joint_tra;

open_manipulator::Motor        motor[LINK_NUM];
open_manipulator::Link         link[LINK_NUM];
open_manipulator::Kinematics*  kinematics;
open_manipulator::MotorDriver* motor_driver;
open_manipulator::Property     start_prop[LINK_NUM];
open_manipulator::Property     end_prop[LINK_NUM];
open_manipulator::Trajectory*  trajectory;

HardwareTimer control_timer(TIMER_CH1);

void initLinkAndMotor();
void initTimer();
void initKinematics();
void initTrajectory();
void initMotorDriver(bool torque);

void establishContactToProcessing();

void setMoveTime(float get_time);
void setJointPropPos(float* joint_pos);
void setGripperPropPos(float gripper);

void setTimer(bool onoff);
void setMotorTorque(bool onoff);
void setMotion(bool onoff);
void setFK(open_manipulator::Link* link, int8_t me);
void setIK(open_manipulator::Link* link, uint8_t to, open_manipulator::Pose goal_pose);

// DYNAMIXEL
void setJointDataToDynamixel();
void setGripperDataToDynamixel();
void getDynamixelPosition();

// PROCESSING
void sendJointDataToProcessing();
void getDataFromProcessing(bool &comm);

void jointMove(float* joint_pos, float mov_time);
void gripMove(float grip_pos, float mov_time);

void getLinkAngle(float* angle);
void getMotorAngle(float* angle);

void split(String data, char separator, String* temp);

#endif // OPEN_MANIPULATOR_SCARA_CONFIG_H_
