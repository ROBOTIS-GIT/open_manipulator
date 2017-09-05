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
#define MOTION_RATE         8000
#define SERIAL_RATE         57600
#define REMOTE_RATE         100
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

#define CHECK_FLAG   0
#define WAIT_FOR_SEC 1

#define JOINT_TRA_TIME       1.0
#define GRIP_TRA_TIME        1.6

#define TASK_TRA_TIME        1.5

const float grip_on  = 0.0;
const float grip_off = -0.9;

float mov_time             = 0.0;
uint16_t step_cnt          = 0;
uint16_t motion_cnt        = 0;
const float control_period = CONTROL_RATE * 1e-6;
const float motion_period  = MOTION_RATE  * 1e-6;

bool moving        = false;
bool comm          = false;
bool motion        = false;
bool circle        = false;
bool reverse       = false;

String cmd[5];

float target_pos[LINK_NUM];

float goal_pos[LINK_NUM];
float goal_vel[LINK_NUM];
float goal_acc[LINK_NUM];

const float circle_x = 0.22;
const float circle_y = 0.01;
float radius = 0.005;

float draw_time = 5.0;

uint8_t motion_state = 0;

open_manipulator::Motor        motor[LINK_NUM];
open_manipulator::Link         link[LINK_NUM];

open_manipulator::Kinematics*  kinematics;
open_manipulator::MotorDriver* motor_driver;
open_manipulator::MinimumJerk* minimum_jerk;
open_manipulator::MinimumJerk* circle_tra;

open_manipulator::Property     start_prop[LINK_NUM];
open_manipulator::Property     end_prop[LINK_NUM];

HardwareTimer control_timer(TIMER_CH1);

// Link
void initLink();

// Joint properties
void initJointProp();
void setJointProp(float* set_goal_pos);
void setGripperProp(float set_goal_pos);

// Timer
void initTimer();
void setTimer(bool onoff);

// kinematics
void initKinematics();
void setFK(open_manipulator::Link* link, int8_t me);
void setIK(open_manipulator::Link* link, uint8_t to, open_manipulator::Pose goal_pose);
void setPose(open_manipulator::Pose target_pose);

// DYNAMIXEL
void initMotor();
void initMotorDriver(bool torque);
void setMotorTorque(bool onoff);
void setJointDataToDynamixel();
void setGripperDataToDynamixel();
void getDynamixelPosition();
void getMotorAngle();

// DATA
void getData(uint32_t wait_time);
void dataFromProcessing(String get);
void split(String data, char separator, String* temp);

// MinimumJerk
void initMinimumJerk();
void jointMove(float* joint_pos, float mov_time);
void gripMove(float grip_pos, float mov_time);
void setMoveTime(float get_time);

// Communication
void establishContactToProcessing();
void sendJointDataToProcessing();

// Joint
void jointControl();

// Motion
void drawCircle();

void setMotion();

#endif // OPEN_MANIPULATOR_SCARA_CONFIG_H_
