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

#ifndef OPEN_MANIPULATOR_CHAIN_CONFIG_H_
#define OPEN_MANIPULATOR_CHAIN_CONFIG_H_

#include "OpenManipulator.h"
#include <RC100.h>

#define CONTROL_RATE        10000
#define SERIAL_RATE         57600
#define REMOTE_RATE         100
#define BAUE_RATE           1000000

#define PROTOCOL_VERSION    2.0

#define JOINT_NUM           4
#define GRIP_NUM            1
#define LINK_NUM            6

#define STORAGE 15

#define BASE    0
#define JOINT1  1
#define JOINT2  2
#define JOINT3  3
#define JOINT4  4
#define END     5

#define CHECK_FLAG   0
#define WAIT_FOR_SEC 1

#define JOINT_TRA_TIME       2.5
#define GRIP_TRA_TIME        1.6

#define MOTION_TRA_TIME      3.0

#define TASK_TRA_TIME        0.16
#define TASK_TRA_UNIT        0.010

const float grip_on  = 1.3;
const float grip_off = 0.0;

float mov_time             = 0.0;
uint16_t step_cnt          = 0;
const float control_period = CONTROL_RATE * 1e-6;

bool moving        = false;
bool comm          = false;
bool motion        = false;
bool repeat        = false;

uint8_t motion_cnt = 0;
uint8_t motion_num = 0;

String cmd[5];

float target_pos[LINK_NUM];

float goal_pos[LINK_NUM];
float goal_vel[LINK_NUM];
float goal_acc[LINK_NUM];

float motion_storage[STORAGE][LINK_NUM];
float motion_set[STORAGE][LINK_NUM] = { //grip, joint1, joint2, joint3, joint, time
                                        { 0.0,   0.0,  1.05, -0.35, -0.70, 3.0}, 
                                        { 0.0,   0.0, -0.05, -0.82,  0.90, 3.0},
                                        { 0.0,  0.35, -0.05, -0.82,  0.90, 3.0},
                                        { 0.0,  0.35, -0.60,  0.05,  0.55, 1.0},
                                        {-1.0,  0.35, -0.60,  0.05,  0.55, 1.0},
                                        { 0.0,  0.35, -0.05, -0.82,  0.90, 1.0},
                                        { 0.0, -0.35, -0.05, -0.82,  0.90, 1.0},
                                        { 0.0, -0.35, -0.60,  0.05,  0.55, 1.0},
                                        { 1.0, -0.35, -0.60,  0.05,  0.55, 3.0},
                                        { 0.0, -0.35, -0.05, -0.82,  0.90, 3.0},
                                        { 0.0,   0.0, -0.05, -0.82,  0.90, 3.0},
                                        { 0.0,   0.0,  1.05, -0.35, -0.70, 3.0}
                                      };

open_manipulator::Motor        motor[LINK_NUM];
open_manipulator::Link         link[LINK_NUM];

open_manipulator::Kinematics  kinematics;
open_manipulator::MotorDriver motor_driver(PROTOCOL_VERSION, BAUE_RATE);
open_manipulator::MinimumJerk minimum_jerk;

open_manipulator::Property     start_prop[LINK_NUM];
open_manipulator::Property     end_prop[LINK_NUM];

HardwareTimer control_timer(TIMER_CH1);

RC100 rc100;

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
void setFK(open_manipulator::Link* link, int8_t me);
void setIK(open_manipulator::Link* link, uint8_t to, open_manipulator::Pose goal_pose);
void setPoseDirection(String dir, float step);

// DYNAMIXEL
void initMotorDriver(bool torque);
void setMotorTorque(bool onoff);
void setJointDataToDynamixel();
void setGripperDataToDynamixel();
void getDynamixelPosition();
void getMotorAngle();

// DATA
void getData(uint32_t wait_time);
void dataFromProcessing(String get);
void dataFromRC100(uint8_t receive_data);
void split(String data, char separator, String* temp);

// MinimumJerk
void jointMove(float* joint_pos, float mov_time);
void gripMove(float grip_pos, float mov_time);
void setMoveTime(float get_time);

// Communication
void establishContactToProcessing();
void sendJointDataToProcessing();

// Motion
void setMotion();


#endif // OPEN_MANIPULATOR_CHAIN_CONFIG_H_
