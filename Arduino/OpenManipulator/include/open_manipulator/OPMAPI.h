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

#ifndef OPMAPI_H_
#define OPMAPI_H_

#include <Arduino.h>
#include <RC100.h>
#include "OPMLink.h"
#include "OPMMath.h"
#include "OPMKinematics.h"
#include "OPMMinimumJerk.h"
#include "OPMDebug.h"
#include "OPMComm.h"
#include "OPMDynamixel.h"

void setJointAngle(float* radian);

void setGripAngle(float radian);

State* getAngle();

State* getState();

void setState(State* copy_state);

bool getMoving();

void setMoveTime(float set_time = 3.0);

void initDynamixel(bool torque_onoff, uint32_t baud_rate = 1000000);

void initProcessing(int8_t link_num);

void setCurrentMode(uint8_t id);

void setCurrentPos(uint8_t id, float pos, int16_t cur = 30);

void setTorque(bool onoff);

Pose setPose(String dir);

void setTimer(bool onoff);

void move(float set_move_time = 3.0);

void forwardKinematics(OPMLink* link, int8_t from);

void inverseKinematics(OPMLink* link, int8_t to, Pose goal_pose, String method = "normal");

void writeDXL(State* data);

void sendAngle2Processing(State* data);

void OPMInit(OPMLink* link, int8_t link_num, bool processing = true, bool dynamixel = true, bool torque_onoff = true);

void OPMRun();

int8_t findMe(String name);

static void jointControl();

#endif  //OPMAPI_H_