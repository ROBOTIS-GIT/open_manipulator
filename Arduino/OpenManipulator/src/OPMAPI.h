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

void getAngle();

bool getMoving();

void setMoveTime(float set_time = 3.0);

void initDynamixel(bool torque_onoff);

void setTorque(bool onoff);

Pose setPose(String dir);

void setTimer(bool onoff);

void move(float set_move_time = 3.0);

void forwardKinematics(OPMLink* link, int8_t from);

void inverseKinematics(OPMLink* link, int8_t to, Pose goal_pose, String method = "normal");

void getSeriesInfo(String series);

void writeDXL(State* data);

void sendAngle2Processing(State* data, int8_t size);

void OPMInit(String series, OPMLink* link, bool dynamixel = true, bool torque_onoff = true);

void OPMRun();

void OPMSimulator(String ctrl);

static void setMotion();

static void handler_control();

static void initProcessing();

static void dataFromProcessing(String get);

static void dataFromRC100(uint16_t receive_data);

static void split(String data, char separator, String* temp);

#endif  //OPMAPI_H_