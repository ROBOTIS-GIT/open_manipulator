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

#ifndef OPMCOMM_H_
#define OPMCOMM_H_

#include <unistd.h>
#include <Eigen.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

typedef struct
{
  Eigen::Vector3f position;
  Eigen::Matrix3f orientation;
} Pose;

typedef struct
{
  float pos;
  float vel;
  float acc;
} State;

typedef struct
{
  float present;
  float target;
} Position;

typedef struct
{
  int8_t link_num;
  int8_t joint_num;
  int8_t grip_num;
  
  int8_t dxl_num;

  int8_t base;
  int8_t joint1;
  int8_t joint2;
  int8_t joint3;
  int8_t joint4;
  int8_t grip;  
} OPM;

#endif // OPMCOMM_H_