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

#ifndef OPMDRAW_H_
#define OPMDRAW_H_

#include "OPMAPI.h"

#define CONTROL_RATE 10000

typedef struct
{
  float x;
  float y;
  float z;
  float radius;
} Object;

class OPMDraw
{
 public:
  Object object;

  float draw_time;
  uint16_t step_cnt;
  const float control_period;

  bool drawing;
  bool platform;

  OPMLink* copy_link;
  int8_t copy_link_num;
  OPMMinimumJerk mj;
  OPMKinematics km;

 public:
  OPMDraw();
  ~OPMDraw();

  void begin(OPMLink *link, int8_t link_num, bool dynamixel = false);
  void setObject(Object set_object);
 
  void setDrawTime(float set_time);
  void setRange(State* start, State* finish);
  bool getDrawing();
  void start();

  Pose Circle(State* tra);
  Pose Heart(State* tra);

  void drawObject(String set_object);
};

#endif //OPMDRAW_H_