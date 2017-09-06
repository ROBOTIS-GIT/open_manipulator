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

#include "../../include/open_manipulator/OPMDraw.h"

OPMDraw::OPMDraw() 
  : control_period(CONTROL_RATE * 1e-6)
{

}

OPMDraw::~OPMDraw()
{

}

void OPMDraw::begin(OPMLink *link, int8_t link_num, bool dynamixel)
{
  copy_link     = link;
  copy_link_num = link_num;
  platform      = dynamixel;

  drawing = false;
}

void OPMDraw::setCircle(Circle set_circle)
{
  circle = set_circle;
}

void OPMDraw::setDrawTime(float set_time)
{
  draw_time = set_time;
}

void OPMDraw::setRange(State* start, State* finish)
{
  mj.setCoeffi(start, finish, 1, draw_time, control_period);
}

bool OPMDraw::getDrawing()
{
  return drawing;
}

void OPMDraw::start()
{
  step_cnt = 0;
  drawing  = true;  
}

void OPMDraw::drawCircle()
{
  uint16_t step_time = uint16_t(floor(draw_time/control_period) + 1.0);
  float tick_time = 0;  

  if (drawing)
  {
    if (step_cnt < step_time)
    {
      tick_time = control_period * step_cnt;

      State* tra_state;
      State goal_state[copy_link_num];
      Pose circle_pose;

      mj.getPosition(tra_state, 1, tick_time);
    
      circle_pose.position << (circle.x-circle.radius) + circle.radius*cos(tra_state->pos),
                               circle.y                + circle.radius*sin(tra_state->pos),
                               circle.z; 
    
      inverseKinematics(copy_link, findMe("Gripper"), circle_pose, "position");   
      
      for (int i = findMe("Joint1"); i < findMe("Gripper"); i++)
      {
        goal_state[i].pos = copy_link[i].joint_angle_;
      }

      if (platform)
        writeDXL(goal_state);

      sendAngle2Processing(goal_state);

      step_cnt++;
    }
    else
    {
      step_cnt = 0;
      drawing  = false; 
    }
  }  
}