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

void OPMDraw::setObject(Object set_object)
{
  State* present_state = getState();

  for (int i = findMe("BASE"); i <= findMe("Gripper"); i++)
    copy_link[i].joint_angle_ = present_state[i].pos;

  km.forward(copy_link, findMe("BASE"));
  object.x = copy_link[findMe("Gripper")].p_(0);
  object.y = copy_link[findMe("Gripper")].p_(1);
  object.z = copy_link[findMe("Gripper")].p_(2);

  object.radius = set_object.radius;
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

Pose OPMDraw::Circle(State* tra)
{
  Pose circle_pose;
  Object circle = object;

  circle_pose.position << (circle.x-circle.radius) + circle.radius*cos(tra->pos),
                           circle.y                + circle.radius*sin(tra->pos),
                           circle.z;

  return circle_pose;
}

Pose OPMDraw::Heart(State* tra)
{
  const float offset = 0.020;

  Pose heart_pose;
  Object heart = object;

  heart_pose.position  <<  heart.radius * (16.0*pow(sin(tra->pos), 3)),
                           heart.radius * (13.0*cos(tra->pos) - 5.0*cos(2*tra->pos) - 2.0*cos(3*tra->pos) - cos(4.0*tra->pos)),
                           0.0;

  OPMMath math;
  heart_pose.position = math.RotationMatrix("yaw", -90*DEG2RAD) * heart_pose.position;

  heart_pose.position  <<  (heart.x - offset) + heart_pose.position(0),
                            heart.y           + heart_pose.position(1),
                            heart.z;

  return heart_pose;            
}

void OPMDraw::drawObject(String object)
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
      Pose object_pose;

      mj.getPosition(tra_state, 1, tick_time);

      if (object == "circle")
        object_pose = Circle(tra_state);
      else if (object == "heart")
        object_pose = Heart(tra_state);
    
      inverseKinematics(copy_link, findMe("Gripper"), object_pose, "position");   
      
      for (int i = findMe("Joint1"); i <= findMe("Gripper"); i++)
      {
        if (i == findMe("Gripper"))
        {
          State* state = getState();
          goal_state[i].pos = state[i].pos;
        }
        else
        {
          goal_state[i].pos = copy_link[i].joint_angle_;
        }
      }

      if (platform)
        writeDXL(goal_state);

      sendAngle2Processing(goal_state);

      setState(goal_state);
      step_cnt++;
    }
    else
    {
      step_cnt = 0;
      drawing  = false; 
    }
  }  
}