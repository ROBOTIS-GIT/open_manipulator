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

#include "../../include/open_manipulator/OPMAPI.h"

#define CONTROL_RATE 10000
#define MOVE_TIME    3.0
#define STORAGE 20

OPMLink* copy_link;
int8_t copy_link_num;
OPMDynamixel dxl;
OPMKinematics km;
OPMMinimumJerk mj;

State state[STORAGE] = {{0.0, 0.0, 0.0}, };
Position pos[STORAGE] = {{0.0, 0.0}, };

HardwareTimer control_timer(TIMER_CH1);

float mov_time             = MOVE_TIME;
uint16_t step_cnt          = 0;
const float control_period = CONTROL_RATE * 1e-6;

bool platform  = true;
bool moving    = false;

void setJointAngle(float* radian)
{
  for (int i = findMe("Joint1"); i < findMe("Gripper"); i++)
  {
    pos[i].target  = radian[i];
  } 
}

void setGripAngle(float radian)
{
  pos[findMe("Gripper")].target  = radian;
}

State* getAngle()
{
  int32_t* dxl_angle;
  dxl_angle = dxl.readPos();  

  for (int i = findMe("Joint1"); i <= findMe("Gripper"); i++)
  {
    int id = i;
    pos[i].present = dxl.convertValue2Radian(id, dxl_angle[i-1]);
    state[i].pos   = dxl.convertValue2Radian(id, dxl_angle[i-1]);
  }

  return state;
}

State* getState()
{
  return state;
}

void setState(State* copy_state)
{
  for (int i = findMe("Joint1"); i <= findMe("Gripper"); i++)
  {
    state[i].pos = copy_state[i].pos;
  }
}

bool getMoving()
{
  return moving;
}

void setMoveTime(float set_time)
{
  mov_time = set_time;
}

void move(float set_move_time)
{
  State start[copy_link_num], target[copy_link_num];

  for (int i = findMe("BASE"); i <= findMe("Gripper"); i++)
  {
    start[i].pos  = state[i].pos;
    start[i].vel  = state[i].vel;
    start[i].acc  = state[i].acc;

    target[i].pos = pos[i].target;
    target[i].vel = 0.0;
    target[i].acc = 0.0;
  }

  setMoveTime(set_move_time);
  mj.setCoeffi(start, target, copy_link_num, mov_time, control_period);

  step_cnt = 0;
  moving   = true;  
}

void initDynamixel(bool torque_onoff, uint32_t baud_rate)
{
  dxl.begin("/dev/ttyUSB0", baud_rate);
  dxl.setTorque(true);
  dxl.setSyncWrite();
  dxl.setSyncRead();

  getAngle();
}

void setCurrentMode(uint8_t id)
{
  const uint8_t current_mode = 5;

  dxl.setMode(id, current_mode);
}

void setCurrentPos(uint8_t id, float pos, int16_t cur)
{
  dxl.writeCur(id, cur);
  dxl.writePos(id, dxl.convertRadian2Value(id, pos));
}

void setTorque(bool onoff)
{
  dxl.setTorque(onoff);
}

Pose setPose(String dir)
{
  Pose target_pose;
  float step = 0.010;

  for (int i = findMe("BASE"); i <= findMe("Gripper"); i++)
    copy_link[i].joint_angle_ = pos[i].present;

  forwardKinematics(copy_link, findMe("BASE"));

  target_pose.position    = copy_link[findMe("Gripper")].p_;
  target_pose.orientation = copy_link[findMe("Gripper")].R_;

  if (dir == "forward")
  {
    target_pose.position(0) += step;
  }
  else if (dir == "back")
  {
    target_pose.position(0) -= step;
  }
  else if (dir == "left")
  {
    target_pose.position(1) += step;
  }
  else if (dir == "right")
  {
    target_pose.position(1) -= step;
  }
  else if (dir == "up")
  {
    target_pose.position(2) += step;
  }
  else if (dir == "down")
  {
    target_pose.position(2) -= step;
  }
  
  return target_pose;
}

void setTimer(bool onoff)
{
  control_timer.stop();
  control_timer.setPeriod(CONTROL_RATE);
  control_timer.attachInterrupt(jointControl);

  if (onoff)
    control_timer.start();
  else
    control_timer.stop();
}

void forwardKinematics(OPMLink* link, int8_t from)
{
  for (int i = findMe("BASE"); i <= findMe("Gripper"); i++)
    link[i].joint_angle_ = pos[i].present;

  km.forward(link, from);
}

void inverseKinematics(OPMLink* link, int8_t to, Pose goal_pose, String method)
{
  if (method == "normal")
    km.inverse(link, to, goal_pose);
  else if (method == "robust")
    km.sr_inverse(link, to, goal_pose);
  else if (method == "position")
    km.position_only_inverse(link, to, goal_pose);
}

void writeDXL(State* data)
{
  int32_t value[copy_link_num] = {0, };

  for (int i = findMe("Joint1"); i <= findMe("Gripper"); i++)
  {
    value[i-1] = dxl.convertRadian2Value(i, data[i].pos);
  }

  dxl.writePos(value);
}

void initProcessing(int8_t link_num)
{  
  int8_t joint_num = link_num-1;

  for (int i = 0; i < joint_num-1; i++)
  {
    Serial.print(0.0);
    Serial.print(",");
  }

  Serial.println(0.0);
  delay(300);

  Serial.println("Init Processing");
}

void sendAngle2Processing(State* data)
{
  Serial.print("angle");

  for (int i = findMe("Joint1"); i <= findMe("Gripper"); i++)
  {
    Serial.print(",");
    Serial.print(data[i].pos);
  }
  Serial.println(" ");
}

void OPMInit(OPMLink* link, int8_t link_num, bool processing, bool dynamixel, bool torque_onoff)
{
  copy_link_num  = link_num;
  copy_link      = link;
  platform       = dynamixel;

  if (processing)
    initProcessing(link_num);

  if (platform)
    initDynamixel(torque_onoff, 1000000);  
  
  forwardKinematics(copy_link, findMe("BASE"));

  //setTimer(true);  
}

void OPMRun()
{
  static uint32_t tmp_time = micros();
  
  if ((micros() - tmp_time) >= CONTROL_RATE)
  {
    tmp_time = micros();
    jointControl();
  }
}

int8_t findMe(String name)
{
  for (int i = 0; i < copy_link_num; i++)
  {
    if (copy_link[i].name_ == name)
      return copy_link[i].me_;
  }
}

void jointControl()
{
  uint16_t step_time = uint16_t(floor(mov_time/control_period) + 1.0);
  float tick_time = 0;

  if (moving)
  {
    if (step_cnt < step_time)
    {
      tick_time = control_period * step_cnt;
      
      mj.getPosition(state, findMe("Gripper"), tick_time);
      mj.getVelocity(state, findMe("Gripper"), tick_time);
      mj.getAcceleration(state, findMe("Gripper"), tick_time);

      if (platform)
        writeDXL(state);

      sendAngle2Processing(state);

      for (int i = findMe("BASE"); i <= findMe("Gripper"); i++)
        pos[i].present = state[i].pos;

      step_cnt++;
    }
    else
    {
      step_cnt = 0;
      moving   = false; 
    }
  }  
}