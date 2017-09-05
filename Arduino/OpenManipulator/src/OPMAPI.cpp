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

#include "OPMAPI.h"

#define CONTROL_RATE 10000
#define MOVE_TIME    3.0

#define CHECK_FLAG   0
#define WAIT_FOR_SEC 1

#define MOTION_NUM 15
#define STORAGE 10

OPM opm;
OPMLink* copy_link;
OPMDynamixel dxl;
OPMKinematics km;
OPMMinimumJerk mj;
RC100 rc100;

State state[STORAGE] = {{0.0, 0.0, 0.0}, };
Position pos[STORAGE] = {{0.0, 0.0}, };

uint8_t motion_cnt = 0;
uint8_t motion_num = 0;
float motion_set[MOTION_NUM][STORAGE] = { // time, grip, joint1, joint2, joint3, joint4, ...
                                            { 3.0,  0.0,   0.0,  1.05, -0.35, -0.70 }, 
                                            { 3.0,  0.0,   0.0, -0.05, -0.82,  0.90 },
                                            { 3.0,  0.0,  0.35, -0.05, -0.82,  0.90 },
                                            { 3.0,  0.0,  0.35, -0.60,  0.05,  0.55 },
                                            { 3.0, -1.0,  0.35, -0.60,  0.05,  0.55 },
                                            { 3.0,  0.0,  0.35, -0.05, -0.82,  0.90 },
                                            { 3.0,  0.0, -0.35, -0.05, -0.82,  0.90 },
                                            { 3.0,  0.0, -0.35, -0.60,  0.05,  0.55 },
                                            { 3.0,  1.0, -0.35, -0.60,  0.05,  0.55 },
                                            { 3.0,  0.0, -0.35, -0.05, -0.82,  0.90 },
                                            { 3.0,  0.0,   0.0, -0.05, -0.82,  0.90 },
                                            { 3.0,  0.0,   0.0,  1.05, -0.35, -0.70 }
                                        };

HardwareTimer control_timer(TIMER_CH1);

float mov_time             = MOVE_TIME;
uint16_t step_cnt          = 0;
const float control_period = CONTROL_RATE * 1e-6;

bool platform  = true;
bool moving    = false;
bool motion    = false;
bool repeat    = false;

String cmd[5];

void setJointAngle(float* radian)
{
  for (int i = opm.joint1; i < opm.grip; i++)
  {
    pos[i].target  = radian[i];
  } 
}

void setGripAngle(float radian)
{
  pos[opm.grip].target  = radian;
}

void getAngle()
{
  int32_t dxl_angle[opm.dxl_num];

  dxl.readPos(dxl_angle);  

  for (int i = opm.joint1; i <= opm.grip; i++)
  {
    int id = i;
    pos[i].present = dxl.convertValue2Radian(id, dxl_angle[i-1]);
    state[i].pos   = dxl.convertValue2Radian(id, dxl_angle[i-1]);
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
  State start[opm.link_num], target[opm.link_num];

  for (int i = opm.base; i <= opm.grip; i++)
  {
    start[i].pos  = state[i].pos;
    start[i].vel  = state[i].vel;
    start[i].acc  = state[i].acc;

    target[i].pos = pos[i].target;
    target[i].vel = 0.0;
    target[i].acc = 0.0;
  }

  setMoveTime(set_move_time);
  mj.setCoeffi(start, target, opm.link_num, mov_time, control_period);

  step_cnt = 0;
  moving   = true;  
}

void initDynamixel(bool torque_onoff)
{
  dxl.begin();
  dxl.setTorque(torque_onoff);
  dxl.setSyncWrite();
  dxl.setSyncRead();

  getAngle();
}

void setTorque(bool onoff)
{
  dxl.setTorque(onoff);
}

Pose setPose(String dir)
{
  Pose target_pose;
  float step = 0.010;

  for (int i = opm.base; i <= opm.grip; i++)
    copy_link[i].joint_angle_ = pos[i].present;

  forwardKinematics(copy_link, opm.base);

  target_pose.position    = copy_link[opm.grip].p_;
  target_pose.orientation = copy_link[opm.grip].R_;

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
  control_timer.attachInterrupt(handler_control);

  if (onoff)
    control_timer.start();
  else
    control_timer.stop();
}

void setMotion()
{
  if (motion)
  {
    if (getMoving())
      return;

    if (motion_cnt >= motion_num)
    {
      if (repeat)
      {
        motion_cnt = 0;
      }
      else
      {
        motion     = false;       
      }
    }

    if (motion_set[motion_cnt][1] == -1.0)
    {
      setGripAngle(1.3);
      move(1.5);

      motion_cnt++;
    }
    else if (motion_set[motion_cnt][1] == 1.0)
    {
      setGripAngle(0.0);
      move(1.5);

      motion_cnt++;
    }
    else
    {
      // for (int num = JOINT1; num <= JOINT4; num++)
      //   target_pos[num] = motion_storage[motion_cnt][num];

      // jointMove(target_pos, motion_storage[motion_cnt][5]);
      // motion_cnt++;

      float target_pos[opm.dxl_num] = {0.0, };
      
      for (int i = opm.joint1; i < opm.grip; i++)
        target_pos[i] = motion_set[motion_cnt][i+1];
  
      setJointAngle(target_pos);
      move(motion_set[motion_cnt][0]);

      motion_cnt++;
    }
  }
  else
  {
    motion_cnt = 0;
  }
}

void forwardKinematics(OPMLink* link, int8_t from)
{
  for (int i = opm.base; i<=opm.grip; i++)
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

void getSeriesInfo(String series)
{
  if (series == "Chain")
  {
    opm.link_num  = 6;
    opm.joint_num = 4;
    opm.grip_num  = 1;
    
    opm.dxl_num = 5;
    
    opm.base   = 0;
    opm.joint1 = 1;
    opm.joint2 = 2;
    opm.joint3 = 3;
    opm.joint4 = 4;
    opm.grip   = 5;
  }
  else if (series == "SCARA")
  {
    opm.link_num  = 6;
    opm.joint_num = 4;
    opm.grip_num  = 1;
    
    opm.dxl_num = 5;
    
    opm.base   = 0;
    opm.joint1 = 1;
    opm.joint2 = 2;
    opm.joint3 = 3;
    opm.joint4 = 4;
    opm.grip   = 5;
  }
}

void writeDXL(State* data)
{
  int32_t value[opm.dxl_num] = {0, };

  for (int i = opm.joint1; i <= opm.grip; i++)
  {
    value[i-1] = dxl.convertRadian2Value(i, data[i].pos);
  }

  dxl.writePos(value);
}

void initProcessing()
{  
  Serial.print(0.0);
  Serial.print(",");
  Serial.print(0.0);
  Serial.print(",");
  Serial.print(0.0);
  Serial.print(",");
  Serial.print(0.0);
  Serial.print(",");
  Serial.println(0.0);
  delay(300);

  Serial.println("Init Processing");
}

void sendAngle2Processing(State* data, int8_t size)
{
  Serial.print("angle");

  for (int i = 1; i <= size; i++)
  {
    Serial.print(",");
    Serial.print(data[i].pos);
  }
  Serial.println(" ");
}

void OPMInit(String series, OPMLink* link, bool dynamixel, bool torque_onoff)
{
  copy_link = link;
  platform = dynamixel;

  initProcessing();
  getSeriesInfo(series);

  if (platform)
    initDynamixel(torque_onoff);  
  
  forwardKinematics(copy_link, opm.base);

  setTimer(true);  
}

void OPMRun()
{
  setMotion();

  if (rc100.available())
    dataFromRC100(rc100.readData());
}

void OPMSimulator(String ctrl)
{
  dataFromProcessing(ctrl);
}

void handler_control()
{
  uint16_t step_time = uint16_t(floor(mov_time/control_period) + 1.0);
  float tick_time = 0;

  if (moving)
  {
    if (step_cnt < step_time)
    {
      tick_time = control_period * step_cnt;
      
      mj.getPosition(state, opm.grip, tick_time);
      mj.getVelocity(state, opm.grip, tick_time);
      mj.getAcceleration(state, opm.grip, tick_time);

      if (platform)
        writeDXL(state);

      sendAngle2Processing(state, opm.dxl_num);

      for (int i = opm.base; i <= opm.grip; i++)
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

void split(String data, char separator, String* temp)
{
	int cnt = 0;
	int get_index = 0;

  String copy = data;
  
	while(true)
	{
		get_index = copy.indexOf(separator);

		if(-1 != get_index)
		{
			temp[cnt] = copy.substring(0, get_index);

			copy = copy.substring(get_index + 1);
		}
		else
		{
      temp[cnt] = copy.substring(0, copy.length());
			break;
		}
		++cnt;
	}
}

void dataFromRC100(uint16_t receive_data)
{
  float target_pos[opm.dxl_num] = {0.0, };

  if (receive_data & RC100_BTN_U)
  {    
    inverseKinematics(copy_link, opm.grip, setPose("forward"), "position");  
    
    for (int i = opm.joint1; i < opm.grip; i++)
      target_pos[i] = copy_link[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_D)
  {
    inverseKinematics(copy_link, opm.grip, setPose("back"), "position");  

    for (int i = opm.joint1; i < opm.grip; i++)
      target_pos[i] = copy_link[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_L)
  {
    inverseKinematics(copy_link, opm.grip, setPose("left"), "position");  

    for (int i = opm.joint1; i < opm.grip; i++)
      target_pos[i] = copy_link[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_R)
  {
    inverseKinematics(copy_link, opm.grip, setPose("right"), "position");  

    for (int i = opm.joint1; i < opm.grip; i++)
      target_pos[i] = copy_link[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_1)
  {
    inverseKinematics(copy_link, opm.grip, setPose("up"), "position");  

    for (int i = opm.joint1; i < opm.grip; i++)
      target_pos[i] = copy_link[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_2)
  {
    setGripAngle(1.3);
    move(1.5);
  }
  else if (receive_data & RC100_BTN_3)
  {
    inverseKinematics(copy_link, opm.grip, setPose("down"), "position");  

    for (int i = opm.joint1; i < opm.grip; i++)
      target_pos[i] = copy_link[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_4)
  {
    setGripAngle(0.0);
    move(1.5);
  }
  else if (receive_data & RC100_BTN_5)
  {
    target_pos[1] = 0.0;
    target_pos[2] = 60.0  * PI/180.0;
    target_pos[3] = -20.0 * PI/180.0;
    target_pos[4] = -40.0 * PI/180.0;

    setJointAngle(target_pos);
    move();
  }
  else if (receive_data & RC100_BTN_6)
  {
    target_pos[1] = 0.0;
    target_pos[2] = 0.0;
    target_pos[3] = 0.0;
    target_pos[4] = 0.0;

    setJointAngle(target_pos);
    move();
  }
}

void dataFromProcessing(String get)
{
  get.trim();

  split(get, ',', cmd);

  if (cmd[0] == "opm")
  {
    if (cmd[1] == "ready")
    {
      if (platform)
        setTorque(true);

      sendAngle2Processing(state, opm.dxl_num); 
    }
    else if (cmd[1] == "end")
    {
      if (platform)
        setTorque(false);
    }
  }
  else if (cmd[0] == "joint")
  {
    float target_pos[opm.dxl_num] = {0.0, };

    for (int i = opm.joint1; i < opm.grip; i++)
      target_pos[i] = cmd[i].toFloat();

    setJointAngle(target_pos);
    move();
  }
  else if (cmd[0] == "gripper")
  {
    setGripAngle(cmd[1].toFloat());
    move(1.5);
  }
  else if (cmd[0] == "grip")
  {
    if (cmd[1] == "on")
      setGripAngle(1.3);
    else if (cmd[1] == "off")
      setGripAngle(0.0);

    move(1.5);
  }
  else if (cmd[0] == "task")
  {
    float target_pos[opm.dxl_num] = {0.0, };

    inverseKinematics(copy_link, opm.grip, setPose(cmd[1]), "position");   
    
    for (int i = opm.joint1; i < opm.grip; i++)
      target_pos[i] = copy_link[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (cmd[0] == "torque")
  {
    if (platform)
    {
      if (cmd[1] == "on")
        setTorque(true);
      else if (cmd[1] == "off")
        setTorque(false);
    }
  }
//   else if (cmd[0] == "get")
//   {
//     if (cmd[1] == "on")
//     {
//       motion_storage[motion_num][0] = -1;
//       motion_num++;
//     }
//     else if (cmd[1] == "off")
//     {
//       motion_storage[motion_num][0] = -2;
//       motion_num++;
//     }
//     else if (cmd[1].toInt() < STORAGE)
//     {
//       getDynamixelPosition();
//       sendJointDataToProcessing();

//       for (int i = 0; i < LINK_NUM; i++)
//       {
//         motion_storage[motion_num][i] = motor[i].present_position;
//       }
//       motion_num++;
//     }
//   }
//   else if (cmd[0] == "hand")
//   {
//     if (cmd[1] == "once")
//     {
//       setMotorTorque(true);

//       getDynamixelPosition();
//       sendJointDataToProcessing();

//       motion = true;
//     }
//     else if (cmd[1] == "repeat")
//     {
//       setMotorTorque(true);

//       getDynamixelPosition();
//       sendJointDataToProcessing();

//       motion = true;
//       repeat = true;
//     }
//     else if (cmd[1] == "stop")
//     {
//       for (int i=0; i<STORAGE; i++)
//       {
//         for (int j=0; j<LINK_NUM; j++)
//         {
//           motion_storage[i][j] = 0;
//         }
//       }

//       motion     = false;
//       repeat     = false;
//       motion_num = 0;
//     }
//   }
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "start")
    {
      if (platform)
        getAngle();
      sendAngle2Processing(state, opm.dxl_num); 

      motion_num = 12;  
      motion_cnt = 0;          
      motion = true;
      repeat = true;
    }
    else if (cmd[1] == "stop")
    {
      motion  = false;
      repeat  = false;
    }
  }
}