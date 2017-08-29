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

#include "open_manipulator_scara_config.h"

// #define DEBUG
// #define DYNAMIXEL
#define SIMULATION

/*******************************************************************************
* Setup
*******************************************************************************/
void setup()
{
  Serial.begin(SERIAL_RATE);
#ifdef DEBUG
   while(!Serial);
#endif

  initLink();
  initMotor();

  initMinimumJerk();

  initJointProp();
  initKinematics();

  initMotorDriver(false);

  initTimer();

  establishContactToProcessing();

  setFK(link, BASE);

#ifdef DEBUG
  Serial.println("OpenManipulator SCARA Initialization Success!!");
#endif
}

/*******************************************************************************
* loop
*******************************************************************************/
void loop()
{
  static uint32_t tmp_time[2];
  
  if ((micros() - tmp_time[0]) >= CONTROL_RATE)
  {
    tmp_time[0] = micros();
    jointControl();
  }

  if ((micros() - tmp_time[1]) >= MOTION_RATE)
  {
    tmp_time[1] = micros();
    drawCircle();
  }

  setMotion();
  
  getData(REMOTE_RATE);

  showLedStatus();
}

/*******************************************************************************
* Timer (8mm)
*******************************************************************************/
void jointControl()
{
  uint16_t step_time = uint16_t(floor(mov_time/control_period) + 1.0);
  float tick_time = 0;

#ifdef DEBUG
  showJointProp(goal_pos, goal_vel, goal_acc, JOINT1, JOINT3);
#endif
  if (moving && comm)
  {
    if (step_cnt >= step_time)
    {
      for (int num = BASE; num <= END; num++)
        link[num].q_ = goal_pos[num];

      setFK(link, BASE);

      moving = false;
      step_cnt = 0;

#ifdef DEBUG
      Serial.println("End Trajectory");
#endif
    }
    else
    {
      tick_time = control_period * step_cnt;

      minimum_jerk->getPosition(goal_pos, END, tick_time);
      minimum_jerk->getVelocity(goal_vel, END, tick_time);
      minimum_jerk->getAcceleration(goal_acc, END, tick_time);

      sendJointDataToProcessing();
      setJointDataToDynamixel();
      setGripperDataToDynamixel();

      step_cnt++;
    }
  }
}

/*******************************************************************************
* draw circle
*******************************************************************************/
void drawCircle()
{
  uint16_t motion_time = uint16_t(floor(draw_time/motion_period) + 1.0);
  float tick_time = 0.0;

  float goal_angle[2] = {0.0, 0.0};
  open_manipulator::Pose goal_pose;

  if (circle)
  {
    if (motion_cnt >= motion_time)
    {
      if (reverse)
        radius -= 0.005;
      else
        radius += 0.005; 

      if (radius >= 0.060 || radius <= 0.005)
      {
        motion_cnt = 0;
        circle = false;
        motion_state++;
        reverse = !reverse;
      }
      else
      {
        motion_cnt = 0;
      }
    }
    else
    {
      tick_time = motion_period * motion_cnt;

      circle_tra->getPosition(goal_angle, 1, tick_time);
      goal_pose.position << (circle_x-radius) + radius*cos(goal_angle[0]),
                            circle_y + radius*sin(goal_angle[0]),
                            0.0661; 

      setPose(goal_pose);

      for (int i=BASE; i <= END; i++)
        goal_pos[i] = target_pos[i];

      sendJointDataToProcessing();
      setJointDataToDynamixel();

      motion_cnt++;
    }
  }
}

/*******************************************************************************
* Set motion
*******************************************************************************/
void setMotion()
{
  open_manipulator::Pose goal_pose;

  if (motion)
  {
    if (moving)
      return;

    if (circle)
      return;

    switch (motion_state)
    {
      case 0:
        goal_pose.position << circle_x,
                              circle_y,
                              0.0661;
    
        setPose(goal_pose);
        jointMove(target_pos, 3.0);

        motion_state = 1;
       break;

      case 1:
        gripMove(grip_on, GRIP_TRA_TIME);

        motion_state = 2;
       break; 

      case 2:
        initJointProp();
        start_prop[0].pos = 0.0;
        end_prop[0].pos   = M_PI*2;

        if (reverse)
          radius = 0.060;  
        else
          radius = 0.005;

        draw_time = 8.0;    
        circle_tra->setCoeffi(start_prop, end_prop, 1, draw_time, motion_period); 

        motion_cnt = 0;
        circle = true;
       break;

       case 3:
        gripMove(grip_off, GRIP_TRA_TIME);

        motion_state = 4;
       break;

     case 4:
       target_pos[JOINT1] = -1.731;
       target_pos[JOINT2] =  0.347;
       target_pos[JOINT3] =  2.247;

       jointMove(target_pos, 3.0);

       motion_state = 5;
      break;

     case 5:
       gripMove(-0.2, GRIP_TRA_TIME);
       
       motion_state = 6;
      break;

     case 6:
       gripMove(grip_off, GRIP_TRA_TIME);

       motion_state = 0;
      break;

      default:
       break;
    }
  }
  else
  {
    motion_state = 0;
  }
}

/*******************************************************************************
* Get Data 
*******************************************************************************/
void getData(uint32_t wait_time)
{
  static uint8_t state = 0;
  static uint32_t tick = 0;

  bool processing_flag = false;

  String get_processing_data = "";

  if (Serial.available())
  {
    get_processing_data = Serial.readStringUntil('\n');
    processing_flag = true;
  }

  switch (state)
  {
    case CHECK_FLAG:
      if (processing_flag)
      {
        dataFromProcessing(get_processing_data);   
        tick = millis();
        state  = WAIT_FOR_SEC;
      }
     break;
    
    case WAIT_FOR_SEC:
      if ((millis() - tick) >= wait_time)
      {
        state = CHECK_FLAG;
      }
     break;
    
    default :
     state = CHECK_FLAG;
     break;
  }
}

/*******************************************************************************
* Data From Processing
*******************************************************************************/
void dataFromProcessing(String get)
{
  get.trim();

  split(get, ',', cmd);

  if (cmd[0] == "mnp")
  {
    if (cmd[1] == "ready")
    {
      setMotorTorque(true);
      getDynamixelPosition();
      sendJointDataToProcessing();

      // setTimer(true);
      comm = true;
    }
    else if (cmd[1] == "end")
    {
      setMotorTorque(false);
      comm = false;
    }
  }
  else if (cmd[0] == "joint")
  {
    for (int num = JOINT1; num <= JOINT3; num++)
      target_pos[num] = cmd[num].toFloat();

    jointMove(target_pos, JOINT_TRA_TIME);
  }
  else if (cmd[0] == "gripper")
  {
    gripMove(cmd[1].toFloat(), GRIP_TRA_TIME);
  }
  else if (cmd[0] == "grip")
  {
    if (cmd[1] == "on")
      gripMove(grip_on, GRIP_TRA_TIME);
    else if (cmd[1] == "off")
      gripMove(grip_off, GRIP_TRA_TIME);
  }
  else if (cmd[0] == "pos")
  {
    open_manipulator::Pose goal_pose;
    goal_pose.position << cmd[1].toFloat(),
                          cmd[2].toFloat(),
                          0.0661;

    setPose(goal_pose);
    jointMove(target_pos, TASK_TRA_TIME);
  }
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "stop")
    {
      motion_state = 0;
      motion_cnt = 0;
      reverse = false;
      motion = false;
      circle = false;
    }
    else
    {
      getDynamixelPosition();
      sendJointDataToProcessing();

      motion_state = 3;
      motion = true;
    }
  }
  else
  {
#ifdef DEBUG
    Serial.println("Error");
#endif
  }
}

/*******************************************************************************
* Set Move Time
*******************************************************************************/
void setMoveTime(float get_time)
{
  mov_time = get_time;
}

/*******************************************************************************
* Send Joint Data to Processing
*******************************************************************************/
void sendJointDataToProcessing()
{
  Serial.print("angle");
  Serial.print(",");
  Serial.print(goal_pos[JOINT1]);
  Serial.print(",");
  Serial.print(goal_pos[JOINT2]);
  Serial.print(",");
  Serial.print(goal_pos[JOINT3]);
  Serial.print(",");
  Serial.println(goal_pos[END]);
}

/*******************************************************************************
* Init Joint Properties
*******************************************************************************/
void initJointProp()
{
  for (int num = BASE; num <= END; num++)
  {
    start_prop[num].pos   = 0.0;
    start_prop[num].vel   = 0.0;
    start_prop[num].acc   = 0.0;

    end_prop[num].pos     = 0.0;
    end_prop[num].vel     = 0.0;
    end_prop[num].acc     = 0.0;
  }
}

/*******************************************************************************
* Set Joint Properties
*******************************************************************************/
void setJointProp(float* set_target_pos)
{
  for (int num = JOINT1; num <= JOINT3; num++)
  {
    start_prop[num].pos   = goal_pos[num];
    start_prop[num].vel   = goal_vel[num];
    start_prop[num].acc   = goal_acc[num];

    end_prop[num].pos     = set_target_pos[num];
    end_prop[num].vel     = 0.0;
    end_prop[num].acc     = 0.0;
  }

  start_prop[END].pos    = goal_pos[END];
  start_prop[END].vel    = 0.0;
  start_prop[END].acc    = 0.0;

  end_prop[END].pos      = goal_pos[END];
  end_prop[END].vel      = 0.0;
  end_prop[END].acc      = 0.0;
}

/*******************************************************************************
* Set Gripper Properties
*******************************************************************************/
void setGripperProp(float get_target_pos)
{
  for (int num = JOINT1; num <= JOINT3; num++)
  {
    start_prop[num].pos   = goal_pos[num];
    start_prop[num].vel   = 0.0;
    start_prop[num].acc   = 0.0;

    end_prop[num].pos     = goal_pos[num];
    end_prop[num].vel     = 0.0;
    end_prop[num].acc     = 0.0;
  }

  start_prop[END].pos    = goal_pos[END];
  start_prop[END].vel    = goal_vel[END];
  start_prop[END].acc    = goal_acc[END];

  end_prop[END].pos      = get_target_pos;
  end_prop[END].vel      = 0.0;
  end_prop[END].acc      = 0.0;
}

/*******************************************************************************
* Initialization Timer
*******************************************************************************/
void initTimer()
{
  control_timer.stop();
  control_timer.setPeriod(CONTROL_RATE);
  control_timer.attachInterrupt(jointControl);
}

/*******************************************************************************
* Set Timer
*******************************************************************************/
void setTimer(bool onoff)
{
  if (onoff)
    control_timer.start();
  else
    control_timer.stop();
}

/*******************************************************************************
* Get Dynamixel Position (rad)
*******************************************************************************/
void getDynamixelPosition()
{
#ifdef DYNAMIXEL
  motor_driver->readPosition(motor);
  getMotorAngle();
#endif
}

/*******************************************************************************
* Set Pose
*******************************************************************************/
void setPose(open_manipulator::Pose target_pose)
{
  for (int num = BASE; num <= END; num++)
    link[num].q_ = goal_pos[num];

  setFK(link, BASE);

  setIK("position", link, END, target_pose);
}

/*******************************************************************************
* Joint move
*******************************************************************************/
void jointMove(float* set_goal_pos, float set_mov_time)
{
  setJointProp(set_goal_pos);
  setMoveTime(set_mov_time);

  minimum_jerk->setCoeffi(start_prop, end_prop, LINK_NUM, mov_time, control_period);

  step_cnt = 0;
  moving = true;
}

/*******************************************************************************
* Grip move
*******************************************************************************/
void gripMove(float set_goal_pos, float set_mov_time)
{
  setGripperProp(set_goal_pos);
  setMoveTime(set_mov_time);

  minimum_jerk->setCoeffi(start_prop, end_prop, LINK_NUM, mov_time, control_period);

  step_cnt = 0;
  moving = true;
}

/*******************************************************************************
* Get Motor Position (rad)
*******************************************************************************/
void getMotorAngle()
{
  for (int num = BASE; num <= END; num++)
  {
    goal_pos[num] = motor[num].present_position;
  }
}

/*******************************************************************************
* Forward Kinematics
*******************************************************************************/
void setFK(open_manipulator::Link* link, int8_t me)
{
  kinematics->forward(link, me);
}

/*******************************************************************************
* Inverse Kinematics
*******************************************************************************/
void setIK(String cmd, open_manipulator::Link* link, uint8_t to, open_manipulator::Pose goal_pose)
{
  if (cmd == "normal")
    kinematics->inverse(link, to, goal_pose);
  else if (cmd == "robust")
    kinematics->sr_inverse(link, to, goal_pose);
  else if (cmd == "position")
    kinematics->position_only_inverse(link, to, goal_pose);

  for (int id = JOINT1; id <= JOINT3; id++)
  {
    target_pos[id] = link[id].q_;
  }
}

/*******************************************************************************
* Write Dynamixel Position (rad)
*******************************************************************************/
void setJointDataToDynamixel()
{
#ifdef DYNAMIXEL
  int32_t joint_value[LINK_NUM] = {0, };

  for (int num = BASE; num <= END; num++)
  {
    joint_value[num] = motor_driver->convertRadian2Value(goal_pos[num]);
  }
  motor_driver->jointControl(joint_value);
#endif
}

/*******************************************************************************
* Set Gripper status
*******************************************************************************/
void setGripperDataToDynamixel()
{
#ifdef DYNAMIXEL
  int32_t gripper_value = 0;

  gripper_value = motor_driver->convertRadian2Value(goal_pos[END]);
  motor_driver->gripControl(gripper_value);
#endif
}

/*******************************************************************************
* Manipulator link initialization
*******************************************************************************/
void initLink()
{
  link[BASE].name_                      = "Base";
  link[BASE].mother_                    = -1;
  link[BASE].sibling_                   = -1;
  link[BASE].child_                     = 1;
  link[BASE].mass_                      = 1.0;
  link[BASE].p_                         = Eigen::Vector3f::Zero();
  link[BASE].R_                         = Eigen::Matrix3f::Identity(3,3);
  link[BASE].q_                         = 0.0;
  link[BASE].dq_                        = 0.0;
  link[BASE].ddq_                       = 0.0;
  link[BASE].a_                         = Eigen::Vector3f::Zero();
  link[BASE].b_                         = Eigen::Vector3f::Zero();
  link[BASE].v_                         = Eigen::Vector3f::Zero();
  link[BASE].w_                         = Eigen::Vector3f::Zero();

  link[JOINT1].name_                    = "Joint1";
  link[JOINT1].mother_                  = 0;
  link[JOINT1].sibling_                 = -1;
  link[JOINT1].child_                   = 2;
  link[JOINT1].mass_                    = 1.0;
  link[JOINT1].p_                       = Eigen::Vector3f::Zero();
  link[JOINT1].R_                       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT1].q_                       = 0.0;
  link[JOINT1].dq_                      = 0.0;
  link[JOINT1].ddq_                     = 0.0;
  link[JOINT1].a_                       << 0, 0, 1;
  link[JOINT1].b_                       << 0.0, 0.0, 0.0661;
  link[JOINT1].v_                       = Eigen::Vector3f::Zero();
  link[JOINT1].w_                       = Eigen::Vector3f::Zero();

  link[JOINT2].name_                    = "Joint2";
  link[JOINT2].mother_                  = 1;
  link[JOINT2].sibling_                 = -1;
  link[JOINT2].child_                   = 3;
  link[JOINT2].mass_                    = 1.0;
  link[JOINT2].p_                       = Eigen::Vector3f::Zero();
  link[JOINT2].R_                       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT2].q_                       = 0.0;
  link[JOINT2].dq_                      = 0.0;
  link[JOINT2].ddq_                     = 0.0;
  link[JOINT2].a_                       << 0, 0, 1;
  link[JOINT2].b_                       << 0.030, 0.0, 0.0;
  link[JOINT2].v_                       = Eigen::Vector3f::Zero();
  link[JOINT2].w_                       = Eigen::Vector3f::Zero();

  link[JOINT3].name_                    = "Joint3";
  link[JOINT3].mother_                  = 2;
  link[JOINT3].sibling_                 = -1;
  link[JOINT3].child_                   = 4;
  link[JOINT3].mass_                    = 1.0;
  link[JOINT3].p_                       = Eigen::Vector3f::Zero();
  link[JOINT3].R_                       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT3].q_                       = 0.0;
  link[JOINT3].dq_                      = 0.0;
  link[JOINT3].ddq_                     = 0.0;
  link[JOINT3].a_                       << 0, 0, 1;
  link[JOINT3].b_                       << 0.09025, 0.0, 0.0;
  link[JOINT3].v_                       = Eigen::Vector3f::Zero();
  link[JOINT3].w_                       = Eigen::Vector3f::Zero();

  link[END].name_                       = "Gripper";
  link[END].mother_                     = 3;
  link[END].sibling_                    = -1;
  link[END].child_                      = -1;
  link[END].mass_                       = 1.0;
  link[END].p_                          = Eigen::Vector3f::Zero();
  link[END].R_                          = Eigen::Matrix3f::Identity(3,3);
  link[END].q_                          = 0.0;
  link[END].dq_                         = 0.0;
  link[END].ddq_                        = 0.0;
  link[END].a_                          << 1, 0, 0;
  link[END].b_                          << 0.114, 0.0, 0.0;
  link[END].v_                          = Eigen::Vector3f::Zero();
  link[END].w_                          = Eigen::Vector3f::Zero();
}

/*******************************************************************************
* Initialization Trajectory
*******************************************************************************/
void initMinimumJerk()
{
  minimum_jerk = new open_manipulator::MinimumJerk();
  circle_tra   = new open_manipulator::MinimumJerk();
}

/*******************************************************************************
* Manipulator link initialization
*******************************************************************************/
void initMotor()
{
  motor[BASE].name                      = link[BASE].name_;
  motor[BASE].id                        = 0;
  motor[BASE].present_position          = 0.0;

  motor[JOINT1].name                    = link[JOINT1].name_;
  motor[JOINT1].id                      = 1;
  motor[JOINT1].present_position        = 0.0;

  motor[JOINT2].name                    = link[JOINT2].name_;
  motor[JOINT2].id                      = 2;
  motor[JOINT2].present_position        = 0.0;

  motor[JOINT3].name                    = link[JOINT3].name_;
  motor[JOINT3].id                      = 3;
  motor[JOINT3].present_position        = 0.0;

  motor[END].name                       = link[END].name_;
  motor[END].id                         = 4;
  motor[END].present_position           = 0.0;
}

/*******************************************************************************
* Initialization Kinematics Library
*******************************************************************************/
void initKinematics()
{
  kinematics = new open_manipulator::Kinematics();
}

/*******************************************************************************
* Initialization Motor Driver Library
*******************************************************************************/
void initMotorDriver(bool torque)
{
#ifdef DYNAMIXEL
  motor_driver = new open_manipulator::MotorDriver(PROTOCOL_VERSION, BAUE_RATE);

  if (motor_driver->init(motor, JOINT_NUM+GRIP_NUM))
    setMotorTorque(torque);
  else
    return;
#endif
}

/*******************************************************************************
* Torque enable or disable
*******************************************************************************/
void setMotorTorque(bool onoff)
{
#ifdef DYNAMIXEL
  motor_driver->setTorque(onoff);
#endif
}

/*******************************************************************************
* send an initial string
*******************************************************************************/
void establishContactToProcessing()
{
#ifdef SIMULATION
  if (Serial.available())
  {
    Serial.print(0.0);
    Serial.print(",");
    Serial.print(0.0);
    Serial.print(",");
    Serial.print(0.0);
    Serial.print(",");
    Serial.print(0.0);
    delay(300);
  }
#endif
}

/*******************************************************************************
* Split String
*******************************************************************************/
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
