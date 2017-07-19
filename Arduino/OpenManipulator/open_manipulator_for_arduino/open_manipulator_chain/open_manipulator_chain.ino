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

#include "open_manipulator_chain_config.h"

// #define DEBUG
// #define DYNAMIXEL
#define SIMULATION

/*******************************************************************************
* Setup
*******************************************************************************/
void setup()
{
  Serial.begin(SERIAL_RATE);
  rc100.begin(1);
#ifdef DEBUG
   while(!Serial);
#endif

  initLinkAndMotor();
  initJointProp();
  initKinematics();

#ifdef DYNAMIXEL
  initMotorDriver(false);
#endif
  initTimer();

#ifdef SIMULATION
  establishContactToProcessing();
#endif

  setFK(link, BASE);

#ifdef DEBUG
  Serial.println("OpenManipulator Chain Initialization Success!!");
#endif
}

/*******************************************************************************
* Loop
*******************************************************************************/
void loop()
{
  getData(REMOTE_RATE);
  setMotion(motion);
  showLedStatus();
}

/*******************************************************************************
* Timer (8mm)
*******************************************************************************/
void handler_control()
{
  uint16_t step_time = mov_time/control_period + 1;
  static uint16_t step_cnt = 0;

  showJointProp(joint_pos, joint_vel, joint_acc, BASE, END);

  if (moving && comm)
  {
    if (step_cnt >= step_time)
    {
      setFK(link, BASE);

      moving = false;
      step_cnt = 0;

      delete minimum_jerk;

#ifdef DEBUG
      Serial.println("End Trajectory");
#endif
    }
    else
    {
      minimum_jerk->getPosition(joint_pos, END, control_period, step_cnt);
      minimum_jerk->getVelocity(joint_vel, END, control_period, step_cnt);
      minimum_jerk->getAcceleration(joint_acc, END, control_period, step_cnt);

      for (int num = BASE; num <= END; num++)
        link[num].q_ = joint_pos[num];

      step_cnt++;

      sendJointDataToProcessing();

#ifdef DYNAMIXEL
      setJointDataToDynamixel();
      setGripperDataToDynamixel();
#endif
    }
  }
}

/*******************************************************************************
* Get Data 
*******************************************************************************/
void getData(uint32_t wait_time)
{
  static uint8_t state = 0;
  static uint32_t t_time = 0;

  bool rc100_flag      = false;
  bool processing_flag = false;

  uint8_t get_rc100_data     = 0;
  String get_processing_data = "";

  if (rc100.available())
  {
    get_rc100_data = rc100.readData();
    rc100_flag = true;
  }

  if (Serial.available())
  {
    get_processing_data = Serial.readStringUntil('\n');
    processing_flag = true;
  }

  switch (state)
  {
    case 0:
      if (rc100_flag)
      {
        dataFromRC100(get_rc100_data);   
        t_time = millis();
        state  = 1;
      }
      else if (processing_flag)
      {
        dataFromProcessing(get_processing_data);   
        t_time = millis();
        state  = 1;
      }
     break;
    
    case 1:
      if ((millis() - t_time) >= wait_time)
      {
        state = 0;
      }
     break;
    
    default :
     state = 0;
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

  if (cmd[0] == "ready")
  {
#ifdef DYNAMIXEL
    setMotorTorque(true);
    getDynamixelPosition();
    sendJointDataToProcessing();
#endif
    setTimer(true);
    comm = true;
  }
  else if (cmd[0] == "end")
  {
#ifdef DYNAMIXEL
    setMotorTorque(false);
#endif
    comm = false;
  }
  else if (cmd[0] == "joint")
  {
    for (int id = JOINT1; id <= JOINT4; id++)
      set_joint_pos[id] = cmd[id].toFloat();

    jointMove(set_joint_pos, JOINT_TRA_TIME);
  }
  else if (cmd[0] == "gripper")
  {
    gripMove(cmd[1].toFloat(), GRIP_TRA_TIME);
  }
  else if (cmd[0] == "on")
  {
    gripMove(grip_on, GRIP_TRA_TIME);
  }
  else if (cmd[0] == "off")
  {
    gripMove(grip_off, GRIP_TRA_TIME);
  }
  else if (cmd[0] == "task")
  {
    setPoseDirection(cmd[1], TASK_TRA_UNIT);

    for (int id = JOINT1; id <= JOINT4; id++)
      set_joint_pos[id] = link[id].q_;

    jointMove(set_joint_pos, TASK_TRA_TIME);
  }
#ifdef DYNAMIXEL
  else if (cmd[0] == "torque")
  {
    if (cmd[1] == "on")
      setMotorTorque(true);
    else if (cmd[1] == "off")
      setMotorTorque(false);
  }
  else if (cmd[0] == "get")
  {
    if (cmd[1] == "on")
    {
      angle_storage[motion_num][0] = -1;
      motion_num++;
    }
    else if (cmd[1] == "off")
    {
      angle_storage[motion_num][0] = -2;
      motion_num++;
    }
    else if (cmd[1].toInt() < STORAGE)
    {
      getDynamixelPosition();
      sendJointDataToProcessing();

      for (int i = 0; i < LINK_NUM; i++)
      {
        angle_storage[motion_num][i] = motor[i].present_position;
      }
      motion_num++;
    }
    else
    {
#ifdef DEBUG
      Serial.println("Overflow");
#endif
    }
  }
  else if (cmd[0] == "once")
  {
    setMotorTorque(true);

    getDynamixelPosition();
    sendJointDataToProcessing();

    motion = true;
  }
  else if (cmd[0] == "repeat")
  {
    setMotorTorque(true);

    getDynamixelPosition();
    sendJointDataToProcessing();

    motion = true;
    repeat = true;
  }
  else if (cmd[0] == "stop")
  {
    for (int i=0; i<STORAGE; i++)
      angle_storage[i][0] = 0;

    motion     = false;
    repeat     = false;
    motion_num = 0;
  }
#endif
  else
  {
#ifdef DEBUG
    Serial.println("Error");
#endif
  }
}

/*******************************************************************************
* Data from RC100 Controller
*******************************************************************************/
void dataFromRC100(uint8_t receive_data)
{
  if (receive_data & RC100_BTN_U)
  {
    setPoseDirection("forward", TASK_TRA_UNIT);

    for (int id = JOINT1; id <= JOINT4; id++)
      joint_pos[id] = link[id].q_;

    jointMove(joint_pos, TASK_TRA_TIME);
  }
  else if (receive_data & RC100_BTN_D)
  {
    setPoseDirection("back", TASK_TRA_UNIT);

    for (int id = JOINT1; id <= JOINT4; id++)
      joint_pos[id] = link[id].q_;

    jointMove(joint_pos, TASK_TRA_TIME);
  }
  else if (receive_data & RC100_BTN_L)
  {
    setPoseDirection("left", TASK_TRA_UNIT);

    for (int id = JOINT1; id <= JOINT4; id++)
      joint_pos[id] = link[id].q_;

    jointMove(joint_pos, TASK_TRA_TIME);
  }
  else if (receive_data & RC100_BTN_R)
  {
    setPoseDirection("right", TASK_TRA_UNIT);

    for (int id = JOINT1; id <= JOINT4; id++)
      joint_pos[id] = link[id].q_;

    jointMove(joint_pos, TASK_TRA_TIME);
  }
  else if (receive_data & RC100_BTN_1)
  {
    setPoseDirection("up", TASK_TRA_UNIT);

    for (int id = JOINT1; id <= JOINT4; id++)
      joint_pos[id] = link[id].q_;

    jointMove(joint_pos, TASK_TRA_TIME);
  }
  else if (receive_data & RC100_BTN_2)
  {
    gripMove(grip_on, 1.5);
  }
  else if (receive_data & RC100_BTN_3)
  {
    setPoseDirection("down", TASK_TRA_UNIT);

    for (int id = JOINT1; id <= JOINT4; id++)
      joint_pos[id] = link[id].q_;

    jointMove(joint_pos, TASK_TRA_TIME);
  }
  else if (receive_data & RC100_BTN_4)
  {
    gripMove(grip_off, 1.5);
  }
  else if (receive_data & RC100_BTN_5)
  {
    joint_pos[1] = 0.0;
    joint_pos[2] = 60.0  * PI/180.0;
    joint_pos[3] = -20.0 * PI/180.0;
    joint_pos[4] = -40.0 * PI/180.0;

    jointMove(joint_pos, 3.0);
  }
  else if (receive_data & RC100_BTN_6)
  {
    joint_pos[1] = 0.0;
    joint_pos[2] = 0.0;
    joint_pos[3] = 0.0;
    joint_pos[4] = 0.0;

    jointMove(joint_pos, 3.0);
  }
  return;
}

/*******************************************************************************
* Set motion
*******************************************************************************/
void setMotion(bool onoff)
{
  static uint8_t motion_cnt = 0;

  if (onoff)
  {
    if (moving)
      return;

    if (motion_cnt >= motion_num)
    {
      if (repeat)
      {
        motion_cnt = 0;
      }
      else
      {
        for (int i = 0; i < STORAGE; i++)
        {
          angle_storage[i][0] = 0;
        }
        motion     = false;
        motion_cnt = 0;
        motion_num = 0;        
#ifdef DEBUG
        Serial.println("End" + String(","));
#endif
        return;
      }
    }

    if (angle_storage[motion_cnt][0] == -1)
    {
      gripMove(grip_on, 1.5);
      motion_cnt++;
    }
    else if (angle_storage[motion_cnt][0] == -2)
    {
      gripMove(grip_off, 1.5);
      motion_cnt++;
    }
    else
    {
      for (int id = JOINT1; id <= JOINT4; id++)
        joint_pos[id] = angle_storage[motion_cnt][id];

      jointMove(joint_pos, 3.0);
      motion_cnt++;
    }
  }
  else
  {
    motion_cnt = 0;
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
  Serial.print(link[JOINT1].q_);
  Serial.print(",");
  Serial.print(link[JOINT2].q_);
  Serial.print(",");
  Serial.print(link[JOINT3].q_);
  Serial.print(",");
  Serial.print(link[JOINT4].q_);
  Serial.print(",");
  Serial.println(link[END].q_);
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
void setJointProp(float* set_joint_pos)
{
  for (int num = JOINT1; num <= JOINT4; num++)
  {
    start_prop[num].pos   = joint_pos[num];
    start_prop[num].vel   = joint_vel[num];
    start_prop[num].acc   = joint_acc[num];

    end_prop[num].pos     = set_joint_pos[num];
    end_prop[num].vel     = 0.0;
    end_prop[num].acc     = 0.0;
  }

  start_prop[END].pos    = joint_pos[END];
  start_prop[END].vel    = 0.0;
  start_prop[END].acc    = 0.0;

  end_prop[END].pos      = joint_pos[END];
  end_prop[END].vel      = 0.0;
  end_prop[END].acc      = 0.0;
}

/*******************************************************************************
* Set Gripper Properties
*******************************************************************************/
void setGripperProp(float get_grip_pos)
{
  for (int num = JOINT1; num <= JOINT4; num++)
  {
    start_prop[num].pos   = joint_pos[num];
    start_prop[num].vel   = 0.0;
    start_prop[num].acc   = 0.0;

    end_prop[num].pos     = joint_pos[num];
    end_prop[num].vel     = 0.0;
    end_prop[num].acc     = 0.0;
  }

  start_prop[END].pos    = joint_pos[END];
  start_prop[END].vel    = joint_vel[END];
  start_prop[END].acc    = joint_acc[END];

  end_prop[END].pos      = get_grip_pos;
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
  control_timer.attachInterrupt(handler_control);
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
  motor_driver->readPosition(motor);
  getMotorAngle(motor_angle);
}

/*******************************************************************************
* Set Pose Direction and step
*******************************************************************************/
void setPoseDirection(String dir, float step)
{
  open_manipulator::Pose target_pose;

  target_pose.position    = link[END].p_;
  target_pose.orientation = link[END].R_;

#ifdef DEBUG
  Serial.println(dir);
#endif

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
  else
  {
    return;
  }

  setIK("position", link, END, target_pose);
}

/*******************************************************************************
* Joint move
*******************************************************************************/
void jointMove(float* set_joint_pos, float set_mov_time)
{
  setJointProp(set_joint_pos);
  setMoveTime(set_mov_time);

  minimum_jerk = new open_manipulator::MinimumJerk(start_prop, end_prop, LINK_NUM, mov_time);

  moving = true;
}

/*******************************************************************************
* Grip move
*******************************************************************************/
void gripMove(float set_grip_pos, float set_mov_time)
{
  setGripperProp(set_grip_pos);
  setMoveTime(set_mov_time);

  minimum_jerk = new open_manipulator::MinimumJerk(start_prop, end_prop, LINK_NUM, mov_time);

  moving = true;
}

/*******************************************************************************
* Get Motor Position (rad)
*******************************************************************************/
void getMotorAngle(float* angle)
{
  for (int num = BASE; num <= END; num++)
  {
    angle[num]   = motor[num].present_position;
    link[num].q_ = angle[num];
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
}

/*******************************************************************************
* Write Dynamixel Position (rad)
*******************************************************************************/
void setJointDataToDynamixel()
{
  int32_t joint_value[LINK_NUM] = {0, };

  for (int num = BASE; num <= END; num++)
  {
    joint_value[num] = motor_driver->convertRadian2Value(link[num].q_);
  }
  motor_driver->jointControl(joint_value);
}

/*******************************************************************************
* Set Gripper status
*******************************************************************************/
void setGripperDataToDynamixel()
{
  int32_t gripper_value = 0;

  gripper_value = motor_driver->convertRadian2Value(link[END].q_);
  motor_driver->gripControl(gripper_value);
}

/*******************************************************************************
* Manipulator link initialization
*******************************************************************************/
void initLinkAndMotor()
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

  motor[BASE].name                      = link[BASE].name_;
  motor[BASE].id                        = 0;
  motor[BASE].goal_position             = 0.0;
  motor[BASE].present_position          = 0.0;

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
  link[JOINT1].b_                       << 0.012, 0, 0.036;
  link[JOINT1].v_                       = Eigen::Vector3f::Zero();
  link[JOINT1].w_                       = Eigen::Vector3f::Zero();

  motor[JOINT1].name                    = link[JOINT1].name_;
  motor[JOINT1].id                      = 1;
  motor[JOINT1].goal_position           = 0.0;
  motor[JOINT1].present_position        = 0.0;

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
  link[JOINT2].a_                       << 0, -1, 0;
  link[JOINT2].b_                       << 0, 0, 0.040;
  link[JOINT2].v_                       = Eigen::Vector3f::Zero();
  link[JOINT2].w_                       = Eigen::Vector3f::Zero();

  motor[JOINT2].name                    = link[JOINT2].name_;
  motor[JOINT2].id                      = 2;
  motor[JOINT2].goal_position           = 0.0;
  motor[JOINT2].present_position        = 0.0;


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
  link[JOINT3].a_                       << 0, -1, 0;
  link[JOINT3].b_                       << 0.022, 0, 0.122;
  link[JOINT3].v_                       = Eigen::Vector3f::Zero();
  link[JOINT3].w_                       = Eigen::Vector3f::Zero();

  motor[JOINT3].name                    = link[JOINT3].name_;
  motor[JOINT3].id                      = 3;
  motor[JOINT3].goal_position           = 0.0;
  motor[JOINT3].present_position        = 0.0;

  link[JOINT4].name_                    = "Joint4";
  link[JOINT4].mother_                  = 3;
  link[JOINT4].sibling_                 = -1;
  link[JOINT4].child_                   = 5;
  link[JOINT4].mass_                    = 1.0;
  link[JOINT4].p_                       = Eigen::Vector3f::Zero();
  link[JOINT4].R_                       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT4].q_                       = 0.0;
  link[JOINT4].dq_                      = 0.0;
  link[JOINT4].ddq_                     = 0.0;
  link[JOINT4].a_                       << 0, -1, 0;
  link[JOINT4].b_                       << 0.124, 0, 0;
  link[JOINT4].v_                       = Eigen::Vector3f::Zero();
  link[JOINT4].w_                       = Eigen::Vector3f::Zero();

  motor[JOINT4].name                    = link[JOINT4].name_;
  motor[JOINT4].id                      = 4;
  motor[JOINT4].goal_position           = 0.0;
  motor[JOINT4].present_position        = 0.0;

  link[END].name_                       = "Gripper";
  link[END].mother_                     = 4;
  link[END].sibling_                    = -1;
  link[END].child_                      = -1;
  link[END].mass_                       = 1.0;
  link[END].p_                          = Eigen::Vector3f::Zero();
  link[END].R_                          = Eigen::Matrix3f::Identity(3,3);
  link[END].q_                          = 0.0;
  link[END].dq_                         = 0.0;
  link[END].ddq_                        = 0.0;
  link[END].a_                          = Eigen::Vector3f::Zero();
  link[END].b_                          << 0.119, 0, 0;
  link[END].v_                          = Eigen::Vector3f::Zero();
  link[END].w_                          = Eigen::Vector3f::Zero();

  motor[END].name                       = link[END].name_;
  motor[END].id                         = 5;
  motor[END].goal_position              = 0.0;
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
  motor_driver = new open_manipulator::MotorDriver(PROTOCOL_VERSION, BAUE_RATE);

  if (motor_driver->init(motor, JOINT_NUM+GRIP_NUM))
    setMotorTorque(torque);
  else
    return;
}

/*******************************************************************************
* Torque enable or disable
*******************************************************************************/
void setMotorTorque(bool onoff)
{
  motor_driver->setTorque(onoff);
}

/*******************************************************************************
* send an initial string
*******************************************************************************/
void establishContactToProcessing()
{
  if (Serial.available())
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
  }
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
