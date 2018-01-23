#include <OpenManipulator.h>
#include <RC100.h>

#include "OpenManipulator_Chain.h"

#define CHECK_FLAG   0
#define WAIT_FOR_SEC 1

#define PROCESSING true
#define DYNAMIXEL  true
#define TORQUE     false

#define MOTION_NUM 12

RC100 rc100;
String cmd[5];

bool motion    = false;
bool repeat    = false;

const float grip_on  = 1.3;
const float grip_off = 0.0;

uint8_t motion_cnt = 0;
uint8_t motion_num = 0;
float motion_storage[MOTION_NUM][6] = {0.0, };
const float motion_set[MOTION_NUM][6] = { // time, grip, joint1, joint2, joint3, joint4
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

void setup() 
{
  Serial.begin(57600);

  initChain();
  OPMInit(chain, LINK_NUM, PROCESSING, DYNAMIXEL, TORQUE); 
  
  initRC100();
}

void loop() 
{
  OPMRun();

  setMotion();

  getData(100); //millis

  showLedStatus();
}

void initRC100()
{
  rc100.begin(1);
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
        motion_cnt = 0;
        motion     = false;     
      }
    }

    if (motion_storage[motion_cnt][1] == -1.0)
    {
      setGripAngle(grip_on);
      move(1.5);

      motion_cnt++;
    }
    else if (motion_storage[motion_cnt][1] == 1.0)
    {
      setGripAngle(grip_off);
      move(1.5);

      motion_cnt++;
    }
    else
    {
      float target_pos[LINK_NUM] = {0.0, };
      
      for (int i = JOINT1; i < GRIP; i++)
        target_pos[i] = motion_storage[motion_cnt][i+1];
  
      setJointAngle(target_pos);
      move(motion_storage[motion_cnt][0]);

      motion_cnt++;
    }
  }
  else
  {
    motion_cnt = 0;
  }
}

void getData(uint32_t wait_time)
{
  static uint8_t state = 0;
  static uint32_t tick = 0;

  bool rc100_flag      = false;
  bool processing_flag = false;

  uint16_t get_rc100_data     = 0;
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
    case CHECK_FLAG:
      if (rc100_flag)
      {
        dataFromRC100(get_rc100_data);   
        tick = millis();
        state  = WAIT_FOR_SEC;
      }
      else if (processing_flag)
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

void dataFromRC100(uint16_t receive_data)
{
  float target_pos[LINK_NUM] = {0.0, };

  if (receive_data & RC100_BTN_U)
  {    
    inverseKinematics(chain, GRIP, setPose("forward"), "position");  
    
    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = chain[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_D)
  {
    inverseKinematics(chain, GRIP, setPose("back"), "position");  

    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = chain[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_L)
  {
    inverseKinematics(chain, GRIP, setPose("left"), "position");  

    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = chain[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_R)
  {
    inverseKinematics(chain, GRIP, setPose("right"), "position");  

    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = chain[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_1)
  {
    inverseKinematics(chain, GRIP, setPose("up"), "position");  

    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = chain[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_2)
  {
    setGripAngle(grip_on);
    move(1.5);
  }
  else if (receive_data & RC100_BTN_3)
  {
    inverseKinematics(chain, GRIP, setPose("down"), "position");  

    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = chain[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_4)
  {
    setGripAngle(grip_off);
    move(1.5);
  }
  else if (receive_data & RC100_BTN_5)
  {
    target_pos[1] = 0.0;
    target_pos[2] = 60.0  * DEG2RAD;
    target_pos[3] = -20.0 * DEG2RAD;
    target_pos[4] = -40.0 * DEG2RAD;

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
      if (DYNAMIXEL)
      {
        setTorque(true);
        sendAngle2Processing(getAngle()); 
      }

      if (PROCESSING)
        sendAngle2Processing(getState()); 
    }
    else if (cmd[1] == "end")
    {
      if (DYNAMIXEL)
        setTorque(false);
    }
  }
  else if (cmd[0] == "joint")
  {
    float target_pos[LINK_NUM] = {0.0, };

    for (int i = JOINT1; i < GRIP; i++)
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
      setGripAngle(grip_on);
    else if (cmd[1] == "off")
      setGripAngle(grip_off);

    move(1.5);
  }
  else if (cmd[0] == "task")
  {
    float target_pos[LINK_NUM] = {0.0, };

    inverseKinematics(chain, GRIP, setPose(cmd[1]), "position");   
    
    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = chain[i].joint_angle_;

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (cmd[0] == "torque")
  {
    if (DYNAMIXEL)
    {
      if (cmd[1] == "on")
        setTorque(true);
      else if (cmd[1] == "off")
        setTorque(false);
    }
  }
  else if (cmd[0] == "get")
  {
    if (cmd[1] == "on")
    {
      motion_storage[motion_num][0] = 3.0;  //mov_time
      motion_storage[motion_num][1] = -1.0;
      motion_num++;
    }
    else if (cmd[1] == "off")
    {
      motion_storage[motion_num][0] = 3.0;  //mov_time
      motion_storage[motion_num][1] = 1.0;
      motion_num++;
    }
    else if (cmd[1] == "clear")
    {
      for (int i = 0; i < MOTION_NUM; i++)
      {
        for (int j = 0; j < 6; j++)
        {
          motion_storage[i][j] = 0.0;
        }
      }
      
      motion_num = 0;
      motion_cnt = 0;
      motion     = false;
      repeat     = false;
    }
    else if (cmd[1] == "pose")
    {
      if (cmd[2].toInt() < MOTION_NUM)
      {
        State* state = getAngle();

        if (DYNAMIXEL)
          sendAngle2Processing(state); 

        for (int i = JOINT1; i < GRIP; i++)
        {
          motion_storage[motion_num][0]   = 3.0;  //mov_time
          motion_storage[motion_num][i+1] = state[i].pos;
        }
        motion_num++;
      }
    }
  }
  else if (cmd[0] == "hand")
  {
    if (cmd[1] == "once")
    {
      if (DYNAMIXEL)
      {
        setTorque(true);
        sendAngle2Processing(getAngle()); 
      }

      motion_cnt = 0;
      motion = true;
    }
    else if (cmd[1] == "repeat")
    {
      if (DYNAMIXEL)
      {
        setTorque(true);
        sendAngle2Processing(getAngle()); 
      }

      motion_cnt = 0;
      motion = true;
      repeat = true;
    }
    else if (cmd[1] == "stop")
    {
      for (int i = 0; i < MOTION_NUM; i++)
      {
        for (int j = 0; j < 6; j++)
        {
          motion_storage[i][j] = 0;
        }
      }

      motion_cnt = 0;
      motion     = false;
      repeat     = false;
    }
  }
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "start")
    {
      if (DYNAMIXEL)
        sendAngle2Processing(getAngle()); 

      if (PROCESSING)
        sendAngle2Processing(getState()); 

      for (int i = 0; i < MOTION_NUM; i++)
      {
        for (int j = 0; j < 6; j++)
        {
          motion_storage[i][j] = motion_set[i][j];
        }
      }

      motion_num = MOTION_NUM;  
      motion_cnt = 0;          
      motion = true;
      repeat = true;
    }
    else if (cmd[1] == "stop")
    {
      motion_cnt = 0;
      motion     = false;
      repeat     = false;
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