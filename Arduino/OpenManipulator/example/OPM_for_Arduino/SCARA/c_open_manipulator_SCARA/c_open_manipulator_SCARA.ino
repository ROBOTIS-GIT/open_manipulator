#include <OpenManipulator.h>
#include "OpenManipulator_SCARA.h"

#define CHECK_FLAG   0
#define WAIT_FOR_SEC 1

#define PROCESSING true
#define DYNAMIXEL  true
#define TORQUE     false

#define START_POSE      0
#define CLOSE_SCREEN    1
#define ATTACH_SCREEN   2
#define DETACH_SCREEN   3
#define RESET_POSE      4
#define PUSH_RESET      5
#define RELEASE_RESET   6

#define DRAW_CIRCLE     10
#define DRAW_HEART      11

String cmd[5];

bool motion  = false;
bool reverse = false;

const float grip_on  = 0.0;
const float grip_off = -0.9;

OPMDraw draw;
Object circle, heart;
String set_object = "heart";
  
uint8_t motion_state = 0;

void setup() 
{
  Serial.begin(57600);

  initSCARA();
  OPMInit(SCARA, LINK_NUM, PROCESSING, DYNAMIXEL, TORQUE); 

  draw.begin(SCARA, LINK_NUM, DYNAMIXEL);
}

void loop() 
{
  static uint32_t tmp_time = micros();

  OPMRun();

  setMotion();

  getData(100); //millis 
  
  if ((micros() - tmp_time) >= CONTROL_RATE)
  {
    tmp_time = micros();    
    draw.drawObject(set_object);
  }

  showLedStatus();
}

float* calcPosition2Angle(Pose set_pose)
{
  static float target_pos[LINK_NUM] = {0.0, };
  Pose goal_pose;

  goal_pose.position(0) = set_pose.position(0);
  goal_pose.position(1) = set_pose.position(1);
  goal_pose.position(2) = set_pose.position(2);
  goal_pose.orientation = set_pose.orientation;

  inverseKinematics(SCARA, GRIP, goal_pose, "position");   
  
  for (int i = JOINT1; i < GRIP; i++)
    target_pos[i] = SCARA[i].joint_angle_;

  return target_pos;
}

void setMotion()
{
  Pose goal_pose;
  float target_pos[LINK_NUM] = {0.0, };
  State* present_state;
  State start  = {0.0, 0.0, 0.0};
  State finish = {M_PI*2, 0.0, 0.0};
  
  if (motion)
  {
    if (getMoving())
      return;

    if (draw.getDrawing())
      return;

    switch (motion_state)
    {
      case START_POSE:    
        if (set_object == "circle")
        {
          goal_pose.position(0) = circle.x;
          goal_pose.position(1) = circle.y;
          goal_pose.position(2) = circle.z;
        }        
        else if (set_object == "heart")
        {
          goal_pose.position(0) = heart.x;
          goal_pose.position(1) = heart.y;
          goal_pose.position(2) = heart.z;
        }
        goal_pose.orientation = Eigen::Matrix3f::Identity(3,3);
    
        setJointAngle(calcPosition2Angle(goal_pose));
        move(3.0);

        motion_state = CLOSE_SCREEN;
       break;

      case CLOSE_SCREEN:
        setGripAngle(-0.35);
        move(1.6);

        motion_state = ATTACH_SCREEN;
       break; 

      case ATTACH_SCREEN:
        present_state = getState();

        for (int i = JOINT1; i < GRIP; i++)
          target_pos[i] = present_state[i].pos;

        target_pos[JOINT3] -= 0.20;

        setJointAngle(target_pos);
        setGripAngle(grip_on);
        move(2.0);

        if (set_object == "circle")
          motion_state = DRAW_CIRCLE;
        else if (set_object == "heart")
          motion_state = DRAW_HEART;

       break; 

      case DRAW_CIRCLE:
        draw.setObject(circle);
        draw.setDrawTime(15.0);
        draw.setRange(&start, &finish);
        draw.start();

        if (reverse)
          circle.radius -= 0.005;
        else
          circle.radius += 0.005;

        if (circle.radius > 0.060 || circle.radius < 0.005)
        {
          if (reverse)
          {
            circle.radius = 0.005;
          }
          else
          {
            circle.radius = 0.060;
          }

          motion_state = DETACH_SCREEN;
          reverse = !reverse;
        }
       break;

      case DRAW_HEART:
        draw.setObject(heart);
        draw.setDrawTime(25.0);
        draw.setRange(&start, &finish);
        draw.start();

        motion_state = DETACH_SCREEN;
       break;

      case DETACH_SCREEN:
        present_state = getState();
        
        for (int i = JOINT1; i < GRIP; i++)
          target_pos[i] = present_state[i].pos;  
          
        target_pos[JOINT3] += 0.40;

        setJointAngle(target_pos);
        setGripAngle(grip_off);
        move(2.0);

        if (set_object == "circle")
          set_object = "heart";
        else if (set_object == "heart")
          set_object = "circle";

        motion_state = RESET_POSE;
       break;

      case RESET_POSE:
        target_pos[JOINT1] = -1.731;
        target_pos[JOINT2] =  0.347;
        target_pos[JOINT3] =  2.247;
    
        setJointAngle(target_pos);
        move(3.0);

        motion_state = PUSH_RESET;
       break;

      case PUSH_RESET:
        setGripAngle(-0.2);
        move(1.6);
        
        motion_state = RELEASE_RESET;
      break;

      case RELEASE_RESET:
        setGripAngle(grip_off);
        move(1.6);

        motion_state = START_POSE;
      break;

      default:
       break;
    }
  }
}

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
    move(1.0);
  }
  else if (cmd[0] == "gripper")
  {
    setGripAngle(cmd[1].toFloat());
    move(1.6);
  }
  else if (cmd[0] == "grip")
  {
    if (cmd[1] == "on")
      setGripAngle(grip_on);
    else if (cmd[1] == "off")
      setGripAngle(grip_off);

    move(1.6);
  }
  else if (cmd[0] == "pos")
  {
    Pose goal_pose;
    goal_pose.position(0) = cmd[1].toFloat();
    goal_pose.position(1) = cmd[2].toFloat();
    goal_pose.position(2) = 0.0661;
    goal_pose.orientation = Eigen::Matrix3f::Identity(3,3);

    setJointAngle(calcPosition2Angle(goal_pose));
    move(1.5);
  }
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "start")
    {
      if (DYNAMIXEL)
        sendAngle2Processing(getAngle()); 
      
      circle = {0.220, 0.030, 0.0661, 0.005};
      heart  = {0.180, 0.025, 0.0661, 0.004};  //0.208

      motion_state = DETACH_SCREEN;
      motion = true;
    }
    else if (cmd[1] == "stop")
    {
      motion = false;
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