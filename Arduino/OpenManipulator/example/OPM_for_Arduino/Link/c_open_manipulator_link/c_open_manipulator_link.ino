#include <OpenManipulator.h>
#include <RC100.h>

#include "OpenManipulator_Link.h"

#define CHECK_FLAG   0
#define WAIT_FOR_SEC 1

#define PROCESSING false
#define DYNAMIXEL  true
#define TORQUE     true

#define MOV_TIME 0.3
#define STEP     3.0

#define RELAY_PIN 8

bool relay = false;
RC100 rc100;

void setup() 
{
  Serial.begin(57600);

  pinMode(RELAY_PIN, OUTPUT);

  initLink();
  OPMInit(links, LINK_NUM, PROCESSING, DYNAMIXEL, TORQUE); 
  
  initRC100();
}

void loop() 
{
  OPMRun();

  getData(100); //millis

  showLedStatus();
}

void initRC100()
{
  rc100.begin(1);
}

void getData(uint32_t wait_time)
{
  static uint8_t state = 0;
  static uint32_t tick = 0;

  bool rc100_flag      = false;

  uint16_t get_rc100_data     = 0;

  if (rc100.available())
  {
    get_rc100_data = rc100.readData();
    rc100_flag = true;
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
  State* present_state;

  if (receive_data & RC100_BTN_U)
  { 
    present_state = getState();

    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = present_state[i].pos;

    target_pos[JOINT2] -= (STEP * DEG2RAD);

    setJointAngle(target_pos);
    move(MOV_TIME);
  }
  else if (receive_data & RC100_BTN_D)
  {
    present_state = getState();
    
    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = present_state[i].pos;

    target_pos[JOINT2] += (STEP * DEG2RAD);

    setJointAngle(target_pos);
    move(MOV_TIME);
  }
  else if (receive_data & RC100_BTN_L)
  {
    present_state = getState();
    
    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = present_state[i].pos;

    target_pos[JOINT1] += (STEP * DEG2RAD);

    setJointAngle(target_pos);
    move(MOV_TIME);
  }
  else if (receive_data & RC100_BTN_R)
  {
    present_state = getState();
    
    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = present_state[i].pos;

    target_pos[JOINT1] -= (STEP * DEG2RAD);

    setJointAngle(target_pos);
    move(0.16);
  }
  else if (receive_data & RC100_BTN_1)
  {
    present_state = getState();
    
    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = present_state[i].pos;

    target_pos[JOINT3] -= (STEP * DEG2RAD);

    setJointAngle(target_pos);
    move(MOV_TIME);
  }
  else if (receive_data & RC100_BTN_2)
  {    
    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = 0.0;

    setJointAngle(target_pos);
    move(2.0);
  }
  else if (receive_data & RC100_BTN_3)
  {
    present_state = getState();
    
    for (int i = JOINT1; i < GRIP; i++)
      target_pos[i] = present_state[i].pos;

    target_pos[JOINT3] += (STEP * DEG2RAD);

    setJointAngle(target_pos);
    move(MOV_TIME);
  }
  else if (receive_data & RC100_BTN_4)
  {
    if (relay)
      digitalWrite(RELAY_PIN, HIGH);
    else
      digitalWrite(RELAY_PIN, LOW);

    relay = !relay;
  }
  else if (receive_data & RC100_BTN_5)
  {
    setTorque(true);
  }
  else if (receive_data & RC100_BTN_6)
  {
    setTorque(false);
  }
}
