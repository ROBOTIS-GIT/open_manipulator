#include <OpenManipulator.h>
#include "OpenManipulator_SCARA.h"

#define PROCESSING false
#define DYNAMIXEL  false
#define TORQUE     false

OPMKinematics kinematics;
float goal_angle[LINK_NUM] = {0.0, -60.0*DEG2RAD, 30.0*DEG2RAD, 30.0*DEG2RAD, 0.0};

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  initSCARA();
  OPMInit(SCARA, LINK_NUM, PROCESSING, DYNAMIXEL, TORQUE);  

  for (int i = BASE; i <= GRIP; i++)
    SCARA[i].joint_angle_ = goal_angle[i];

  kinematics.forward(SCARA, BASE);
  showFKResult(SCARA, BASE, GRIP);
}

void loop() 
{

}


