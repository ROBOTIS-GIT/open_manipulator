#include <OpenManipulator.h>
#include "OpenManipulator_Chain.h"

#define PROCESSING false
#define DYNAMIXEL  false
#define TORQUE     false

OPMKinematics kinematics;
Pose goal_pose;

float error = 0.0;

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  initChain();
  OPMInit(chain, LINK_NUM, PROCESSING, DYNAMIXEL, TORQUE);  

  goal_pose.position(0) = 0.144;
  goal_pose.position(1) = 0.000;
  goal_pose.position(2) = 0.218;
  goal_pose.orientation = Eigen::Matrix3f::Identity(3,3);

  error = kinematics.inverse(chain, GRIP, goal_pose, 0.7);

  for (int i = BASE; i <= GRIP; i++)
  {
    Serial.print(chain[i].name_);
    Serial.print(" : ");
    Serial.println(chain[i].joint_angle_ * RAD2DEG);
  }

  Serial.print("Error : ");
  Serial.println(error, 15);
}

void loop() 
{

}

