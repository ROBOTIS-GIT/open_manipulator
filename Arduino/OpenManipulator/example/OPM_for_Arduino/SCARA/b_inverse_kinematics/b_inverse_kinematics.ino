#include <OpenManipulator.h>
#include "OpenManipulator_SCARA.h"

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

  initSCARA();
  OPMInit(SCARA, LINK_NUM, PROCESSING, DYNAMIXEL, TORQUE);  

  goal_pose.position(0) = 0.207;
  goal_pose.position(1) = -0.071;
  goal_pose.position(2) = 0.066;
  goal_pose.orientation = Eigen::Matrix3f::Identity(3,3);

  error = kinematics.position_only_inverse(SCARA, GRIP, goal_pose);

  for (int i = BASE; i <= GRIP; i++)
  {
    Serial.print(SCARA[i].name_);
    Serial.print(" : ");
    Serial.println(SCARA[i].joint_angle_ * RAD2DEG);
  }

  Serial.print("Error : ");
  Serial.println(error, 15);
}

void loop() 
{

}
