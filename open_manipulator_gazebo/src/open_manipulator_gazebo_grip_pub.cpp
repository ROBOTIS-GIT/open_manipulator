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


#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

void jointStateMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_gazebo");
  ros::NodeHandle nh("~");

  ros::Subscriber joint_states_sub_ = nh.subscribe("/open_manipulator_chain/joint_states", 10, jointStateMsgCallback);

  ros::spin();

  return 0;
}
