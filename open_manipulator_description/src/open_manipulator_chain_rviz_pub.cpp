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

/* Authors: Taehoon Lim (Darby) */


#include <ros/ros.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <sensor_msgs/JointState.h>

ros::Publisher goal_dynamixel_states_pub;
ros::Publisher present_joint_states_pub;

void presentDynamixelStateMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  sensor_msgs::JointState present_joint_msg;

  for (int index = 0; index < msg->name.size(); index++)
  {
    if (msg->name[index] == "id_5")
    {
      present_joint_msg.name.push_back("grip_joint");
      present_joint_msg.position.push_back(msg->position[index] * 0.01);
      present_joint_msg.name.push_back("grip_joint_sub");
      present_joint_msg.position.push_back(present_joint_msg.position[index]);
    }
    else
    {
      std::stringstream joint_num;
      joint_num << "joint" << index+1;
      present_joint_msg.name.push_back(joint_num.str());
      present_joint_msg.position.push_back(msg->position[index]);
    }
  }

  present_joint_states_pub.publish(present_joint_msg);
}

void goalJointStatesMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  sensor_msgs::JointState goal_joint_msg;

  for (int index = 0; index < msg->name.size(); index++)
  {
    goal_joint_msg.name.push_back(msg->name[index]);
    goal_joint_msg.position.push_back(msg->position[index]);
  }

  goal_dynamixel_states_pub.publish(goal_joint_msg);
}

int main( int argc , char **argv )
{
  ros::init( argc , argv , "open_manipulator_description_publisher" );
  ros::NodeHandle nh("~");

  goal_dynamixel_states_pub  = nh.advertise<sensor_msgs::JointState>("/robotis/dynamixel/goal_states", 10);
  present_joint_states_pub  = nh.advertise<sensor_msgs::JointState>("/robotis/open_manipulator/present_joint_states", 10);

  ros::Subscriber present_dynamixel_states_sub = nh.subscribe("/robotis/dynamixel/present_states", 10, presentDynamixelStateMsgCallback);
  ros::Subscriber goal_joint_states_sub = nh.subscribe("/robotis/open_manipulator/goal_joint_states", 10, goalJointStatesMsgCallback);

  ros::spin();

  return 0;
}



