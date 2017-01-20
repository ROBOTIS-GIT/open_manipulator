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
#include <sensor_msgs/JointState.h>

ros::Publisher present_joint_states_pub;
ros::Publisher goal_joint_states_pub;

void present_joint_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{
  sensor_msgs::JointState _present_msg;

  for ( int _index = 0 ; _index < msg->name.size(); _index++ )
  {
    if ( msg->name[ _index ] == "grip_joint" )
    {
      _present_msg.name.push_back( msg->name[ _index ] );
      _present_msg.position.push_back( msg->position[ _index ] * 0.01);
      _present_msg.name.push_back("grip_joint_sub");
      _present_msg.position.push_back(_present_msg.position[ _index ]);
    }
    else
    {
      _present_msg.name.push_back( msg->name[ _index ] );
      _present_msg.position.push_back( msg->position[ _index ] );
    }
  }
  present_joint_states_pub.publish( _present_msg );
}

void goal_joint_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{
  sensor_msgs::JointState _goal_msg;

  for ( int _index = 0 ; _index < msg->name.size(); _index++ )
  {
    if ( msg->name[ _index ] == "grip_joint" )
    {
      _goal_msg.name.push_back( msg->name[ _index ] );
      _goal_msg.position.push_back( msg->position[ _index ] * 0.01);
      _goal_msg.name.push_back("grip_joint_sub");
      _goal_msg.position.push_back(_goal_msg.position[ _index ]);
    }
    else
    {
      _goal_msg.name.push_back( msg->name[ _index ] );
      _goal_msg.position.push_back( msg->position[ _index ] );
    }
  }
  goal_joint_states_pub.publish( _goal_msg );
}

int main( int argc , char **argv )
{
  ros::init( argc , argv , "manipulator_x4_publisher" );
  ros::NodeHandle nh("~");

  present_joint_states_pub  = nh.advertise<sensor_msgs::JointState>("/robotis/manipulator_x4/present_joint_states", 0);
  goal_joint_states_pub  = nh.advertise<sensor_msgs::JointState>("/robotis/manipulator_x4/goal_joint_states", 0);

  ros::Subscriber present_joint_states_sub = nh.subscribe("/robotis/present_joint_states", 5, present_joint_states_callback);
  ros::Subscriber goal_joint_states_sub = nh.subscribe("/robotis/goal_joint_states", 5, goal_joint_states_callback);

  ros::spin();

  return 0;
}



