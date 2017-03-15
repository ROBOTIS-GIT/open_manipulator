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

#include <pthread.h>

#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include "open_manipulator_msgs/KinematicsPose.h"

ros::Publisher execute_planned_path_msg_pub;
pthread_t trajectory_generate;
open_manipulator_msgs::KinematicsPose kinematics_pose_msg;

void publishStatusMsg(unsigned int type, std::string msg);

void* plan_trajectory_proc(void* arg)
{
  static moveit::planning_interface::MoveGroup group("arm");
  moveit::planning_interface::MoveGroup::Plan my_plan;

  bool success;

  /* ----- set planning time ----- */

  group.setPlanningTime(5.0);

  /* set start state */
  group.setStartState(*group.getCurrentState());

  /* set target state */
  geometry_msgs::Pose target_pose;

  target_pose.position.x = kinematics_pose_msg.pose.position.x;
  target_pose.position.y = kinematics_pose_msg.pose.position.y;
  target_pose.position.z = kinematics_pose_msg.pose.position.z;

  target_pose.orientation.x = kinematics_pose_msg.pose.orientation.x;
  target_pose.orientation.y = kinematics_pose_msg.pose.orientation.y;
  target_pose.orientation.z = kinematics_pose_msg.pose.orientation.z;
  target_pose.orientation.w = kinematics_pose_msg.pose.orientation.w;

//  group.setPoseTarget( target_pose );
//  group.setPositionTarget(kinematics_pose_msg.pose.position.x,
//                          kinematics_pose_msg.pose.position.y,
//                          kinematics_pose_msg.pose.position.z);
  group.setApproximateJointValueTarget(target_pose);

  /* motion planning */
  success = group.plan( my_plan );

  if ( success == true )
  {
    std_msgs::String msg;
    msg.data = "execute";

    execute_planned_path_msg_pub.publish( msg );
  }
  else
  {
    std_msgs::String msg;
    msg.data = "fail";

    execute_planned_path_msg_pub.publish( msg );
  }
}

void motionPlanningTargetPoseMsgCallback( const open_manipulator_msgs::KinematicsPose::ConstPtr& msg )
{
  kinematics_pose_msg = *msg;

  pthread_create( &trajectory_generate , NULL , plan_trajectory_proc , NULL );

  return;
}

int main( int argc , char **argv )
{
  ros::init( argc , argv , "open_manipulator_motion_planning_controller" );
  ros::NodeHandle nh("~");

  execute_planned_path_msg_pub = nh.advertise<std_msgs::String>("/robotis/open_manipulator/execute_planned_path", 10);

  ros::Subscriber motion_planning_target_pose_msg_sub = nh.subscribe("/robotis/open_manipulator/motion_planning_target_pose", 10,
                                                                      motionPlanningTargetPoseMsgCallback);

  ros::spin();

  return 0;
}
