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

#ifndef OPEN_MANIPULATOR_PICK_AND_PLACE_CONTROLLER_H
#define OPEN_MANIPULATOR_PICK_AND_PLACE_CONTROLLER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread.hpp>

namespace open_manipulator
{
#define JOINT_NUM 4
#define GRIP_NUM  1

#define LEFT_PALM   0
#define RIGHT_PALM  1

#define ITERATION_FREQUENCY 25 //Hz

class PickAndPlaceController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;

  // ROS Parameters
  bool using_gazebo_;
  std::string robot_name_;

  // ROS Publisher
  ros::Publisher gazebo_goal_joint_position_pub_[JOINT_NUM];
  ros::Publisher gazebo_gripper_position_pub_[2];

  // ROS Subscribers
  ros::Subscriber gazebo_present_joint_position_sub_;
  // ROS Service Server

  // ROS Service Client

  // Joint states
  std::map<std::string, uint8_t> joint_id_;
  Eigen::VectorXd present_joint_position_;

  // thread
  boost::thread* trajectory_generate_thread_;

 public:
  PickAndPlaceController();
  virtual ~PickAndPlaceController();

  void process(void);

  // Process state variables
  bool is_moving_;

 private:
  void initPublisher(bool using_gazebo);
  void initSubscriber(bool using_gazebo);

  void gazeboPresentJointPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg);
};
}

#endif /*OPEN_MANIPULATOR_PICK_AND_PLACE_CONTROLLER_H*/
