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

#ifndef OPEN_MANIPULATOR_POSITION_CONTROLLER_H
#define OPEN_MANIPULATOR_POSITION_CONTROLLER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "open_manipulator_msgs/KinematicsPose.h"

#include <map>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>
#include <fstream>

#include "robotis_math/robotis_math.h"

#include "open_manipulator_position_ctrl/motion_planning_tool.h"
#include "moveit_msgs/DisplayTrajectory.h"

namespace open_manipulator_position_ctrl
{
#define MAX_JOINT_NUM        (4)
#define MAX_GRIPPER_NUM      (1)
#define ITERATION_FREQUENCY  (25)
#define ITERATION_TIME       (0.04)

class PositionController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  bool is_debug_;

  // ROS Publisher
  ros::Publisher goal_joint_position_pub_;
  ros::Publisher goal_gripper_position_pub_;
  ros::Publisher bottle_pose_msg_pub_;
  ros::Publisher place_pose_msg_pub_;
  ros::Publisher demo_order_pub_;

  // ROS Subscribers
  ros::Subscriber demo_order_sub_;
  ros::Subscriber present_joint_position_sub_;
  ros::Subscriber present_gripper_position_sub_;
  ros::Subscriber motion_planning_target_pose_msg_sub_;
  ros::Subscriber execute_planned_path_msg_sub_;
  ros::Subscriber display_planned_path_sub_;
  ros::Subscriber bottle_pose_msg_sub_;

  // Process state variables
  bool is_moving_;
  bool enable_joint_;
  bool enable_gripper_;
  bool moveit_execution_;
  std_msgs::String order_;

  // Time variables
  double move_time_;
  int all_time_steps_;
  int step_cnt_;

  // Target Pose
  open_manipulator_msgs::KinematicsPose bottle_pose_;

  // Place Pose
  open_manipulator_msgs::KinematicsPose place_pose_;

  // Dynamixel position Vector and Matrix
  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd present_gripper_position_;
  Eigen::VectorXd goal_joint_position_;
  Eigen::VectorXd goal_gripper_position_;

  Eigen::MatrixXd goal_joint_trajectory_;
  Eigen::MatrixXd goal_gripper_trajectory_;

  // Joint states
  std::map<std::string, uint8_t> joint_id_;

  // Motion Planning Tool
  motion_planning_tool::MotionPlanningTool *motionPlanningTool_;

  // thread
  boost::thread* trajectory_generate_tread_;

  void demoMsgCallback(const std_msgs::String::ConstPtr &msg);

  void presentJointPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void presentGripperPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg);

  void motionPlanningTargetPoseMsgCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void executePlannedPathMsgCallback(const std_msgs::String::ConstPtr &msg);
  void displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);
  void bottlePoseMsgCallback(const geometry_msgs::Pose::ConstPtr &msg);

  void parsePoseData(const std::string &path);
  void calculateJointGoalTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position);
  void calculateGripperGoalTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position);
  void moveItTragectoryGenerateThread();
  
 public:
  PositionController();
  virtual ~PositionController();

  bool initPositionController(void);
  bool shutdownPositionController(void);

  void process(void);
};
}

#endif /*OPEN_MANIPULATOR_POSITION_CONTROLLER_H*/
