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

/* Authors: Taehun Lim (Darby) */

#ifndef OPEN_MANIPULATOR_ARM_CONTROLLER_H
#define OPEN_MANIPULATOR_ARM_CONTROLLER_H

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <vector>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include "open_manipulator_msgs/OpenManipulatorState.h"

#include "open_manipulator_msgs/GetJointPosition.h"
#include "open_manipulator_msgs/GetKinematicsPose.h"

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include <eigen3/Eigen/Eigen>

namespace open_manipulator
{

typedef struct
{
  std::vector<std::string> name;
} Joints;

typedef struct
{
  uint8_t group;
  uint16_t waypoints;                                  // planned number of via-points
  double time_from_start;
  Eigen::MatrixXd planned_path_positions;              // planned position trajectory
} PlannedPathInfo;

class MoveItBridge
{
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  // ROS Parameters
  bool use_platform_;
  bool set_home_position_;
  double publish_period_;
  std::map<std::string, Joints> planning_group_;

  // ROS Publisher
  std::vector<ros::Publisher> gazebo_goal_joint_position_pub_;

  ros::Publisher dynamixel_workbench_pub_;

  ros::Publisher joint_position_pub_;
  ros::Publisher joint_move_time_pub_;

  ros::Publisher gripper_position_pub_;
  ros::Publisher gripper_move_time_pub_;

  ros::Publisher moving_state_pub_;

  // ROS Subscribers
  ros::Subscriber display_planned_path_sub_;

  // ROS Service Server
  ros::ServiceServer get_joint_position_server_;
  ros::ServiceServer get_kinematics_pose_server_;
  ros::ServiceServer set_joint_position_server_;
  ros::ServiceServer set_kinematics_pose_server_;

  ros::ServiceClient open_manipulator_joint_control_client_;

  // ROS Service Client

  // MoveIt! interface
  std::map<std::string, moveit::planning_interface::MoveGroupInterface*> move_group_;
  trajectory_msgs::JointTrajectory joint_trajectory_;
  PlannedPathInfo planned_path_info_;

  // Process state variables
  bool     is_moving_;
  uint16_t all_time_steps_;

 public:
  MoveItBridge();
  virtual ~MoveItBridge();

  void controlCallback(const ros::TimerEvent&);
  double getPublishPeriod(void){ return publish_period_;}

  bool getPlanningGroupInfo(const std::string yaml_file);

  void initPublisher();
  void initSubscriber();

  void initServer();
  void initClient();

  void initPlanningGroup();

  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::JointPosition msg);
  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::KinematicsPose msg);

  void displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);

  bool setJointPositionMsgCallback(open_manipulator_msgs::SetJointPosition::Request &req,
                                   open_manipulator_msgs::SetJointPosition::Response &res);

  bool setKinematicsPoseMsgCallback(open_manipulator_msgs::SetKinematicsPose::Request &req,
                                    open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool getJointPositionMsgCallback(open_manipulator_msgs::GetJointPosition::Request &req,
                                   open_manipulator_msgs::GetJointPosition::Response &res);

  bool getKinematicsPoseMsgCallback(open_manipulator_msgs::GetKinematicsPose::Request &req,
                                    open_manipulator_msgs::GetKinematicsPose::Response &res);
};
}

#endif /*OPEN_MANIPULATOR_ARM_CONTROLLER_H*/
