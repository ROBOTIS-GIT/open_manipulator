/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_CONTROLLER_H
#define OPEN_MANIPULATOR_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <boost/thread.hpp>
#include <unistd.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"
#include "open_manipulator_msgs/GetJointPosition.h"
#include "open_manipulator_msgs/GetKinematicsPose.h"
#include "open_manipulator_msgs/OpenManipulatorState.h"

#include "open_manipulator_libs/open_manipulator.h"

namespace open_manipulator_controller
{

class OpenManipulatorController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters
  bool using_platform_;
  bool using_moveit_;
  double control_period_;

  // ROS Publisher
  ros::Publisher open_manipulator_states_pub_;
  std::vector<ros::Publisher> open_manipulator_kinematics_pose_pub_;
  ros::Publisher open_manipulator_joint_states_pub_;
  std::vector<ros::Publisher> gazebo_goal_joint_position_pub_;
  ros::Publisher moveit_update_start_state_pub_;

  // ROS Subscribers
  ros::Subscriber open_manipulator_option_sub_;
  ros::Subscriber display_planned_path_sub_;
  ros::Subscriber move_group_goal_sub_;
  ros::Subscriber execute_traj_goal_sub_;

  // ROS Service Server
  ros::ServiceServer goal_joint_space_path_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_pose_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_position_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_orientation_server_;
  ros::ServiceServer goal_task_space_path_server_;
  ros::ServiceServer goal_task_space_path_position_only_server_;
  ros::ServiceServer goal_task_space_path_orientation_only_server_;
  ros::ServiceServer goal_joint_space_path_from_present_server_;
  ros::ServiceServer goal_task_space_path_from_present_position_only_server_;
  ros::ServiceServer goal_task_space_path_from_present_orientation_only_server_;
  ros::ServiceServer goal_task_space_path_from_present_server_;
  ros::ServiceServer goal_tool_control_server_;
  ros::ServiceServer set_actuator_state_server_;
  ros::ServiceServer goal_drawing_trajectory_server_;
  ros::ServiceServer get_joint_position_server_;
  ros::ServiceServer get_kinematics_pose_server_;
  ros::ServiceServer set_joint_position_server_;
  ros::ServiceServer set_kinematics_pose_server_;

  // MoveIt! interface
  moveit::planning_interface::MoveGroupInterface* move_group_;
  trajectory_msgs::JointTrajectory joint_trajectory_;
  double moveit_sampling_time_;
  bool moveit_plan_only_;

  // Thread parameter
  pthread_t timer_thread_;
  pthread_attr_t attr_;

  // Related robotis_manipulator
  OpenManipulator open_manipulator_;

  // flag parameter
  bool tool_ctrl_state_;
  bool timer_thread_state_;
  bool moveit_plan_state_;

 public:

  OpenManipulatorController(std::string usb_port, std::string baud_rate);
  ~OpenManipulatorController();

  void publishCallback(const ros::TimerEvent&);

  void initPublisher();
  void initSubscriber();
  void initServer();

  void openManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg);
  void displayPlannedPathCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);
  void moveGroupGoalCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg);
  void executeTrajGoalCallback(const moveit_msgs::ExecuteTrajectoryActionGoal::ConstPtr &msg);

  double getControlPeriod(void){return control_period_;}

  bool goalJointSpacePathCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                  open_manipulator_msgs::SetJointPosition::Response &res);

  bool goalJointSpacePathToKinematicsPoseCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                  open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathToKinematicsPositionCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                  open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathToKinematicsOrientationCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                  open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                 open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathPositionOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                             open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathOrientationOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathFromPresentCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                             open_manipulator_msgs::SetJointPosition::Response &res);

  bool goalTaskSpacePathFromPresentCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                            open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathFromPresentPositionOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                        open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathFromPresentOrientationOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                           open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                               open_manipulator_msgs::SetJointPosition::Response &res);

  bool setActuatorStateCallback(open_manipulator_msgs::SetActuatorState::Request  &req,
                                open_manipulator_msgs::SetActuatorState::Response &res);

  bool goalDrawingTrajectoryCallback(open_manipulator_msgs::SetDrawingTrajectory::Request  &req,
                                     open_manipulator_msgs::SetDrawingTrajectory::Response &res);

  bool setJointPositionMsgCallback(open_manipulator_msgs::SetJointPosition::Request &req,
                                   open_manipulator_msgs::SetJointPosition::Response &res);

  bool setKinematicsPoseMsgCallback(open_manipulator_msgs::SetKinematicsPose::Request &req,
                                    open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool getJointPositionMsgCallback(open_manipulator_msgs::GetJointPosition::Request &req,
                                   open_manipulator_msgs::GetJointPosition::Response &res);

  bool getKinematicsPoseMsgCallback(open_manipulator_msgs::GetKinematicsPose::Request &req,
                                    open_manipulator_msgs::GetKinematicsPose::Response &res);

  void startTimerThread();
  static void *timerThread(void *param);

  void moveitTimer(double present_time);
  void process(double time);

  void publishOpenManipulatorStates();
  void publishKinematicsPose();
  void publishJointStates();
  void publishGazeboCommand();

  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::JointPosition msg);
  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::KinematicsPose msg);
};
}

#endif //OPEN_MANIPULATOR_CONTROLLER_H
