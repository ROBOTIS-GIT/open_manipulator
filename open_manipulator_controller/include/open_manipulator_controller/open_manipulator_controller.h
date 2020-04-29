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

#ifndef OPEN_MANIPULATOR_CONTROLLER_H_
#define OPEN_MANIPULATOR_CONTROLLER_H_

#include <boost/thread.hpp>
#include <unistd.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "open_manipulator_libs/open_manipulator.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"
#include "open_manipulator_msgs/GetJointPosition.h"
#include "open_manipulator_msgs/GetKinematicsPose.h"
#include "open_manipulator_msgs/OpenManipulatorState.h"

namespace open_manipulator_controller
{
class OpenManipulatorController
{
 public:
  OpenManipulatorController(std::string usb_port, std::string baud_rate);
  ~OpenManipulatorController();

  // update
  void publishCallback(const ros::TimerEvent&);
  void startTimerThread();
  static void *timerThread(void *param);
  void process(double time);
  double getControlPeriod(void){return control_period_;}

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  bool using_platform_;
  double control_period_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  // Thread parameter
  pthread_t timer_thread_;
  pthread_attr_t attr_;
  bool timer_thread_state_;

  // Robotis_manipulator related 
  OpenManipulator open_manipulator_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();
  void initServer();

  /*****************************************************************************
  ** ROS Publishers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Publisher open_manipulator_states_pub_;
  std::vector<ros::Publisher> open_manipulator_kinematics_pose_pub_;
  ros::Publisher open_manipulator_joint_states_pub_;
  std::vector<ros::Publisher> gazebo_goal_joint_position_pub_;

  void publishOpenManipulatorStates();
  void publishKinematicsPose();
  void publishJointStates();
  void publishGazeboCommand();

  /*****************************************************************************
  ** ROS Subscribers and Callback Functions
  *****************************************************************************/
  ros::Subscriber open_manipulator_option_sub_;

  void openManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg);

  /*****************************************************************************
  ** ROS Servers and Callback Functions
  *****************************************************************************/
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
};
}
#endif //OPEN_MANIPULATOR_CONTROLLER_H_
