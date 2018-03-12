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

#ifndef MOTION_PLANNING_TOOL_H
#define MOTION_PLANNING_TOOL_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include "robotis_math/robotis_math.h"

namespace motion_planning_tool
{
#define MAX_JOINT_NUM  (4)

class MotionPlanningTool
{
 public:
  MotionPlanningTool();
  ~MotionPlanningTool();

  int points_; // planned number of via-points

  ros::Duration time_from_start_; // planned movement time

  Eigen::MatrixXd display_planned_path_positions_; // planned position trajectory
  Eigen::MatrixXd display_planned_path_velocities_; // planned velocity trajectory
  Eigen::MatrixXd display_planned_path_accelerations_; // planned acceleration trajectory

  moveit_msgs::DisplayTrajectory moveit_msg_;

  // initialization
  boost::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;

  robot_state::JointModelGroup* arm_joint_model_group_;
  std::vector<std::string> arm_joint_names_;

  std::vector<double> _arm_joint_values_ , arm_joint_values_ ;

  boost::shared_ptr<planning_scene::PlanningScene> planning_scene_;

  // moveit path planning
  boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_planning_group_;

  void init(std::string description);
};

}

#endif /* MOTION_PLANNING_TOOL_H */
