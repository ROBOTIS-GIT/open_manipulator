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

#include "open_manipulator_position_ctrl/motion_planning_tool.h"

using namespace motion_planning_tool;

MotionPlanningTool::MotionPlanningTool()
{
  points_ = 10;

  time_from_start_ = ros::Duration(10.0); // movement duration

  display_planned_path_positions_ = Eigen::MatrixXd::Zero(points_ , MAX_JOINT_NUM);     // positions of planned path
  display_planned_path_velocities_ = Eigen::MatrixXd::Zero(points_ , MAX_JOINT_NUM);    // positions of planned path
  display_planned_path_accelerations_ = Eigen::MatrixXd::Zero(points_ , MAX_JOINT_NUM); // positions of planned path

  // initialization
  robot_model_loader_.reset();
  planning_scene_.reset();
}

MotionPlanningTool::~MotionPlanningTool(){}

void MotionPlanningTool::init(std::string description)
{
  // initialization
  robot_model_loader_.reset( new robot_model_loader::RobotModelLoader(description));
  kinematic_model_ = robot_model_loader_->getModel();
  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();

  arm_joint_model_group_ 	= 	kinematic_model_->getJointModelGroup("arm");
  arm_joint_names_        = 	arm_joint_model_group_->getJointModelNames();

  planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model_));
}


