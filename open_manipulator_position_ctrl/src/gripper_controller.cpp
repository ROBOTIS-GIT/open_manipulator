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

#include "open_manipulator_position_ctrl/gripper_controller.h"

using namespace open_manipulator;

GripperController::GripperController()
    :priv_nh_("~"),
     using_gazebo_(false),
     robot_name_(""),
     palm_num_(2),
     is_moving_(false)
{
  // Init parameter
  nh_.getParam("gazebo", using_gazebo_);
  nh_.getParam("robot_name", robot_name_);

  planned_path_info_.waypoints = 10;
  planned_path_info_.planned_path_positions = Eigen::MatrixXd::Zero(planned_path_info_.waypoints, 2);

  move_group = new moveit::planning_interface::MoveGroupInterface("gripper");

  initPublisher(using_gazebo_);
  initSubscriber(using_gazebo_);

  initServer();
}

GripperController::~GripperController()
{
  ros::shutdown();
  return;
}

void GripperController::initPublisher(bool using_gazebo)
{
  if (using_gazebo)
  {
    ROS_INFO("SET Gazebo Simulation Mode(Gripper)");

    if (robot_name_ == "open_manipulator")
    {
      gazebo_gripper_position_pub_[LEFT_PALM]  = nh_.advertise<std_msgs::Float64>(robot_name_ + "/grip_joint_position/command", 10);
      gazebo_gripper_position_pub_[RIGHT_PALM] = nh_.advertise<std_msgs::Float64>(robot_name_ + "/grip_joint_sub_position/command", 10);
    }
    else
    {
      gazebo_gripper_position_pub_[LEFT_PALM]  = nh_.advertise<std_msgs::Float64>("/grip_joint_position/command", 10);
      gazebo_gripper_position_pub_[RIGHT_PALM] = nh_.advertise<std_msgs::Float64>("/grip_joint_sub_position/command", 10);
    }
  }
  else
  {
    gripper_position_pub_ = nh_.advertise<sensor_msgs::JointState>(robot_name_ + "/goal_gripper_position", 10);
  }

  gripper_state_pub_ = nh_.advertise<open_manipulator_msgs::State>(robot_name_ + "/gripper_state", 10);
}

void GripperController::initSubscriber(bool using_gazebo)
{
  gripper_onoff_sub_ = nh_.subscribe(robot_name_ + "/gripper", 10,
                                     &GripperController::gripperOnOffMsgCallback, this);

  display_planned_path_sub_ = nh_.subscribe("/move_group/display_planned_path", 10,
                                            &GripperController::displayPlannedPathMsgCallback, this);
}

void GripperController::initServer()
{
  set_gripper_position_server_ = nh_.advertiseService(robot_name_ + "/set_gripper_position", &GripperController::setGripperPositionMsgCallback, this);
}

bool GripperController::setGripperPositionMsgCallback(open_manipulator_msgs::SetJointPosition::Request &req,
                                                      open_manipulator_msgs::SetJointPosition::Response &res)
{
  open_manipulator_msgs::JointPosition msg = req.joint_position;
  res.isPlanned = calcPlannedPath(msg);
}

bool GripperController::calcPlannedPath(open_manipulator_msgs::JointPosition msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool isPlanned = false;

  const robot_state::JointModelGroup *joint_model_group = move_group->getCurrentState()->getJointModelGroup("gripper");

  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  msg.joint_name.push_back("grip_joint");
  msg.joint_name.push_back("grip_joint_sub");

  for (uint8_t index = 0; index < palm_num_; index++)
  {
    joint_group_positions[index] = msg.position[0];
    joint_group_positions[index] = msg.position[0];
  }

  move_group->setJointValueTarget(joint_group_positions);

  move_group->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (is_moving_ == false)
  {
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
      isPlanned = true;
    }
    else
    {
      ROS_WARN("Planning (joint space goal) is FAILED");
      isPlanned = false;
    }
  }
  else
  {
    ROS_WARN("ROBOT IS WORKING");
    isPlanned = false;
  }

  spinner.stop();

  return isPlanned;
}

void GripperController::gripperOnOffMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  open_manipulator_msgs::JointPosition joint_msg;

  joint_msg.max_velocity_scaling_factor = 0.3;
  joint_msg.max_accelerations_scaling_factor = 0.01;

  if (msg->data == "grip_on")
  {
    joint_msg.position.push_back(GRIP_ON);
    calcPlannedPath(joint_msg);
  }
  else if (msg->data == "grip_off")
  {
    joint_msg.position.push_back(GRIP_OFF);
    calcPlannedPath(joint_msg);
  }
    else if (msg->data == "neutral")
  {
    joint_msg.position.push_back(NEUTRAL);
    calcPlannedPath(joint_msg);
  }
  else
  {
    ROS_ERROR("If you want to grip or release something, publish 'grip_on', 'grip_off' or 'neutral'");
  }
}

void GripperController::displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
  // Can find 'grip'
  if (msg->trajectory[0].joint_trajectory.joint_names[0].find("grip") != std::string::npos)
  {
    ROS_INFO("Get Gripper Planned Path");
    uint8_t gripper_num = 2;

    planned_path_info_.waypoints = msg->trajectory[0].joint_trajectory.points.size();

    planned_path_info_.planned_path_positions.resize(planned_path_info_.waypoints, gripper_num);

    for (uint16_t point_num = 0; point_num < planned_path_info_.waypoints; point_num++)
    {
      for (uint8_t num = 0; num < gripper_num; num++)
      {
        float joint_position = msg->trajectory[0].joint_trajectory.points[point_num].positions[num];

        planned_path_info_.planned_path_positions.coeffRef(point_num , num) = joint_position;
      }
    }

    all_time_steps_ = planned_path_info_.waypoints - 1;

    ros::WallDuration sleep_time(0.5);
    sleep_time.sleep();

    is_moving_  = true; 
  }
}

void GripperController::process(void)
{
  static uint16_t step_cnt = 0;
  std_msgs::Float64 gazebo_goal_gripper_position;
  sensor_msgs::JointState goal_gripper_position;
  open_manipulator_msgs::State state;

  if (is_moving_)
  {
    if (using_gazebo_)
    {
      gazebo_goal_gripper_position.data = planned_path_info_.planned_path_positions(step_cnt, 0);
      gazebo_gripper_position_pub_[LEFT_PALM].publish(gazebo_goal_gripper_position);

      gazebo_goal_gripper_position.data = planned_path_info_.planned_path_positions(step_cnt, 1);
      gazebo_gripper_position_pub_[RIGHT_PALM].publish(gazebo_goal_gripper_position);
    }
    else
    {
      goal_gripper_position.position.push_back(planned_path_info_.planned_path_positions(step_cnt, 0));
      goal_gripper_position.position.push_back(planned_path_info_.planned_path_positions(step_cnt, 1));

      gripper_position_pub_.publish(goal_gripper_position);
    }

    if (step_cnt >= all_time_steps_)
    {
      is_moving_ = false;
      step_cnt   = 0;

      ROS_INFO("Complete Execution");
    }
    else
    {
      step_cnt++;
    }

    state.robot = state.IS_MOVING;
    gripper_state_pub_.publish(state);
  }
  else
  {
    state.robot = state.STOPPED;
    gripper_state_pub_.publish(state);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_controller_for_OpenManipulator");

  ros::WallDuration sleep_time(3.0);
  sleep_time.sleep();

  GripperController controller;

  ros::Rate loop_rate(ITERATION_FREQUENCY);

  while (ros::ok())
  {
    controller.process();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
