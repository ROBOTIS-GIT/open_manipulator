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

#include "open_manipulator_moveit_bridge/moveit_bridge.h"

using namespace open_manipulator;

MoveItBridge::MoveItBridge()
    :nh_(""),
     priv_nh_("~"),
     publish_period_(0.010f),
     use_platform_(true),
     set_home_position_(true),
     is_moving_(false)
{
  // Init parameter
  set_home_position_ = priv_nh_.param<bool>("set_home_position", true);
  use_platform_ = priv_nh_.param<bool>("use_platform", true);
  publish_period_ = priv_nh_.param<double>("publish_period", 0.010f);

  //  planned_path_info_.waypoints = 10;
  //  planned_path_info_.planned_path_positions = Eigen::MatrixXd::Zero(planned_path_info_.waypoints, joint_num_);

}

MoveItBridge::~MoveItBridge()
{
  ros::shutdown();
  return;
}

bool MoveItBridge::getPlanningGroupInfo(const std::string yaml_file)
{
  YAML::Node file;
  file = YAML::LoadFile(yaml_file.c_str());

  if (file == NULL)
    return false;

  for (YAML::const_iterator it_file = file.begin(); it_file != file.end(); it_file++)
  {
    std::string planning_group_name = it_file->first.as<std::string>();
    if (planning_group_name.size() == 0)
    {
      continue;
    }

    YAML::Node joint = file[planning_group_name];
    uint8_t joint_size = joint["names"].size();

    for (uint8_t index = 0; index < joint_size; index++)
    {
      std::string joint_name = joint["names"][index].as<std::string>();
      planning_group_[planning_group_name].name.push_back(joint_name);
    }
  }

  return true;
}

void MoveItBridge::initPlanningGroup()
{
  for (auto const& group:planning_group_)
  {
    moveit::planning_interface::MoveGroupInterface *move_group;
    move_group = new moveit::planning_interface::MoveGroupInterface(group.first);

    move_group_[group.first] = move_group;
  }
}

void MoveItBridge::initPublisher()
{
  if (use_platform_ == false)
  {
    ROS_INFO("SET Gazebo Simulation Mode(Joint)");

    for (auto const& group:planning_group_)
    {
      Joints joints = group.second;
      for (auto const& joint_name:joints.name)
      {
        ros::Publisher publisher = priv_nh_.advertise<std_msgs::Float64>(joint_name + "_position/command", 10);
        gazebo_goal_joint_position_pub_.push_back(publisher);
      }
    }
  }
  else
  {
    dynamixel_workbench_pub_ = priv_nh_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);

    joint_position_pub_ = priv_nh_.advertise<std_msgs::Float64MultiArray>("joint_position", 10);
    joint_move_time_pub_ = priv_nh_.advertise<std_msgs::Float64>("joint_move_time", 10);

    gripper_position_pub_ = priv_nh_.advertise<std_msgs::Float64MultiArray>("gripper_position", 10);
    gripper_move_time_pub_ = priv_nh_.advertise<std_msgs::Float64>("gripper_move_time", 10);
  }

  moving_state_pub_ = priv_nh_.advertise<open_manipulator_msgs::OpenManipulatorState>("moving_state", 10);
}

void MoveItBridge::initSubscriber()
{
  display_planned_path_sub_ = nh_.subscribe("/move_group/display_planned_path", 100,
                                            &MoveItBridge::displayPlannedPathMsgCallback, this);
}

void MoveItBridge::initServer()
{
  get_joint_position_server_  = priv_nh_.advertiseService("get_joint_position", &MoveItBridge::getJointPositionMsgCallback, this);
  get_kinematics_pose_server_ = priv_nh_.advertiseService("get_kinematics_pose", &MoveItBridge::getKinematicsPoseMsgCallback, this);
  set_joint_position_server_  = priv_nh_.advertiseService("set_joint_position", &MoveItBridge::setJointPositionMsgCallback, this);
  set_kinematics_pose_server_ = priv_nh_.advertiseService("set_kinematics_pose", &MoveItBridge::setKinematicsPoseMsgCallback, this);
}

void MoveItBridge::initClient()
{
  open_manipulator_joint_control_client_ = priv_nh_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
}

bool MoveItBridge::getJointPositionMsgCallback(open_manipulator_msgs::GetJointPosition::Request &req,
                                                open_manipulator_msgs::GetJointPosition::Response &res)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const std::vector<std::string> &joint_names = move_group_[req.planning_group]->getJointNames();
  std::vector<double> joint_values = move_group_[req.planning_group]->getCurrentJointValues();

  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    res.joint_position.joint_name.push_back(joint_names[i]);
    res.joint_position.position.push_back(joint_values[i]);
  }

  spinner.stop();
  return true;
}

bool MoveItBridge::getKinematicsPoseMsgCallback(open_manipulator_msgs::GetKinematicsPose::Request &req,
                                                 open_manipulator_msgs::GetKinematicsPose::Response &res)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

//  const std::string &pose_reference_frame = move_group_[req.planning_group]->getPoseReferenceFrame();
  geometry_msgs::PoseStamped current_pose = move_group_[req.planning_group]->getCurrentPose();

  res.header                     = current_pose.header;
  res.kinematics_pose.group_name = req.planning_group;
  res.kinematics_pose.pose       = current_pose.pose;

  spinner.stop();
  return true;
}

bool MoveItBridge::setJointPositionMsgCallback(open_manipulator_msgs::SetJointPosition::Request &req,
                                                open_manipulator_msgs::SetJointPosition::Response &res)
{
  open_manipulator_msgs::JointPosition msg = req.joint_position;
  res.isPlanned = calcPlannedPath(req.planning_group, msg);

  return true;
}

bool MoveItBridge::setKinematicsPoseMsgCallback(open_manipulator_msgs::SetKinematicsPose::Request &req,
                                                 open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  open_manipulator_msgs::KinematicsPose msg = req.kinematics_pose;
  res.isPlanned = calcPlannedPath(req.planning_group, msg);

  return true;
}

bool MoveItBridge::calcPlannedPath(const std::string planning_group, open_manipulator_msgs::KinematicsPose msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool isPlanned = false;
  geometry_msgs::Pose target_pose = msg.pose;

  move_group_[planning_group]->setPoseTarget(target_pose);

  move_group_[planning_group]->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group_[planning_group]->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  move_group_[planning_group]->setGoalTolerance(msg.tolerance);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (is_moving_ == false)
  {
    bool success = (move_group_[planning_group]->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
      isPlanned = true;
    }
    else
    {
      ROS_WARN("Planning (task space goal) is FAILED");
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

bool MoveItBridge::calcPlannedPath(const std::string planning_group, open_manipulator_msgs::JointPosition msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool isPlanned = false;

  const robot_state::JointModelGroup *joint_model_group = move_group_[planning_group]->getCurrentState()->getJointModelGroup("arm");

  moveit::core::RobotStatePtr current_state = move_group_[planning_group]->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  uint8_t joint_num = planning_group_[planning_group].name.size();
  for (uint8_t index = 0; index < joint_num; index++)
  {
    joint_group_positions[index] = msg.position[index];
  }

  move_group_[planning_group]->setJointValueTarget(joint_group_positions);

  move_group_[planning_group]->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group_[planning_group]->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (is_moving_ == false)
  {
    bool success = (move_group_[planning_group]->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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

void MoveItBridge::displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
  ROS_INFO("Get Planned Path");

  joint_trajectory_ = msg->trajectory[0].joint_trajectory;
  all_time_steps_ = msg->trajectory[0].joint_trajectory.points.size();

//  planned_path_info_.waypoints = msg->trajectory[0].joint_trajectory.points.size();
//  planned_path_info_.planned_path_positions.resize(planned_path_info_.waypoints, joint_num);

//  for (uint16_t point_num = 0; point_num < planned_path_info_.waypoints; point_num++)
//  {
//    for (uint8_t num = 0; num < joint_num; num++)
//    {
//      float joint_position = msg->trajectory[0].joint_trajectory.points[point_num].positions[num];

//      planned_path_info_.time_from_start = msg->trajectory[0].joint_trajectory.points[point_num].time_from_start.toSec();
//      planned_path_info_.planned_path_positions.coeffRef(point_num , num) = joint_position;
//    }
//  }

//  all_time_steps_ = planned_path_info_.waypoints;

//  ros::WallDuration sleep_time(0.5);
//  sleep_time.sleep();

  is_moving_  = true;
}

void MoveItBridge::controlCallback(const ros::TimerEvent&)
{
  static uint16_t step_cnt = 0;
  std_msgs::Float64 gazebo_goal_joint_position;

  open_manipulator_msgs::SetJointPosition srv;
  open_manipulator_msgs::OpenManipulatorState state;

  trajectory_msgs::JointTrajectory jnt_tra;

  if (is_moving_)
  {
    uint8_t joint_num = joint_trajectory_.points[0].positions.size();
    if (use_platform_ == false)
    {
      for (uint8_t i = 0; i < joint_num; i++)
      {
        gazebo_goal_joint_position.data = joint_trajectory_.points[step_cnt].positions[i];
        gazebo_goal_joint_position_pub_.at(i).publish(gazebo_goal_joint_position);
      }
      step_cnt++;
    }
    else
    {
      jnt_tra.points[step_cnt] = joint_trajectory_.points[step_cnt];

      // dynamixel_workbench_controller
      jnt_tra.joint_names = joint_trajectory_.joint_names;
      dynamixel_workbench_pub_.publish(jnt_tra);

      // open_manipulator_controller
      std::vector<double> joint_angle;
      double path_time = 0.010f;

      for (uint8_t i = 0; i < joint_num; i++)
      {
        joint_angle.push_back(joint_trajectory_.points[step_cnt].positions[i]);
        path_time = joint_trajectory_.points[step_cnt].time_from_start.toSec() + 0.010f;
      }

      srv.request.joint_position.joint_name = joint_trajectory_.joint_names;
      srv.request.joint_position.position = joint_angle;
      srv.request.path_time = path_time;

      if (open_manipulator_joint_control_client_.call(srv) == false)
        ROS_ERROR("Failed to call");

      // turtlebot3_with_open_manipulator_core.ino
      std_msgs::Float64 joint_move_time;
      joint_move_time.data = joint_trajectory_.points[step_cnt].time_from_start.toSec();

      std_msgs::Float64MultiArray joint_position;
      for (uint8_t i = 0; i < joint_num; i++)
      {
        joint_position.data.push_back(joint_trajectory_.points[step_cnt].positions[i]);
      }
      joint_position_pub_.publish(joint_position);
      joint_move_time_pub_.publish(joint_move_time);

      step_cnt++;
    }

    if (step_cnt >= all_time_steps_)
    {
      is_moving_ = false;
      step_cnt   = 0;

      ROS_INFO("Complete Execution");
    }

    state.open_manipulator_moving_state = state.IS_MOVING;
    moving_state_pub_.publish(state);
  }
  else
  {
    state.open_manipulator_moving_state = state.STOPPED;
    moving_state_pub_.publish(state);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_bridge");
  ros::NodeHandle node_handle("");

//  ros::WallDuration sleep_time(3.0);
//  sleep_time.sleep();

  MoveItBridge bridge;

  std::string yaml_file = node_handle.param<std::string>("planning_group", "");

  bool result = bridge.getPlanningGroupInfo(yaml_file);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return 0;
  }

  bridge.initPlanningGroup();

  bridge.initPublisher();
  bridge.initSubscriber();

  bridge.initServer();
  bridge.initClient();

  ros::Timer control_timer = node_handle.createTimer(ros::Duration(bridge.getPublishPeriod()), &MoveItBridge::controlCallback, &bridge);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
