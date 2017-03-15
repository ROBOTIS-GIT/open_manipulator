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

#include "open_manipulator_position_ctrl/position_controller.h"

using namespace open_manipulator_position_ctrl;

PositionController::PositionController()
    :nh_priv_("~"),
     is_moving_(false),
     move_time_(0.0),
     all_time_steps_(0.0),
     step_cnt_(0),
     enable_joint_(false),
     enable_gripper_(false)
{
  // Init parameter
  nh_.param("is_debug", is_debug_, is_debug_);

  // ROS Publisher
  goal_joint_position_pub_   = nh_.advertise<sensor_msgs::JointState>("/robotis/dynamixel/goal_joint_states", 10);
  goal_gripper_position_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/dynamixel/goal_gripper_states", 10);
  bottle_pose_msg_pub_       = nh_.advertise<open_manipulator_msgs::KinematicsPose>("/robotis/open_manipulator/motion_planning_target_pose", 10);
  place_pose_msg_pub_        = nh_.advertise<open_manipulator_msgs::KinematicsPose>("/robotis/open_manipulator/motion_planning_target_pose", 10);
  demo_order_pub_            = nh_.advertise<std_msgs::String>("/robotis/open_manipulator/pick_and_place", 10);

  // ROS Subscriber
  demo_order_sub_                      = nh_.subscribe("/robotis/open_manipulator/pick_and_place",10,
                                                      &PositionController::demoMsgCallback, this);
  present_joint_position_sub_          = nh_.subscribe("/robotis/dynamixel/present_joint_states", 10,
                                                      &PositionController::presentJointPositionMsgCallback, this);
  present_gripper_position_sub_        = nh_.subscribe("/robotis/dynamixel/present_gripper_states", 10,
                                                      &PositionController::presentGripperPositionMsgCallback, this);
  motion_planning_target_pose_msg_sub_ = nh_.subscribe("/robotis/open_manipulator/motion_planning_target_pose", 10,
                                                      &PositionController::motionPlanningTargetPoseMsgCallback, this);
  execute_planned_path_msg_sub_        = nh_.subscribe("/robotis/open_manipulator/execute_planned_path", 10,
                                                       &PositionController::executePlannedPathMsgCallback, this);
  display_planned_path_sub_            = nh_.subscribe("/move_group/display_planned_path", 10,
                                                       &PositionController::displayPlannedPathMsgCallback, this);
  bottle_pose_msg_sub_                 = nh_.subscribe("/robotis/open_manipulator/ar_transform", 10,
                                                       &PositionController::bottlePoseMsgCallback, this);

  // Init target name
  ROS_ASSERT(initPositionController());
}

PositionController::~PositionController()
{
  ROS_ASSERT(shutdownPositionController());
}

bool PositionController::initPositionController(void)
{
  joint_id_["joint1"] = 1;
  joint_id_["joint2"] = 2;
  joint_id_["joint3"] = 3;
  joint_id_["joint4"] = 4;

  present_joint_position_   = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  present_gripper_position_ = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);
  goal_gripper_position_    = Eigen::VectorXd::Zero(MAX_GRIPPER_NUM);

  motionPlanningTool_ = new motion_planning_tool::MotionPlanningTool();

  motionPlanningTool_->init("robot_description");

  ROS_INFO("open_manipulator_position_controller : Init OK!");
  return true;
}

bool PositionController::shutdownPositionController(void)
{
  ros::shutdown();
  return true;
}

void PositionController::demoMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  order_.data = msg->data;

  if (is_moving_ == false)
  {
    if (msg->data == "init_joint")
    {
      ROS_INFO("Set Init Position");

      std::string pose_path;
      pose_path = ros::package::getPath("open_manipulator_position_ctrl") + "/config/initial_pose.yaml";

      parsePoseData(pose_path);
    }
    else if (msg->data == "zero_joint")
    {
      ROS_INFO("Set Init Position");

      std::string pose_path;
      pose_path = ros::package::getPath("open_manipulator_position_ctrl") + "/config/zero_pose.yaml";

      parsePoseData(pose_path);
    }
    else if (msg->data == "grip_on")
    {
      Eigen::VectorXd initial_position = present_gripper_position_;

      goal_gripper_position_(0) = 50 * DEGREE2RADIAN;
      Eigen::VectorXd target_position = goal_gripper_position_;

      move_time_ = 2.0;
      calculateGripperGoalTrajectory(initial_position, target_position);
    }
    else if (msg->data == "grip_off")
    {
      Eigen::VectorXd initial_position = present_gripper_position_;

      goal_gripper_position_(0) = 0 * DEGREE2RADIAN;
      Eigen::VectorXd target_position = goal_gripper_position_;

      move_time_ = 2.0;
      calculateGripperGoalTrajectory(initial_position, target_position);
    }
    else if (msg->data == "pick")
    {
      bottle_pose_msg_pub_.publish(bottle_pose_);
    }
    else if (msg->data == "closed")
    {
      bottle_pose_.pose.position.x = bottle_pose_.pose.position.x + 0.015;

      bottle_pose_msg_pub_.publish(bottle_pose_);
    }
    else if (msg->data == "place")
    {
      // Set place pose
      place_pose_.pose.position.x = 0.255;
      place_pose_.pose.position.y = 0.000;
      place_pose_.pose.position.z = 0.165;

      place_pose_.pose.orientation.x = 0.0;
      place_pose_.pose.orientation.y = 0.0;
      place_pose_.pose.orientation.z = 0.0;
      place_pose_.pose.orientation.w = 1.0;

      place_pose_msg_pub_.publish(place_pose_);
    }
    else if (msg->data == "init_pose_1")
    {
      open_manipulator_msgs::KinematicsPose init_pose;

      init_pose.pose.position.x = 0.140;
      init_pose.pose.position.y = 0.000;
      init_pose.pose.position.z = 0.280;

      init_pose.pose.orientation.x = 0.000;
      init_pose.pose.orientation.y = 0.000;
      init_pose.pose.orientation.z = 0.000;
      init_pose.pose.orientation.w = 1.000;

      bottle_pose_msg_pub_.publish(init_pose);
    }
    else if (msg->data == "init_pose_2")
    {
      open_manipulator_msgs::KinematicsPose init_pose;

      init_pose.pose.position.x = 0.040;
      init_pose.pose.position.y = 0.000;
      init_pose.pose.position.z = 0.323;

      init_pose.pose.orientation.x = 0.000;
      init_pose.pose.orientation.y = 0.000;
      init_pose.pose.orientation.z = 0.000;
      init_pose.pose.orientation.w = 1.000;

      bottle_pose_msg_pub_.publish(init_pose);
    }
    else if (msg->data == "zero_pose")
    {
      open_manipulator_msgs::KinematicsPose init_pose;

      init_pose.pose.position.x = 0.198;
      init_pose.pose.position.y = 0.000;
      init_pose.pose.position.z = 0.301;

      init_pose.pose.orientation.x = 0.000;
      init_pose.pose.orientation.y = 0.000;
      init_pose.pose.orientation.z = 0.000;
      init_pose.pose.orientation.w = 1.000;

      bottle_pose_msg_pub_.publish(init_pose);
    }
  }
  else
  {
    ROS_WARN("Other process is alive");
  }
}

void PositionController::bottlePoseMsgCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  float distance_gripper_to_joint_4 = 0.070;
  float distance_camera_to_bottle = msg->position.x - distance_gripper_to_joint_4;

  float distance_bottle_to_world_x = distance_camera_to_bottle;// + 0.110; // distance_camera_to_bottle + distance_camera_to_world
  float distance_bottle_to_world_y = msg->position.y;

  float distance_bottle_to_world = sqrt(distance_bottle_to_world_x*distance_bottle_to_world_x + distance_bottle_to_world_y*distance_bottle_to_world_y);

  bottle_pose_.pose.position.x = distance_camera_to_bottle;          // 104.2mm
  bottle_pose_.pose.position.y = 0.0;                                // msg->position.y;
  bottle_pose_.pose.position.z = 0.240;                              // msg->position.z;

  Eigen::Quaterniond bottle_orientationQua(msg->orientation.w,
                                           msg->orientation.x,
                                           msg->orientation.y,
                                           msg->orientation.z);

  Eigen::Vector3d bottle_orientationRPY;

  bottle_orientationRPY = robotis_framework::convertQuaternionToRPY(bottle_orientationQua);

  double bottle_yaw = acos(distance_bottle_to_world_x/distance_bottle_to_world) * robotis_framework::sign(distance_bottle_to_world_y); //bottle_orientationRPY.coeff(2) + 1.5708;

  bottle_orientationQua = robotis_framework::convertRPYToQuaternion(0.0,
                                                                    0.0,
                                                                    bottle_yaw);
  bottle_pose_.pose.orientation.x = bottle_orientationQua.x();
  bottle_pose_.pose.orientation.y = bottle_orientationQua.y();
  bottle_pose_.pose.orientation.z = bottle_orientationQua.z();
  bottle_pose_.pose.orientation.w = bottle_orientationQua.w();

  ROS_WARN("Position X = %lf", bottle_pose_.pose.position.x);
  ROS_INFO("Position Y = %lf", msg->position.y);
  ROS_INFO("Position Z = %lf", bottle_pose_.pose.position.z);
//  ROS_INFO("Roll       = %lf", bottle_orientationRPY(0) * RADIAN2DEGREE);
//  ROS_INFO("Pitch      = %lf", bottle_orientationRPY(1) * RADIAN2DEGREE);
  ROS_WARN("Yaw        = %lf", bottle_yaw * RADIAN2DEGREE);
}

void PositionController::motionPlanningTargetPoseMsgCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  // Get Target Pose for Motion Planning

  if (is_moving_ == false)
  {
    moveit_execution_ = true;
  }
  else
  {
    ROS_WARN("Other process is alive");
  }
}

void PositionController::executePlannedPathMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  // Get message from open_manipulator_motion_planning_ctrl

  if (is_moving_ == false)
  {
    if (msg->data == "execute")
    {
      ROS_INFO("Planning Execute...");
    }
    else if (msg->data == "fail")
    {
      ROS_ERROR("Planning Fail");
      ROS_INFO("End Trajectory");
      moveit_execution_ = false;
    }
  }
}

void PositionController::displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
  /*
  # The model id for which this path has been generated

  string model_id

  # The representation of the path contains position values for all the joints that are moving along the path;
  # a sequence of trajectories may be specified

  RobotTrajectory[] trajectory

  # The robot state is used to obtain positions for all/some of the joints of the robot.
  # It is used by the path display node to determine the positions of the joints that are not specified in the joint path message above.
  # If the robot state message contains joint position information for joints that are also mentioned in the joint path message,
  # the positions in the joint path message will overwrite the positions specified in the robot state message.

  RobotState trajectory_start
  */

  motionPlanningTool_->moveit_msg_ = *msg;

  if (is_moving_ == false)
  {
    trajectory_generate_tread_ = new boost::thread(boost::bind(&PositionController::moveItTragectoryGenerateThread, this));
    delete trajectory_generate_tread_;
  }
  else
  {
    ROS_WARN("Other process is alive");
  }
}

void PositionController::moveItTragectoryGenerateThread()
{
  std::vector<double> via_time;

  for (int _tra_index = 0; _tra_index < motionPlanningTool_->moveit_msg_.trajectory.size(); _tra_index++)
  {
    //      ROS_INFO("_tra_index = %d", motionPlanningTool_->moveit_msg_.trajectory.size());
    motionPlanningTool_->points_ = motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.points.size();

    motionPlanningTool_->display_planned_path_positions_.resize(motionPlanningTool_->points_, MAX_JOINT_NUM);
    motionPlanningTool_->display_planned_path_velocities_.resize(motionPlanningTool_->points_, MAX_JOINT_NUM);
    motionPlanningTool_->display_planned_path_accelerations_.resize(motionPlanningTool_->points_, MAX_JOINT_NUM);

    for (int _point_index = 0; _point_index < motionPlanningTool_->points_; _point_index++)
    {
      //        ROS_INFO("_point_index = %d", motionPlanningTool_->points_);
      for(int _name_index = 0; _name_index < MAX_JOINT_NUM; _name_index++)
      {
        motionPlanningTool_->display_planned_path_positions_.coeffRef(_point_index, _name_index) = goal_joint_position_(_name_index);
      }
    }

    for (int _point_index = 0; _point_index < motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.points.size(); _point_index++)
    {
      motionPlanningTool_->time_from_start_ = motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.points[_point_index].time_from_start;
      via_time.push_back(motionPlanningTool_->time_from_start_.toSec());

      for (int _joint_index = 0; _joint_index < motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.joint_names.size(); _joint_index++)
      {
        //          ROS_INFO("_joint_index = %d", motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.joint_names.size());
        std::string _joint_name 	  = motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.joint_names[_joint_index];

        double _joint_position 		  = motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.points[_point_index].positions	   [_joint_index];
        double _joint_velocity 	   	= motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.points[_point_index].velocities	 [_joint_index];
        double _joint_acceleration 	= motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.points[_point_index].accelerations[_joint_index];

        motionPlanningTool_->display_planned_path_positions_.coeffRef     (_point_index , joint_id_[_joint_name]-1) = _joint_position;
        motionPlanningTool_->display_planned_path_velocities_.coeffRef	  (_point_index , joint_id_[_joint_name]-1) = _joint_velocity;
        motionPlanningTool_->display_planned_path_accelerations_.coeffRef (_point_index , joint_id_[_joint_name]-1) = _joint_acceleration;
      }
    }
  }

  move_time_ = motionPlanningTool_->time_from_start_.toSec();
  //    ROS_INFO("move_time_ = %f", move_time_);

  all_time_steps_ = motionPlanningTool_->points_;
  //    ROS_INFO("all_time_steps_ = %d", all_time_steps_ );

  //    ROS_INFO("[end] plan trajectory");

  //    PRINT_MAT( Moveit->display_planned_path_positions );

  ros::Duration seconds(0.5);
  seconds.sleep();

  //    if ( Robotis->execute_planned_path == true )
  if (moveit_execution_ == true)
  {
    //joint_goal_trajectory_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_NUM);
    goal_joint_trajectory_ = motionPlanningTool_->display_planned_path_positions_;

    is_moving_    = true;
    enable_joint_ = true;
    step_cnt_     = 0;

    moveit_execution_ = false;

    ROS_INFO("Send Motion Trajectory");

    //    Robotis->execute_planned_path = false;

    //        ROS_INFO("[start] send trajectory");
  }
}

void PositionController::presentJointPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  for (int index = 0; index < MAX_JOINT_NUM; index++)
  {
    present_joint_position_(index) = msg->position.at(index);
  }
}

void PositionController::presentGripperPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  present_gripper_position_(0) = msg->position.at(0);
}

void PositionController::calculateJointGoalTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  /* set movement time */
  all_time_steps_ = int(floor((move_time_ / ITERATION_TIME) + 1.0));
  move_time_ = double(all_time_steps_ - 1) * ITERATION_TIME;

  goal_joint_trajectory_.resize(all_time_steps_, MAX_JOINT_NUM);

  /* calculate joint trajectory */
  for (int index = 0; index < MAX_JOINT_NUM; index++)
  {
    double init_position_value = initial_position(index);
    double target_position_value = target_position(index);

    Eigen::MatrixXd trajectory =
        robotis_framework::calcMinimumJerkTra(init_position_value, 0.0, 0.0,
                                              target_position_value, 0.0, 0.0,
                                              ITERATION_TIME, move_time_);

    // Block of size (p,q), starting at (i,j)
    // block(i,j,p,q)
    goal_joint_trajectory_.block(0, index, all_time_steps_, 1) = trajectory;
  }

  step_cnt_ = 0;
  is_moving_ = true;
  enable_joint_ = true;

  ROS_INFO("Start Joint Trajectory");
}

void PositionController::calculateGripperGoalTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  /* set movement time */
  all_time_steps_ = int(floor((move_time_ / ITERATION_TIME) + 1.0));
  move_time_ = double(all_time_steps_ - 1) * ITERATION_TIME;

  goal_gripper_trajectory_.resize(all_time_steps_, MAX_GRIPPER_NUM);

  /* calculate gripper trajectory */
  for (int index = 0; index < MAX_GRIPPER_NUM; index++)
  {
    double init_position_value = initial_position(index);
    double target_position_value = target_position(index);

    Eigen::MatrixXd trajectory =
        robotis_framework::calcMinimumJerkTra(init_position_value, 0.0, 0.0,
                                              target_position_value, 0.0, 0.0,
                                              ITERATION_TIME, move_time_);

    // Block of size (p,q), starting at (i,j)
    // block(i,j,p,q)
    goal_gripper_trajectory_.block(0, index, all_time_steps_, 1) = trajectory;
  }

  step_cnt_ = 0;
  is_moving_ = true;
  enable_gripper_ = true;

  ROS_INFO("Start Gripper Trajectory");
}

void PositionController::parsePoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  Eigen::VectorXd initial_position = present_joint_position_;

  // parse movement time
  move_time_ = doc["move_time"].as<double>();

  // parse target position
  Eigen::VectorXd target_position = Eigen::VectorXd::Zero(MAX_JOINT_NUM);

  YAML::Node target_position_node = doc["target_position"];
  for(YAML::iterator it = target_position_node.begin(); it != target_position_node.end(); ++it)
  {
    std::string joint_name;
    double value;

    joint_name = it->first.as<std::string>();
    value = it->second.as<double>();

    target_position(joint_id_[joint_name]-1) = value * DEGREE2RADIAN;
  }

  calculateJointGoalTrajectory(initial_position, target_position);
}

void PositionController::process(void)
{
  // Get Joint & Gripper State
  // present_joint_position, present_gripper_position

  if (is_moving_ == true)
  {
    if (enable_joint_ == true)
    {
      for (int index = 0; index < MAX_JOINT_NUM; index++)
      {
        goal_joint_position_(index) = goal_joint_trajectory_(step_cnt_, index);
      }
    }

    if (enable_gripper_ == true)
    {
      for (int index = 0; index < MAX_GRIPPER_NUM; index++)
      {
        goal_gripper_position_(index) = goal_gripper_trajectory_(step_cnt_, index);
      }
    }

    step_cnt_++;
  }

  sensor_msgs::JointState send_to_joint_position, send_to_gripper_position;

  for (std::map<std::string, uint8_t>::iterator state_iter = joint_id_.begin();
       state_iter != joint_id_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    send_to_joint_position.name.push_back(joint_name);
    send_to_joint_position.position.push_back(goal_joint_position_(joint_id_[joint_name]-1));
  }

  std::string gripper_name = "grip_joint";
  send_to_gripper_position.name.push_back(gripper_name);
  send_to_gripper_position.position.push_back(goal_gripper_position_(0));

  if (is_moving_ == true)
  {
    if (enable_joint_ == true)
      goal_joint_position_pub_.publish(send_to_joint_position);
    else if (enable_gripper_ == true)
      goal_gripper_position_pub_.publish(send_to_gripper_position);
  }

  if (is_moving_ == true)
  {
    if (step_cnt_ >= all_time_steps_)
    {
      ROS_INFO("End Trajectory");

      is_moving_ = false;
      step_cnt_  = 0;
      enable_joint_ = false;
      enable_gripper_ = false;

      if (order_.data == "pick")
      {
        std_msgs::String msg;
        msg.data = "grip_on";

        demo_order_pub_.publish(msg);
      }
      else if (order_.data == "grip_on")
      {
        std_msgs::String msg;
        msg.data = "init_pose_1";

        demo_order_pub_.publish(msg);
      }

      if (order_.data == "place")
      {
        std_msgs::String msg;
        msg.data = "grip_off";

        demo_order_pub_.publish(msg);
      }
      else if (order_.data == "grip_off")
      {
        std_msgs::String msg;
        msg.data = "init_joint";

        demo_order_pub_.publish(msg);
      }
    }
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_position_controller");
  PositionController position_controller;
  ros::Rate loop_rate(ITERATION_FREQUENCY);

  while (ros::ok())
  {
    position_controller.process();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
