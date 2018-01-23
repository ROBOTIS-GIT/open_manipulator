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
     using_gazebo_(false),
     using_moveit_(false),
     is_moving_(false),
     move_time_(0.0),
     all_time_steps_(0.0),
     step_cnt_(0),
     moveit_execution_(false),
     gripper_(false)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_.getParam("gazebo", using_gazebo_);
  nh_.getParam("moveit", using_moveit_);
  nh_.getParam("gazebo_robot_name", robot_name_);

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

  present_joint_position_   = Eigen::VectorXd::Zero(MAX_JOINT_NUM+MAX_GRIP_JOINT_NUM);
  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_NUM);
  goal_gripper_position_    = Eigen::VectorXd::Zero(MAX_GRIP_JOINT_NUM);

  initStatePublisher(using_gazebo_);
  initStateSubscriber(using_gazebo_);

  if (using_moveit_)
  {
    motionPlanningTool_ = new motion_planning_tool::MotionPlanningTool();

    motionPlanningTool_->init("robot_description");
  }

  ROS_INFO("open_manipulator_position_controller : Init OK!");
  return true;
}

bool PositionController::shutdownPositionController(void)
{
  ros::shutdown();
  return true;
}

bool PositionController::initStatePublisher(bool using_gazebo)
{
  // ROS Publisher
  if (using_gazebo)
  {
    ROS_WARN("SET Gazebo Simulation Mode");
    for (std::map<std::string, uint8_t>::iterator state_iter = joint_id_.begin();
         state_iter != joint_id_.end(); state_iter++)
    {
      std::string joint_name = state_iter->first;
      gazebo_goal_joint_position_pub_[joint_id_[joint_name]-1]
        = nh_.advertise<std_msgs::Float64>("/" + robot_name_ + "/" + joint_name + "_position/command", 10);
    }

    gazebo_gripper_position_pub_[LEFT_GRIP]  = nh_.advertise<std_msgs::Float64>("/" + robot_name_ + "/grip_joint_position/command", 10);
    gazebo_gripper_position_pub_[RIGHT_GRIP] = nh_.advertise<std_msgs::Float64>("/" + robot_name_ + "/grip_joint_sub_position/command", 10);
  }
  else
  {
    goal_joint_position_pub_   = nh_.advertise<sensor_msgs::JointState>("/robotis/open_manipulator/goal_joint_states", 10);
  }

  moving_pub_   = nh_.advertise<std_msgs::Bool>("/robotis/open_manipulator/moving", 10);
}

bool PositionController::initStateSubscriber(bool using_gazebo)
{
  // ROS Subscriber
  if (using_gazebo)
  {
    gazebo_present_joint_position_sub_ = nh_.subscribe("/" + robot_name_ + "/joint_states", 10,
                                                       &PositionController::gazeboPresentJointPositionMsgCallback, this);
  }
  else
  {
    present_joint_position_sub_     = nh_.subscribe("/robotis/open_manipulator/present_joint_states", 10,
                                                      &PositionController::presentJointPositionMsgCallback, this);
  }

  move_group_feedback_sub_        = nh_.subscribe("/move_group/feedback", 10,
                                                    &PositionController::moveGroupActionFeedbackMsgCallback, this);
  display_planned_path_sub_       = nh_.subscribe("/move_group/display_planned_path", 10,
                                                    &PositionController::displayPlannedPathMsgCallback, this);

  gripper_position_sub_           = nh_.subscribe("/robotis/open_manipulator/gripper", 10,
                                                    &PositionController::gripperPositionMsgCallback, this);

  joint_position_sub_             = nh_.subscribe("/robotis/open_manipulator/joints", 10,
                                                    &PositionController::jointPositionMsgCallback, this);
}

bool PositionController::getPresentPosition()
{
  for (int it = 0; it < MAX_JOINT_NUM; it++)
  {
    goal_joint_position_(it)  = present_joint_position_(it);
  }

  goal_gripper_position_(0)  = present_joint_position_(GRIPPER);
}

void PositionController::gripOn(void)
{
  Eigen::VectorXd initial_position = Eigen::VectorXd::Zero(MAX_JOINT_NUM + MAX_GRIP_JOINT_NUM);
  Eigen::VectorXd target_position  = Eigen::VectorXd::Zero(MAX_JOINT_NUM + MAX_GRIP_JOINT_NUM);

  for (int it = 0; it < MAX_JOINT_NUM; it++)
  {
    initial_position(it)  = goal_joint_position_(it);
    target_position(it)   = goal_joint_position_(it);
  }

  initial_position(GRIPPER)  = goal_gripper_position_(0);
  target_position(GRIPPER)   = goal_gripper_position_(0);

  target_position(GRIPPER) = 75.0 * DEGREE2RADIAN;

  move_time_ = 3.0;
  gripper_    = true;
  calculateGoalTrajectory(initial_position, target_position);

  ROS_INFO("Start Gripper Trajectory");
}

void PositionController::gripOff(void)
{
  Eigen::VectorXd initial_position = Eigen::VectorXd::Zero(MAX_JOINT_NUM + MAX_GRIP_JOINT_NUM);
  Eigen::VectorXd target_position  = Eigen::VectorXd::Zero(MAX_JOINT_NUM + MAX_GRIP_JOINT_NUM);

  for (int it = 0; it < MAX_JOINT_NUM; it++)
  {
    initial_position(it)  = goal_joint_position_(it);
    target_position(it)   = goal_joint_position_(it);
  }

  initial_position(GRIPPER)  = goal_gripper_position_(0);
  target_position(GRIPPER)   = goal_gripper_position_(0);

  target_position(GRIPPER) = 0.0 * DEGREE2RADIAN;

  move_time_ = 3.0;
  gripper_    = true;
  calculateGoalTrajectory(initial_position, target_position);

  ROS_INFO("Start Gripper Trajectory");
}

void PositionController::calculateGoalTrajectory(Eigen::VectorXd initial_position, Eigen::VectorXd target_position)
{
  /* set movement time */
  all_time_steps_ = int(floor((move_time_ / ITERATION_TIME) + 1.0));
  move_time_ = double(all_time_steps_ - 1) * ITERATION_TIME;

  goal_trajectory_.resize(all_time_steps_, MAX_JOINT_NUM + MAX_GRIP_JOINT_NUM);

  /* calculate gripper trajectory */
  for (int index = 0; index < MAX_JOINT_NUM + MAX_GRIP_JOINT_NUM; index++)
  {
    double init_position_value = initial_position(index);
    double target_position_value = target_position(index);

    Eigen::MatrixXd trajectory =
        robotis_framework::calcMinimumJerkTra(init_position_value, 0.0, 0.0,
                                              target_position_value, 0.0, 0.0,
                                              ITERATION_TIME, move_time_);

    // Block of size (p,q), starting at (i,j)
    // block(i,j,p,q)
    goal_trajectory_.block(0, index, all_time_steps_, 1) = trajectory;
  }

  step_cnt_   = 0;
  is_moving_  = true;
}

void PositionController::moveGroupActionFeedbackMsgCallback(const moveit_msgs::MoveGroupActionFeedback::ConstPtr &msg)
{
  if (is_moving_ == false && msg->feedback.state == "MONITOR")
  {
    moveit_execution_ = true;
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

  trajectory_generate_thread_ = new boost::thread(boost::bind(&PositionController::moveItTragectoryGenerateThread, this));
  delete trajectory_generate_thread_;
}

void PositionController::moveItTragectoryGenerateThread()
{
  std::vector<double> via_time;

  for (int _tra_index = 0; _tra_index < motionPlanningTool_->moveit_msg_.trajectory.size(); _tra_index++)
  {
    motionPlanningTool_->points_ = motionPlanningTool_->moveit_msg_.trajectory[_tra_index].joint_trajectory.points.size();

    motionPlanningTool_->display_planned_path_positions_.resize(motionPlanningTool_->points_, MAX_JOINT_NUM);
    motionPlanningTool_->display_planned_path_velocities_.resize(motionPlanningTool_->points_, MAX_JOINT_NUM);
    motionPlanningTool_->display_planned_path_accelerations_.resize(motionPlanningTool_->points_, MAX_JOINT_NUM);

    for (int _point_index = 0; _point_index < motionPlanningTool_->points_; _point_index++)
    {
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

  all_time_steps_ = motionPlanningTool_->points_;

  ros::Duration seconds(0.5);
  seconds.sleep();


  goal_trajectory_ = motionPlanningTool_->display_planned_path_positions_;
  ROS_INFO("Get Joint Trajectory");

  if (moveit_execution_ == true)
  {
    is_moving_    = true;
    step_cnt_     = 0;

    moveit_execution_ = false;

    ROS_INFO("Send Motion Trajectory");
  }
}

void PositionController::gazeboPresentJointPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  uint8_t gripper_joint_num = 0;

  gripper_joint_num = MAX_GRIP_JOINT_NUM + 1;

 for (int index = gripper_joint_num; index < MAX_JOINT_NUM + gripper_joint_num; index++)
 {
   present_joint_position_(index - gripper_joint_num) = msg->position.at(index);
 }

 for (int index = 0; index < MAX_GRIP_JOINT_NUM; index++)
 {
   present_joint_position_(index + MAX_JOINT_NUM) = msg->position.at(index);
 }
}

void PositionController::presentJointPositionMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  for (int index = 0; index < MAX_JOINT_NUM + MAX_GRIP_JOINT_NUM; index++)
  {
    present_joint_position_(index) = msg->position.at(index);
  }
}

void PositionController::gripperPositionMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "grip_on")
  {
    gripOn();
  }
  else if (msg->data == "grip_off")
  {
    gripOff();
  }
  else
  {
    ROS_ERROR("If you want to grip or release something, publish 'grip_on' or 'grip_off'");
  }
}

void PositionController::jointPositionMsgCallback(const open_manipulator_msgs::JointPose::ConstPtr &msg)
{
  static bool check = true;

  if (check)
  {
    getPresentPosition();
    check = false;
  }

  Eigen::VectorXd initial_position = Eigen::VectorXd::Zero(MAX_JOINT_NUM + MAX_GRIP_JOINT_NUM);
  Eigen::VectorXd target_position  = Eigen::VectorXd::Zero(MAX_JOINT_NUM + MAX_GRIP_JOINT_NUM);

  for (int it = 0; it < MAX_JOINT_NUM; it++)
  {
    initial_position(it)  = goal_joint_position_(it);
    target_position(it)   = goal_joint_position_(it);
  }

  initial_position(GRIPPER)  = goal_gripper_position_(0);
  target_position(GRIPPER)   = goal_gripper_position_(0);

  for (int it = 0; it < msg->joint_name.size(); it++)
  {
    target_position(joint_id_[msg->joint_name[it]]-1) = msg->position[it];
  }

  move_time_ = msg->move_time;
  calculateGoalTrajectory(initial_position, target_position);

  ROS_INFO("Start Joint Trajectory");
}

void PositionController::process(void)
{
  // Get Joint & Gripper State
  // present_joint_position, present_gripper_position

  if (is_moving_)
  {
    if (gripper_)
    {
      goal_gripper_position_(0) = goal_trajectory_(step_cnt_, GRIPPER);
    }
    else
    {
      for (int index = 0; index < MAX_JOINT_NUM; index++)
      {
        goal_joint_position_(index) = goal_trajectory_(step_cnt_, index);
      }
    }

    step_cnt_++;
  }

  sensor_msgs::JointState send_to_joint_position;

  for (std::map<std::string, uint8_t>::iterator state_iter = joint_id_.begin();
       state_iter != joint_id_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    send_to_joint_position.name.push_back(joint_name);
    send_to_joint_position.position.push_back(goal_joint_position_(joint_id_[joint_name]-1));
  }

  std::string gripper_name = "grip_joint";
  send_to_joint_position.name.push_back(gripper_name);
  send_to_joint_position.position.push_back(goal_gripper_position_(0));

  if (is_moving_)
  {
    if (using_gazebo_)
    {
      for (int id = 1; id <= MAX_JOINT_NUM; id++)
      {
        std_msgs::Float64 joint_position;
        joint_position.data = send_to_joint_position.position.at(id-1);
        gazebo_goal_joint_position_pub_[id-1].publish(joint_position);
      }

      std_msgs::Float64 gripper_position;
      gripper_position.data = send_to_joint_position.position.at(4) * 0.01;
      gazebo_gripper_position_pub_[LEFT_GRIP].publish(gripper_position);
      gazebo_gripper_position_pub_[RIGHT_GRIP].publish(gripper_position);
    }
    else
    {
      goal_joint_position_pub_.publish(send_to_joint_position);
    }
  }

  if (is_moving_)
  {
    if (step_cnt_ >= all_time_steps_)
    {
      is_moving_ = false;
      step_cnt_  = 0;
      gripper_   = false;

      ROS_INFO("End Trajectory");
    }
  }

  std_msgs::Bool get_moving;
  get_moving.data = is_moving_;
  moving_pub_.publish(get_moving);
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
