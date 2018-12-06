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

#include "open_manipulator_controller/open_manipulator_controller.h"

using namespace open_manipulator_controller;

OM_CONTROLLER::OM_CONTROLLER(std::string usb_port, std::string baud_rate)
    :node_handle_(""),
     priv_node_handle_("~"),
     tool_ctrl_flag_(false),
     timer_thread_flag_(false),
     using_platform_(false),
     using_moveit_(false),
     control_period_(0.010),
     tool_position_(0.0)
{
  control_period_ = priv_node_handle_.param<double>("control_period", 0.010f);
  using_platform_ = priv_node_handle_.param<bool>("using_platform", false);
  using_moveit_ = priv_node_handle_.param<bool>("using_moveit", false);

  open_manipulator_.initManipulator(using_platform_, usb_port, baud_rate);

  if (using_platform_ == true)    ROS_INFO("Succeeded to init %s", priv_node_handle_.getNamespace().c_str());
  else if (using_platform_ == false)    ROS_INFO("Ready to simulate %s on Gazebo", priv_node_handle_.getNamespace().c_str());
}

OM_CONTROLLER::~OM_CONTROLLER()
{
  timer_thread_flag_ = false;
  usleep(100 * 1000); // 100ms
  RM_LOG::INFO("Shutdown the OpenManipulator");
  open_manipulator_.allActuatorDisable();
  ros::shutdown();
}

void OM_CONTROLLER::setTimerThread()
{
  int error;
  struct sched_param param;
  pthread_attr_init(&attr_);

  error = pthread_attr_setschedpolicy(&attr_, SCHED_RR);
  if (error != 0)
    RM_LOG::ERROR("pthread_attr_setschedpolicy error = ", (double)error);
  error = pthread_attr_setinheritsched(&attr_, PTHREAD_EXPLICIT_SCHED);
  if (error != 0)
    RM_LOG::ERROR("pthread_attr_setinheritsched error = ", (double)error);

  memset(&param, 0, sizeof(param));
  param.sched_priority = 31;    // RT
  error = pthread_attr_setschedparam(&attr_, &param);
  if (error != 0)
    RM_LOG::ERROR("pthread_attr_setschedparam error = ", (double)error);
}
void OM_CONTROLLER::startTimerThread()
{
  int error;
  if ((error = pthread_create(&this->timer_thread_, /*&attr_*/NULL, this->timerThread, this)) != 0)
  {
    RM_LOG::ERROR("Creating timer thread failed!!", (double)error);
    exit(-1);
  }
  timer_thread_flag_ = true;
}

void *OM_CONTROLLER::timerThread(void *param)
{
  OM_CONTROLLER *controller = (OM_CONTROLLER *) param;
  static struct timespec next_time;
  static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(controller->timer_thread_flag_)
  {
    next_time.tv_sec += (next_time.tv_nsec + ((int)controller->getControlPeriod() * 1000) * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + ((int)controller->getControlPeriod() * 1000) * 1000000) % 1000000000;

    double time = next_time.tv_sec + (next_time.tv_nsec*0.000000001);
    controller->process(time);

    clock_gettime(CLOCK_MONOTONIC, &curr_time);

    /////
    double delta_nsec = (next_time.tv_sec - curr_time.tv_sec) + (next_time.tv_nsec - curr_time.tv_nsec)*0.000000001;
//    RM_LOG::INFO("control time : %f", controller->getControlPeriod() - delta_nsec);
    if(delta_nsec < 0.0)
    {
//      RM_LOG::WARN("Over the control time : %f", controller->getControlPeriod() - delta_nsec);
      next_time = curr_time;
    }
    else
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    /////
  }

  return 0;
}

void OM_CONTROLLER::initPublisher()
{
  auto opm_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  for (auto const& name:opm_tools_name)
  {
    ros::Publisher pb;
    pb = priv_node_handle_.advertise<open_manipulator_msgs::KinematicsPose>(name + "/kinematics_pose", 10);
    open_manipulator_kinematics_pose_pub_.push_back(pb);
  }
  open_manipulator_state_pub_ = priv_node_handle_.advertise<open_manipulator_msgs::OpenManipulatorState>("states", 10);

  if(using_platform_ == true)
  {
    open_manipulator_joint_states_pub_ = priv_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
  }
  else
  {
    auto gazebo_joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();

    gazebo_joints_name.reserve(gazebo_joints_name.size() + opm_tools_name.size());

    gazebo_joints_name.insert(gazebo_joints_name.end(),
                            opm_tools_name.begin(),
                            opm_tools_name. end());

    for (auto const& name:gazebo_joints_name)
    {
      ros::Publisher pb;
      pb = priv_node_handle_.advertise<std_msgs::Float64>(name + "_position/command", 10);
      gazebo_goal_joint_position_pub_.push_back(pb);
    }
  }

  if (using_moveit_ == true)
  {

  }
}
void OM_CONTROLLER::initSubscriber()
{
  // msg subscriber
  open_manipulator_option_sub_ = priv_node_handle_.subscribe("option", 10, &OM_CONTROLLER::printManipulatorSettingCallback, this);
}

void OM_CONTROLLER::initServer()
{
  goal_joint_space_path_server_             = priv_node_handle_.advertiseService("goal_joint_space_path", &OM_CONTROLLER::goalJointSpacePathCallback, this);
  goal_task_space_path_server_              = priv_node_handle_.advertiseService("goal_task_space_path", &OM_CONTROLLER::goalTaskSpacePathCallback, this);
  goal_joint_space_path_to_present_server_  = priv_node_handle_.advertiseService("goal_joint_space_path_to_present", &OM_CONTROLLER::goalJointSpacePathToPresentCallback, this);
  goal_task_space_path_to_present_server_   = priv_node_handle_.advertiseService("goal_task_space_path_to_present", &OM_CONTROLLER::goalTaskSpacePathToPresentCallback, this);
  goal_tool_control_server_                 = priv_node_handle_.advertiseService("goal_tool_control", &OM_CONTROLLER::goalToolControlCallback, this);
  set_actuator_state_server_                = priv_node_handle_.advertiseService("set_actuator_state", &OM_CONTROLLER::setActuatorStateCallback, this);
  goal_drawing_trajectory_server_           = priv_node_handle_.advertiseService("goal_drawing_trajectory", &OM_CONTROLLER::goalDrawingTrajectoryCallback, this);
}

void OM_CONTROLLER::printManipulatorSettingCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "print_open_manipulator_setting")
    open_manipulator_.checkManipulatorSetting();
}

bool OM_CONTROLLER::goalJointSpacePathCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                               open_manipulator_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));

  open_manipulator_.jointTrajectoryMove(target_angle, req.path_time);

  res.is_planned = true;
  return true;
}
bool OM_CONTROLLER::goalTaskSpacePathCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                              open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Pose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                        req.kinematics_pose.pose.orientation.x,
                        req.kinematics_pose.pose.orientation.y,
                        req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = RM_MATH::convertQuaternionToRotation(q);
  open_manipulator_.taskTrajectoryMove(req.end_effector_name, target_pose, req.path_time);

  res.is_planned = true;
  return true;
}

bool OM_CONTROLLER::goalJointSpacePathToPresentCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                                        open_manipulator_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));

  open_manipulator_.jointTrajectoryMoveToPresentValue(target_angle, req.path_time);

  res.is_planned = true;
  return true;
}
bool OM_CONTROLLER::goalTaskSpacePathToPresentCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                      open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Pose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator_.taskTrajectoryMoveToPresentPosition(req.planning_group, target_pose.position, req.path_time);

  res.is_planned = true;
  return true;
}
bool OM_CONTROLLER::goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                            open_manipulator_msgs::SetJointPosition::Response &res)
{
  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
  {
    open_manipulator_.toolMove(req.joint_position.joint_name.at(i), req.joint_position.position.at(i));
  }
  res.is_planned = true;
  return true;
}

bool OM_CONTROLLER::setActuatorStateCallback(open_manipulator_msgs::SetActuatorState::Request  &req,
                                             open_manipulator_msgs::SetActuatorState::Response &res)
{
  if(req.set_actuator_state == true) // torque on
  {
    RM_LOG::INFO("Wait a second for actuator enable");
    timer_thread_flag_ = false;
    usleep(100 * 1000); // 100ms
    open_manipulator_.allActuatorEnable();
    startTimerThread();
  }
  else // torque off
  {
    RM_LOG::INFO("Wait a second for actuator disable");
    timer_thread_flag_ = false;
    usleep(100 * 1000); // 100ms
    open_manipulator_.allActuatorDisable();
    startTimerThread();
  }

  res.is_planned = true;
  return true;
}

bool OM_CONTROLLER::goalDrawingTrajectoryCallback(open_manipulator_msgs::SetDrawingTrajectory::Request  &req,
                                                  open_manipulator_msgs::SetDrawingTrajectory::Response &res)
{
  try
  {
    if(req.drawing_trajectory_name == "circle")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];  // radius (m)
      draw_circle_arg[1] = req.param[1];  // revolution (rev)
      draw_circle_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;
      open_manipulator_.drawingTrajectoryMove(DRAWING_CIRCLE, req.end_effector_name, p_draw_circle_arg, req.path_time);

    }
    else if(req.drawing_trajectory_name == "line")
    {
      Pose present_pose = open_manipulator_.getPose(req.end_effector_name);
      WayPoint draw_goal_pose[6];
      draw_goal_pose[0].value = present_pose.position(0) + req.param[0];
      draw_goal_pose[1].value = present_pose.position(1) + req.param[1];
      draw_goal_pose[2].value = present_pose.position(2) + req.param[2];
      draw_goal_pose[3].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[0];
      draw_goal_pose[4].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[1];
      draw_goal_pose[5].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[2];

      void *p_draw_goal_pose = &draw_goal_pose;
      open_manipulator_.drawingTrajectoryMove(DRAWING_LINE, req.end_effector_name, p_draw_goal_pose, req.path_time);
    }
    else if(req.drawing_trajectory_name == "rhombus")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];  // radius (m)
      draw_circle_arg[1] = req.param[1];  // revolution (rev)
      draw_circle_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;
      open_manipulator_.drawingTrajectoryMove(DRAWING_RHOMBUS, req.end_effector_name, p_draw_circle_arg, req.path_time);
    }
    else if(req.drawing_trajectory_name == "heart")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];  // radius (m)
      draw_circle_arg[1] = req.param[1];  // revolution (rev)
      draw_circle_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;
      open_manipulator_.drawingTrajectoryMove(DRAWING_HEART, req.end_effector_name, p_draw_circle_arg, req.path_time);
    }
    res.is_planned = true;
    return true;
  }
  catch ( ros::Exception &e )
  {
    RM_LOG::ERROR("Creation the drawing trajectory is failed!");
  }

  return true;
}

void OM_CONTROLLER::publishOpenManipulatorStates()
{
  open_manipulator_msgs::OpenManipulatorState msg;
  if(open_manipulator_.isMoving())
    msg.open_manipulator_moving_state = msg.IS_MOVING;
  else
    msg.open_manipulator_moving_state = msg.STOPPED;

  if(open_manipulator_.isEnabled(JOINT_DYNAMIXEL))
    msg.open_manipulator_actuator_state = msg.ACTUATOR_ENABLED;
  else
    msg.open_manipulator_actuator_state = msg.ACTUATOR_DISABLED;

  open_manipulator_state_pub_.publish(msg);
}


void OM_CONTROLLER::publishKinematicsPose()
{
  open_manipulator_msgs::KinematicsPose msg;
  auto opm_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  uint8_t index = 0;
  for (auto const& tools:opm_tools_name)
  {
    Pose pose = open_manipulator_.getPose(tools);
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond orientation = RM_MATH::convertRotationToQuaternion(pose.orientation);
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

    open_manipulator_kinematics_pose_pub_.at(index).publish(msg);
    index++;
  }
}

void OM_CONTROLLER::publishJointStates()
{
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();

  auto joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  auto tool_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  auto joint_value = open_manipulator_.getAllActiveJointValue();
  auto tool_value = open_manipulator_.getAllToolValue();

  for(uint8_t i = 0; i < joints_name.size(); i ++)
  {
    msg.name.push_back(joints_name.at(i));

    msg.position.push_back(joint_value.at(i).value);
    msg.velocity.push_back(joint_value.at(i).velocity);
    msg.effort.push_back(joint_value.at(i).effort);
  }

  for(uint8_t i = 0; i < tool_name.size(); i ++)
  {
    msg.name.push_back(tool_name.at(i));

    msg.position.push_back(tool_value.at(i));
    msg.velocity.push_back(0.0f);
    msg.effort.push_back(0.0f);
  }


//  msg.name.push_back("joint1");           msg.position.push_back(jointValue.at(0).value);
//                                          msg.velocity.push_back(jointValue.at(0).velocity);
//                                          msg.effort.push_back(jointValue.at(0).effort);

//  msg.name.push_back("joint2");           msg.position.push_back(jointValue.at(1).value);
//                                          msg.velocity.push_back(jointValue.at(1).velocity);
//                                          msg.effort.push_back(jointValue.at(1).effort);

//  msg.name.push_back("joint3");           msg.position.push_back(jointValue.at(2).value);
//                                          msg.velocity.push_back(jointValue.at(2).velocity);
//                                          msg.effort.push_back(jointValue.at(2).effort);

//  msg.name.push_back("joint4");           msg.position.push_back(jointValue.at(3).value);
//                                          msg.velocity.push_back(jointValue.at(3).velocity);
//                                          msg.effort.push_back(jointValue.at(3).effort);

//  msg.name.push_back("gripper");          msg.position.push_back(tool_value);
//                                          msg.velocity.push_back(0.0);
//                                          msg.effort.push_back(0.0);

//  msg.name.push_back("grip_joint_sub");   msg.position.push_back(tool_value);
//                                          msg.velocity.push_back(0.0);
//                                          msg.effort.push_back(0.0);
  open_manipulator_joint_states_pub_.publish(msg);
}

void OM_CONTROLLER::publishGazeboCommand()
{
  std::vector<WayPoint> joint_value = open_manipulator_.getAllActiveJointValue();
  std::vector<double> tool_value = open_manipulator_.getAllToolValue();

  for(uint8_t i = 0; i < joint_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = joint_value.at(i).value;

    gazebo_goal_joint_position_pub_.at(i).publish(msg);
  }

  for(uint8_t i = 0; i < tool_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = tool_value.at(i);

    gazebo_goal_joint_position_pub_.at(joint_value.size() + i).publish(msg);
  }
}

void OM_CONTROLLER::publishCallback(const ros::TimerEvent&)
{
  if (using_platform_ == true)  publishJointStates();
  else  publishGazeboCommand();

  publishOpenManipulatorStates();
  publishKinematicsPose();
}


void OM_CONTROLLER::process(double time)
{
  open_manipulator_.openManipulatorProcess(time);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_controller");
  ros::NodeHandle node_handle("");

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc < 3)
  {
    ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  }
  else
  {
    usb_port = argv[1];
    baud_rate = argv[2];
  }

  OM_CONTROLLER om_controller(usb_port, baud_rate);

  om_controller.initPublisher();
  om_controller.initSubscriber();
  om_controller.initServer();

  om_controller.setTimerThread();
  om_controller.startTimerThread();

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(om_controller.getControlPeriod()), &OM_CONTROLLER::publishCallback, &om_controller);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
