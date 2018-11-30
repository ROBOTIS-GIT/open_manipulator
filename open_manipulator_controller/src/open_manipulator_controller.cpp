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

OM_CONTROLLER::OM_CONTROLLER()
    :node_handle_(""),
     priv_node_handle_("~"),
     tool_ctrl_flag_(false),
     timer_thread_flag_(false),
     using_platform_(false),
     tool_position_(0.0)
{
  robot_name_             = node_handle_.param<std::string>("robot_name", "open_manipulator");
  std::string usb_port    = priv_node_handle_.param<std::string>("usb_port", "/dev/ttyUSB0");
  std::string baud_rate   = priv_node_handle_.param<std::string>("baud_rate", "1000000");

  using_platform_ = priv_node_handle_.param<bool>("using_platform", false);

  initPublisher();
  initSubscriber();

  open_manipulator_.initManipulator(using_platform_, usb_port, baud_rate);

  setTimerThread();
  startTimerThread();
  RM_LOG::INFO("Successed to OpenManipulator initialization");
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
    next_time.tv_sec += (next_time.tv_nsec + ACTUATOR_CONTROL_TIME_MSEC * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + ACTUATOR_CONTROL_TIME_MSEC * 1000000) % 1000000000;

    double time = next_time.tv_sec + (next_time.tv_nsec*0.000000001);
    controller->process(time);

    clock_gettime(CLOCK_MONOTONIC, &curr_time);

    /////
    double delta_nsec = (next_time.tv_sec - curr_time.tv_sec) + (next_time.tv_nsec - curr_time.tv_nsec)*0.000000001;
    //RM_LOG::INFO("control time : ", ACTUATOR_CONTROL_TIME - delta_nsec);
    if(delta_nsec < 0.0)
    {
      RM_LOG::WARN("Over the control time :", ACTUATOR_CONTROL_TIME - delta_nsec);
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
  // msg publisher
  open_manipulator_state_pub_ = node_handle_.advertise<open_manipulator_msgs::OpenManipulatorState>(robot_name_ + "/states", 10);
  open_manipulator_kinematics_pose_pub_  = node_handle_.advertise<open_manipulator_msgs::KinematicsPose>(robot_name_ + "/kinematics_pose", 10);
  if(using_platform_)
    open_manipulator_joint_states_pub_  = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/joint_states", 10);
  else
  {
    open_manipulator_joint_states_to_gazebo_pub_[0] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/joint1_position/command", 10);
    open_manipulator_joint_states_to_gazebo_pub_[1] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/joint2_position/command", 10);
    open_manipulator_joint_states_to_gazebo_pub_[2] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/joint3_position/command", 10);
    open_manipulator_joint_states_to_gazebo_pub_[3] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/joint4_position/command", 10);
    open_manipulator_gripper_states_to_gazebo_pub_[0] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/grip_joint_position/command", 10);
    open_manipulator_gripper_states_to_gazebo_pub_[1] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/grip_joint_sub_position/command", 10);
  }
}
void OM_CONTROLLER::initSubscriber()
{
  // msg subscriber
  open_manipulator_option_client_ = node_handle_.subscribe(robot_name_ + "/option", 10, &OM_CONTROLLER::printManipulatorSettingCallback, this);
  // service server
  goal_joint_space_path_server_ = node_handle_.advertiseService(robot_name_ + "/goal_joint_space_path", &OM_CONTROLLER::goalJointSpacePathCallback, this);
  goal_task_space_path_server_ =  node_handle_.advertiseService(robot_name_ + "/goal_task_space_path", &OM_CONTROLLER::goalTaskSpacePathCallback, this);
  goal_joint_space_path_to_present_server_ = node_handle_.advertiseService(robot_name_ + "/goal_joint_space_path_to_present", &OM_CONTROLLER::goalJointSpacePathToPresentCallback, this);
  goal_task_space_path_to_present_server_ =  node_handle_.advertiseService(robot_name_ + "/goal_task_space_path_to_present", &OM_CONTROLLER::goalTaskSpacePathToPresentCallback, this);
  goal_tool_control_server_ =      node_handle_.advertiseService(robot_name_ + "/goal_tool_control", &OM_CONTROLLER::goalToolControlCallback, this);
  set_actuator_state_server_ =     node_handle_.advertiseService(robot_name_ + "/set_actuator_state", &OM_CONTROLLER::setActuatorStateCallback, this);
  goal_drawing_trajectory_server_= node_handle_.advertiseService(robot_name_ + "/goal_drawing_trajectory", &OM_CONTROLLER::goalDrawingTrajectoryCallback, this);

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

  res.isPlanned = true;
}
bool OM_CONTROLLER::goalTaskSpacePathCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                              open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Pose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;
  open_manipulator_.taskTrajectoryMove("tool", target_pose.position, req.path_time);

  res.isPlanned = true;
}

bool OM_CONTROLLER::goalJointSpacePathToPresentCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                                        open_manipulator_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));

  open_manipulator_.jointTrajectoryMoveToPresentValue(target_angle, req.path_time);

  res.isPlanned = true;
}
bool OM_CONTROLLER::goalTaskSpacePathToPresentCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                      open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Pose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator_.taskTrajectoryMoveToPresentPosition("tool", target_pose.position, req.path_time);

  res.isPlanned = true;
}
bool OM_CONTROLLER::goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                            open_manipulator_msgs::SetJointPosition::Response &res)
{
  open_manipulator_.toolMove("tool", req.joint_position.position.at(0));
  res.isPlanned = true;
}

bool OM_CONTROLLER::setActuatorStateCallback(open_manipulator_msgs::SetActuatorState::Request  &req,
                                             open_manipulator_msgs::SetActuatorState::Response &res)
{
  if(req.setActuatorState == true) // torque on
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

  res.isPlanned = true;
}

bool OM_CONTROLLER::goalDrawingTrajectoryCallback(open_manipulator_msgs::SetDrawingTrajectory::Request  &req,
                                                  open_manipulator_msgs::SetDrawingTrajectory::Response &res)
{
  try
  {
    if(req.drawingTrajectoryName == "circle")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];  // radius (m)
      draw_circle_arg[1] = req.param[1];  // revolution (rev)
      draw_circle_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;
      open_manipulator_.drawingTrajectoryMove(DRAWING_CIRCLE, "tool", p_draw_circle_arg, req.path_time);

    }
    else if(req.drawingTrajectoryName == "line")
    {
      Pose present_pose = open_manipulator_.getPose("tool");
      WayPoint draw_goal_pose[6];
      draw_goal_pose[0].value = present_pose.position(0) + req.param[0];
      draw_goal_pose[1].value = present_pose.position(1) + req.param[1];
      draw_goal_pose[2].value = present_pose.position(2) + req.param[2];
      draw_goal_pose[3].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[0];
      draw_goal_pose[4].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[1];
      draw_goal_pose[5].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[2];

      void *p_draw_goal_pose = &draw_goal_pose;
      open_manipulator_.drawingTrajectoryMove(DRAWING_LINE, "tool", p_draw_goal_pose, req.path_time);
    }
    else if(req.drawingTrajectoryName == "rhombus")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];  // radius (m)
      draw_circle_arg[1] = req.param[1];  // revolution (rev)
      draw_circle_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;
      open_manipulator_.drawingTrajectoryMove(DRAWING_RHOMBUS, "tool", p_draw_circle_arg, req.path_time);
    }
    else if(req.drawingTrajectoryName == "heart")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];  // radius (m)
      draw_circle_arg[1] = req.param[1];  // revolution (rev)
      draw_circle_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;
      open_manipulator_.drawingTrajectoryMove(DRAWING_HEART, "tool", p_draw_circle_arg, req.path_time);
    }
    res.isPlanned = true;
  }
  catch ( ros::Exception &e )
  {
    RM_LOG::ERROR("Creation the drawing trajectory is failed!");
  }
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

  Pose pose = open_manipulator_.getPose("tool");
  msg.pose.position.x = pose.position[0];
  msg.pose.position.y = pose.position[1];
  msg.pose.position.z = pose.position[2];
  open_manipulator_kinematics_pose_pub_.publish(msg);
}

void OM_CONTROLLER::publishJointStates()
{
  if(using_platform_)
  {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    std::vector<WayPoint> jointValue = open_manipulator_.getAllActiveJointValue();
    double tool_value = open_manipulator_.getToolValue("tool");
    msg.name.push_back("joint1");           msg.position.push_back(jointValue.at(0).value);
                                            msg.velocity.push_back(jointValue.at(0).velocity);
                                            msg.effort.push_back(jointValue.at(0).effort);

    msg.name.push_back("joint2");           msg.position.push_back(jointValue.at(1).value);
                                            msg.velocity.push_back(jointValue.at(1).velocity);
                                            msg.effort.push_back(jointValue.at(1).effort);

    msg.name.push_back("joint3");           msg.position.push_back(jointValue.at(2).value);
                                            msg.velocity.push_back(jointValue.at(2).velocity);
                                            msg.effort.push_back(jointValue.at(2).effort);

    msg.name.push_back("joint4");           msg.position.push_back(jointValue.at(3).value);
                                            msg.velocity.push_back(jointValue.at(3).velocity);
                                            msg.effort.push_back(jointValue.at(3).effort);

    msg.name.push_back("grip_joint");       msg.position.push_back(tool_value);
                                            msg.velocity.push_back(0.0);
                                            msg.effort.push_back(0.0);

    msg.name.push_back("grip_joint_sub");   msg.position.push_back(tool_value);
                                            msg.velocity.push_back(0.0);
                                            msg.effort.push_back(0.0);
    open_manipulator_joint_states_pub_.publish(msg);
  }
  else // gazebo
  {
    std::vector<WayPoint> jointValue = open_manipulator_.getAllActiveJointValue();
    for(int i = 0; i < jointValue.size(); i ++)
    {
      std_msgs::Float64 msg;
      msg.data = jointValue.at(i).value;
      open_manipulator_joint_states_to_gazebo_pub_[i].publish(msg);
    }

    double tool_value = open_manipulator_.getToolValue("tool");
    for(int i = 0; i < 2; i ++)
    {
      std_msgs::Float64 msg;
      msg.data = tool_value;
      open_manipulator_gripper_states_to_gazebo_pub_[i].publish(msg);
    }
  }
}


void OM_CONTROLLER::process(double time)
{
  open_manipulator_.openManipulatorProcess(time);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_controller");

  OM_CONTROLLER om_controller;
  ros::Rate loop_rate(ITERATION_FREQUENCY);

  while (ros::ok())
  {
    om_controller.publishOpenManipulatorStates();
    om_controller.publishJointStates();
    om_controller.publishKinematicsPose();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
