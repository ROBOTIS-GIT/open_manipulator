
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
  robot_name_             = priv_node_handle_.param<std::string>("robot_name", "open_manipulator");
  std::string usb_port    = priv_node_handle_.param<std::string>("usb_port", "/dev/ttyUSB0");
  std::string baud_rate   = priv_node_handle_.param<std::string>("baud_rate", "1000000");
  robot_name_             = priv_node_handle_.param<std::string>("robot_name", "open_manipulator");

  using_platform_ = priv_node_handle_.param<bool>("using_platform", false);

  initPublisher();
  initSubscriber();

  chain_.initManipulator(using_platform_, usb_port, baud_rate);

  setTimerThread();
  ROS_INFO("OpenManipulator initialization");
}

OM_CONTROLLER::~OM_CONTROLLER()
{
  timer_thread_flag_ = false;
  usleep(10 * 1000); // 10ms
  ROS_INFO("Shutdown the OpenManipulator");
  chain_.allActuatorDisable();
  ros::shutdown();
}

void OM_CONTROLLER::setTimerThread()
{
  int error;
  struct sched_param param;
  pthread_attr_t attr;
  pthread_attr_init(&attr);

  error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
  if (error != 0)
    ROS_ERROR("pthread_attr_setschedpolicy error = %d\n", error);
  error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (error != 0)
    ROS_ERROR("pthread_attr_setinheritsched error = %d\n", error);

  memset(&param, 0, sizeof(param));
  param.sched_priority = 31;    // RT
  error = pthread_attr_setschedparam(&attr, &param);
  if (error != 0)
    ROS_ERROR("pthread_attr_setschedparam error = %d\n", error);

  // create and start the thread
  if ((error = pthread_create(&this->timer_thread_, /*&attr*/NULL, this->timerThread, this)) != 0)
  {
    ROS_ERROR("Creating timer thread failed!! %d", error);
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
    // ROS_INFO("%lf", ACTUATOR_CONTROL_TIME - delta_nsec);
    if(delta_nsec < 0.0)
      next_time = curr_time;
    else
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    /////
  }

  return 0;
}

void OM_CONTROLLER::initPublisher()
{
  // msg publisher
  chain_kinematics_pose_pub_  = node_handle_.advertise<open_manipulator_msgs::KinematicsPose>(robot_name_ + "/kinematics_pose", 10);
  if(using_platform_)
    chain_joint_states_pub_  = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/joint_states", 10);
  else
  {
    chain_joint_states_to_gazebo_pub_[0] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/joint1_position/command", 10);
    chain_joint_states_to_gazebo_pub_[1] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/joint2_position/command", 10);
    chain_joint_states_to_gazebo_pub_[2] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/joint3_position/command", 10);
    chain_joint_states_to_gazebo_pub_[3] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/joint4_position/command", 10);
    chain_gripper_states_to_gazebo_pub_[0] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/grip_joint_position/command", 10);
    chain_gripper_states_to_gazebo_pub_[1] = node_handle_.advertise<std_msgs::Float64>(robot_name_ + "/grip_joint_sub_position/command", 10);
  }
}
void OM_CONTROLLER::initSubscriber()
{
  // service server
  goal_joint_space_path_server_ = node_handle_.advertiseService(robot_name_ + "/goal_joint_space_path", &OM_CONTROLLER::goalJointSpacePathCallback, this);
  goal_task_space_path_server_ = node_handle_.advertiseService(robot_name_ + "/goal_task_space_path", &OM_CONTROLLER::goalTaskSpacePathCallback, this);
  goal_joint_space_path_to_present_server_ = node_handle_.advertiseService(robot_name_ + "/goal_joint_space_path_to_present", &OM_CONTROLLER::goalJointSpacePathToPresentCallback, this);
  goal_task_space_path_to_present_server_ = node_handle_.advertiseService(robot_name_ + "/goal_task_space_path_to_present", &OM_CONTROLLER::goalTaskSpacePathToPresentCallback, this);
  goal_tool_control_server_ = node_handle_.advertiseService(robot_name_ + "/goal_tool_control", &OM_CONTROLLER::goalToolControlCallback, this);
}

bool OM_CONTROLLER::goalJointSpacePathCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                               open_manipulator_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));

  chain_.jointTrajectoryMove(target_angle, req.path_time);

  res.isPlanned = true;
  return true;
}
bool OM_CONTROLLER::goalTaskSpacePathCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                              open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Vector3d target_position;
  target_position[0] = req.kinematics_pose.pose.position.x;
  target_position[1] = req.kinematics_pose.pose.position.y;
  target_position[2] = req.kinematics_pose.pose.position.z;

  chain_.taskTrajectoryMove(TOOL, target_position, req.path_time);

  res.isPlanned = true;
  return true;
}

bool OM_CONTROLLER::goalJointSpacePathToPresentCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                                        open_manipulator_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));

  chain_.jointTrajectoryMoveToPresentValue(target_angle, req.path_time);

  res.isPlanned = true;
  return true;
}
bool OM_CONTROLLER::goalTaskSpacePathToPresentCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                      open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Vector3d target_position;
  target_position[0] = req.kinematics_pose.pose.position.x;
  target_position[1] = req.kinematics_pose.pose.position.y;
  target_position[2] = req.kinematics_pose.pose.position.z;

  chain_.taskTrajectoryMoveToPresentPosition(TOOL, target_position, req.path_time);

  res.isPlanned = true;
  return true;
}
bool OM_CONTROLLER::goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                            open_manipulator_msgs::SetJointPosition::Response &res)
{
  tool_position_ = req.joint_position.position.at(0);
  tool_ctrl_flag_ = true;

  res.isPlanned = true;
  return true;
}

void OM_CONTROLLER::publishKinematicsPose()
{
  open_manipulator_msgs::KinematicsPose msg;

  Vector3d position = chain_.getManipulator()->getComponentPositionToWorld(TOOL);
  msg.pose.position.x = position[0];
  msg.pose.position.y = position[1];
  msg.pose.position.z = position[2];
  chain_kinematics_pose_pub_.publish(msg);
}

void OM_CONTROLLER::publishJointStates()
{
  if(using_platform_)
  {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    std::vector<double> position, velocity, effort;
    chain_.getManipulator()->getAllActiveJointValue(&position, &velocity, &effort);
    double tool_value = -chain_.getManipulator()->getToolGoalValue(TOOL) * 0.01;
    msg.name.push_back("joint1");           msg.position.push_back(position.at(0));
                                            msg.velocity.push_back(velocity.at(0));
                                            msg.effort.push_back(effort.at(0));

    msg.name.push_back("joint2");           msg.position.push_back(position.at(1));
                                            msg.velocity.push_back(velocity.at(1));
                                            msg.effort.push_back(effort.at(1));

    msg.name.push_back("joint3");           msg.position.push_back(position.at(2));
                                            msg.velocity.push_back(velocity.at(2));
                                            msg.effort.push_back(effort.at(2));

    msg.name.push_back("joint4");           msg.position.push_back(position.at(3));
                                            msg.velocity.push_back(velocity.at(3));
                                            msg.effort.push_back(effort.at(3));

    msg.name.push_back("grip_joint");       msg.position.push_back(tool_value);
                                            msg.velocity.push_back(0.0);
                                            msg.effort.push_back(0.0);

    msg.name.push_back("grip_joint_sub");   msg.position.push_back(tool_value);
                                            msg.velocity.push_back(0.0);
                                            msg.effort.push_back(0.0);
    chain_joint_states_pub_.publish(msg);
  }
  else // gazebo
  {
    std::vector<double> value = chain_.getManipulator()->getAllActiveJointValue();
    for(int i = 0; i < value.size(); i ++)
    {
      std_msgs::Float64 msg;
      msg.data = value.at(i);
      chain_joint_states_to_gazebo_pub_[i].publish(msg);
    }
    double tool_value = -chain_.getManipulator()->getToolGoalValue(TOOL) * 0.01;
    for(int i = 0; i < 2; i ++)
    {
      std_msgs::Float64 msg;
      msg.data = tool_value;
      chain_gripper_states_to_gazebo_pub_[i].publish(msg);
    }
  }
}


void OM_CONTROLLER::process(double time)
{
  chain_.chainProcess(time);

  if(tool_ctrl_flag_)
  {
    chain_.toolMove(TOOL, tool_position_);
    tool_ctrl_flag_ = false;
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_controller");

  OM_CONTROLLER om_controller;
  ros::Rate loop_rate(ITERATION_FREQUENCY);

  ROS_INFO("OpenManipulator control loop start");
  while (ros::ok())
  {
    om_controller.publishJointStates();
    om_controller.publishKinematicsPose();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
