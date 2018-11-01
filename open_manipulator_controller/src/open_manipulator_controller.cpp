
#include "open_manipulator_controller/open_manipulator_controller.h"

using namespace open_manipulator_controller;

OM_CONTROLLER::OM_CONTROLLER()
    :node_handle_(""),
     priv_node_handle_("~"),
     toolCtrlFlag(false),
     toolPosition(0.0)
{
  robot_name_   = priv_node_handle_.param<std::string>("robot_name", "open_manipulator");

  initPublisher();
  initSubscriber();

  chain.initManipulator();

  setTimerThread();
  ROS_INFO("OpenManipulator initialization");
}

OM_CONTROLLER::~OM_CONTROLLER()
{
  chain.actuatorDisable(JOINT_DYNAMIXEL);
  chain.actuatorDisable(TOOL_DYNAMIXEL);
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
}


void *OM_CONTROLLER::timerThread(void *param)
{
  OM_CONTROLLER *controller = (OM_CONTROLLER *) param;
  static struct timespec next_time;
  static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(1)
  {
    next_time.tv_sec += (next_time.tv_nsec + ACTUATOR_CONTROL_TIME_MSEC * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + ACTUATOR_CONTROL_TIME_MSEC * 1000000) % 1000000000;

    double time = next_time.tv_sec + (next_time.tv_nsec*0.000000001);
    controller->process(time);

    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    long delta_nsec = (next_time.tv_sec - curr_time.tv_sec) * 1000000000 + (next_time.tv_nsec - curr_time.tv_nsec);
    /////
    //double current_time = curr_time.tv_sec + (curr_time.tv_nsec*0.000000001);
    //ROS_INFO("%lf", current_time);
    /////
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }

  return 0;
}

void OM_CONTROLLER::initPublisher()
{
  // msg publisher
  chain_kinematics_pose_pub_  = node_handle_.advertise<open_manipulator_msgs::KinematicsPose>(robot_name_ + "/chain_kinematics_pose", 10);
  chain_joint_states_pub_  = node_handle_.advertise<open_manipulator_msgs::JointPosition>(robot_name_ + "/chain_joint_states", 10);
}
void OM_CONTROLLER::initSubscriber()
{
  // service server
  goal_joint_space_path_server_ = node_handle_.advertiseService(robot_name_ + "/goal_joint_space_path", &OM_CONTROLLER::goalJointSpacePathCallback, this);
  goal_task_space_path_server_ = node_handle_.advertiseService(robot_name_ + "/goal_task_space_path", &OM_CONTROLLER::goalTaskSpacePathCallback, this);
  goal_tool_control_server_ = node_handle_.advertiseService(robot_name_ + "/goal_tool_control", &OM_CONTROLLER::goalToolControlCallback, this);
}


bool OM_CONTROLLER::goalJointSpacePathCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                               open_manipulator_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;
  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));
  //chain.setJointTrajectory(target_angle,req.path_time);

  res.isPlanned = true;
  return true;
}
bool OM_CONTROLLER::goalTaskSpacePathCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                              open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Pose target_pose;
  target_pose.position(0) = req.kinematics_pose.pose.position.x;
  target_pose.position(1) = req.kinematics_pose.pose.position.y;
  target_pose.position(2) = req.kinematics_pose.pose.position.z;
  //target_pose.orientation = chain.getComponentOrientationToWorld(TOOL);
  //chain.setTaskTrajectory(TOOL, target_pose, req.path_time);

  res.isPlanned = true;
  return true;
}
bool OM_CONTROLLER::goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                            open_manipulator_msgs::SetJointPosition::Response &res)
{
  toolCtrlFlag = true;
  toolPosition = req.joint_position.position.at(0);

  res.isPlanned = true;
  return true;
}

void OM_CONTROLLER::publishKinematicsPose()
{
  open_manipulator_msgs::KinematicsPose msg;

  Vector3f position;// = chain.getComponentPositionToWorld(TOOL);
  msg.pose.position.x = position[0];
  msg.pose.position.y = position[1];
  msg.pose.position.z = position[2];
  chain_kinematics_pose_pub_.publish(msg);
}

void OM_CONTROLLER::publishJointStates()
{
  open_manipulator_msgs::JointPosition msg;

  std::vector<double> position;// = chain.getAllActiveJointAngle();
  msg.position = position;
  chain_joint_states_pub_.publish(msg);
}


void OM_CONTROLLER::process(double time)
{
  chain.chainProcess(time);

  if(toolCtrlFlag)
  {
    //chain.toolMove(TOOL_DYNAMIXEL, TOOL, toolPosition);
    toolCtrlFlag = false;
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
