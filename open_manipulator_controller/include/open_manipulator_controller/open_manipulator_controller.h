#ifndef OPEN_MANIPULATOR_CONTROLLER_H
#define OPEN_MANIPULATOR_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <unistd.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include "open_manipulator_libs/om_chain.h"

#define ACTUATOR_CONTROL_TIME 0.010 // ms
#define ITERATION_FREQUENCY   100   // hz 10ms
#define ACTUATOR_CONTROL_TIME_MSEC 10 // ms

namespace open_manipulator_controller
{

class OM_CONTROLLER
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  ros::ServiceServer goal_joint_space_path_server_;
  ros::ServiceServer goal_task_space_path_server_;
  ros::ServiceServer goal_tool_control_server_;

  ros::Publisher chain_kinematics_pose_pub_;
  ros::Publisher chain_joint_states_pub_;

  std::string robot_name_;

  pthread_t timer_thread_;

  bool toolCtrlFlag_;
  double toolPosition_;

 public:
  bool timerThreadFlag_;
  OM_CHAIN chain_;

  OM_CONTROLLER();
  ~OM_CONTROLLER();

  void initPublisher();
  void initSubscriber();

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

  bool goalJointSpacePathCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                  open_manipulator_msgs::SetJointPosition::Response &res);
  bool goalTaskSpacePathCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                  open_manipulator_msgs::SetKinematicsPose::Response &res);
  bool goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                               open_manipulator_msgs::SetJointPosition::Response &res);

  void setTimerThread();
  static void *timerThread(void *param);

  void process(double time);

  void publishKinematicsPose();
  void publishJointStates();

};
}

#endif //OPEN_MANIPULATOR_CONTROLLER_H
