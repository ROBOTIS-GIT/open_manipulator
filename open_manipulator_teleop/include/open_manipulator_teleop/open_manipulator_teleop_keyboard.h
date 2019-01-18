#ifndef OPEN_MANIPULATOR_CONTROLLER_H
#define OPEN_MANIPULATOR_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <termios.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#define NUM_OF_JOINT 4
#define DELTA 0.01
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5

namespace open_manipulator_teleop
{

class OM_TELEOP
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_tool_control_client_;

  ros::Subscriber chain_joint_states_sub_;
  ros::Subscriber chain_kinematics_pose_sub_;

  std::vector<double> present_joint_angle;
  std::vector<double> present_kinematic_position;

  struct termios oldt;

 public:

  OM_TELEOP();
  ~OM_TELEOP();

  void initClient();
  void initSubscriber();

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();

  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  void printText();
  void setGoal(char ch);

  void restore_terminal_settings(void);
  void disable_waiting_for_enter(void);

};
}

#endif //OPEN_MANIPULATOR_CONTROLLER_H
