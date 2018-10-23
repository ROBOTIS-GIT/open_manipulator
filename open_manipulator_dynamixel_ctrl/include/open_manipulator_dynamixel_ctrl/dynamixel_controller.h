

#ifndef OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H
#define OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H

#include <ros/ros.h>

#include <vector>
#include <string>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <sensor_msgs/JointState.h>

namespace dynamixel
{
#define ITERATION_FREQUENCY  (100)
#define JOINT_NUM   4
#define GRIPPER_NUM 1
#define DXL_NUM     5
#define PALM_NUM    2

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber goal_joint_states_sub_;
  ros::Subscriber goal_gripper_states_sub_;

  // ROS Service Server

  // ROS Service Client

  // Dynamixel Workbench Parameters
  std::string robot_name_;
  float protocol_version_;

  DynamixelWorkbench *joint_controller_;
  DynamixelWorkbench *gripper_controller_;

  std::vector<uint8_t> joint_id_;
  std::vector<uint8_t> gripper_id_;

  std::string joint_mode_;
  std::string gripper_mode_;

 public:
  DynamixelController();
  ~DynamixelController();
  bool control_loop();

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void getDynamixelInst();
  void setOperatingMode();
  void setSyncFunction();
  void readPosition(double *value);
  void readVelocity(double *value);
  void updateJointStates();

  void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void goalGripperPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
};
}

#endif //OPEN_MANIPULATOR_DYNAMIXEL_CONTROLLER_H
