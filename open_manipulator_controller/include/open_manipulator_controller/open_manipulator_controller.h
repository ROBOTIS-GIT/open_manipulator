#ifndef OPEN_MANIPULATOR_CONTROLLER_H
#define OPEN_MANIPULATOR_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include "robotis_manipulator/OpenManipulator.h"
#include "open_manipulator_libs/OMKinematics.h"
#include "open_manipulator_libs/OMPath.h"

#define WORLD 0
#define COMP1 1
#define COMP2 2
#define COMP3 3
#define COMP4 4
#define TOOL 5

#define X_AXIS RM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS RM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS RM_MATH::makeVector3(0.0, 0.0, 1.0)

#define ITERATION_FREQUENCY 1/ACTUATOR_CONTROL_TIME
#define NUM_OF_JOINT 4

#define LINE 0
#define CIRCLE 1
#define RHOMBUS 2
#define HEART 3

namespace open_manipulator_controller
{

using namespace OPEN_MANIPULATOR;

class OM_CONTROLLER
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  ros::Subscriber joint_states_sub_;

  ros::ServiceServer goal_joint_space_path_server_;
  ros::ServiceServer goal_task_space_path_server_;
  ros::ServiceServer goal_tool_control_server_;


  ros::Publisher goal_joint_states_pub_;
  ros::Publisher goal_gripper_states_pub_;
  ros::Publisher present_kinematics_pose_pub_;

  std::string robot_name_;

  OpenManipulator chain;
  Kinematics* kinematics;

  sensor_msgs::JointState present_joint_states;
  std::vector<double> present_joint_angle;
  std::vector<double> present_gripper_angle;

  bool initPresentAngleFlag;

  //path
  OM_PATH::Line line_;
  OM_PATH::Circle circle_;
  OM_PATH::Rhombus rhombus_;
  OM_PATH::Heart heart_;

 public:
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

  void initManipulator();
  void updateAllJointAngle();
  void process();


  void setGripperAngle(double data, int gripper_id);
  void setAllActiveJointAngle(std::vector<double> angle_vector);
  void publishKinematicsPose();


};
}

#endif //OPEN_MANIPULATOR_CONTROLLER_H
