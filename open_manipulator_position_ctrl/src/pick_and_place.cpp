#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>

#include <open_manipulator_msgs/JointPose.h>
#include <open_manipulator_msgs/KinematicsPose.h>

#include <open_manipulator_msgs/State.h>

#include <open_manipulator_msgs/Pick.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

const uint8_t is_moving = 0;
const uint8_t stopped   = 1;

#define MARKER_ID 8

enum
{
  WAITING_FOR_SIGNAL = 1,
  CHECK_AR_MARKER_POSE,
  GRIPPER_OFF,
  MOVE_ARM,
  CLOSE_TO_OBJECT,
  GRIP_OBJECT,
  PICK_OBJECT_UP
};

ros::Publisher target_joint_pose_pub;
ros::Publisher target_kinematics_pose_pub;

ros::Publisher grip_pub;

typedef struct
{
  bool arm;
  bool gripper;
  bool marker;
} State;

geometry_msgs::PoseStamped ar_marker_pose;
State state = {false, false, false};

uint8_t task = 0;

void init_joint_position()
{
  open_manipulator_msgs::JointPose joint_positions;

  joint_positions.joint_name.push_back("joint1");
  joint_positions.joint_name.push_back("joint2");
  joint_positions.joint_name.push_back("joint3");
  joint_positions.joint_name.push_back("joint4");

  joint_positions.position.push_back(0.0);
  joint_positions.position.push_back(-1.5707);
  joint_positions.position.push_back(1.37);
  joint_positions.position.push_back(0.2258);

  target_joint_pose_pub.publish(joint_positions);
}

bool pickMsgCallback(open_manipulator_msgs::Pick::Request &req,
                     open_manipulator_msgs::Pick::Response &res)
{
//  if ((state.gripper == stopped) && (state.arm == stopped))
//  {
//    res.result = "START PICK TASK!";
//    task = CHECK_AR_MARKER_POSE;
//  }
//  else
//  {
//    res.result = "SOME TASKS IS WORKING";
//    task = WAITING_FOR_SIGNAL;
//  }

  open_manipulator_msgs::KinematicsPose eef_pose;

  if (state.marker == true)
  {
    eef_pose.group_name = "arm";
    eef_pose.pose = ar_marker_pose.pose;

    target_kinematics_pose_pub.publish(eef_pose);
  }
  else
  {
    ROS_ERROR("FUck");
  }
}

void robotStateMsgCallback(const open_manipulator_msgs::State::ConstPtr &msg)
{
  std::string get_arm_state     = msg->arm;
  std::string get_gripper_state = msg->gripper;

  if (get_arm_state == msg->STOPPED)
    state.arm = stopped;
  else if (get_arm_state == msg->IS_MOVING)
    state.arm = is_moving;

  if (get_gripper_state == msg->STOPPED)
    state.gripper = stopped;
  else if (get_gripper_state == msg->IS_MOVING)
    state.gripper = is_moving;
}

void arMarkerPoseMsgCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  ar_track_alvar_msgs::AlvarMarker marker = msg->markers[0];

  if (marker.id == MARKER_ID)
  {
    state.marker = true;
    ar_marker_pose = marker.pose;
  }
  else
  {
    state.marker = false;
  }
}

void pick()
{
  open_manipulator_msgs::KinematicsPose eef_pose;
  std_msgs::String grip;

  switch (task)
  {
    case CHECK_AR_MARKER_POSE:
      if (state.marker == true)
      {
        ROS_INFO("SAVE POSE OF AR MARKER");
        task = GRIPPER_OFF;
      }
      else
      {
        ROS_ERROR("CAN NOT FIND AR MARKER(ID : 8)");
        task = WAITING_FOR_SIGNAL;
      }
     break;

    case GRIPPER_OFF:
      grip.data = "grip_off";

      if (state.gripper == stopped)
      {
        grip_pub.publish(grip);
        ROS_INFO("SUCCESS TO GRIPPER OFF");
        task = MOVE_ARM;
      }
      else
      {
        ROS_WARN("WAITING FOR STOP(GRIPPER)");
        task = GRIPPER_OFF;
      }
     break;

    case MOVE_ARM:
      eef_pose.group_name = "arm";
      eef_pose.pose = ar_marker_pose.pose;

      if (state.gripper == stopped)
      {
        target_kinematics_pose_pub.publish(eef_pose);
        ROS_INFO("SUCCESS TO MOVE ARM");
        task = CLOSE_TO_OBJECT;
      }
      else
      {
        ROS_WARN("WAITING FOR STOP(GRIPPER ON)");
        task = MOVE_ARM;
      }
     break;

    case CLOSE_TO_OBJECT:
      task = GRIP_OBJECT;
     break;

    case GRIP_OBJECT:
      grip.data = "grip_on";

      if (state.arm == stopped)
      {
        grip_pub.publish(grip);
        ROS_INFO("SUCCESS TO GRIPPER ON");
        task = GRIP_OBJECT;
      }
      else
      {
        ROS_WARN("WAITING FOR STOP(ARM)");
        task = PICK_OBJECT_UP;
      }
     break;

    case PICK_OBJECT_UP:
      if ((state.gripper == stopped) && (state.arm == stopped))
      {
        init_joint_position();
        ROS_INFO("SUCCESS TO PICKING UP");
        task = WAITING_FOR_SIGNAL;
      }
      else
      {
        ROS_WARN("WAITING FOR PICKING UP");
        task = PICK_OBJECT_UP;
      }
     break;

    default:
     break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle nh;

  std::string robot_name = "open_manipulator_with_tb3";

  target_joint_pose_pub = nh.advertise<open_manipulator_msgs::JointPose>(robot_name + "/joint_pose", 10);
  target_kinematics_pose_pub = nh.advertise<open_manipulator_msgs::KinematicsPose>(robot_name + "/kinematics_pose", 10);
  grip_pub = nh.advertise<std_msgs::String>(robot_name + "/gripper", 10);

  ros::Subscriber robot_state_sub = nh.subscribe(robot_name + "/state", 10, robotStateMsgCallback);
  ros::Subscriber ar_marker_pose_sub = nh.subscribe("/ar_pose_marker", 10, arMarkerPoseMsgCallback);

  ros::ServiceServer pick_server = nh.advertiseService(robot_name + "/pick", pickMsgCallback);

  ros::Rate loop_rate(25);

  while (ros::ok())
  {
    pick();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
