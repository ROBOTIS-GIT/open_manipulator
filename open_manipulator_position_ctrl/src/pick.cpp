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

#define DEG2RAD 57.2957795131
#define RAD2DEG 0.01745329251

const uint8_t is_moving = 0;
const uint8_t stopped   = 1;

#define MARKER_ID 8

#define DIST_GRIPPER_TO_JOINT4 0.130
#define OFFSET_FOR_GRIP_HEIGHT 8.0

enum
{
  WAITING_FOR_SIGNAL = 1,
  CHECK_AR_MARKER_POSE,
  INIT_POSITION,
  GRIPPER_OFF,
  MOVE_ARM,
  CLOSE_TO_OBJECT,
  GRIP_OBJECT,
  PICK_OBJECT_UP,
  WAITING_FOR_STOP
};

ros::Publisher target_joint_position_pub;
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

uint8_t task = 0, pre_task = 0;

void init_joint_position()
{
  open_manipulator_msgs::JointPose joint_positions;

  joint_positions.joint_name.push_back("joint1");
  joint_positions.joint_name.push_back("joint2");
  joint_positions.joint_name.push_back("joint3");
  joint_positions.joint_name.push_back("joint4");

  joint_positions.position.push_back( 0.0);
  joint_positions.position.push_back(-1.22173);
  joint_positions.position.push_back( 0.523599);
  joint_positions.position.push_back( 0.698132);

  joint_positions.max_velocity_scaling_factor = 0.3;
  joint_positions.max_accelerations_scaling_factor = 0.5;

  target_joint_position_pub.publish(joint_positions);
}

bool pickMsgCallback(open_manipulator_msgs::Pick::Request &req,
                     open_manipulator_msgs::Pick::Response &res)
{
  if ((state.arm == stopped) && (state.gripper == stopped))
  {
    res.result = "START PICK TASK!";
    task = CHECK_AR_MARKER_POSE;
  }
  else
  {
    res.result = "SOME TASKS IS WORKING";
    task = WAITING_FOR_SIGNAL;
  }

//  open_manipulator_msgs::KinematicsPose eef_pose;

//  if (state.marker == true)
//  {
//    eef_pose.group_name = "arm";
//    eef_pose.pose = ar_marker_pose.pose;

//    target_kinematics_pose_pub.publish(eef_pose);
//  }
//  else
//  {
//    ROS_ERROR("nono");
//  }

//  init_joint_position();
}

void armStateMsgCallback(const open_manipulator_msgs::State::ConstPtr &msg)
{
  std::string get_arm_state = msg->robot;

  if (get_arm_state == msg->STOPPED)
    state.arm = stopped;
  else if (get_arm_state == msg->IS_MOVING)
    state.arm = is_moving;
}

void gripperStateMsgCallback(const open_manipulator_msgs::State::ConstPtr &msg)
{
  std::string get_gripper_state = msg->robot;

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
    ar_marker_pose = marker.pose;

    ar_marker_pose.pose.position.x = marker.pose.pose.position.x - DIST_GRIPPER_TO_JOINT4;
    ar_marker_pose.pose.position.y = 0.0;
    ar_marker_pose.pose.position.z = marker.pose.pose.position.z + OFFSET_FOR_GRIP_HEIGHT;

    double dist = sqrt(ar_marker_pose.pose.position.x * ar_marker_pose.pose.position.x +
                       marker.pose.pose.position.y * marker.pose.pose.position.y);

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    if (marker.pose.pose.position.y > 0) yaw =        acos(ar_marker_pose.pose.position.x/dist);
    else                                 yaw = (-1) * acos(ar_marker_pose.pose.position.x/dist);

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    ar_marker_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    ar_marker_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    ar_marker_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    ar_marker_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;

//    ROS_INFO("x = %.3f", ar_marker_pose.pose.position.x);
//    ROS_INFO("y = %.3f", ar_marker_pose.pose.position.y);
//    ROS_INFO("z = %.3f", ar_marker_pose.pose.position.z);
//    ROS_INFO("yaw = %.3f", yaw * RAD2DEG);

    state.marker = true;
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
        task = INIT_POSITION;
      }
      else
      {
        ROS_ERROR("CAN NOT FIND AR MARKER(ID : 8)");
        task = WAITING_FOR_SIGNAL;
      }
     break;

    case INIT_POSITION:
      if (state.arm == stopped)
      {
        init_joint_position();
        state.arm = is_moving;

        pre_task = INIT_POSITION;
        task = WAITING_FOR_STOP;
      }
     break;

    case GRIPPER_OFF:
      if (state.gripper == stopped)
      {
        state.gripper = is_moving;

        grip.data = "grip_off";
        grip_pub.publish(grip);

        pre_task = GRIPPER_OFF;
        task = WAITING_FOR_STOP;
      }
     break;

    case MOVE_ARM:
      if (state.arm == stopped)
      {
        state.arm = is_moving;

        eef_pose.group_name = "arm";
        eef_pose.pose = ar_marker_pose.pose;
        target_kinematics_pose_pub.publish(eef_pose);

        pre_task = MOVE_ARM;
        task = WAITING_FOR_STOP;
      }
     break;

    case CLOSE_TO_OBJECT:
      task = GRIP_OBJECT;
     break;

    case GRIP_OBJECT:
      if (state.gripper == stopped)
      {
        state.gripper = is_moving;

        grip.data = "grip_on";
        grip_pub.publish(grip);

        pre_task = GRIP_OBJECT;
        task = WAITING_FOR_STOP;
      }
     break;

    case PICK_OBJECT_UP:
      if (state.arm == stopped)
      {
        state.arm = is_moving;

        init_joint_position();

        pre_task = PICK_OBJECT_UP;
        task = WAITING_FOR_SIGNAL;
      }
     break;

    case WAITING_FOR_STOP:
      if ((state.arm == stopped) && (state.gripper == stopped))
      {
        task = pre_task + 1;
      }
      else
      {
        ROS_WARN("WAITING FOR STOP");
      }
     break;

    default:
     break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick");
  ros::NodeHandle nh;

  std::string robot_name = "open_manipulator_with_tb3";

  target_joint_position_pub = nh.advertise<open_manipulator_msgs::JointPose>(robot_name + "/joint_pose", 10);
  target_kinematics_pose_pub = nh.advertise<open_manipulator_msgs::KinematicsPose>(robot_name + "/kinematics_pose", 10);
  grip_pub = nh.advertise<std_msgs::String>(robot_name + "/gripper", 10);

  ros::Subscriber arm_state_sub = nh.subscribe(robot_name + "/arm_state", 10, armStateMsgCallback);
  ros::Subscriber gripper_state_sub = nh.subscribe(robot_name + "/gripper_state", 10, gripperStateMsgCallback);
  ros::Subscriber ar_marker_pose_sub = nh.subscribe("/ar_pose_marker", 10, arMarkerPoseMsgCallback);

  ros::ServiceServer pick_server = nh.advertiseService(robot_name + "/pick", pickMsgCallback);

  ROS_INFO("Ready to PICK UP Task");

  ros::Rate loop_rate(25);

  while (ros::ok())
  {
    pick();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
