#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>

#include <open_manipulator_msgs/SetJointPosition.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>

#include <open_manipulator_msgs/State.h>

#include <open_manipulator_msgs/Pick.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131

const uint8_t is_moving = 0;
const uint8_t stopped   = 1;

#define MARKER_ID 8

#define DIST_OBJECT_TO_ARMARKER 0.030
#define DIST_EDGE_TO_CENTER_OF_PALM 0.030
#define DIST_GRIPPER_TO_JOINT4  0.145
#define OFFSET_FOR_GRIP_HEIGHT  0.08

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

ros::ServiceClient joint_position_command_client;
ros::ServiceClient kinematics_pose_command_client;

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

double roll = 0.0, pitch = 0.0, yaw = 0.0;

double tolerance = 0.01;

ar_track_alvar_msgs::AlvarMarker markers;

bool init_joint_position()
{
  open_manipulator_msgs::SetJointPosition msg;

  msg.request.joint_position.joint_name.push_back("joint1");
  msg.request.joint_position.joint_name.push_back("joint2");
  msg.request.joint_position.joint_name.push_back("joint3");
  msg.request.joint_position.joint_name.push_back("joint4");

  msg.request.joint_position.position.push_back( 0.0);
  msg.request.joint_position.position.push_back(-0.65);
  msg.request.joint_position.position.push_back( 1.20);
  msg.request.joint_position.position.push_back(-0.54);

  msg.request.joint_position.max_velocity_scaling_factor = 0.3;
  msg.request.joint_position.max_accelerations_scaling_factor = 0.5;

  if (joint_position_command_client.call(msg))
  {
    return msg.response.isPlanned;
  }
  else
    ROS_ERROR("FAILED TO CALL SERVER");
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
//  ar_track_alvar_msgs::AlvarMarker marker = msg->markers[0];

//  if (marker.id == MARKER_ID)
//  {
//    ar_marker_pose = marker.pose;

//    ar_marker_pose.pose.position.x = marker.pose.pose.position.x - DIST_GRIPPER_TO_JOINT4;
//    ar_marker_pose.pose.position.y = 0.0;
//    ar_marker_pose.pose.position.z = marker.pose.pose.position.z + OFFSET_FOR_GRIP_HEIGHT;

//    double dist = sqrt((ar_marker_pose.pose.position.x * ar_marker_pose.pose.position.x) +
//                       (marker.pose.pose.position.y * marker.pose.pose.position.y));

//    if (marker.pose.pose.position.y > 0) yaw =        acos(ar_marker_pose.pose.position.x/dist);
//    else                                 yaw = (-1) * acos(ar_marker_pose.pose.position.x/dist);

//    double cy = cos(yaw * 0.5);
//    double sy = sin(yaw * 0.5);
//    double cr = cos(roll * 0.5);
//    double sr = sin(roll * 0.5);
//    double cp = cos(pitch * 0.5);
//    double sp = sin(pitch * 0.5);

//    ar_marker_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
//    ar_marker_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
//    ar_marker_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
//    ar_marker_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;

//    state.marker = true;
//  }
//  else
//  {
//    state.marker = false;
//  }
}

void pick()
{
  open_manipulator_msgs::SetKinematicsPose eef_pose;
  std_msgs::String grip;

  switch (task)
  {
    case CHECK_AR_MARKER_POSE:
      if (state.marker == true)
      {
        ROS_WARN("SAVE POSE OF AR MARKER");
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
        ROS_WARN("SET INIT POSITION");

        bool result = init_joint_position();

        if (result)
        {
          ROS_INFO("PLANNING IS SUCCESSED");

          ros::WallDuration sleep_time(1.0);
          sleep_time.sleep();

          pre_task = INIT_POSITION;
          task = WAITING_FOR_STOP;
        }
        else
        {
          ROS_ERROR("PLANNING IS FAILED");
          task = INIT_POSITION;
        }
      }
     break;

    case GRIPPER_OFF:
      if (state.gripper == stopped)
      {
        ROS_WARN("OPEN GRIPPER");

        grip.data = "grip_off";
        grip_pub.publish(grip);

        ros::WallDuration sleep_time(1.0);
        sleep_time.sleep();

        pre_task = GRIPPER_OFF;
        task = WAITING_FOR_STOP;
      }
     break;

    case MOVE_ARM:
      if (state.arm == stopped)
      {
        ROS_WARN("MOVE ARM TO PICK");

        ROS_INFO("x = %.3f", ar_marker_pose.pose.position.x);
        ROS_INFO("y = %.3f", ar_marker_pose.pose.position.y);
        ROS_INFO("z = %.3f", ar_marker_pose.pose.position.z);

        ROS_INFO("qx = %.3f", ar_marker_pose.pose.orientation.x);
        ROS_INFO("qy = %.3f", ar_marker_pose.pose.orientation.y);
        ROS_INFO("qz = %.3f", ar_marker_pose.pose.orientation.z);
        ROS_INFO("qw = %.3f", ar_marker_pose.pose.orientation.w);

        ROS_INFO("yaw = %.3f", yaw * RAD2DEG);

        eef_pose.request.kinematics_pose.group_name = "arm";
        eef_pose.request.kinematics_pose.pose = ar_marker_pose.pose;
        eef_pose.request.kinematics_pose.max_velocity_scaling_factor = 0.3;
        eef_pose.request.kinematics_pose.max_accelerations_scaling_factor = 0.5;
        eef_pose.request.kinematics_pose.tolerance = tolerance;

        if (kinematics_pose_command_client.call(eef_pose))
        {
          if (eef_pose.response.isPlanned == true)
          {
            ROS_INFO("PLANNING IS SUCCESSED");

            ros::WallDuration sleep_time(1.0);
            sleep_time.sleep();

            pre_task = MOVE_ARM;
            task = WAITING_FOR_STOP;
          }
          else
          {
            ROS_ERROR("PLANNING IS FAILED");
            tolerance += 0.01;
            task = MOVE_ARM;
          }
        }
      }
     break;

    case CLOSE_TO_OBJECT:
      if (state.arm == stopped)
      {
        ROS_WARN("CLOSE TO OBJECT");

        geometry_msgs::PoseStamped object_pose = ar_marker_pose;

        object_pose.pose.position.x = object_pose.pose.position.x - (DIST_EDGE_TO_CENTER_OF_PALM + DIST_OBJECT_TO_ARMARKER);

        eef_pose.request.kinematics_pose.group_name = "arm";
        eef_pose.request.kinematics_pose.pose = object_pose.pose;

        eef_pose.request.kinematics_pose.max_velocity_scaling_factor = 0.1;
        eef_pose.request.kinematics_pose.max_accelerations_scaling_factor = 0.5;
        eef_pose.request.kinematics_pose.tolerance = tolerance;

        if (kinematics_pose_command_client.call(eef_pose))
        {
          if (eef_pose.response.isPlanned == true)
          {
            ROS_INFO("PLANNING IS SUCCESSED");

            ros::WallDuration sleep_time(1.0);
            sleep_time.sleep();

            pre_task = CLOSE_TO_OBJECT;
            task = WAITING_FOR_STOP;
          }
          else
          {
            ROS_ERROR("PLANNING IS FAILED");
            task = CLOSE_TO_OBJECT;
          }
        }
      }
     break;

    case GRIP_OBJECT:
      if (state.gripper == stopped)
      {
        ROS_WARN("GRIP OBJECT");

        grip.data = "grip_on";
        grip_pub.publish(grip);

        ros::WallDuration sleep_time(1.0);
        sleep_time.sleep();

        pre_task = GRIP_OBJECT;
        task = WAITING_FOR_STOP;
      }
     break;

    case PICK_OBJECT_UP:
      if (state.arm == stopped)
      {
        ROS_WARN("PICK OBJECT UP");

        init_joint_position();

        pre_task = PICK_OBJECT_UP;
        task = WAITING_FOR_STOP;
      }
     break;

    case WAITING_FOR_STOP:
      if ((state.arm == stopped) && (state.gripper == stopped))
      {
        tolerance = 0.01;

        if (pre_task == PICK_OBJECT_UP)
        {
          task = WAITING_FOR_SIGNAL;
          ROS_WARN("SUCCESS TO PICK UP");
        }
        else
          task = pre_task + 1;

      }
      else
      {
        ROS_INFO("WAITING FOR ROBOT HAS STOPPED");
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

  joint_position_command_client = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name + "/set_joint_position");
  kinematics_pose_command_client = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>(robot_name + "/set_kinematics_pose");

  grip_pub = nh.advertise<std_msgs::String>(robot_name + "/gripper", 10);

  ros::Subscriber arm_state_sub = nh.subscribe(robot_name + "/arm_state", 10, armStateMsgCallback);
  ros::Subscriber gripper_state_sub = nh.subscribe(robot_name + "/gripper_state", 10, gripperStateMsgCallback);

//  ar_track_alvar_msgs::AlvarMarkers am;
//  ar_track_alvar_msgs::AlvarMarkersConstPtr msg = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>(markers, ros::Duration(25));
//  if (msg == NULL)
//      ROS_INFO("No point clound messages received");
//  else
//      am = * msg;

//  ros::Subscriber ar_marker_pose_sub = nh.subscribe("/ar_pose_marker", 10, arMarkerPoseMsgCallback);

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
