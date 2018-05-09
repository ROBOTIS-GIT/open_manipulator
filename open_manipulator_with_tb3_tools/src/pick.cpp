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

#define ON  true
#define OFF false

const uint8_t IS_MOVING = 0;
const uint8_t STOPPED   = 1;

#define DIST_GRIPPER_TO_JOINT4       0.145
#define DIST_EDGE_TO_CENTER_OF_PALM  0.025

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
ros::ServiceClient gripper_position_command_client;

ros::ServiceClient pick_result_client;

ros::Publisher grip_pub;

typedef struct
{
  bool arm;
  bool gripper;
  bool marker;
} State;

ar_track_alvar_msgs::AlvarMarker ar_marker_pose;
geometry_msgs::PoseStamped desired_pose;

State state = {false, false, false};

uint8_t task = 0, pre_task = 0;

double roll = 0.0, pitch = 0.0, yaw = 0.0;

double tolerance = 0.01;

ar_track_alvar_msgs::AlvarMarker markers;

std::string robot_name;
double offset_for_object_height, dist_ar_marker_to_object;

int get_marker_id;

bool initJointPosition()
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

  ros::service::waitForService(robot_name + "/set_joint_position");
  if (joint_position_command_client.call(msg))
  {
    return msg.response.isPlanned;
  }
  else
  {
    ROS_ERROR("FAILED TO CALL SERVER");
    return false;
  }
}

bool pickUpJointPosition()
{
  open_manipulator_msgs::SetJointPosition msg;

  msg.request.joint_position.joint_name.push_back("joint1");
  msg.request.joint_position.joint_name.push_back("joint2");
  msg.request.joint_position.joint_name.push_back("joint3");
  msg.request.joint_position.joint_name.push_back("joint4");

  msg.request.joint_position.position.push_back( 0.0);
  msg.request.joint_position.position.push_back(-0.95);
  msg.request.joint_position.position.push_back( 0.95);
  msg.request.joint_position.position.push_back(-0.20);

  msg.request.joint_position.max_velocity_scaling_factor = 0.1;
  msg.request.joint_position.max_accelerations_scaling_factor = 0.5;

  ros::service::waitForService(robot_name + "/set_joint_position");
  if (joint_position_command_client.call(msg))
  {
    return msg.response.isPlanned;
  }
  else
  {
    ROS_ERROR("FAILED TO CALL SERVER");
    return false;
  }
}

bool gripper(bool onoff)
{
  open_manipulator_msgs::SetJointPosition msg;

  msg.request.joint_position.joint_name.push_back("grip_joint");
  msg.request.joint_position.joint_name.push_back("grip_joint_sub");

  if (onoff == true)
    msg.request.joint_position.position.push_back(0.01);
  else
    msg.request.joint_position.position.push_back(-0.01);

  msg.request.joint_position.max_velocity_scaling_factor = 0.3;
  msg.request.joint_position.max_accelerations_scaling_factor = 0.01;

  ros::service::waitForService(robot_name + "/set_gripper_position");
  if (gripper_position_command_client.call(msg))
  {
    return msg.response.isPlanned;
  }
  else
  {
    ROS_ERROR("FAILED TO CALL SERVER");
    return false;
  }
}

bool resultOfPickUp(std_msgs::String res_msg)
{
  open_manipulator_msgs::Pick msg;

  msg.request.state = res_msg.data;

  ros::service::waitForService(robot_name + "/result_of_pick_up");
  if (pick_result_client.call(msg))
  {
    return true;
  }
  else
  {
    ROS_ERROR("FAILED TO CALL SERVER");
    return false;
  }
}

geometry_msgs::PoseStamped calcDesiredPose(ar_track_alvar_msgs::AlvarMarker marker)
{
  geometry_msgs::PoseStamped get_pose;

  if (marker.id == get_marker_id)
  {
    get_pose = marker.pose;

    get_pose.pose.position.x = marker.pose.pose.position.x - DIST_GRIPPER_TO_JOINT4;
    get_pose.pose.position.y = 0.0;
    get_pose.pose.position.z = marker.pose.pose.position.z + offset_for_object_height;

    double dist = sqrt((marker.pose.pose.position.x * marker.pose.pose.position.x) +
                       (marker.pose.pose.position.y * marker.pose.pose.position.y));

    if (marker.pose.pose.position.y > 0) yaw =        acos(marker.pose.pose.position.x/dist);
    else                                 yaw = (-1) * acos(marker.pose.pose.position.x/dist);

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    get_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    get_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    get_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    get_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;

    state.marker = true;
  }
  else
  {
    state.marker = false;
  }

  return get_pose;
}


bool pickMsgCallback(open_manipulator_msgs::Pick::Request &req,
                     open_manipulator_msgs::Pick::Response &res)
{
  if ((state.arm == STOPPED) && (state.gripper == STOPPED))
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
    state.arm = STOPPED;
  else if (get_arm_state == msg->IS_MOVING)
    state.arm = IS_MOVING;
}

void gripperStateMsgCallback(const open_manipulator_msgs::State::ConstPtr &msg)
{
  std::string get_gripper_state = msg->robot;

  if (get_gripper_state == msg->STOPPED)
    state.gripper = STOPPED;
  else if (get_gripper_state == msg->IS_MOVING)
    state.gripper = IS_MOVING;
}

void arMarkerPoseMsgCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{  
  if (msg->markers.size() == 0)
    return;

  ar_marker_pose = msg->markers[0];
}

void pick()
{
  open_manipulator_msgs::SetKinematicsPose eef_pose;
  std_msgs::String result_msg;

  static uint8_t planning_cnt = 0;

  switch (task)
  {
    case CHECK_AR_MARKER_POSE:
      desired_pose = calcDesiredPose(ar_marker_pose);
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
      if (state.arm == STOPPED)
      {
        ROS_WARN("SET INIT POSITION");

        bool result = initJointPosition();

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
          planning_cnt++;
          ROS_ERROR("PLANNING IS FAILED (%d)", planning_cnt);
          task = INIT_POSITION;
        }
      }
     break;

    case GRIPPER_OFF:
      if (state.gripper == STOPPED)
      {
        ROS_WARN("OPEN GRIPPER");

        bool result = gripper(OFF);

        if (result)
        {
          ROS_INFO("PLANNING IS SUCCESSED");

          ros::WallDuration sleep_time(1.0);
          sleep_time.sleep();

          pre_task = GRIPPER_OFF;
          task = WAITING_FOR_STOP;
        }
        else
        {
          planning_cnt++;
          ROS_ERROR("PLANNING IS FAILED (%d)", planning_cnt);
          task = GRIPPER_OFF;
        }
      }
     break;

    case MOVE_ARM:
      if (state.arm == STOPPED)
      {
        ROS_WARN("MOVE ARM TO PICK");

        ROS_INFO("x = %.3f", desired_pose.pose.position.x);
        ROS_INFO("y = %.3f", desired_pose.pose.position.y);
        ROS_INFO("z = %.3f", desired_pose.pose.position.z);

        ROS_INFO("qx = %.3f", desired_pose.pose.orientation.x);
        ROS_INFO("qy = %.3f", desired_pose.pose.orientation.y);
        ROS_INFO("qz = %.3f", desired_pose.pose.orientation.z);
        ROS_INFO("qw = %.3f", desired_pose.pose.orientation.w);

        ROS_INFO("yaw = %.3f", yaw * RAD2DEG);

        eef_pose.request.kinematics_pose.group_name = "arm";
        eef_pose.request.kinematics_pose.pose = desired_pose.pose;
        eef_pose.request.kinematics_pose.max_velocity_scaling_factor = 0.1;
        eef_pose.request.kinematics_pose.max_accelerations_scaling_factor = 0.5;
        eef_pose.request.kinematics_pose.tolerance = tolerance;

        ros::service::waitForService(robot_name + "/set_kinematics_pose");
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
            if (planning_cnt > 10)
            {
              result_msg.data = "Failed to plan";
              resultOfPickUp(result_msg);

              planning_cnt = 0;
              task = WAITING_FOR_SIGNAL;
            }
            else
            {
              planning_cnt++;

              tolerance += 0.005;
              ROS_ERROR("PLANNING IS FAILED (%d, tolerance : %.2f)", planning_cnt, tolerance);

              task = MOVE_ARM;
            }
          }
        }
      }
     break;

    case CLOSE_TO_OBJECT:
      if (state.arm == STOPPED)
      {
        ROS_WARN("CLOSE TO OBJECT");

        geometry_msgs::PoseStamped object_pose = desired_pose;

        object_pose.pose.position.x = object_pose.pose.position.x + (DIST_EDGE_TO_CENTER_OF_PALM + dist_ar_marker_to_object);

        ROS_INFO("x = %.3f", object_pose.pose.position.x);
        ROS_INFO("y = %.3f", object_pose.pose.position.y);
        ROS_INFO("z = %.3f", object_pose.pose.position.z);

        ROS_INFO("qx = %.3f", object_pose.pose.orientation.x);
        ROS_INFO("qy = %.3f", object_pose.pose.orientation.y);
        ROS_INFO("qz = %.3f", object_pose.pose.orientation.z);
        ROS_INFO("qw = %.3f", object_pose.pose.orientation.w);

        ROS_INFO("yaw = %.3f", yaw * RAD2DEG);

        eef_pose.request.kinematics_pose.group_name = "arm";
        eef_pose.request.kinematics_pose.pose = object_pose.pose;

        eef_pose.request.kinematics_pose.max_velocity_scaling_factor = 0.1;
        eef_pose.request.kinematics_pose.max_accelerations_scaling_factor = 0.1;
        eef_pose.request.kinematics_pose.tolerance = tolerance;

        ros::service::waitForService(robot_name + "/set_kinematics_pose");
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
            if (planning_cnt > 10)
            {
              result_msg.data = "Failed to plan";
              resultOfPickUp(result_msg);

              planning_cnt = 0;
              task = WAITING_FOR_SIGNAL;
            }
            else
            {
              planning_cnt++;
              tolerance += 0.005;
              ROS_ERROR("PLANNING IS FAILED (%d, tolerance : %.2f)", planning_cnt, tolerance);

              task = CLOSE_TO_OBJECT;
            }
          }
        }
      }
     break;

    case GRIP_OBJECT:
      if (state.gripper == STOPPED)
      {
        ROS_WARN("GRIP OBJECT");

        bool result = gripper(ON);

        if (result)
        {
          ROS_INFO("PLANNING IS SUCCESSED");

          ros::WallDuration sleep_time(1.0);
          sleep_time.sleep();

          pre_task = GRIP_OBJECT;
          task = WAITING_FOR_STOP;
        }
        else
        {
          planning_cnt++;
          ROS_ERROR("PLANNING IS FAILED (%d)", planning_cnt);
          task = GRIP_OBJECT;
        }
      }
     break;

    case PICK_OBJECT_UP:
      if (state.arm == STOPPED)
      {
        ROS_WARN("PICK OBJECT UP");

        bool result = pickUpJointPosition();

        if (result)
        {
          ROS_INFO("PLANNING IS SUCCESSED");

          ros::WallDuration sleep_time(1.0);
          sleep_time.sleep();

          pre_task = PICK_OBJECT_UP;
          task = WAITING_FOR_STOP;
        }
        else
        {
          planning_cnt++;
          ROS_ERROR("PLANNING IS FAILED (%d)", planning_cnt);
          task = PICK_OBJECT_UP;
        }
      }
     break;

    case WAITING_FOR_STOP:
      if ((state.arm == STOPPED) && (state.gripper == STOPPED))
      {
        tolerance = 0.01;
        planning_cnt = 0;

        if (pre_task == PICK_OBJECT_UP)
        {
          task = WAITING_FOR_SIGNAL;

          result_msg.data = "Success";
          resultOfPickUp(result_msg);
        }
        else
          task = pre_task + 1;

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
  ros::NodeHandle priv_nh("~");

  priv_nh.getParam("robot_name", robot_name);
  priv_nh.getParam("marker_id", get_marker_id);
  priv_nh.getParam("offset_for_object_height", offset_for_object_height);
  priv_nh.getParam("dist_ar_marker_to_object", dist_ar_marker_to_object);

  joint_position_command_client = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name + "/set_joint_position");
  kinematics_pose_command_client = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>(robot_name + "/set_kinematics_pose");

  gripper_position_command_client = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name + "/set_gripper_position");

  pick_result_client = nh.serviceClient<open_manipulator_msgs::Pick>(robot_name + "/result_of_pick_up");

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
