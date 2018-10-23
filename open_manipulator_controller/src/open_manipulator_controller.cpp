
#include "open_manipulator_controller/open_manipulator_controller.h"

using namespace open_manipulator_controller;

OM_CONTROLLER::OM_CONTROLLER()
    :node_handle_(""),
     priv_node_handle_("~")
{
  robot_name_   = priv_node_handle_.param<std::string>("robot_name", "open_manipulator");

  present_joint_angle.resize(4);
  present_gripper_angle.resize(1);

  initPresentAngleFlag = false;

  initPublisher();
  initSubscriber();

  initManipulator();

  chain.addDraw(LINE, &line_);
  chain.addDraw(CIRCLE, &circle_);
  chain.addDraw(RHOMBUS, &rhombus_);
  chain.addDraw(HEART, &heart_);

  ROS_INFO("OpenManipulator initialization");

}

OM_CONTROLLER::~OM_CONTROLLER()
{
  ros::shutdown();
}

void OM_CONTROLLER::initPublisher()
{
  goal_joint_states_pub_    = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/goal_joint_position", 10);
  goal_gripper_states_pub_  = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/goal_gripper_position", 10);
  present_kinematics_pose_pub_  = node_handle_.advertise<open_manipulator_msgs::KinematicsPose>(robot_name_ + "/present_kinematics_pose", 10);
}
void OM_CONTROLLER::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe(robot_name_ + "/joint_states", 10, &OM_CONTROLLER::jointStatesCallback, this);
  goal_joint_space_path_server_ = node_handle_.advertiseService(robot_name_ + "/goal_joint_space_path", &OM_CONTROLLER::goalJointSpacePathCallback, this);
  goal_task_space_path_server_ = node_handle_.advertiseService(robot_name_ + "/goal_task_space_path", &OM_CONTROLLER::goalTaskSpacePathCallback, this);
  goal_tool_control_server_ = node_handle_.advertiseService(robot_name_ + "/goal_tool_control", &OM_CONTROLLER::goalToolControlCallback, this);
}

void OM_CONTROLLER::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

//  for(int i = 0; i < msg->name.size(); i ++)
//  {
//    present_joint_states.name[i] = msg->name.at(i);
//    present_joint_states.position[i] = msg->position.at(i);
//    present_joint_states.velocity[i] = msg->velocity.at(i);
//    present_joint_states.effort[i] = msg->effort.at(i);
//  }
  std::vector<double> temp_angle;
  for(int i = 0; i < NUM_OF_JOINT; i ++) temp_angle.push_back(msg->position.at(i));
  present_joint_angle = temp_angle;

  present_gripper_angle.push_back(msg->position.at(NUM_OF_JOINT));

  if(!initPresentAngleFlag)
  {
    chain.setPreviousGoalPosition(present_joint_angle);
    initPresentAngleFlag = true;
  }


  spinner.stop();
}

bool OM_CONTROLLER::goalJointSpacePathCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                               open_manipulator_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;
  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));
  chain.setJointTrajectory(target_angle,req.path_time);

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
  target_pose.orientation = chain.getComponentOrientationToWorld(TOOL);
  chain.setTaskTrajectory(TOOL, target_pose, req.path_time);
  //chain.setDrawing(TOOL, LINE, 1.0, OM_MATH::makeVector3(0.0,0.0,-0.05));


  res.isPlanned = true;
  return true;
}
bool OM_CONTROLLER::goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                            open_manipulator_msgs::SetJointPosition::Response &res)
{
  setGripperAngle(chain.toolMove(TOOL, req.joint_position.position.at(0)), chain.getComponentToolId(TOOL));

  res.isPlanned = true;
  return true;
}

void OM_CONTROLLER::initManipulator()
{
  chain.addWorld(WORLD,
                 COMP1);

  chain.addComponent(COMP1,
                     WORLD,
                     COMP2,
                     RM_MATH::makeVector3(-0.278, 0.0, 0.017),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     11);

  chain.addComponent(COMP2,
                     COMP1,
                     COMP3,
                     RM_MATH::makeVector3(0.0, 0.0, 0.058),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     12);

  chain.addComponent(COMP3,
                     COMP2,
                     COMP4,
                     RM_MATH::makeVector3(0.024, 0.0, 0.128),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     13);

  chain.addComponent(COMP4,
                     COMP3,
                     TOOL,
                     RM_MATH::makeVector3(0.124, 0.0, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     14);

  chain.addTool(TOOL,
                COMP4,
                RM_MATH::makeVector3(0.130, 0.0, 0.0),
                Eigen::Matrix3f::Identity(3, 3),
                15,
                1.0f); // Change unit from `meter` to `radian`

  kinematics = new OM_KINEMATICS::Chain();
  chain.initKinematics(kinematics);
  chain.initTrajectory(present_joint_angle);
  chain.setControlTime(ACTUATOR_CONTROL_TIME);

  updateAllJointAngle();
  chain.forward(COMP1);
}
void OM_CONTROLLER::updateAllJointAngle()
{
  chain.setAllActiveJointAngle(chain.getPreviousGoalPosition());
}


void OM_CONTROLLER::setGripperAngle(double data, int gripper_id)
{
  sensor_msgs::JointState goal_gripper_position;
  goal_gripper_position.position.push_back((double)data);
  goal_gripper_position.name.push_back(std::to_string(gripper_id));

  goal_gripper_states_pub_.publish(goal_gripper_position);
}


void OM_CONTROLLER::setAllActiveJointAngle(std::vector<double> angle_vector)
{
  sensor_msgs::JointState goal_joint_position;

  for(int i = 0; i < NUM_OF_JOINT; i ++)
  {
    goal_joint_position.name.push_back(std::to_string(chain.getComponentJointId(i+1)));
    goal_joint_position.position.push_back(angle_vector.at(i));
    goal_joint_position.effort.push_back(0);

  }
  goal_joint_states_pub_.publish(goal_joint_position);
}

void OM_CONTROLLER::publishKinematicsPose()
{
  open_manipulator_msgs::KinematicsPose msg;

  Vector3f position = chain.getComponentPositionToWorld(TOOL);
  msg.pose.position.x = position[0];
  msg.pose.position.y = position[1];
  msg.pose.position.z = position[2];
  present_kinematics_pose_pub_.publish(msg);
}


void OM_CONTROLLER::process()
{
  std::vector<double> setJointAngle = chain.controlLoop(ros::Time::now().toSec(), TOOL);
  if(!setJointAngle.empty())
    setAllActiveJointAngle(setJointAngle);

  publishKinematicsPose();


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
    om_controller.process();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
