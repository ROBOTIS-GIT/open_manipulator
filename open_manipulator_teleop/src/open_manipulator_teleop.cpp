
#include "open_manipulator_teleop/open_manipulator_teleop.h"

using namespace open_manipulator_teleop;

OM_TELEOP::OM_TELEOP()
    :node_handle_(""),
     priv_node_handle_("~")
{
  robot_name_     = priv_node_handle_.param<std::string>("robot_name", "open_manipulator");
  present_joint_angle.resize(NUM_OF_JOINT);
  present_kinematic_position.resize(3);

  initPublisher();
  initSubscriber();

  ROS_INFO("OpenManipulator initialization");
}

OM_TELEOP::~OM_TELEOP()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void OM_TELEOP::initPublisher()
{

  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name_ + "/goal_joint_space_path");
  goal_task_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>(robot_name_ + "/goal_task_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name_ + "/goal_tool_control");

}
void OM_TELEOP::initSubscriber()
{
  chain_joint_states_sub_ = node_handle_.subscribe("open_manipulator/joint_states", 10, &OM_TELEOP::jointStatesCallback, this);
  chain_kinematics_pose_sub_ = node_handle_.subscribe("open_manipulator/kinematics_pose", 10, &OM_TELEOP::kinematicsPoseCallback, this);

}

void OM_TELEOP::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(int i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle = temp_angle;

}

void OM_TELEOP::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position = temp_position;
}

std::vector<double> OM_TELEOP::getPresentJointAngle()
{
  return present_joint_angle;
}
std::vector<double> OM_TELEOP::getPresentKinematicsPose()
{
  return present_kinematic_position;
}

bool OM_TELEOP::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}

bool OM_TELEOP::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}

bool OM_TELEOP::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if(goal_task_space_path_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}

void OM_TELEOP::printText()
{
  printf("Control Your OpenManipulator!\n");
  printf("---------------------------\n");
  printf("Moving around:\n");
  printf("x+ : increase x axis in cartesian space\n");
  printf("x- : decrease x axis in cartesian space\n");
  printf("y+ : increase y axis in cartesian space\n");
  printf("y- : decrease y axis in cartesian space\n");
  printf("z+ : increase z axis in cartesian space\n");
  printf("z- : decrease z axis in cartesian space\n");
  printf("\n");
  printf("j1+ : increase joint 1 angle\n");
  printf("j1- : decrease joint 1 angle\n");
  printf("j2+ : increase joint 2 angle\n");
  printf("j2- : decrease joint 2 angle\n");
  printf("j3+ : increase joint 3 angle\n");
  printf("j3- : decrease joint 3 angle\n");
  printf("j4+ : increase joint 4 angle\n");
  printf("j4- : decrease joint 4 angle\n");
  printf("\n");
  printf("g : gripper open\n");
  printf("f : gripper close\n");
  printf("       \n");
  printf("h : home pose\n");
  printf("i : init pose\n");
  printf("       \n");
  printf("CTRL-C to quit\n");

  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematic Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));

}

void OM_TELEOP::setGoal(std::string inputString)
{
  std::vector<double> goalPose = getPresentKinematicsPose();
  std::vector<double> goalJoint = getPresentJointAngle();
  if((!inputString.compare("x+")) || (!inputString.compare("X+")))
  {
    printf("input : x+ \tincrease(++) x axis in cartesian space\n");
    goalPose.at(0) += DELTA;
    setTaskSpacePath(goalPose, PATH_TIME);
  }
  else if((!inputString.compare("x-")) || (!inputString.compare("X-")))
  {
    printf("input : x- \tdecrease(--) x axis in cartesian space\n");
    goalPose.at(0) -= DELTA;
    setTaskSpacePath(goalPose, PATH_TIME);
  }
  else if((!inputString.compare("y+")) || (!inputString.compare("Y+")))
  {
    printf("input : y+ \tincrease(++) y axis in cartesian space\n");
    goalPose.at(1) += DELTA;
    setTaskSpacePath(goalPose, PATH_TIME);
  }
  else if((!inputString.compare("y-")) || (!inputString.compare("Y-")))
  {
    printf("input : y- \tdecrease(--) y axis in cartesian space\n");
    goalPose.at(1) -= DELTA;
    setTaskSpacePath(goalPose, PATH_TIME);
  }
  else if((!inputString.compare("z+")) || (!inputString.compare("Z+")))
  {
    printf("input : z+ \tincrease(++) z axis in cartesian space\n");
    goalPose.at(2) += DELTA;
    setTaskSpacePath(goalPose, PATH_TIME);
  }
  else if((!inputString.compare("z-")) || (!inputString.compare("Z-")))
  {
    printf("input : z- \tdecrease(--) z axis in cartesian space\n");
    goalPose.at(2) -= DELTA;
    setTaskSpacePath(goalPose, PATH_TIME);
  }
  else if((!inputString.compare("j1+")) || (!inputString.compare("J1+")))
  {
    printf("input : j1+ \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) += JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePath(joint_name, goalJoint, PATH_TIME);
  }
  else if((!inputString.compare("j1-")) || (!inputString.compare("J1-")))
  {
    printf("input : j1- \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) -= JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePath(joint_name, goalJoint, PATH_TIME);
  }

  else if((!inputString.compare("j2+")) || (!inputString.compare("J2+")))
  {
    printf("input : j2+ \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) += JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePath(joint_name, goalJoint, PATH_TIME);
  }
  else if((!inputString.compare("j2-")) || (!inputString.compare("J2-")))
  {
    printf("input : j2- \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) -= JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePath(joint_name, goalJoint, PATH_TIME);
  }

  else if((!inputString.compare("j3+")) || (!inputString.compare("J3+")))
  {
    printf("input : j3+ \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) += JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePath(joint_name, goalJoint, PATH_TIME);
  }
  else if((!inputString.compare("j3-")) || (!inputString.compare("J3-")))
  {
    printf("input : j3- \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) -= JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePath(joint_name, goalJoint, PATH_TIME);
  }

  else if((!inputString.compare("j4+")) || (!inputString.compare("J4+")))
  {
    printf("input : j4+ \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) += JOINT_DELTA;
    setJointSpacePath(joint_name, goalJoint, PATH_TIME);
  }
  else if((!inputString.compare("j4-")) || (!inputString.compare("J4-")))
  {
    printf("input : j4- \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) -= JOINT_DELTA;
    setJointSpacePath(joint_name, goalJoint, PATH_TIME);
  }

  else if(!inputString.compare("g"))
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(-1.0);
    setToolControl(joint_angle);
  }
  else if(!inputString.compare("f"))
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(0.5);
    setToolControl(joint_angle);
  }


  else if(!inputString.compare("h"))
  {
    printf("input : h \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);


  }
  else if(!inputString.compare("i"))
  {
    printf("input : i \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_TELEOP");

  OM_TELEOP om_TELEOP;
  ros::Rate loop_rate(10);

  ROS_INFO("OpenManipulator teleoperation loop start");

  om_TELEOP.printText();

  std::string inputString;
  while (ros::ok())
  {
    om_TELEOP.printText();
    std::getline(std::cin, inputString);
    ros::spinOnce();
    om_TELEOP.setGoal(inputString);
    ros::spinOnce();
    loop_rate.sleep();
  }
  printf("Teleop. is finished\n");

  return 0;
}
