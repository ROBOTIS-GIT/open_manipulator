/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "open_manipulator_x_teleop/open_manipulator_x_teleop_keyboard.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;


namespace open_manipulator_x_teleop_keyboard
{
OpenManipulatorXTeleopKeyboard::OpenManipulatorXTeleopKeyboard()
: Node("open_manipulator_x_teleop_keyboard")
{
  /********************************************************************************
  ** Initialise variables
  ********************************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /********************************************************************************
  ** Initialise Subscribers
  ********************************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "open_manipulator_x/joint_states", qos, std::bind(&OpenManipulatorXTeleopKeyboard::joint_states_callback, this, _1));
  kinematics_pose_sub_ = this->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
    "open_manipulator_x/kinematics_pose", qos, std::bind(&OpenManipulatorXTeleopKeyboard::kinematics_pose_callback, this, _1));

  /********************************************************************************
  ** Initialise Clients
  ********************************************************************************/
  goal_joint_space_path_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_joint_space_path");
  goal_tool_control_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_tool_control");
  goal_task_space_path_from_present_position_only_client_ = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>(
    "open_manipulator_x/goal_task_space_path_from_present_position_only");
  goal_joint_space_path_from_present_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_joint_space_path_from_present");

  /********************************************************************************
  ** Display in terminal
  ********************************************************************************/
  this->disable_waiting_for_enter();
  update_timer_ = this->create_wall_timer(10ms, std::bind(&OpenManipulatorXTeleopKeyboard::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X Teleop Keyboard Initialised");
}

OpenManipulatorXTeleopKeyboard::~OpenManipulatorXTeleopKeyboard() 
{
  this->restore_terminal_settings();
  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X Teleop Keyboard Terminated");
}

/********************************************************************************
** Callback Functions
********************************************************************************/
void OpenManipulatorXTeleopKeyboard::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (uint8_t i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1")) temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2")) temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3")) temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4")) temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorXTeleopKeyboard::kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

/********************************************************************************
** Callback Functions and Relevant Functions
********************************************************************************/
void OpenManipulatorXTeleopKeyboard::set_goal(char ch)
{
  std::vector<double> goalPose; goalPose.resize(3);
  std::vector<double> goalJoint; goalJoint.resize(4);
  const double delta = 0.01;
  const double joint_delta = 0.05;
  double path_time = 0.5;

  if (ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = delta;
    set_task_space_path_from_present_position_only(goalPose, path_time);
  }
  else if (ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -delta;
    set_task_space_path_from_present_position_only(goalPose, path_time);
  }
  else if (ch == 'a' || ch == 'A')
  {
    printf("input : a \tincrease(++) y axis in task space\n");
    goalPose.at(1) = delta;
    set_task_space_path_from_present_position_only(goalPose, path_time);
  }
  else if (ch == 'd' || ch == 'D')
  {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goalPose.at(1) = -delta;
    set_task_space_path_from_present_position_only(goalPose, path_time);
  }
  else if (ch == 'z' || ch == 'Z')
  {
    printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = delta;
    set_task_space_path_from_present_position_only(goalPose, path_time);
  }
  else if (ch == 'x' || ch == 'X')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -delta;
    set_task_space_path_from_present_position_only(goalPose, path_time);
  }
  else if (ch == 'y' || ch == 'Y')
  {
    printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = joint_delta;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goalJoint, path_time);
  }
  else if (ch == 'h' || ch == 'H')
  {
    printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = -joint_delta;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goalJoint, path_time);
  }
  else if (ch == 'u' || ch == 'U')
  {
    printf("input : u \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = joint_delta;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goalJoint, path_time);
  }
  else if (ch == 'j' || ch == 'J')
  {
    printf("input : j \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = -joint_delta;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goalJoint, path_time);
  }

  else if (ch == 'i' || ch == 'I')
  {
    printf("input : i \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = joint_delta;
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goalJoint, path_time);
  }
  else if (ch == 'k' || ch == 'K')
  {
    printf("input : k \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = -joint_delta;
    joint_name.push_back("joint4");
    set_joint_space_path_from_present(joint_name, goalJoint, path_time);
  }

  else if (ch == 'o' || ch == 'O')
  {
    printf("input : o \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = joint_delta;
    set_joint_space_path_from_present(joint_name, goalJoint, path_time);
  }
  else if (ch == 'l' || ch == 'L')
  {
    printf("input : l \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = -joint_delta;
    set_joint_space_path_from_present(joint_name, goalJoint, path_time);
  }
  else if (ch == 'g' || ch == 'G')
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    set_tool_control(joint_angle);
  }
  else if (ch == 'f' || ch == 'F')
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    set_tool_control(joint_angle);
  }
  else if (ch == '2')
  {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-PI/3);
    joint_name.push_back("joint3"); joint_angle.push_back(PI/9);
    joint_name.push_back("joint4"); joint_angle.push_back(PI*2/9);
    set_joint_space_path(joint_name, joint_angle, path_time);
  }
  else if (ch == '1')
  {
    printf("input : 1 \tinit pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    set_joint_space_path(joint_name, joint_angle, path_time);
  }
  else if (ch == 'q')
  {
    printf("Pressed Quit \n");
    rclcpp::shutdown();
  }
}

bool OpenManipulatorXTeleopKeyboard::set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;
  
  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXTeleopKeyboard::set_tool_control(std::vector<double> joint_angle)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name.push_back("gripper");
  request->joint_position.position = joint_angle;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_tool_control_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXTeleopKeyboard::set_task_space_path_from_present_position_only(std::vector<double> kinematics_pose, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
  request->planning_group = "gripper";
  request->kinematics_pose.pose.position.x = kinematics_pose.at(0);
  request->kinematics_pose.pose.position.y = kinematics_pose.at(1);
  request->kinematics_pose.pose.position.z = kinematics_pose.at(2);
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_task_space_path_from_present_position_only_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXTeleopKeyboard::set_joint_space_path_from_present(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_from_present_client_->async_send_request(request, response_received_callback);

  return false;
}

/********************************************************************************
** Other Functions
********************************************************************************/
void OpenManipulatorXTeleopKeyboard::print_text()
{
  system("clear");
  printf("\n");
  printf("-----------------------------\n");
  printf("Control Your OpenManipulator!\n");
  printf("-----------------------------\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  printf("z : increase z axis in task space\n");
  printf("x : decrease z axis in task space\n");
  printf("\n");
  printf("y : increase joint 1 angle\n");
  printf("h : decrease joint 1 angle\n");
  printf("u : increase joint 2 angle\n");
  printf("j : decrease joint 2 angle\n");
  printf("i : increase joint 3 angle\n");
  printf("k : decrease joint 3 angle\n");
  printf("o : increase joint 4 angle\n");
  printf("l : decrease joint 4 angle\n");
  printf("\n");
  printf("g : gripper open\n");
  printf("f : gripper close\n");
  printf("       \n");
  printf("1 : init pose\n");
  printf("2 : home pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
    get_present_joint_angle().at(0),
    get_present_joint_angle().at(1),
    get_present_joint_angle().at(2),
    get_present_joint_angle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
    get_present_kinematics_pose().at(0),
    get_present_kinematics_pose().at(1),
    get_present_kinematics_pose().at(2));
  printf("---------------------------\n");  
}

std::vector<double> OpenManipulatorXTeleopKeyboard::get_present_joint_angle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorXTeleopKeyboard::get_present_kinematics_pose()
{
  return present_kinematic_position_;
}

void OpenManipulatorXTeleopKeyboard::restore_terminal_settings()
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorXTeleopKeyboard::disable_waiting_for_enter()
{
  struct termios newt;

  tcgetattr(0, &oldt_);             /* Save terminal settings */
  newt = oldt_;                     /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO); /* Change settings */
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &newt);     /* Apply settings */
}

void OpenManipulatorXTeleopKeyboard::update_callback()  
{
  this->print_text();  
  
  char ch = std::getchar();
  this->set_goal(ch);
}
}  // namespace open_manipulator_x_teleop_keyboard

/********************************************************************************
** Main
********************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<open_manipulator_x_teleop_keyboard::OpenManipulatorXTeleopKeyboard>());
  rclcpp::shutdown();

  return 0;
}
