// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Hye-jong KIM, Sungho Woo

#include <algorithm>
#include <memory>

#include "open_manipulator_x_teleop/open_manipulator_x_teleop.hpp"

// KeyboardReader
KeyboardReader::KeyboardReader()
: kfd(0)
{
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  struct termios raw;
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}

void KeyboardReader::readOne(char * c)
{
  int rc = read(kfd, c, 1);
  if (rc < 0) {
    throw std::runtime_error("read failed");
  }
}

void KeyboardReader::shutdown()
{
  tcsetattr(kfd, TCSANOW, &cooked);
}

// KeyboardServo

KeyboardServo::KeyboardServo()
: publish_joint_(false), publish_gripper_(false)
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");

  servo_start_client_ =
    nh_->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  servo_stop_client_ =
    nh_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");

  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(ARM_JOINT_TOPIC, ROS_QUEUE_SIZE);
  client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(nh_, "gripper_controller/gripper_cmd");
}

KeyboardServo::~KeyboardServo()
{
  stop_moveit_servo();
}

int KeyboardServo::keyLoop()
{
  char c;

  // Ros Spin
  std::thread{std::bind(&KeyboardServo::spin, this)}.detach();
  connect_moveit_servo();
  start_moveit_servo();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Joint Control Keys:");
  puts("  1/q: Joint1 +/-");
  puts("  2/w: Joint2 +/-");
  puts("  3/e: Joint3 +/-");
  puts("  4/r: Joint4 +/-");
  puts("Use o|p to open/close the gripper.");
  puts("'ESC' to quit.");

  std::thread{std::bind(&KeyboardServo::pub, this)}.detach();

  bool servoing = true;
  while (servoing) {
    // get the next event from the keyboard
    try {
      input.readOne(&c);
    } catch (const std::runtime_error &) {
      perror("read():");
      return -1;
    }

    RCLCPP_INFO(nh_->get_logger(), "value: 0x%02X", c);

    // Use read key-press
    joint_msg_.joint_names.clear();
    joint_msg_.velocities.clear();

    switch (c) {
      case KEYCODE_1:
        joint_msg_.joint_names.push_back("joint1");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint1 +");
        break;
      case KEYCODE_2:
        joint_msg_.joint_names.push_back("joint2");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint2 +");
        break;
      case KEYCODE_3:
        joint_msg_.joint_names.push_back("joint3");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint3 +");
        break;
      case KEYCODE_4:
        joint_msg_.joint_names.push_back("joint4");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint4 +");
        break;
      case KEYCODE_Q:
        joint_msg_.joint_names.push_back("joint1");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint1 -");
        break;
      case KEYCODE_W:
        joint_msg_.joint_names.push_back("joint2");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint2 -");
        break;
      case KEYCODE_E:
        joint_msg_.joint_names.push_back("joint3");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint3 -");
        break;
      case KEYCODE_R:
        joint_msg_.joint_names.push_back("joint4");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint4 -");
        break;
      case KEYCODE_O:
        send_goal(0.019);
        publish_gripper_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Gripper Open");
        break;
      case KEYCODE_P:
        send_goal(-0.01);
        publish_gripper_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Gripper Close");
        break;
      case KEYCODE_ESC:
        RCLCPP_INFO_STREAM(nh_->get_logger(), "quit");
        servoing = false;
        break;
      default:
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Unassigned input : " << c);
        break;
    }
  }

  return 0;
}

void KeyboardServo::send_goal(float position)
{
    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 100.0;

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&KeyboardServo::goal_result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(nh_->get_logger(), "Sending goal");
    client_->async_send_goal(goal_msg, send_goal_options);
}

void KeyboardServo::connect_moveit_servo()
{
  for (int i = 0; i < 10; i++) {
    if (servo_start_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS TO CONNECT SERVO START SERVER");
      break;
    }
    RCLCPP_WARN_STREAM(nh_->get_logger(), "WAIT TO CONNECT SERVO START SERVER");
    if (i == 9) {
      RCLCPP_ERROR_STREAM(
        nh_->get_logger(),
        "fail to connect moveit_servo." <<
          "please launch 'servo.launch' at 'open_manipulator_x_moveit_configs' pkg.");
    }
  }
  for (int i = 0; i < 10; i++) {
    if (servo_stop_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS TO CONNECT SERVO STOP SERVER");
      break;
    }
    RCLCPP_WARN_STREAM(nh_->get_logger(), "WAIT TO CONNECT SERVO STOP SERVER");
    if (i == 9) {
      RCLCPP_ERROR_STREAM(
        nh_->get_logger(),
        "fail to connect moveit_servo." <<
          "please launch 'servo.launch' at 'open_manipulator_x_moveit_configs' pkg.");
    }
  }
}

void KeyboardServo::start_moveit_servo()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "call 'moveit_servo' start srv.");
  auto future = servo_start_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  auto result = future.wait_for(std::chrono::seconds(1));
  if (result == std::future_status::ready) {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS to start 'moveit_servo'");
    future.get();
  } else {
    RCLCPP_ERROR_STREAM(
      nh_->get_logger(), "FAIL to start 'moveit_servo', execute without 'moveit_servo'");
  }
}

void KeyboardServo::stop_moveit_servo()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "call 'moveit_servo' END srv.");
  auto future = servo_stop_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  auto result = future.wait_for(std::chrono::seconds(1));
  if (result == std::future_status::ready) {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS to stop 'moveit_servo'");
    future.get();
  }
}

void KeyboardServo::pub()
{
  while (rclcpp::ok()) {
    if (publish_joint_) {
      joint_msg_.header.stamp = nh_->now();
      joint_msg_.header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(joint_msg_);
      publish_joint_ = false;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint PUB");
    }
    if (publish_gripper_) {
      publish_gripper_ = false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

void KeyboardServo::spin()
{
  while (rclcpp::ok()) {
    rclcpp::spin_some(nh_);
  }
}
