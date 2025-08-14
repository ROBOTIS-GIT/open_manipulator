// Copyright 2021 ros2_control development team
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

#include "joint_trajectory_command_broadcaster/joint_trajectory_command_broadcaster.hpp"

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <atomic>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/bool.hpp"
#include "urdf/model.h"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace joint_trajectory_command_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_POSITION;

JointTrajectoryCommandBroadcaster::JointTrajectoryCommandBroadcaster() {}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointTrajectoryCommandBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration JointTrajectoryCommandBroadcaster::
state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  if (use_all_available_interfaces()) {
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
  } else {
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : params_.joints) {
      for (const auto & interface : params_.interfaces) {
        state_interfaces_config.names.push_back(joint + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (use_all_available_interfaces()) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'joints' or 'interfaces' parameter is empty. "
      "All available state interfaces will be considered.");
    params_.joints.clear();
    params_.interfaces.clear();
  } else {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Publishing trajectory states for the defined 'joints' and 'interfaces' parameters.");
  }

  // Map interface if needed
  map_interface_to_joint_state_.clear();
  map_interface_to_joint_state_[HW_IF_POSITION] = params_.map_interface_to_joint_state.position;

  try {
    const std::string topic_name_prefix = params_.use_local_topics ? "~/" : "";
    // Create publisher for JointTrajectory
    joint_trajectory_publisher_ =
      get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      topic_name_prefix + "joint_trajectory", rclcpp::SystemDefaultsQoS());

    realtime_joint_trajectory_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>(
      joint_trajectory_publisher_);
  } catch (const std::exception & e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  collision_flag_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/collision_flag", rclcpp::QoS(10),
    std::bind(&JointTrajectoryCommandBroadcaster::collision_callback, this, std::placeholders::_1));

  const std::string & urdf = get_robot_description();
  is_model_loaded_ = !urdf.empty() && model_.initString(urdf);
  if (!is_model_loaded_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to parse robot description. Will proceed without URDF-based filtering.");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }
  // Check offsets and create mapping based on params_.joints order
  joint_offsets_.clear();
  joint_offsets_.resize(params_.joints.size(), 0.0);

  if (!params_.offsets.empty()) {
    if (params_.offsets.size() != params_.joints.size()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "The number of provided offsets (%zu) does not match the number of joints in params (%zu).",
        params_.offsets.size(), params_.joints.size());
      return CallbackReturn::ERROR;
    }

    // Create mapping from joint name to offset based on params_.joints order
    std::unordered_map<std::string, double> joint_offset_map;
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      joint_offset_map[params_.joints[i]] = params_.offsets[i];
    }

    // Apply offsets to joint_names_ in their actual order
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = joint_offset_map.find(joint_names_[i]);
      if (it != joint_offset_map.end()) {
        joint_offsets_[i] = it->second;
      }
      // If joint not found in params_.joints, offset remains 0.0
    }
  }
  // No need to init JointState or DynamicJointState messages, only JointTrajectory
  // will be published. We'll construct it on-the-fly in update()

  if (
    !use_all_available_interfaces() &&
    state_interfaces_.size() != (params_.joints.size() * params_.interfaces.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Not all requested interfaces exist. "
      "Check ControllerManager output for more detailed information.");
  }

  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  name_if_value_mapping_.clear();

  return CallbackReturn::SUCCESS;
}

template<typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  for (const auto & key_item : map) {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend()) {
      return true;
    }
  }
  return false;
}

bool JointTrajectoryCommandBroadcaster::init_joint_data()
{
  joint_names_.clear();
  if (state_interfaces_.empty()) {
    return false;
  }

  // Initialize mapping
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++) {
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0) {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    std::string interface_name = si->get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = kUninitializedValue;
  }

  // Filter out joints without position interface (since we want positions)
  for (const auto & name_ifv : name_if_value_mapping_) {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {HW_IF_POSITION})) {
      if (
        !params_.use_urdf_to_filter || !params_.joints.empty() || !is_model_loaded_ ||
        model_.getJoint(name_ifv.first))
      {
        joint_names_.push_back(name_ifv.first);
      }
    }
  }

  // Add extra joints if needed
  rclcpp::Parameter extra_joints;
  if (get_node()->get_parameter("extra_joints", extra_joints)) {
    const std::vector<std::string> & extra_joints_names = extra_joints.as_string_array();
    for (const auto & extra_joint_name : extra_joints_names) {
      if (name_if_value_mapping_.count(extra_joint_name) == 0) {
        name_if_value_mapping_[extra_joint_name] = {
          {HW_IF_POSITION, 0.0}};
        joint_names_.push_back(extra_joint_name);
      }
    }
  }

  return true;
}

bool JointTrajectoryCommandBroadcaster::use_all_available_interfaces() const
{
  return params_.joints.empty() || params_.interfaces.empty();
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend()) {
    return interface_and_value->second;
  } else {
    return kUninitializedValue;
  }
}

controller_interface::return_type JointTrajectoryCommandBroadcaster::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (collision_detected_.load()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Collision detected. Skipping joint_trajectory publish.");
    return controller_interface::return_type::OK;
  }
  // Update stored values
  for (const auto & state_interface : state_interfaces_) {
    std::string interface_name = state_interface.get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    auto value = state_interface.get_optional();
    if (value) {
      name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] = *value;
    }
  }

  // Publish JointTrajectory message with current positions
  if (realtime_joint_trajectory_publisher_ && realtime_joint_trajectory_publisher_->trylock()) {
    auto & traj_msg = realtime_joint_trajectory_publisher_->msg_;
    traj_msg.header.stamp = rclcpp::Time(0, 0);
    traj_msg.joint_names = joint_names_;

    const size_t num_joints = joint_names_.size();
    traj_msg.points.clear();
    traj_msg.points.resize(1);
    traj_msg.points[0].positions.resize(num_joints, kUninitializedValue);

    for (size_t i = 0; i < num_joints; ++i) {
      double pos_value =
        get_value(name_if_value_mapping_, joint_names_[i], HW_IF_POSITION);

      // Check if the current joint is in the reverse_joints parameter
      if (
        std::find(
          params_.reverse_joints.begin(),
          params_.reverse_joints.end(),
          joint_names_[i]) != params_.reverse_joints.end())
      {
        pos_value = -pos_value;
      }

      // Apply offset
      pos_value += joint_offsets_[i];

      traj_msg.points[0].positions[i] = pos_value;
    }

    // Optionally set velocities/accelerations/time_from_start if needed
    traj_msg.points[0].time_from_start = rclcpp::Duration(0, 0);  // immediate

    realtime_joint_trajectory_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

void JointTrajectoryCommandBroadcaster::collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  collision_detected_.store(msg->data);
}

}  // namespace joint_trajectory_command_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_command_broadcaster::JointTrajectoryCommandBroadcaster,
  controller_interface::ControllerInterface)
