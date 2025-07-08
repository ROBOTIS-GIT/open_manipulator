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
// Author: Woojin Wie

#include <spring_actuator_controller/spring_actuator_controller.hpp>

#include <chrono>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/helpers.hpp"

namespace spring_actuator_controller
{

SpringActuatorController::SpringActuatorController()
: controller_interface::ControllerInterface(),
  dither_switch_(false)
{
}

controller_interface::InterfaceConfiguration
SpringActuatorController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : command_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
SpringActuatorController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

controller_interface::return_type SpringActuatorController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ----------------------------------------------------------------------------
  // 1) Read joint states from hardware interfaces
  // ----------------------------------------------------------------------------
  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface) {
      for (size_t index = 0; index < dof_; ++index) {
        trajectory_point_interface[index] =
          joint_interface[index].get().get_optional().value_or(0.0);
      }
    };

  // Fill in current joint positions and velocities
  assign_point_from_interface(joint_positions_, joint_state_interface_[0]);  // position
  assign_point_from_interface(joint_velocities_, joint_state_interface_[1]);  // velocity

  // We will set torques for each joint
  std::vector<double> torques(dof_, 0.0);

  for (size_t trigger_joint_idx = 0; trigger_joint_idx < dof_; ++trigger_joint_idx) {
    double q_trigger = joint_positions_[trigger_joint_idx];
    double spring_stiffness = params_.trigger_spring_stiffness[trigger_joint_idx];
    double neutral_position = params_.trigger_neutral_position[trigger_joint_idx];

    double spring_torque = -spring_stiffness * (q_trigger - neutral_position);

    // Optional friction or damping or dithering if needed:
    // For example, a small damping: T -= damping * q_dot
    double damping_coefficient = params_.trigger_damping[trigger_joint_idx];
    double q_dot_trigger = joint_velocities_[trigger_joint_idx];
    spring_torque -= damping_coefficient * q_dot_trigger;

    // If you still want dithering to overcome static friction:
    if (std::abs(q_dot_trigger) < params_.static_friction_velocity_thresholds[trigger_joint_idx]) {
      double dither = params_.static_friction_scalars[trigger_joint_idx] * std::abs(spring_torque);
      spring_torque += (dither_switch_ ? dither : -dither);
    }

    torques[trigger_joint_idx] = spring_torque;
  }

  // Flip the dither switch for next update cycle
  dither_switch_ = !dither_switch_;

  // ----------------------------------------------------------------------------
  // 3) Output the final torques to hardware command interfaces
  // ----------------------------------------------------------------------------
  for (size_t i = 0; i < dof_; ++i) {
    // Multiply by any user-defined torque scaling if desired
    double scaled_torque = torques[i] * params_.torque_scaling_factors[i];
    bool set_ok = joint_command_interface_[0][i].get().set_value(scaled_torque);
    if (!set_ok) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command value for joint %zu", i);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn SpringActuatorController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpringActuatorController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // degrees of freedom
  dof_ = params_.joints.size();

  joint_positions_.resize(dof_);
  joint_velocities_.resize(dof_);

  if (params_.joints.empty()) {
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  joint_names_ = params_.joints;
  n_joints_ = joint_names_.size();

  command_joint_names_ = params_.command_joints;
  if (command_joint_names_.empty()) {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names provided for command interfaces. Using 'joints'.");
  } else if (command_joint_names_.size() != params_.joints.size()) {
    RCLCPP_ERROR(
      logger, "'command_joints' parameter must match size of 'joints' parameter.");
    return CallbackReturn::FAILURE;
  }

  joint_command_interface_.resize(command_interface_types_.size());
  joint_state_interface_.resize(state_interface_types_.size());

  RCLCPP_INFO(get_node()->get_logger(), "SpringActuatorController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpringActuatorController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // Order all joints in the storage
  for (const auto & interface : params_.command_interfaces) {
    auto it =
      std::find(command_interface_types_.begin(), command_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(command_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' command interfaces, got %zu.",
        dof_, interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  for (const auto & interface : params_.state_interfaces) {
    auto it =
      std::find(state_interface_types_.begin(), state_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(state_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.",
        dof_, interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "SpringActuatorController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpringActuatorController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < n_joints_; ++i) {
    for (size_t j = 0; j < command_interface_types_.size(); ++j) {
      bool set_ok = command_interfaces_[i * command_interface_types_.size() + j].set_value(0.0);
      if (!set_ok) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to reset command value for joint %zu, interface %zu", i, j);
      }
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "SpringActuatorController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpringActuatorController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  dither_switch_ = false;

  RCLCPP_INFO(get_node()->get_logger(), "SpringActuatorController cleaned up successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpringActuatorController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_node()->get_logger(), "Error occurred in SpringActuatorController.");
  return CallbackReturn::ERROR;
}

controller_interface::CallbackReturn SpringActuatorController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Shutting down SpringActuatorController.");
  return CallbackReturn::SUCCESS;
}

std::string SpringActuatorController::formatVector(const std::vector<double> & vec)
{
  std::ostringstream oss;
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];
    if (i != vec.size() - 1) {
      oss << ", ";
    }
  }
  return oss.str();
}

}  // namespace spring_actuator_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  spring_actuator_controller::SpringActuatorController,
  controller_interface::ControllerInterface)
