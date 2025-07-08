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

#include <gravity_compensation_controller/gravity_compensation_controller.hpp>
#include <chrono>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/helpers.hpp>

namespace gravity_compensation_controller
{
GravityCompensationController::GravityCompensationController()
: controller_interface::ControllerInterface(),
  dither_switch_(false)
{
}

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
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
GravityCompensationController::state_interface_configuration() const
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

controller_interface::return_type GravityCompensationController::update(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface) {
      for (size_t index = 0; index < n_joints_; ++index) {
        trajectory_point_interface[index] =
          joint_interface[index].get().get_optional().value_or(0.0);
      }
    };

  assign_point_from_interface(joint_positions_, joint_state_interface_[0]);
  assign_point_from_interface(joint_velocities_, joint_state_interface_[1]);

  // Temporary: match the velocity unit to the previous dynamixel_hardware_interface version
  // TODO(Woojin Wie): remove this after the gain is tuned for the new dynamixel_hardware_interface
  //                  version
  for (size_t i = 0; i < joint_velocities_.size(); i++) {
    joint_velocities_[i] = joint_velocities_[i] * 0.01 / 0.229;
  }

  // Calculate acceleration from velocity using finite difference
  std::vector<double> joint_accelerations(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i) {
    joint_accelerations[i] = (joint_velocities_[i] - previous_velocities_[i]) / period.seconds();
  }

  // Create KDL objects for computation
  KDL::TreeIdSolver_RNE idsolver(tree_, KDL::Vector(0, 0, -9.81));
  KDL::JntArray q(tree_.getNrOfJoints());
  KDL::JntArray q_dot(tree_.getNrOfJoints());
  KDL::JntArray q_ddot(tree_.getNrOfJoints());
  KDL::JntArray torques(tree_.getNrOfJoints());

  // Populate joint positions, velocities and accelerations from state interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q(i) = joint_positions_[i];
    q_dot(i) = joint_velocities_[i];
    q_ddot(i) = joint_accelerations[i];
  }

  // Compute torques
  idsolver.CartToJnt(q, q_dot, q_ddot, f_ext_, torques);

  // Add a spring effect to joint 2
  if (q(2) < 0.5) {
    torques(2) += std::abs(q(2) - 0.5) * 2.5;
  }
  // Add leader sync function
  double gain_joint_1_to_3 = 6.0;
  double default_gain = 1.0;
  bool collision = *collision_flag_buffer_.readFromRT();

  if (collision && has_follower_data_) {
    auto follower_positions_ptr = follower_joint_positions_buffer_.readFromRT();
    if (follower_positions_ptr) {
      for (size_t i = 0; i < n_joints_; ++i) {
        double error = (*follower_positions_ptr)[i] - joint_positions_[i];
        double gain = (i <= 2) ? gain_joint_1_to_3 : default_gain;
        torques(i) += gain * error;
      }
    }
  }

  // Apply friction compensation
  for (size_t i = 0; i < tree_.getNrOfJoints(); ++i) {
    if (i >= joint_names_.size()) {
      continue;
    }

    double kinetic_friction_scalar = params_.kinetic_friction_scalars[i] *
      (1.0 + std::abs(torques(i) * params_.kinetic_friction_torque_scalars[i]));

    // Kinetic friction compensation
    double kinetic_friction_rate = 1.0 -
      (std::abs(q_dot(i)) * 10.0 - params_.friction_compensation_velocity_thresholds[i]);
    if (kinetic_friction_rate < 0.0) {
      kinetic_friction_rate = 0.0;
    }
    kinetic_friction_scalar *= kinetic_friction_rate;

    if (q_dot(i) > 0.0) {
      torques(i) += kinetic_friction_scalar * std::abs(q_dot(i));

      if (std::abs(torques(i)) < params_.unloaded_effort_thresholds[i]) {
        torques(i) += params_.unloaded_effort_offsets[i];
      }
    } else if (q_dot(i) < 0.0) {
      torques(i) -= kinetic_friction_scalar * std::abs(q_dot(i));

      if (std::abs(torques(i)) < params_.unloaded_effort_thresholds[i]) {
        torques(i) -= params_.unloaded_effort_offsets[i];
      }
    }

    // Static friction compensation (dithering)
    if (std::abs(q_dot(i)) < params_.static_friction_velocity_thresholds[i]) {
      if (dither_switch_) {
        torques(i) += params_.static_friction_scalars[i] * std::abs(torques(i));
      } else {
        torques(i) -= params_.static_friction_scalars[i] * std::abs(torques(i));
      }
    }

    bool set_ok = joint_command_interface_[0][i].get().set_value(
      torques(i) * params_.torque_scaling_factors[i]);
    if (!set_ok) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to set command value for joint %zu, interface %u", i, 0);
    }
  }

  // Update previous velocities for next iteration
  previous_velocities_ = joint_velocities_;

  dither_switch_ = !dither_switch_;  // Flip the dither switch

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GravityCompensationController::on_init()
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

controller_interface::CallbackReturn GravityCompensationController::on_configure(
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

  // get degrees of freedom
  n_joints_ = params_.joints.size();
  joint_names_ = params_.joints;
  collision_flag_buffer_.writeFromNonRT(false);
  joint_positions_.resize(n_joints_);
  joint_velocities_.resize(n_joints_);
  previous_velocities_.resize(n_joints_);  // Initialize previous velocities vector
  joint_name_to_index_.resize(joint_names_.size(), -1);
  tmp_positions_.resize(joint_names_.size(), 0.0);

  follower_joint_state_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (msg->name.size() != msg->position.size()) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "JointState message has mismatched name/position sizes");
        return;
      }

      if (!joint_index_initialized_) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
          if (it != msg->name.end()) {
            joint_name_to_index_[i] = static_cast<int>(std::distance(msg->name.begin(), it));
          } else {
            RCLCPP_ERROR(
              get_node()->get_logger(),
              "Joint name '%s' not found in the first joint state message",
              joint_names_[i].c_str());
            return;
          }
        }
        joint_index_initialized_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "Joint index mapping initialized.");
      }

      for (size_t i = 0; i < joint_names_.size(); ++i) {
        tmp_positions_[i] = msg->position[joint_name_to_index_[i]];
      }

      follower_joint_positions_buffer_.writeFromNonRT(tmp_positions_);
      has_follower_data_ = true;
    });

  collision_flag_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/collision_flag", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      collision_flag_buffer_.writeFromNonRT(msg->data);
    });

  if (params_.joints.empty()) {
    // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty()) {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }

  joint_command_interface_.resize(command_interface_types_.size());
  joint_state_interface_.resize(state_interface_types_.size());

  const std::string & urdf = get_robot_description();
  if (!urdf.empty()) {
    if (!kdl_parser::treeFromString(urdf, tree_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot description!");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "[KDL] number of joints: %d", tree_.getNrOfJoints());

    for (const auto & segment : tree_.getSegments()) {
      RCLCPP_INFO(get_node()->get_logger(), "[KDL] segment name: %s", segment.first.c_str());
    }

    RCLCPP_INFO(get_node()->get_logger(), "Successfully parsed the robot description.");

    q_ddot_.resize(tree_.getNrOfJoints());
  } else {
    // empty URDF is used for some tests
    RCLCPP_DEBUG(get_node()->get_logger(), "No URDF file given");
  }

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();
  // order all joints in the storage
  for (const auto & interface : params_.command_interfaces) {
    auto it =
      std::find(command_interface_types_.begin(), command_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(command_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' command interfaces, got %zu.", n_joints_, interface.c_str(),
        joint_command_interface_[index].size());
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
        logger, "Expected %zu '%s' state interfaces, got %zu.", n_joints_, interface.c_str(),
        joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
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
  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset flags and parameters
  dither_switch_ = false;

  // Clear KDL tree and joint name map
  tree_ = KDL::Tree();

  // Clear vectors
  f_ext_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController cleaned up successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_node()->get_logger(), "Error occurred in GravityCompensationController.");
  return CallbackReturn::ERROR;
}

controller_interface::CallbackReturn GravityCompensationController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Shutting down GravityCompensationController.");
  return CallbackReturn::SUCCESS;
}

std::string GravityCompensationController::formatVector(const std::vector<double> & vec)
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

}  // namespace gravity_compensation_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gravity_compensation_controller::GravityCompensationController,
  controller_interface::ControllerInterface)
