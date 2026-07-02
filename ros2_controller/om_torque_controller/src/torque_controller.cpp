#include <torque_controller/torque_controller.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

namespace torque_controller
{

TorqueController::TorqueController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration TorqueController::command_interface_configuration()
const
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

controller_interface::InterfaceConfiguration TorqueController::state_interface_configuration() const
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

controller_interface::return_type TorqueController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
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

  for (size_t i = 0; i < joint_velocities_.size(); ++i) {
    joint_velocities_[i] *= params_.input_velocity_scaling_factors[i];
  }

  std::vector<double> joint_accelerations(n_joints_);
  const double period_seconds = period.seconds();
  for (size_t i = 0; i < n_joints_; ++i) {
    if (period_seconds > 0.0) {
      joint_accelerations[i] =
        (joint_velocities_[i] - previous_velocities_[i]) / period_seconds *
        params_.input_acceleration_scaling_factors[i];
    } else {
      joint_accelerations[i] = 0.0;
    }
  }

  auto command_received_ptr = command_received_buffer_.readFromRT();
  auto last_command_time_ptr = last_command_time_buffer_.readFromRT();
  const bool command_received = command_received_ptr && *command_received_ptr;
  const bool command_is_fresh =
    command_received &&
    last_command_time_ptr &&
    (
    params_.command_timeout <= 0.0 ||
    (time - *last_command_time_ptr).seconds() <= params_.command_timeout);

  KDL::JntArray q(tree_.getNrOfJoints());
  KDL::JntArray q_dot(tree_.getNrOfJoints());
  KDL::JntArray q_ddot(tree_.getNrOfJoints());
  KDL::JntArray torques(tree_.getNrOfJoints());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q(i) = joint_positions_[i];
    q_dot(i) = command_is_fresh ? joint_velocities_[i] : 0.0;
    q_ddot(i) = command_is_fresh ? joint_accelerations[i] : 0.0;
    torques(i) = 0.0;
  }

  if (compensate_gravity_.load()) {
    KDL::TreeIdSolver_RNE idsolver(tree_, KDL::Vector(0, 0, -9.81));
    idsolver.CartToJnt(q, q_dot, q_ddot, f_ext_, torques);
  }

  auto command_torques_ptr = command_torque_buffer_.readFromRT();
  if (command_is_fresh && command_torques_ptr) {
    for (size_t i = 0; i < n_joints_; ++i) {
      torques(i) += (*command_torques_ptr)[i];
    }
  }

  for (size_t i = 0; i < tree_.getNrOfJoints(); ++i) {
    if (i >= joint_names_.size()) {
      continue;
    }

    const double friction_velocity = joint_velocities_[i];
    double kinetic_friction_scalar = params_.kinetic_friction_scalars[i] *
      (1.0 + std::abs(torques(i) * params_.kinetic_friction_torque_scalars[i]));

    double kinetic_friction_rate = 1.0 -
      (std::abs(friction_velocity) * 10.0 - params_.friction_compensation_velocity_thresholds[i]);
    if (kinetic_friction_rate < 0.0) {
      kinetic_friction_rate = 0.0;
    }
    kinetic_friction_scalar *= kinetic_friction_rate;

    if (friction_velocity > 0.0) {
      torques(i) += kinetic_friction_scalar * std::abs(friction_velocity);

      if (std::abs(torques(i)) < params_.unloaded_effort_thresholds[i]) {
        torques(i) += params_.unloaded_effort_offsets[i];
      }
    } else if (friction_velocity < 0.0) {
      torques(i) -= kinetic_friction_scalar * std::abs(friction_velocity);

      if (std::abs(torques(i)) < params_.unloaded_effort_thresholds[i]) {
        torques(i) -= params_.unloaded_effort_offsets[i];
      }
    }

    if (std::abs(friction_velocity) < params_.static_friction_velocity_thresholds[i]) {
      const double static_dither_torque =
        params_.static_friction_offsets[i] +
        params_.static_friction_scalars[i] * std::abs(torques(i));
      if (dither_switch_) {
        torques(i) += static_dither_torque;
      } else {
        torques(i) -= static_dither_torque;
      }
    }

    bool set_ok = joint_command_interface_[0][i].get().set_value(
      torques(i) * params_.torque_scaling_factors[i]);
    if (!set_ok) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to set command value for joint %zu, interface %u", i, 0);
    }
  }

  previous_velocities_ = joint_velocities_;
  dither_switch_ = !dither_switch_;

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TorqueController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    compensate_gravity_.store(params_.compensate_gravity);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void TorqueController::configure_subscribers()
{
  torque_command_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory", rclcpp::QoS(1),
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      if (msg->points.empty()) {
        RCLCPP_WARN(get_node()->get_logger(), "Ignoring empty torque trajectory command");
        return;
      }

      const auto & effort = msg->points.front().effort;
      if (effort.empty()) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Ignoring torque trajectory command without point effort values");
        return;
      }

      std::vector<double> command_torques(n_joints_, 0.0);

      if (msg->joint_names.empty()) {
        if (effort.size() != n_joints_) {
          RCLCPP_WARN(
            get_node()->get_logger(),
            "Ignoring torque trajectory command with %zu effort values; expected %zu",
            effort.size(), n_joints_);
          return;
        }
        command_torques = effort;
        command_torque_buffer_.writeFromNonRT(command_torques);
        command_received_buffer_.writeFromNonRT(true);
        last_command_time_buffer_.writeFromNonRT(get_node()->now());
        return;
      }

      if (msg->joint_names.size() != effort.size()) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Ignoring torque trajectory command with %zu joint names and %zu effort values",
          msg->joint_names.size(), effort.size());
        return;
      }

      for (size_t i = 0; i < n_joints_; ++i) {
        auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_names_[i]);
        if (it == msg->joint_names.end()) {
          RCLCPP_WARN(
            get_node()->get_logger(),
            "Ignoring torque trajectory command missing joint '%s'", joint_names_[i].c_str());
          return;
        }
        command_torques[i] =
          effort[static_cast<size_t>(std::distance(msg->joint_names.begin(), it))];
      }

      command_torque_buffer_.writeFromNonRT(command_torques);
      command_received_buffer_.writeFromNonRT(true);
      last_command_time_buffer_.writeFromNonRT(get_node()->now());
    });
}

controller_interface::CallbackReturn TorqueController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
  compensate_gravity_.store(params_.compensate_gravity);

  n_joints_ = params_.joints.size();
  joint_names_ = params_.joints;
  joint_positions_.resize(n_joints_);
  joint_velocities_.resize(n_joints_);
  previous_velocities_.resize(n_joints_);
  command_torque_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
  command_received_buffer_.writeFromNonRT(false);
  last_command_time_buffer_.writeFromNonRT(get_node()->now());

  if (params_.joints.empty()) {
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  command_joint_names_ = params_.command_joints;
  if (command_joint_names_.empty()) {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  } else if (command_joint_names_.size() != params_.joints.size()) {
    RCLCPP_ERROR(logger, "'command_joints' parameter must match size of 'joints' parameter.");
    return CallbackReturn::ERROR;
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
    RCLCPP_INFO(get_node()->get_logger(), "Successfully parsed the robot description.");
  } else {
    RCLCPP_DEBUG(get_node()->get_logger(), "No URDF file given");
  }

  configure_subscribers();

  RCLCPP_INFO(get_node()->get_logger(), "TorqueController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
  compensate_gravity_.store(params_.compensate_gravity);
  previous_velocities_.assign(n_joints_, 0.0);

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

  RCLCPP_INFO(get_node()->get_logger(), "TorqueController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueController::on_deactivate(
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
  RCLCPP_INFO(get_node()->get_logger(), "TorqueController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  dither_switch_ = false;
  tree_ = KDL::Tree();
  f_ext_.clear();
  command_torque_buffer_.writeFromNonRT(std::vector<double>());
  command_received_buffer_.writeFromNonRT(false);
  last_command_time_buffer_.writeFromNonRT(get_node()->now());
  torque_command_sub_.reset();

  RCLCPP_INFO(get_node()->get_logger(), "TorqueController cleaned up successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_node()->get_logger(), "Error occurred in TorqueController.");
  return CallbackReturn::ERROR;
}

controller_interface::CallbackReturn TorqueController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Shutting down TorqueController.");
  return CallbackReturn::SUCCESS;
}

}  // namespace torque_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  torque_controller::TorqueController,
  controller_interface::ControllerInterface)
