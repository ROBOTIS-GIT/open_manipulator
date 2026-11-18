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

#include <friction_compensation_controller/friction_compensation_controller.hpp>
#include "friction_compensation_controller/kinetic_bank.hpp"
#include <chrono>
#include <string>
#include <stdexcept>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/helpers.hpp>

namespace friction_compensation_controller
{
namespace
{
int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool fresh_update(int64_t stamp)
{
  const auto age = steady_now_ns()-stamp;
  return stamp > 0 && age >= 0 && age < 50000000;
}
}  // namespace

FrictionCompensationController::FrictionCompensationController()
: controller_interface::ControllerInterface(),
  dither_switch_(false)
{
}

controller_interface::InterfaceConfiguration
FrictionCompensationController::command_interface_configuration() const
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
FrictionCompensationController::state_interface_configuration() const
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

controller_interface::return_type FrictionCompensationController::update(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Sample the gate once. Service acknowledgement is distinct from completion
  // of an RT cycle with assistance and external effort removed.
  const bool motor_ready_in_update = motor_ready_.load();
  if (!friction_compensation_enabled()) {
    friction_enabled_.store(false);
    automatic_enable_pending_.store(false);
  }
  if (!motor_ready_in_update) {friction_enabled_.store(false);}
  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface) {
      for (size_t index = 0; index < n_joints_; ++index) {
        trajectory_point_interface[index] =
          joint_interface[index].get().get_optional().value_or(
          (params_.calibrated_friction_enabled || params_.stateful_friction_enabled) ?
          std::numeric_limits<double>::quiet_NaN() : 0.0);
      }
    };

  assign_point_from_interface(joint_positions_, joint_state_interface_[0]);
  assign_point_from_interface(joint_velocities_, joint_state_interface_[1]);

  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0) {
    return controller_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < n_joints_; ++i) {
    if (!std::isfinite(joint_positions_[i]) || !std::isfinite(joint_velocities_[i])) {
      return controller_interface::return_type::ERROR;
    }
  }
  if (params_.calibrated_friction_enabled || params_.stateful_friction_enabled) {
    PositionVelocityObserver::Position positions{};
    std::copy_n(joint_positions_.begin(), positions.size(), positions.begin());
    position_velocity_observation_ = position_velocity_observer_.update(time.seconds(), positions);
    if (params_.stateful_friction_enabled &&
      position_velocity_observation_.status != PositionVelocityObserver::Status::Valid &&
      position_velocity_observation_.status != PositionVelocityObserver::Status::Warming)
    {
      friction_fault_.store(true);
      friction_fault_reason_.fetch_or(StatefulFriction::InvalidObservation);
      friction_enabled_.store(false);
    }
    if (params_.stateful_friction_enabled) {
      const bool ready = friction_reset_interlock_.update(positions,
        position_velocity_observation_.velocity, dt,
        position_velocity_observation_.valid, !friction_enabled_.load());
      friction_reset_ready_.store(ready);
      friction_last_update_ns_.store(steady_now_ns());
      // Service callbacks only queue work. Internal spring state is changed by
      // the RT owner, with a fresh second check; gravity stays active throughout.
      if (friction_reset_pending_.load()) {
        const bool allowed = ready && !friction_enabled_.load();
        if (allowed) {
          for (auto & friction : stateful_friction_) {if (friction) {friction->reset();}}
          friction_fault_.store(false);
          friction_fault_reason_.store(0);
        }
        friction_reset_succeeded_.store(allowed);
        friction_reset_completed_.store(friction_reset_requested_.load());
        friction_reset_pending_.store(false);
      }
    }
  }
  if (automatic_enable_pending_.load() && motor_ready_in_update &&
    position_velocity_observation_.valid && friction_reset_ready_.load() && !friction_fault_.load())
  {
    friction_enabled_.store(true);
    automatic_enable_pending_.store(false);
  }
  const EffortReference external_reference = *external_effort_buffer_.readFromRT();
  const auto external_effort = external_effort_gate_.consume(
    external_reference, steady_now_ns(), period.nanoseconds(), motor_ready_in_update);
  // Apply velocity scaling factors from parameters
  for (size_t i = 0; i < joint_velocities_.size(); i++) {
    joint_velocities_[i] = joint_velocities_[i] * params_.input_velocity_scaling_factors[i];
  }
  if (first_update_) {
    previous_velocities_ = joint_velocities_;
    first_update_ = false;
  }
  // Calculate acceleration from velocity using finite difference
  std::vector<double> joint_accelerations(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i) {
    joint_accelerations[i] = (joint_velocities_[i] - previous_velocities_[i]) / period.seconds() *
      params_.input_acceleration_scaling_factors[i];
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

  KDL::JntArray gravity_only(tree_.getNrOfJoints());
  if (params_.calibrated_friction_enabled || params_.stateful_bank_enabled ||
    params_.stateful_load_joint > 0 || !gravity_compensation_enabled()) {
    KDL::JntArray stationary(tree_.getNrOfJoints());
    KDL::SetToZero(stationary);
    // Load feature uses the original rigid-body model and original torque scale.
    idsolver.CartToJnt(q, stationary, stationary, f_ext_, gravity_only);
  }
  if (!gravity_compensation_enabled()) {
    // Remove gravity only from the output, retaining its original value for
    // load-dependent friction. Velocity/inertia terms and torque scaling stay intact.
    for (size_t i = 0; i < joint_names_.size(); ++i) {torques(i) -= gravity_only(i);}
  }

  // Optional spring effect on joint 2
  if (params_.enable_spring_effect) {
    if (q(2) < 0.5) {
      torques(2) += std::abs(q(2) - 0.5) * 2.5;
    }
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

  if (params_.stateful_friction_enabled) {
    const auto reason=step_kinetic_bank(stateful_friction_,
      [&](size_t i){return q(i);},
      [&](size_t i){return position_velocity_observation_.velocity[i];},
      [&](size_t i){return (params_.stateful_bank_enabled || params_.stateful_load_joint > 0) ?
        gravity_only(i)*params_.torque_scaling_factors[i] : 0.;},
      dt,position_velocity_observation_.valid,
      motor_ready_in_update && friction_enabled_.load() && !friction_fault_.load(),friction_details_);
    if(reason) {
      friction_fault_.store(true);friction_fault_reason_.fetch_or(reason);
      friction_enabled_.store(false);
    }
  }
  // Apply commands only after every selected axis has been checked.
  for (size_t i = 0; i < tree_.getNrOfJoints(); ++i) {
    if (i >= joint_names_.size()) {
      continue;
    }

    if (params_.stateful_friction_enabled) {
      double assistance = friction_details_[i].applied;
      if (friction_fault_.load()) {
        assistance = 0.; friction_details_[i].applied = 0.;
        friction_details_[i].flags |= Fault;
      }
      const double baseline = torques(i) * params_.torque_scaling_factors[i];
      const double command = baseline + assistance + external_effort[i];
      actual_commands_[i] = command;
      if (!std::isfinite(command) || !joint_command_interface_[0][i].get().set_value(command)) {
        return controller_interface::return_type::ERROR;
      }
      if (friction_diagnostics_publisher_) {
        friction_baseline_[i] = baseline; friction_command_[i] = command;
        friction_raw_velocity_[i] = joint_state_interface_[1][i].get().get_optional().value_or(0.);
        friction_windows_[i].add(friction_details_[i]);
      }
      continue;  // No legacy friction or dither in this path, including disabled axes.
    }
    if (params_.calibrated_friction_enabled) {
      // Read raw velocity: baseline velocity scaling is for the original KDL calculation only.
      const double raw_velocity = joint_state_interface_[1][i].get().get_optional().value_or(
        std::numeric_limits<double>::quiet_NaN());
      const int context_joint = static_cast<int>(params_.calibrated_context_joint_1based[i]);
      const double assistance = calibrated_friction_[i].step(
        q(i), raw_velocity, gravity_only(i) * params_.torque_scaling_factors[i],
        dt, motor_ready_in_update && friction_enabled_.load(), context_joint > 0 ? q(context_joint-1) : 0.);
      if (calibrated_friction_[i].fault()) {friction_fault_.store(true);}
      // Coefficients are identified in the hardware effort interface's Nm convention.
      // Preserve baseline torque scaling, do not scale the measured friction a second time.
      const double command = torques(i) * params_.torque_scaling_factors[i] + assistance +
        external_effort[i];
      actual_commands_[i] = command;
      if (!std::isfinite(command) || !joint_command_interface_[0][i].get().set_value(command)) {
        return controller_interface::return_type::ERROR;
      }
      if (friction_diagnostics_publisher_) {
        friction_baseline_[i] = torques(i) * params_.torque_scaling_factors[i];
        friction_command_[i] = command;
        friction_raw_velocity_[i] = raw_velocity;
        friction_details_[i] = calibrated_friction_[i].diagnostics();
        friction_windows_[i].add(calibrated_friction_[i].diagnostics());
      }
      continue;  // Never combine calibrated and legacy friction/dither.
    }

    if (friction_compensation_enabled()) {
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

    }
    actual_commands_[i] = torques(i) * params_.torque_scaling_factors[i] +
      external_effort[i];
    bool set_ok = joint_command_interface_[0][i].get().set_value(actual_commands_[i]);
    if (!set_ok) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to set command value for joint %zu, interface %u", i, 0);
    }
  }

  motor_ready_applied_.store(motor_ready_in_update);
  if (friction_diagnostics_publisher_) {
    friction_publish_elapsed_ += dt;
    if (friction_publish_elapsed_ >= 1. / params_.friction_diagnostics_rate &&
      friction_diagnostics_publisher_->trylock())
    {
      // Names/storage were allocated during configure. Never wait on a subscriber.
      auto & message = friction_diagnostics_publisher_->msg_;
      message.header.stamp = time;
      for (size_t i = 0; i < n_joints_; ++i) {
        const auto & d = friction_details_[i];
        const auto & w = friction_windows_[i];
        auto & values = message.interface_values[i].values;
        values[0] = joint_positions_[i];
        values[1] = friction_raw_velocity_[i];
        values[2] = d.model_magnitude;
        values[3] = d.requested;
        values[4] = d.target;
        values[5] = d.applied;
        values[6] = friction_baseline_[i];
        values[7] = friction_command_[i];
        values[8] = d.flags;
        values[9] = dt;
        values[15] = position_velocity_observation_.velocity[i];
        values[16] = position_velocity_observation_.age;
        values[17] = position_velocity_observation_.valid ? 1. : 0.;
        values[18] = position_velocity_observation_.quantization_bound[i];
        if (params_.stateful_friction_enabled) {
          const auto components = stateful_friction_[i] ? stateful_friction_[i]->components() :
            ElastoplasticReplay::Output{};
          values[19] = components.elastic;
          values[20] = components.rate_term;
          values[21] = components.damping;
          values[22] = components.plastic_velocity;
          values[23] = stateful_friction_[i] ? stateful_friction_[i]->fault_reason() : 0;
          const auto dither = stateful_friction_[i] ? stateful_friction_[i]->dither_components() :
            RestDither::Output{};
          values[24] = dither.torque;
          values[25] = dither.envelope;
        }
        values[10] = static_cast<double>(w.cycles);
        values[11] = static_cast<double>(w.assist_cycles);
        values[12] = w.peak_abs;
        values[13] = w.cycles ? w.sum_abs / w.cycles : 0.;
        values[14] = w.flags;
      }
      friction_diagnostics_publisher_->unlockAndPublish();
      friction_publish_elapsed_ = 0.;
      for (auto & window : friction_windows_) {window = {};}
    }
  }

  command_publish_elapsed_ += dt;
  if (command_publish_elapsed_ >= .01 && command_state_publisher_ && command_state_publisher_->trylock()) {
    auto & message = command_state_publisher_->msg_;
    message.header.stamp = time;
    for (size_t i = 0; i < n_joints_; ++i) {
      message.position[i] = joint_positions_[i];
      message.velocity[i] = joint_velocities_[i];
      message.effort[i] = actual_commands_[i];
    }
    command_state_publisher_->unlockAndPublish();
    command_publish_elapsed_ = 0.;
  }
  // Update previous velocities for next iteration
  previous_velocities_ = joint_velocities_;

  dither_switch_ = !dither_switch_;  // Flip the dither switch

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn FrictionCompensationController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    if (params_.stateful_bank_enabled) {
      rcl_interfaces::msg::ParameterDescriptor immutable;
      immutable.read_only=true;
      for(size_t i=0;i<params_.stateful_joint_mask.size();++i) {
        if(!params_.stateful_joint_mask[i]) {continue;}
        const auto prefix="stateful_bank.joint"+std::to_string(i+1)+".";
        get_node()->declare_parameter<int64_t>(prefix+"kind",0,immutable);
        get_node()->declare_parameter<bool>(prefix+"load_hat_mean_extension",false,immutable);
        get_node()->declare_parameter<std::vector<int64_t>>(prefix+"orders",{1,2},immutable);
        for(const auto name:{"speeds","loads","positive","negative"}) {
          get_node()->declare_parameter<std::vector<double>>(prefix+name,{},immutable);
        }
        for(const auto name:{"width","load_scale","fraction","dither_frequency"}) {
          get_node()->declare_parameter<double>(prefix+name,1.,immutable);
        }
        for(const auto name:{"frequency","transition_distance","dither_amplitude"}) {
          get_node()->declare_parameter<double>(prefix+name,0.,immutable);
        }
        get_node()->declare_parameter<double>(prefix+"dither_full_velocity",.004,immutable);
        get_node()->declare_parameter<double>(prefix+"dither_zero_velocity",.012,immutable);
        get_node()->declare_parameter<double>(prefix+"dither_ramp",.5,immutable);
      }
    }
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrictionCompensationController::on_configure(
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
  if (n_joints_ != 6) {RCLCPP_ERROR(logger, "Exactly six ordered arm joints are required"); return CallbackReturn::ERROR;}
  actual_commands_.assign(n_joints_, 0.);
  external_effort_buffer_.writeFromNonRT(EffortReference{});
  auto command_publisher = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/command_state", rclcpp::SensorDataQoS());
  command_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(command_publisher);
  command_state_publisher_->msg_.name = joint_names_;
  command_state_publisher_->msg_.position.resize(n_joints_);
  command_state_publisher_->msg_.velocity.resize(n_joints_);
  command_state_publisher_->msg_.effort.resize(n_joints_);
  effort_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::QoS(1), [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
      if (!motor_ready_.load() || !controller_active_.load()) {return;}
      if (message->data.size() != 6) {
        external_effort_buffer_.writeFromNonRT(EffortReference{});
        RCLCPP_WARN(get_node()->get_logger(), "Ignoring effort command: expected six Nm values"); return;
      }
      EffortReference reference;
      std::copy_n(message->data.begin(), 6, reference.values.begin());
      reference.stamp_ns = steady_now_ns();
      if (!valid_effort_reference(reference)) {
        external_effort_buffer_.writeFromNonRT(EffortReference{});
        RCLCPP_WARN(get_node()->get_logger(), "Ignoring non-finite effort command"); return;
      }
      reference.sequence = external_effort_sequence_.fetch_add(1) + 1;
      external_effort_buffer_.writeFromNonRT(reference);
    });
  effort_gate_service_ = get_node()->create_service<std_srvs::srv::SetBool>(
    "~/accept_external_effort", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      external_effort_buffer_.writeFromNonRT(EffortReference{});
      motor_ready_.store(request->data);
      automatic_enable_pending_.store(request->data && friction_compensation_enabled());
      if (!request->data) {friction_enabled_.store(false);}
      response->success = true;
      response->message = request->data ? "Motor ready: external effort accepted; friction starts with a valid observer" :
        "External effort and friction disabled; gravity remains active";
    });
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

  calibrated_friction_.clear();
  stateful_friction_.clear();
  friction_details_.assign(n_joints_, FrictionDiagnostics{});
  if (params_.stateful_friction_enabled) {
    try {
      if (params_.calibrated_friction_enabled || n_joints_ != 6 ||
        params_.stateful_joint_mask.size() != 6)
      {throw std::invalid_argument("Stateful path requires six joints and no legacy calibrated path");}
      const std::vector<const std::vector<double> *> arrays = {
        &params_.stateful_positive_yield, &params_.stateful_negative_yield,
        &params_.stateful_positive_viscous, &params_.stateful_negative_viscous,
        &params_.stateful_stiffness, &params_.stateful_decay,
        &params_.stateful_damping, &params_.stateful_maximum_velocity};
      for (const auto * values : arrays) {
        if (values->size() != 6) {throw std::invalid_argument("Stateful array length");}
      }
      stateful_friction_.resize(6);
      if(params_.stateful_bank_enabled && (params_.stateful_load_joint!=0 ||
        params_.stateful_curve_j6_enabled || params_.stateful_dither_with_elastic || params_.stateful_dither_j6_only)) {
        throw std::invalid_argument("Do not mix bank and legacy single-axis settings");
      }
      const bool dither_enabled = params_.stateful_dither_j6_only || params_.stateful_dither_with_elastic;
      if (params_.stateful_load_joint < 0 || params_.stateful_load_joint > 6) {
        throw std::invalid_argument("Invalid load-dependent joint selection");
      }
      if (!std::isfinite(params_.stateful_load_smooth_width_nm) ||
        params_.stateful_load_smooth_width_nm<0. ||
        (params_.stateful_load_smooth_width_nm>0. && params_.stateful_load_joint==0)) {
        throw std::invalid_argument("Invalid smooth-load activation");
      }
      if (params_.stateful_load_joint > 0) {
        std::vector<int64_t> expected(6, 0);
        expected[params_.stateful_load_joint-1] = 1;
        if (params_.stateful_joint_mask != expected || params_.stateful_curve_j6_enabled ||
          params_.stateful_dither_j6_only || (params_.stateful_load_positive.size() != 4 &&
          params_.stateful_load_positive.size() != 6) ||
          params_.stateful_load_negative.size() != params_.stateful_load_positive.size()) {
          throw std::invalid_argument("Load-dependent experiment requires exactly one matching joint and no mixed model");
        }
      }
      if ((params_.stateful_curve_j6_enabled || (dither_enabled && params_.stateful_load_joint==0)) &&
        params_.stateful_joint_mask != std::vector<int64_t>({0, 0, 0, 0, 0, 1}))
      {throw std::invalid_argument("Kinetic curve experiment is J6 only");}
      if ((params_.stateful_curve_j6_enabled && dither_enabled &&
        (!params_.stateful_dither_with_elastic || !params_.stateful_curve_hold_endpoint)) ||
        (params_.stateful_dither_j6_only && params_.stateful_dither_with_elastic)) {
        throw std::invalid_argument("Dither/curve combination requires elastic mode and constant endpoint extension");
      }
      for (size_t i = 0; i < 6; ++i) {
        if (params_.stateful_joint_mask[i] != 0 && params_.stateful_joint_mask[i] != 1) {
          throw std::invalid_argument("Stateful joint mask must contain 0 or 1");
        }
        if (params_.stateful_joint_mask[i] == 1) {
          if(params_.stateful_bank_enabled) {
            const auto prefix="stateful_bank.joint"+std::to_string(i+1)+".";
            auto value=[&](const char* name){return get_node()->get_parameter(prefix+name);};
            KineticBankJoint c;
            c.kind=value("kind").as_int();c.orders=value("orders").as_integer_array();
            c.load_hat_mean_extension=value("load_hat_mean_extension").as_bool();
            c.width=value("width").as_double();c.load_scale=value("load_scale").as_double();
            c.frequency=value("frequency").as_double();c.fraction=value("fraction").as_double();
            c.speeds=value("speeds").as_double_array();c.loads=value("loads").as_double_array();
            c.positive=value("positive").as_double_array();c.negative=value("negative").as_double_array();
            c.dither_amplitude=value("dither_amplitude").as_double();
            c.dither_frequency=value("dither_frequency").as_double();
            c.dither_full_velocity=value("dither_full_velocity").as_double();
            c.dither_zero_velocity=value("dither_zero_velocity").as_double();
            c.dither_ramp=value("dither_ramp").as_double();
            c.state={params_.stateful_positive_yield[i],params_.stateful_negative_yield[i],
              params_.stateful_positive_viscous[i],params_.stateful_negative_viscous[i],
              params_.stateful_stiffness[i],params_.stateful_decay[i],params_.stateful_damping[i],
              params_.stateful_maximum_velocity[i],value("transition_distance").as_double()};
            stateful_friction_[i].emplace(make_kinetic_bank_joint(c));
            continue;
          }
          std::optional<DirectionalKineticCurve> curve;
          std::optional<RestDither> dither;
          std::optional<DirectionalLoadLaw> load_law;
          if (params_.stateful_load_joint == static_cast<int64_t>(i+1)) {
            DirectionalLoadLaw::Coefficients positive{}, negative{};
            std::copy_n(params_.stateful_load_positive.begin(), 4, positive.begin());
            std::copy_n(params_.stateful_load_negative.begin(), 4, negative.begin());
            if (params_.stateful_load_periodic_frequency > 0.) {
              load_law.emplace(DirectionalLoadLaw::PeriodicLoad{
                params_.stateful_load_smooth_width_nm,params_.stateful_load_feature_scale_nm,
                params_.stateful_load_periodic_frequency,params_.stateful_load_periodic_orders,
                params_.stateful_load_speed_knots,params_.stateful_load_smooth_positive,
                params_.stateful_load_smooth_negative,
                params_.stateful_load_gravity_knots.size()>1?params_.stateful_load_gravity_knots:std::vector<double>{},
                params_.stateful_load_hat_mean_extension});
            } else if (params_.stateful_load_smooth_width_nm > 0.) {
              if (params_.stateful_load_gravity_knots.size()>1) {
                throw std::invalid_argument("Do not mix smooth and tabulated load laws");
              }
              load_law.emplace(DirectionalLoadLaw::SmoothLoad{
                params_.stateful_load_smooth_width_nm,params_.stateful_load_feature_scale_nm,
                params_.stateful_load_speed_knots,params_.stateful_load_smooth_positive,
                params_.stateful_load_smooth_negative});
            } else if (params_.stateful_load_gravity_knots.size() > 1) {
              load_law.emplace(params_.stateful_load_gravity_knots, params_.stateful_load_speed_knots,
                params_.stateful_load_positive_table, params_.stateful_load_negative_table);
            } else if (params_.stateful_load_positive.size() == 6) {
              load_law.emplace(positive, negative, params_.stateful_load_speed_endpoint,
                DirectionalLoadLaw::HighLoad{{params_.stateful_load_positive[4], params_.stateful_load_positive[5]}},
                DirectionalLoadLaw::HighLoad{{params_.stateful_load_negative[4], params_.stateful_load_negative[5]}});
            } else {
              load_law.emplace(positive, negative, params_.stateful_load_speed_endpoint);
            }
          }
          if (params_.stateful_curve_j6_enabled) {
            curve.emplace(params_.stateful_curve_speed, params_.stateful_curve_positive,
              params_.stateful_curve_negative, params_.stateful_curve_hold_endpoint);
          }
          if (dither_enabled) {
            dither.emplace(RestDitherParameters{params_.stateful_dither_amplitude_nm,
              params_.stateful_dither_frequency_hz, params_.stateful_dither_full_velocity,
              params_.stateful_dither_zero_velocity, params_.stateful_dither_ramp_seconds});
          }
          stateful_friction_[i].emplace(StatefulParameters{
            params_.stateful_positive_yield[i], params_.stateful_negative_yield[i],
            params_.stateful_positive_viscous[i], params_.stateful_negative_viscous[i],
            params_.stateful_stiffness[i], params_.stateful_decay[i],
            params_.stateful_damping[i], params_.stateful_maximum_velocity[i],
            params_.stateful_load_transition_distance}, std::move(curve),
            (params_.stateful_curve_j6_enabled || params_.stateful_dither_with_elastic ||
            params_.stateful_load_joint > 0) ?
            params_.stateful_curve_output_scale : 1.,
            std::move(dither), params_.stateful_dither_with_elastic, std::move(load_law));
        }
      }
      position_velocity_observer_ = PositionVelocityObserver(params_.stateful_observer_window);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Invalid stateful friction: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }
  if (params_.calibrated_friction_enabled) {
    try {
      if (n_joints_ != 6) {throw std::invalid_argument("Calibration requires six arm joints");}
      const std::vector<const std::vector<double> *> arrays = {
        &params_.calibrated_fc, &params_.calibrated_fs, &params_.calibrated_vs,
        &params_.calibrated_viscous, &params_.calibrated_max_torque,
        &params_.calibrated_slew_rate, &params_.calibrated_load_gain,
        &params_.calibrated_load_min, &params_.calibrated_load_max,
        &params_.calibrated_load_peak, &params_.calibrated_load_vs,
        &params_.calibrated_position_peak, &params_.calibrated_position_order,
        &params_.calibrated_position_phase, &params_.calibrated_context_peak,
        &params_.calibrated_context_vs, &params_.calibrated_context_joint_1based,
        &params_.calibrated_context_power, &params_.calibrated_sigma0,
        &params_.calibrated_sigma1, &params_.calibrated_smoothing_k};
      for (const auto * values : arrays) {
        if (values->size() != n_joints_) {throw std::invalid_argument("Calibration array length");}
      }
      for (size_t i = 0; i < n_joints_; ++i) {
        const double context_joint = params_.calibrated_context_joint_1based[i];
        if (!std::isfinite(context_joint) || context_joint < 0. ||
          context_joint > static_cast<double>(joint_names_.size()) ||
          std::floor(context_joint) != context_joint ||
          (params_.calibrated_context_peak[i] > 0. && context_joint == 0.))
        {
          throw std::invalid_argument("Invalid calibrated context joint index");
        }
        calibrated_friction_.emplace_back(FrictionParameters{
          params_.calibrated_fc[i], params_.calibrated_fs[i], params_.calibrated_vs[i],
          params_.calibrated_viscous[i], params_.calibrated_gain,
          params_.calibrated_max_torque[i], params_.calibrated_slew_rate[i],
          params_.calibrated_deadband, params_.calibrated_min_velocity,
          params_.calibrated_max_velocity, params_.calibrated_load_gain[i],
          params_.calibrated_load_min[i], params_.calibrated_load_max[i],
          params_.calibrated_enforce_load_bounds, params_.calibrated_unscaled,
          params_.calibrated_load_peak[i], params_.calibrated_load_vs[i],
          params_.calibrated_position_peak[i], params_.calibrated_position_order[i],
          params_.calibrated_position_phase[i],params_.calibrated_context_peak[i],
          params_.calibrated_context_vs[i],params_.calibrated_context_power[i],
          params_.calibrated_lugre, params_.calibrated_sigma0[i], params_.calibrated_sigma1[i],
          params_.calibrated_smoothing_k[i], params_.calibrated_max_acceleration});
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Invalid calibrated friction: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }
  friction_enabled_.store(params_.calibrated_friction_enabled && friction_compensation_enabled());
  friction_diagnostics_publisher_.reset();
  const double diagnostics_rate = params_.friction_diagnostics_rate;
  if (!std::isfinite(diagnostics_rate) || diagnostics_rate < 0. || diagnostics_rate > 400.) {
    RCLCPP_ERROR(logger, "friction_diagnostics_rate must be in [0, 400]");
    return CallbackReturn::ERROR;
  }
  if ((params_.calibrated_friction_enabled || params_.stateful_friction_enabled) && diagnostics_rate > 0.) {
    auto publisher = get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
      "~/friction_diagnostics", rclcpp::SensorDataQoS());
    friction_diagnostics_publisher_ = std::make_unique<
      realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>(publisher);
    auto & message = friction_diagnostics_publisher_->msg_;
    message.joint_names = joint_names_;
    message.interface_values.resize(n_joints_);
    std::vector<std::string> names = {
      "position", "velocity", "model_magnitude_nm", "requested_nm", "target_nm",
      "assistance_nm", "baseline_nm", "total_command_nm", "flags", "period_s",
      "window_cycles", "window_assist_cycles", "window_peak_nm", "window_mean_abs_nm",
      "window_flags", "position_velocity_rad_s", "position_velocity_age_s",
      "position_velocity_valid", "position_velocity_quantization_bound_rad_s"};
    if (params_.stateful_friction_enabled) {
      names.insert(names.end(), {"elastic_nm", "rate_term_nm", "damping_nm", "plastic_velocity_rad_s",
        "stateful_fault_reason", "dither_nm", "dither_envelope"});
    }
    for (auto & entry : message.interface_values) {
      entry.interface_names = names;
      entry.values.resize(names.size(), 0.);
    }
    friction_windows_.assign(n_joints_, FrictionWindow{});
    friction_baseline_.assign(n_joints_, 0.);
    friction_command_.assign(n_joints_, 0.);
    friction_raw_velocity_.assign(n_joints_, 0.);
  }
  friction_switch_ = get_node()->create_service<std_srvs::srv::SetBool>(
    "~/set_friction_enabled",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      if (request->data && !friction_compensation_enabled()) {
        response->success = false;
        response->message = "Friction disabled by controller YAML; change configuration and restart launch";
        return;
      }
      if (request->data && friction_fault_.load()) {
        response->success = false;
        response->message = params_.stateful_friction_enabled ?
          "Assistance fault latched; hold still with assistance OFF, then request reset_friction_fault" :
          "Friction timing/state fault latched; reactivate controller to reset";
        return;
      }
      if (request->data && params_.stateful_friction_enabled &&
        (friction_reset_pending_.load() || !fresh_update(friction_last_update_ns_.load()) ||
        (!friction_enabled_.load() && !friction_reset_ready_.load())))
      {
        response->success = false;
        response->message = "Hold all joints still with assistance OFF for 0.5 s; no reset may be pending";
        return;
      }
      response->success = controller_active_.load() &&
        (params_.calibrated_friction_enabled || params_.stateful_friction_enabled);
      response->message = response->success ? "Assistance toggled; gravity unchanged" :
        "No validated calibration loaded";
      if (response->success) {
        automatic_enable_pending_.store(false);
        friction_enabled_.store(request->data);
      }
    });
  friction_reset_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/reset_friction_fault",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = controller_active_.load() && params_.stateful_friction_enabled &&
        !friction_enabled_.load() && !friction_reset_pending_.load() &&
        friction_reset_ready_.load() && fresh_update(friction_last_update_ns_.load());
      if (!response->success) {
        response->message = "Reset rejected: need active stateful controller, assistance OFF, fresh state and 0.5 s rest";
        return;
      }
      const auto request_id = friction_reset_requested_.fetch_add(1)+1;
      friction_reset_succeeded_.store(false);
      friction_reset_pending_.store(true);
      response->message = "Reset queued (request " + std::to_string(request_id) +
        "); check reset_completed/reset_succeeded. Assistance remains OFF; gravity unchanged";
    });
  friction_status_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/get_friction_status",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = true;
      std::ostringstream status;
      status << std::boolalpha << "{\"active\":" << controller_active_.load() <<
        ",\"configured\":" << (params_.calibrated_friction_enabled || params_.stateful_friction_enabled) <<
        ",\"stateful\":" << params_.stateful_friction_enabled <<
        ",\"enabled\":" << friction_enabled_.load() <<
        ",\"motor_ready_applied\":" << motor_ready_applied_.load() <<
        ",\"fault\":" << friction_fault_.load() <<
        ",\"fault_reason_code\":" << friction_fault_reason_.load() <<
        ",\"reset_ready\":" << (friction_reset_ready_.load() &&
        fresh_update(friction_last_update_ns_.load())) <<
        ",\"reset_pending\":" << friction_reset_pending_.load() <<
        ",\"reset_requested\":" << friction_reset_requested_.load() <<
        ",\"reset_completed\":" << friction_reset_completed_.load() <<
        ",\"reset_succeeded\":" << friction_reset_succeeded_.load() << "}";
      response->message = status.str();
    });

  RCLCPP_INFO(get_node()->get_logger(), "FrictionCompensationController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrictionCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();
  // order all joints in the storage
  // get_ordered_interfaces appends references. Previous activation's loans
  // have been released by controller_manager and must not be retained.
  for (auto & interfaces : joint_command_interface_) {interfaces.clear();}
  for (auto & interfaces : joint_state_interface_) {interfaces.clear();}
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
  first_update_ = true;
  external_effort_buffer_.writeFromNonRT(EffortReference{});
  motor_ready_.store(!params_.require_motor_ready);
  motor_ready_applied_.store(false);
  automatic_enable_pending_.store(!params_.require_motor_ready && friction_compensation_enabled());
  command_publish_elapsed_ = 0.;
  position_velocity_observer_.reset();
  position_velocity_observation_ = {};
  friction_fault_.store(false);
  friction_fault_reason_.store(0);
  friction_reset_interlock_.reset();
  friction_reset_ready_.store(false); friction_reset_pending_.store(false);
  friction_reset_requested_.store(0); friction_reset_completed_.store(0);
  friction_reset_succeeded_.store(false); friction_last_update_ns_.store(0);
  if (params_.stateful_friction_enabled) {friction_enabled_.store(false);}
  for (auto & friction : calibrated_friction_) {friction.reset();}
  for (auto & friction : stateful_friction_) {if (friction) {friction->reset();}}
  for (auto & window : friction_windows_) {window = {};}
  friction_publish_elapsed_ = 0.;
  controller_active_.store(true);
  RCLCPP_INFO(get_node()->get_logger(), "FrictionCompensationController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrictionCompensationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  controller_active_.store(false);
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
  RCLCPP_INFO(get_node()->get_logger(), "FrictionCompensationController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrictionCompensationController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset flags and parameters
  dither_switch_ = false;
  controller_active_.store(false);
  friction_diagnostics_publisher_.reset();

  // Clear KDL tree and joint name map
  tree_ = KDL::Tree();

  // Clear vectors
  f_ext_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "FrictionCompensationController cleaned up successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrictionCompensationController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  controller_active_.store(false);
  RCLCPP_ERROR(get_node()->get_logger(), "Error occurred in FrictionCompensationController.");
  return CallbackReturn::ERROR;
}

controller_interface::CallbackReturn FrictionCompensationController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  controller_active_.store(false);
  RCLCPP_INFO(get_node()->get_logger(), "Shutting down FrictionCompensationController.");
  return CallbackReturn::SUCCESS;
}

std::string FrictionCompensationController::formatVector(const std::vector<double> & vec)
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

}  // namespace friction_compensation_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  friction_compensation_controller::FrictionCompensationController,
  controller_interface::ControllerInterface)
