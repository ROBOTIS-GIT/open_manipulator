#include "joint_impedance_controller/joint_impedance_controller.hpp"
#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <controller_interface/helpers.hpp>

namespace omy_joint_impedance
{
namespace
{
int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

controller_interface::InterfaceConfiguration JointImpedanceController::state_interface_configuration() const
{
  auto config = Base::state_interface_configuration();
  if (wrench_enabled_) {
    for (const auto & joint : joint_names_) {config.names.push_back(joint + "/effort");}
  }
  return config;
}

controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  const auto result = Base::on_init();
  if (result != controller_interface::CallbackReturn::SUCCESS) {return result;}
  rcl_interfaces::msg::ParameterDescriptor startup_only;
  startup_only.read_only = true;
  startup_only.description = "Compensation output switch; restart launch to change";
  get_node()->declare_parameter<bool>("gravity_compensation_enabled", true, startup_only);
  get_node()->declare_parameter<bool>("friction_compensation_enabled", true, startup_only);
  get_node()->declare_parameter<bool>("wrench.enabled", true, startup_only);
  get_node()->declare_parameter<std::string>("wrench.root_link", "link0", startup_only);
  get_node()->declare_parameter<std::string>("wrench.tip_link", "end_effector_link", startup_only);
  get_node()->declare_parameter<double>("wrench.publish_rate_hz", 50., startup_only);
  get_node()->declare_parameter<double>("wrench.damping_m", .005, startup_only);
  get_node()->declare_parameter<double>("wrench.characteristic_length_m", .25, startup_only);
  get_node()->declare_parameter<double>("wrench.residual_filter_time_s", .1, startup_only);
  get_node()->declare_parameter<double>("wrench.acceleration_filter_time_s", .05, startup_only);
  auto_declare<std::vector<double>>("impedance.stiffness", {5., 5., 4., 2., 1., .8});
  auto_declare<std::vector<double>>("impedance.damping", {.6, .8, .6, .25, .15, .12});
  auto_declare<std::vector<double>>("impedance.integral_gain", std::vector<double>(6, 0.));
  auto_declare<std::vector<double>>("impedance.integral_limit", {.4, .4, .4, .2, .2, .2});
  auto_declare<double>("impedance.integral_leak", 0.);
  return result;
}

controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & state)
{
  gravity_compensation_enabled_ = get_node()->get_parameter("gravity_compensation_enabled").as_bool();
  friction_compensation_enabled_ = get_node()->get_parameter("friction_compensation_enabled").as_bool();
  Gains gains;
  const auto load = [&](const std::string & name, Vector & destination) {
      const auto values = get_node()->get_parameter(name).as_double_array();
      if (values.size() != destination.size()) {return false;}
      std::copy(values.begin(), values.end(), destination.begin());
      return true;
    };
  if (!load("impedance.stiffness", gains.stiffness) ||
    !load("impedance.damping", gains.damping) ||
    !load("impedance.integral_gain", gains.integral_gain) ||
    !load("impedance.integral_limit", gains.integral_limit))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Each impedance gain requires six values: joint1 ... joint6");
    return controller_interface::CallbackReturn::ERROR;
  }
  gains.integral_leak = get_node()->get_parameter("impedance.integral_leak").as_double();
  if (!impedance_.configure(gains)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid impedance gains or integral limits");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto result = Base::on_configure(state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {return result;}
  RCLCPP_INFO(get_node()->get_logger(), "Compensation outputs: gravity=%s friction/dither=%s (controller YAML)",
    gravity_compensation_enabled_ ? "ON" : "OFF", friction_compensation_enabled_ ? "ON" : "OFF");
  wrench_enabled_ = get_node()->get_parameter("wrench.enabled").as_bool();
  wrench_publisher_.reset();
  wrench_valid_publisher_.reset();
  if (wrench_enabled_) {
    const auto root = get_node()->get_parameter("wrench.root_link").as_string();
    const auto tip = get_node()->get_parameter("wrench.tip_link").as_string();
    const double rate = get_node()->get_parameter("wrench.publish_rate_hz").as_double();
    if (!std::isfinite(rate) || rate <= 0. || rate > 400. ||
      !wrench_projection_.configure(tree_, joint_names_, root, tip,
        get_node()->get_parameter("wrench.damping_m").as_double(),
        get_node()->get_parameter("wrench.characteristic_length_m").as_double()))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid wrench settings or chain (must contain ordered joint1 ... joint6)");
      return controller_interface::CallbackReturn::ERROR;
    }
    wrench_period_ = 1. / rate;
    if (!params_.stateful_bank_enabled ||
      !torque_observer_.configure(tree_, params_.torque_scaling_factors, stateful_friction_,
        std::min(wrench_period_, .02), get_node()->get_parameter("wrench.residual_filter_time_s").as_double(),
        get_node()->get_parameter("wrench.acceleration_filter_time_s").as_double()))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "External wrench needs a complete friction bank and positive filter times");
      return controller_interface::CallbackReturn::ERROR;
    }
    wrench_publisher_ = std::make_unique<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
      get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/external_wrench", rclcpp::SensorDataQoS()));
    wrench_publisher_->msg_.header.frame_id = tip;
    wrench_valid_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
      get_node()->create_publisher<std_msgs::msg::Bool>("~/external_wrench_valid", rclcpp::SensorDataQoS()));
  }

  position_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/reference", rclcpp::QoS(1), [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
      if (!motor_ready_.load() || !controller_active_.load() || message->data.size() != 6) {return;}
      EffortReference reference;
      for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(message->data[i])) {return;}
        reference.values[i] = message->data[i];
      }
      reference.stamp_ns = steady_now_ns();
      reference.sequence = position_sequence_.fetch_add(1) + 1;
      position_input_.writeFromNonRT(reference);
    });

  // Keep external user commands in a separate non-RT -> RT buffer. Only this
  // plugin's update writes the base's additive input during steady operation.
  // This removes the ROS transport deadline from the internal impedance loop.
  effort_subscription_.reset();
  effort_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::QoS(1), [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
      if (!motor_ready_.load() || !controller_active_.load()) {return;}
      EffortReference reference;
      if (message->data.size() != reference.values.size()) {
        input_.writeFromNonRT(reference);
        return;
      }
      std::copy(message->data.begin(), message->data.end(), reference.values.begin());
      reference.stamp_ns = steady_now_ns();
      if (!friction_compensation_controller::valid_effort_reference(reference)) {
        input_.writeFromNonRT(EffortReference{});
        return;
      }
      reference.sequence = input_sequence_.fetch_add(1) + 1;
      input_.writeFromNonRT(reference);
    });
  auto publisher = get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
    "~/impedance_state", rclcpp::SensorDataQoS());
  state_publisher_ = std::make_unique<
    realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>(publisher);
  auto & message = state_publisher_->msg_;
  message.joint_names = joint_names_;
  message.interface_values.resize(6);
  for (auto & joint : message.interface_values) {
    joint.interface_names = {"reference_position", "position_error", "spring_nm", "damping_nm",
      "integral_nm", "impedance_effort_nm", "external_effort_nm", "enabled", "update_sequence",
      "measured_effort_nm", "external_joint_torque_nm"};
    joint.values.resize(joint.interface_names.size());
  }
  return result;
}

controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & state)
{
  const auto result = Base::on_activate(state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {return result;}
  measured_effort_interfaces_.clear();
  if (wrench_enabled_ && !controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names_, "effort", measured_effort_interfaces_))
  {
    controller_active_.store(false);
    motor_ready_.store(false);
    RCLCPP_ERROR(get_node()->get_logger(), "External wrench requires all six measured effort interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  Vector position;
  for (size_t i = 0; i < 6; ++i) {
    position[i] = joint_state_interface_[0][i].get().get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
  }
  if (!impedance_.activate(position)) {
    controller_active_.store(false);
    motor_ready_.store(false);
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot capture a nonfinite activation pose");
    return controller_interface::CallbackReturn::ERROR;
  }
  input_.writeFromNonRT(EffortReference{});
  position_input_.writeFromNonRT(EffortReference{});
  applied_position_sequence_ = 0;
  input_gate_ = {};
  output_ = {};
  state_elapsed_ = 0.;
  updates_ = 0;
  wrench_elapsed_ = 0.;
  wrench_result_ = {};
  torque_observer_.reset();
  torque_observer_valid_ = false;
  RCLCPP_INFO(get_node()->get_logger(), "Joint impedance activation pose captured: %s rad",
    formatVector(std::vector<double>(position.begin(), position.end())).c_str());
  return result;
}

controller_interface::return_type JointImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  Vector position, velocity, measured{};
  for (size_t i = 0; i < 6; ++i) {
    position[i] = joint_state_interface_[0][i].get().get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
    velocity[i] = joint_state_interface_[1][i].get().get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
    if (wrench_enabled_) {
      measured[i] = measured_effort_interfaces_[i].get().get_optional().value_or(
        std::numeric_limits<double>::quiet_NaN());
    }
  }
  const bool ready = motor_ready_.load();
  const auto position_reference = *position_input_.readFromRT();
  if (position_reference.sequence != applied_position_sequence_) {
    // Never replay a target received before a communication/enable interruption.
    applied_position_sequence_ = position_reference.sequence;
    const auto age = steady_now_ns() - position_reference.stamp_ns;
    if (ready && position_reference.sequence != 0 && age >= 0 && age < 300000000) {
      impedance_.set_reference(position_reference.values);
    }
  }
  if (!impedance_.step(position, velocity, period.seconds(), ready, output_)) {
    return controller_interface::return_type::ERROR;
  }
  const auto external = input_gate_.consume(*input_.readFromRT(), steady_now_ns(),
    period.nanoseconds(), ready);
  EffortReference combined;
  for (size_t i = 0; i < 6; ++i) {combined.values[i] = output_.effort[i] + external[i];}
  combined.stamp_ns = steady_now_ns();
  combined.sequence = external_effort_sequence_.fetch_add(1) + 1;
  // readFromRT() returns the RT-owned slot; writing it performs no allocation
  // or blocking writeFromNonRT() call. The base consumes it in this same update.
  // Its existing gate callback can still clear inputs during start/stop.
  *external_effort_buffer_.readFromRT() = combined;
  const auto result = Base::update(time, period);
  if (wrench_enabled_) {
    torque_observer_valid_ = torque_observer_.update(position, velocity, measured, period.seconds(),
      result == controller_interface::return_type::OK && ready && motor_ready_applied_.load());
  }
  // Diagnostic only: no result is fed back into command generation. Run the
  // fixed-size solve at the telemetry rate, not on every 400 Hz control cycle.
  wrench_elapsed_ += period.seconds();
  if (wrench_enabled_ && wrench_elapsed_ >= wrench_period_) {
    const bool calculated = result == controller_interface::return_type::OK &&
      torque_observer_valid_ &&
      wrench_projection_.calculate(position, torque_observer_.external(), wrench_result_);
    if (wrench_valid_publisher_->trylock()) {
      wrench_valid_publisher_->msg_.data = calculated && wrench_result_.valid;
      wrench_valid_publisher_->unlockAndPublish();
    }
    if (calculated && wrench_publisher_->trylock()) {
      auto & message = wrench_publisher_->msg_;
      message.header.stamp = time;
      const auto & w = wrench_result_.wrench;
      message.wrench.force.x = w[0]; message.wrench.force.y = w[1]; message.wrench.force.z = w[2];
      message.wrench.torque.x = w[3]; message.wrench.torque.y = w[4]; message.wrench.torque.z = w[5];
      wrench_publisher_->unlockAndPublish();
    }
    wrench_elapsed_ = std::fmod(wrench_elapsed_, wrench_period_);
  }
  ++updates_;
  state_elapsed_ += period.seconds();
  if (state_elapsed_ >= .02 && state_publisher_ && state_publisher_->trylock()) {
    auto & message = state_publisher_->msg_;
    message.header.stamp = time;
    for (size_t i = 0; i < 6; ++i) {
      auto & values = message.interface_values[i].values;
      values[0] = impedance_.reference()[i]; values[1] = output_.error[i];
      values[2] = output_.spring[i]; values[3] = output_.damping[i];
      values[4] = output_.integral[i]; values[5] = output_.effort[i];
      values[6] = external[i]; values[7] = motor_ready_applied_.load() ? 1. : 0.;
      values[8] = static_cast<double>(updates_);
      values[9] = torque_observer_valid_ ? torque_observer_.measured()[i] : std::numeric_limits<double>::quiet_NaN();
      values[10] = torque_observer_valid_ ? torque_observer_.external()[i] : std::numeric_limits<double>::quiet_NaN();
    }
    state_publisher_->unlockAndPublish();
    state_elapsed_ = 0.;
  }
  return result;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  output_ = {};
  torque_observer_.reset();
  torque_observer_valid_ = false;
  if (wrench_valid_publisher_ && wrench_valid_publisher_->trylock()) {
    wrench_valid_publisher_->msg_.data = false;
    wrench_valid_publisher_->unlockAndPublish();
  }
  return Base::on_deactivate(state);
}
}  // namespace omy_joint_impedance

PLUGINLIB_EXPORT_CLASS(omy_joint_impedance::JointImpedanceController,
  controller_interface::ControllerInterface)
