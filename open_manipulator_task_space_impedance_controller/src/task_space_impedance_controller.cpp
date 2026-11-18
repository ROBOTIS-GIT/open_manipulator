#include "task_space_impedance_controller/task_space_impedance_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <algorithm>
#include <chrono>

namespace omy_task
{
namespace
{
int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
void copy_pose(const Pose & source, geometry_msgs::msg::Pose & destination)
{
  destination.position.x = source.position.x(); destination.position.y = source.position.y();
  destination.position.z = source.position.z(); destination.orientation.x = source.orientation.x();
  destination.orientation.y = source.orientation.y(); destination.orientation.z = source.orientation.z();
  destination.orientation.w = source.orientation.w();
}
}

controller_interface::CallbackReturn TaskSpaceImpedanceController::on_init()
{
  const auto result = Base::on_init();
  if (result != controller_interface::CallbackReturn::SUCCESS) {return result;}
  rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only = true;
  get_node()->declare_parameter<std::vector<double>>("task.stiffness",
    std::vector<double>(gains_.stiffness.data(), gains_.stiffness.data() + 6), fixed);
  get_node()->declare_parameter<std::vector<double>>("task.damping",
    std::vector<double>(gains_.damping.data(), gains_.damping.data() + 6), fixed);
  get_node()->declare_parameter<std::string>("task.root_link", "link0", fixed);
  get_node()->declare_parameter<std::string>("task.tip_link", "end_effector_link", fixed);
  get_node()->declare_parameter<double>("task.publish_rate_hz", 50., fixed);
  get_node()->declare_parameter<bool>("gravity_compensation_enabled", true, fixed);
  get_node()->declare_parameter<bool>("friction_compensation_enabled", true, fixed);
  return result;
}

controller_interface::CallbackReturn TaskSpaceImpedanceController::on_configure(const rclcpp_lifecycle::State & state)
{
  const auto fail = [&](const char * message) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", message);
      return controller_interface::CallbackReturn::ERROR;
    };
  auto states = get_node()->get_parameter("state_interfaces").as_string_array();
  std::sort(states.begin(), states.end());
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  const auto commands = get_node()->get_parameter("command_joints").as_string_array();
  if (states != std::vector<std::string>({"position", "velocity"}) || joints.size() != 6 ||
    get_node()->get_parameter("command_interfaces").as_string_array() != std::vector<std::string>({"effort"}) ||
    (!commands.empty() && commands != joints)) {return fail("Require six matching arm joints, position/velocity states and effort commands");}
  const auto stiffness = get_node()->get_parameter("task.stiffness").as_double_array();
  const auto damping = get_node()->get_parameter("task.damping").as_double_array();
  if (stiffness.size() != 6 || damping.size() != 6) {return fail("Task gains require six values: xyz translation, xyz rotation");}
  for (int i = 0; i < 6; ++i) {gains_.stiffness[i] = stiffness[i]; gains_.damping[i] = damping[i];}
  const double rate = get_node()->get_parameter("task.publish_rate_hz").as_double();
  if (!valid_gains(gains_) || !std::isfinite(rate) || rate <= 0. || rate > 400.) {
    return fail("Require finite nonnegative gains and publish rate in (0,400]");
  }
  publish_period_ = 1. / rate;
  root_ = get_node()->get_parameter("task.root_link").as_string();
  tip_ = get_node()->get_parameter("task.tip_link").as_string();
  gravity_enabled_ = get_node()->get_parameter("gravity_compensation_enabled").as_bool();
  friction_policy_ = get_node()->get_parameter("friction_compensation_enabled").as_bool();
  const auto result = Base::on_configure(state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {return result;}
  if (!tree_.getChain(root_, tip_, chain_) || chain_.getNrOfJoints() != 6) {
    return fail("Task root-to-tip chain must contain six arm joints");
  }
  size_t index = 0;
  for (const auto & segment : chain_.segments) {
    if (segment.getJoint().getType() != KDL::Joint::Fixed &&
      segment.getJoint().getName() != joint_names_[index++]) {return fail("Task chain joint order differs from hardware order");}
  }
  fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  const auto receive_target = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg, bool leader) {
      if (!controller_active_.load() || !motor_ready_.load()) {return;}
      // Same capture-generation protocol as the Cartesian contact follower.
      // Direct user targets retain their ordinary (ignored) header timestamp.
      if (leader && (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u)) {return;}
      const int64_t capture_stamp = leader ? rclcpp::Time(msg->header.stamp).nanoseconds() : -1;
      if (leader && capture_stamp != reference_stamp_ns_.load()) {return;}
      Target target;
      target.capture_stamp_ns = capture_stamp;
      target.pose.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
      target.pose.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
        msg->pose.orientation.y, msg->pose.orientation.z);
      if (msg->header.frame_id != root_ || !valid_pose(target.pose)) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Ignoring target: require configured root frame, finite position and nonzero quaternion");
        return;
      }
      target.pose.orientation.normalize(); target.stamp_ns = steady_ns();
      target.sequence = target_sequence_.fetch_add(1) + 1; target_buffer_.writeFromNonRT(target);
    };
  target_subscription_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", rclcpp::QoS(1), [receive_target](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      receive_target(msg, false);
    });
  leader_target_subscription_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/leader_target_pose", rclcpp::QoS(1), [receive_target](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      receive_target(msg, true);
    });
  // Preserve the external joint-effort interface, but combine it with Cartesian
  // effort in this update before passing one fresh command to the base class.
  effort_subscription_.reset();
  effort_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::QoS(1), [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (!controller_active_.load() || !motor_ready_.load()) {return;}
      Effort reference;
      reference.sequence = effort_sequence_.fetch_add(1) + 1;
      if (msg->data.size() == 6) {
        std::copy(msg->data.begin(), msg->data.end(), reference.values.begin());
        reference.stamp_ns = steady_ns();
      }
      effort_buffer_.writeFromNonRT(reference);
    });
  // Keep the existing readiness/compensation semantics, adding a queued pose
  // capture on EVERY enable request. Sampling a bool edge alone misses an
  // OFF -> ON pair processed between two control updates.
  effort_gate_service_.reset();
  effort_gate_service_ = get_node()->create_service<std_srvs::srv::SetBool>(
    "~/accept_external_effort", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      external_effort_buffer_.writeFromNonRT(Effort{});
      // Publish the capture request before readiness becomes visible to RT.
      if (request->data) {capture_requested_.store(true);}
      motor_ready_.store(request->data);
      automatic_enable_pending_.store(request->data && friction_compensation_enabled());
      if (!request->data) {friction_enabled_.store(false);}
      response->success = true;
      response->message = request->data ? "Motor ready; current-pose capture queued" :
        "External effort and friction disabled; gravity remains active";
    });
  capture_service_ = get_node()->create_service<std_srvs::srv::Trigger>("~/capture_current_pose",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = controller_active_.load() && motor_ready_.load();
      if (response->success) {capture_requested_.store(true);}
      response->message = response->success ? "Capture queued for next control cycle" : "Controller not ready";
    });
  const auto qos = rclcpp::SensorDataQoS();
  pose_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
    get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", qos));
  target_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
    get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/target_pose_state", qos));
  wrench_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
    get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/task_wrench", qos));
  pose_publisher_->msg_.header.frame_id = target_publisher_->msg_.header.frame_id = root_;
  wrench_publisher_->msg_.header.frame_id = root_;
  state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>(
    get_node()->create_publisher<control_msgs::msg::DynamicJointState>("~/task_state", qos));
  state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.interface_values.resize(6);
  for (auto & item : state_publisher_->msg_.interface_values) {
    item.interface_names = {"position", "velocity", "task_effort_nm", "external_effort_nm", "enabled", "update_sequence", "reference_epoch"};
    item.values.resize(7);
  }
  return result;
}

bool TaskSpaceImpedanceController::read_state(Vector6 & q, Vector6 & v) const
{
  if (joint_state_interface_.size() != 2 || joint_state_interface_[0].size() != 6 ||
    joint_state_interface_[1].size() != 6) {return false;}
  for (size_t i = 0; i < 6; ++i) {
    q[i] = joint_state_interface_[0][i].get().get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    v[i] = joint_state_interface_[1][i].get().get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
  }
  return q.allFinite() && v.allFinite();
}
bool TaskSpaceImpedanceController::kinematics(const Vector6 & q, Pose & pose)
{
  q_kdl_.data = q; KDL::Frame frame;
  if (fk_->JntToCart(q_kdl_, frame) < 0 || jac_solver_->JntToJac(q_kdl_, jacobian_) < 0) {return false;}
  pose.position << frame.p.x(), frame.p.y(), frame.p.z();
  double x, y, z, w; frame.M.GetQuaternion(x, y, z, w);
  pose.orientation = Eigen::Quaterniond(w, x, y, z);
  return valid_pose(pose) && jacobian_.data.allFinite();
}

controller_interface::CallbackReturn TaskSpaceImpedanceController::on_activate(const rclcpp_lifecycle::State & state)
{
  const auto result = Base::on_activate(state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {return result;}
  Vector6 q, v;
  if (!read_state(q, v) || joint_command_interface_.size() != 1 ||
    joint_command_interface_[0].size() != 6 || !kinematics(q, target_)) {
    controller_active_.store(false); motor_ready_.store(false);
    return controller_interface::CallbackReturn::ERROR;
  }
  target_buffer_.writeFromNonRT(Target{}); effort_buffer_.writeFromNonRT(Effort{});
  effort_gate_ = {}; applied_target_ = 0; was_ready_ = false; capture_requested_.store(false);
  output_ = {}; publish_elapsed_ = 0.; updates_ = 0;
  reference_epoch_ = 0; reference_stamp_ns_.store(-1);
  return result;
}

controller_interface::return_type TaskSpaceImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  Vector6 q, v; Pose pose;
  if (!read_state(q, v) || !std::isfinite(period.seconds()) || period.seconds() <= 0. || !kinematics(q, pose)) {
    return controller_interface::return_type::ERROR;
  }
  const bool ready = motor_ready_.load();
  const bool capture = capture_requested_.exchange(false);
  const Target reference = *target_buffer_.readFromRT();
  if (ready && (!was_ready_ || capture)) {
    target_ = pose; applied_target_ = reference.sequence;
    ++reference_epoch_; reference_stamp_ns_.store(time.nanoseconds());
    // Discard external effort received before motor readiness/pose recapture.
    effort_gate_.consume(*effort_buffer_.readFromRT(), steady_ns(), period.nanoseconds(), false);
  }
  was_ready_ = ready;
  if (reference.sequence != applied_target_) {
    applied_target_ = reference.sequence;
    const auto age = steady_ns() - reference.stamp_ns;
    // Recheck after the RT capture; a callback may race with pose recapture.
    if (ready && reference.sequence && age >= 0 && age < 300000000 &&
      (reference.capture_stamp_ns < 0 || reference.capture_stamp_ns == reference_stamp_ns_.load())) {
      target_ = reference.pose;
    }
  }
  const auto external = effort_gate_.consume(*effort_buffer_.readFromRT(), steady_ns(), period.nanoseconds(), ready);
  output_ = {};
  if (ready && !evaluate(gains_, pose, target_, Matrix6(jacobian_.data), v, output_)) {
    return controller_interface::return_type::ERROR;
  }
  Effort combined;
  for (int i = 0; i < 6; ++i) {combined.values[i] = output_.torque[i] + external[i];}
  combined.stamp_ns = steady_ns(); combined.sequence = external_effort_sequence_.fetch_add(1) + 1;
  *external_effort_buffer_.readFromRT() = combined;
  const auto result = Base::update(time, period);
  ++updates_; publish_elapsed_ += period.seconds();
  if (publish_elapsed_ >= publish_period_) {
    publish(time, pose, q, v, external, ready && motor_ready_applied_.load()); publish_elapsed_ = 0.;
  }
  return result;
}

void TaskSpaceImpedanceController::publish(const rclcpp::Time & time, const Pose & pose,
  const Vector6 & q, const Vector6 & v, const std::array<double, 6> & external, bool ready)
{
  if (pose_publisher_->trylock()) {
    pose_publisher_->msg_.header.stamp = time; copy_pose(pose, pose_publisher_->msg_.pose);
    pose_publisher_->unlockAndPublish();
  }
  if (target_publisher_->trylock()) {
    // This stamp identifies a synchronization generation, not publication time.
    target_publisher_->msg_.header.stamp = rclcpp::Time(std::max(int64_t{0}, reference_stamp_ns_.load()));
    copy_pose(target_, target_publisher_->msg_.pose);
    target_publisher_->unlockAndPublish();
  }
  if (wrench_publisher_->trylock()) {
    auto & msg = wrench_publisher_->msg_; msg.header.stamp = time;
    msg.wrench.force.x = output_.wrench[0]; msg.wrench.force.y = output_.wrench[1];
    msg.wrench.force.z = output_.wrench[2]; msg.wrench.torque.x = output_.wrench[3];
    msg.wrench.torque.y = output_.wrench[4]; msg.wrench.torque.z = output_.wrench[5];
    wrench_publisher_->unlockAndPublish();
  }
  if (state_publisher_->trylock()) {
    auto & msg = state_publisher_->msg_; msg.header.stamp = time;
    for (int i = 0; i < 6; ++i) {
      auto & a = msg.interface_values[i].values;
      a[0] = q[i]; a[1] = v[i]; a[2] = output_.torque[i]; a[3] = external[i];
      a[4] = ready ? 1. : 0.; a[5] = static_cast<double>(updates_);
      a[6] = static_cast<double>(reference_epoch_);
    }
    state_publisher_->unlockAndPublish();
  }
}
controller_interface::CallbackReturn TaskSpaceImpedanceController::on_deactivate(const rclcpp_lifecycle::State & state)
{
  was_ready_ = false; capture_requested_.store(false); output_ = {};
  return Base::on_deactivate(state);
}
}  // namespace omy_task
PLUGINLIB_EXPORT_CLASS(omy_task::TaskSpaceImpedanceController, controller_interface::ControllerInterface)
