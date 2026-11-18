#pragma once
#include "friction_compensation_controller/friction_compensation_controller.hpp"
#include "joint_impedance_controller/impedance_law.hpp"
#include "joint_impedance_controller/wrench_projection.hpp"
#include "joint_impedance_controller/external_torque_observer.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace omy_joint_impedance
{
// A distinct plugin: the original compensation plugin and launch are untouched.
// The base class owns compensation, hardware loans, readiness and brake services.
class JointImpedanceController : public friction_compensation_controller::FrictionCompensationController
{
public:
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  bool gravity_compensation_enabled() const override {return gravity_compensation_enabled_;}
  bool friction_compensation_enabled() const override {return friction_compensation_enabled_;}
  bool gravity_compensation_enabled_{true};
  bool friction_compensation_enabled_{true};
  using Base = friction_compensation_controller::FrictionCompensationController;
  using EffortReference = friction_compensation_controller::EffortReference;
  ImpedanceLaw impedance_;
  Output output_;
  realtime_tools::RealtimeBuffer<EffortReference> input_;
  friction_compensation_controller::EffortReferenceGate input_gate_;
  std::atomic<uint64_t> input_sequence_{0};
  // Optional position targets. Without a publisher, activation-pose hold is unchanged.
  realtime_tools::RealtimeBuffer<EffortReference> position_input_;
  std::atomic<uint64_t> position_sequence_{0};
  uint64_t applied_position_sequence_{0};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_subscription_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>> state_publisher_;
  double state_elapsed_{0.};
  uint64_t updates_{0};
  WrenchProjection wrench_projection_;
  ExternalTorqueObserver torque_observer_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> measured_effort_interfaces_;
  WrenchProjection::Result wrench_result_;
  bool wrench_enabled_{true};
  double wrench_elapsed_{0.}, wrench_period_{.02};
  bool torque_observer_valid_{false};
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> wrench_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>> wrench_valid_publisher_;
};
}  // namespace omy_joint_impedance
