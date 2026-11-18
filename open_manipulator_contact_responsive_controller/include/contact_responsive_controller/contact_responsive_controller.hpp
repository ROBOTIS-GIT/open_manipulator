#pragma once
#include "contact_responsive_controller/contact_law.hpp"
#include "friction_compensation_controller/friction_compensation_controller.hpp"
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

namespace omy_contact
{
class ContactResponsiveController : public friction_compensation_controller::FrictionCompensationController
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  using Base=friction_compensation_controller::FrictionCompensationController;
  using Reference=friction_compensation_controller::EffortReference;
  bool gravity_compensation_enabled() const override {return gravity_enabled_;}
  bool friction_compensation_enabled() const override {return friction_enabled_policy_;}
  bool gravity_enabled_{true},friction_enabled_policy_{true},was_ready_{false},fault_latched_{false};
  Settings settings_;
  ContactLaw law_;
  Result output_;
  Vector goal_{Vector::Zero()},hold_{Vector::Zero()},last_motion_{Vector::Zero()},fault_start_{Vector::Zero()};
  Vector rotor_inertia_{Vector::Zero()};
  double fault_elapsed_{0.},fallback_ramp_{.25},state_elapsed_{0.};
  uint64_t updates_{0},applied_reference_{0};
  std::atomic<uint64_t> target_sequence_{0},effort_sequence_{0};
  std::atomic<bool> reset_requested_{false};
  realtime_tools::RealtimeBuffer<Reference> target_input_,effort_input_;
  friction_compensation_controller::EffortReferenceGate effort_gate_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>> telemetry_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dynamics_;
  KDL::JntArray qn_{6},vn_{6},coriolis_{6};
  KDL::JntSpaceInertiaMatrix inertia_{6};
};
}  // namespace omy_contact
