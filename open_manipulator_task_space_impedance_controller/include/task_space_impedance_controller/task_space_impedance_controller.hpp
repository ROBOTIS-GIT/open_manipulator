#pragma once
#include "task_space_impedance_controller/task_law.hpp"
#include "friction_compensation_controller/friction_compensation_controller.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace omy_task
{
class TaskSpaceImpedanceController : public friction_compensation_controller::FrictionCompensationController
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  using Base = friction_compensation_controller::FrictionCompensationController;
  using Effort = friction_compensation_controller::EffortReference;
  struct Target {Pose pose; int64_t stamp_ns{0}, capture_stamp_ns{-1}; uint64_t sequence{0};};
  bool gravity_compensation_enabled() const override {return gravity_enabled_;}
  bool friction_compensation_enabled() const override {return friction_policy_;}
  bool read_state(Vector6 & position, Vector6 & velocity) const;
  bool kinematics(const Vector6 & position, Pose & pose);
  void publish(const rclcpp::Time & time, const Pose & pose, const Vector6 & q,
    const Vector6 & velocity, const std::array<double, 6> & external, bool ready);
  Gains gains_;
  Result output_;
  Pose target_;
  bool gravity_enabled_{true}, friction_policy_{true}, was_ready_{false};
  std::string root_, tip_;
  double publish_period_{.02}, publish_elapsed_{0.};
  uint64_t applied_target_{0}, updates_{0}, reference_epoch_{0};
  std::atomic<uint64_t> target_sequence_{0}, effort_sequence_{0};
  std::atomic<int64_t> reference_stamp_ns_{-1};
  std::atomic<bool> capture_requested_{false};
  realtime_tools::RealtimeBuffer<Target> target_buffer_;
  realtime_tools::RealtimeBuffer<Effort> effort_buffer_;
  friction_compensation_controller::EffortReferenceGate effort_gate_;
  KDL::Chain chain_;
  KDL::JntArray q_kdl_{6};
  KDL::Jacobian jacobian_{6};
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_target_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> pose_publisher_, target_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> wrench_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>> state_publisher_;
};
}  // namespace omy_task
