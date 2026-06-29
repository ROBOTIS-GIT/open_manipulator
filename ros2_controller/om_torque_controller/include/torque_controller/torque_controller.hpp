#ifndef TORQUE_CONTROLLER__TORQUE_CONTROLLER_HPP_
#define TORQUE_CONTROLLER__TORQUE_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "torque_controller/visibility_control.h"

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <om_torque_controller/torque_controller_parameters.hpp>

namespace torque_controller
{

class TorqueController : public controller_interface::ControllerInterface
{
public:
  TORQUE_CONTROLLER_PUBLIC
  TorqueController();

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  TORQUE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  void configure_subscribers();

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  KDL::Tree tree_;
  KDL::WrenchMap f_ext_;

  std::vector<std::string> state_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
  };
  std::vector<std::string> command_interface_types_ = {
    hardware_interface::HW_IF_EFFORT,
  };

  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_joint_names_;
  size_t n_joints_{0};

  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> previous_velocities_;

  realtime_tools::RealtimeBuffer<std::vector<double>> command_torque_buffer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr torque_command_sub_;

  std::atomic<bool> compensate_gravity_{true};
  bool dither_switch_{false};
};

}  // namespace torque_controller

#endif  // TORQUE_CONTROLLER__TORQUE_CONTROLLER_HPP_
