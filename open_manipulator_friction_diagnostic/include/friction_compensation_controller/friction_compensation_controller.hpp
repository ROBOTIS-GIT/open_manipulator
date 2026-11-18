// Copyright 2017 Open Source Robotics Foundation, Inc.
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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef GRAVITY_COMPENSATION_CONTROLLER__GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__GRAVITY_COMPENSATION_CONTROLLER_HPP_

#include <atomic>
#include "friction_compensation_controller/effort_reference.hpp"
#include "friction_compensation_controller/calibrated_friction.hpp"
#include "friction_compensation_controller/position_velocity_observer.hpp"
#include "friction_compensation_controller/stateful_friction.hpp"
#include "friction_compensation_controller/friction_reset_interlock.hpp"
#include <optional>
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "friction_compensation_controller/visibility_control.h"

#include "std_msgs/msg/bool.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <open_manipulator_friction_diagnostic/friction_compensation_controller_parameters.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>


namespace friction_compensation_controller
{
class FrictionCompensationController : public controller_interface::ControllerInterface
{
public:
  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  FrictionCompensationController();

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // Optional output policy for derived controllers. The original plugin always
  // retains both compensations and its existing parameter/launch behavior.
  virtual bool gravity_compensation_enabled() const {return true;}
  virtual bool friction_compensation_enabled() const {return true;}
  std::string formatVector(const std::vector<double> & vec);

  // Parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // KDL
  KDL::Tree tree_;
  KDL::JntArray q_ddot_;
  KDL::WrenchMap f_ext_;

  realtime_tools::RealtimeBuffer<EffortReference> external_effort_buffer_;
  std::atomic<uint64_t> external_effort_sequence_{0};
  EffortReferenceGate external_effort_gate_;  // Only the control update thread consumes.
  std::atomic<bool> motor_ready_{false};
  std::atomic<bool> motor_ready_applied_{false};
  std::atomic<bool> automatic_enable_pending_{false};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr effort_gate_service_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> command_state_publisher_;
  std::vector<double> actual_commands_;
  double command_publish_elapsed_{0.};

  // State
  bool dither_switch_;
  std::vector<CalibratedFriction> calibrated_friction_;
  std::vector<std::optional<StatefulFriction>> stateful_friction_;
  std::vector<FrictionDiagnostics> friction_details_;
  std::atomic<bool> friction_enabled_{false};
  std::atomic<bool> friction_fault_{false};
  std::atomic<unsigned int> friction_fault_reason_{0};
  FrictionResetInterlock friction_reset_interlock_;
  std::atomic<bool> friction_reset_ready_{false}, friction_reset_pending_{false};
  std::atomic<bool> friction_reset_succeeded_{false};
  std::atomic<unsigned int> friction_reset_requested_{0}, friction_reset_completed_{0};
  std::atomic<int64_t> friction_last_update_ns_{0};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr friction_switch_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr friction_status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr friction_reset_service_;
  std::atomic<bool> controller_active_{false};
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>
  friction_diagnostics_publisher_;
  std::vector<FrictionWindow> friction_windows_;
  std::vector<double> friction_baseline_, friction_command_, friction_raw_velocity_;
  double friction_publish_elapsed_ = 0.;
  bool first_update_ = true;
  // State-derived velocity is used by the new stateful path. The original KDL
  // baseline and retired calibrated branch retain their original input units.
  PositionVelocityObserver position_velocity_observer_;
  PositionVelocityObserver::Result position_velocity_observation_;

  // Interfaces
  std::vector<std::string> state_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
  };
  std::vector<std::string> command_interface_types_ = {
    hardware_interface::HW_IF_EFFORT
  };

  // Storing command joint names for interfaces
  std::vector<std::string> command_joint_names_;

  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in joint_names_
  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  std::vector<std::string> joint_names_;
  size_t n_joints_;

  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> previous_velocities_;
  std::vector<double> tmp_positions_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_flag_sub_;
  realtime_tools::RealtimeBuffer<bool> collision_flag_buffer_;
  bool joint_index_initialized_ = false;
  std::vector<int> joint_name_to_index_;
  realtime_tools::RealtimeBuffer<std::vector<double>> follower_joint_positions_buffer_;
  std::atomic<bool> has_follower_data_{false};
};
}  // namespace friction_compensation_controller

#endif  // GRAVITY_COMPENSATION_CONTROLLER__GRAVITY_COMPENSATION_CONTROLLER_HPP_
