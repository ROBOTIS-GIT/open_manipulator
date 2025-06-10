// Copyright 2024 ROBOTIS CO., LTD.
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
//
// Author: Woojin Wie

#ifndef SPRING_ACTUATOR_CONTROLLER__SPRING_ACTUATOR_CONTROLLER_HPP_
#define SPRING_ACTUATOR_CONTROLLER__SPRING_ACTUATOR_CONTROLLER_HPP_

#include <spring_actuator_controller/visibility_control.h>

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <sstream>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Visibility and parameter definitions
#include <om_spring_actuator_controller/spring_actuator_controller_parameters.hpp>

namespace spring_actuator_controller
{

/**
 * @class SpringActuatorController
 * @brief A ROS 2 controller that simulates a spring-like behavior on a specified joint
 *        (e.g., a “trigger”), while optionally leaving other joints unaltered or with minimal torque.
 *
 * The parameters for this controller (e.g., stiffness, neutral position, friction compensation, etc.)
 * are obtained via a parameter listener defined in @c spring_actuator_controller_parameters.hpp.
 */
class SpringActuatorController : public controller_interface::ControllerInterface
{
public:
  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  SpringActuatorController();

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  //----------------------------------------------------------------------------
  // Lifecycle node interface
  //----------------------------------------------------------------------------

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  SPRING_ACTUATOR_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  /**
   * @brief Utility to convert a vector of doubles into a comma-separated string.
   * @param vec The vector of doubles to format.
   * @return A string representation of the vector.
   */
  std::string formatVector(const std::vector<double> & vec);

  //----------------------------------------------------------------------------
  // Parameters and internal state
  //----------------------------------------------------------------------------

  /**
   * @brief Param listener and the parameter struct holding all configuration values
   *        (e.g., spring stiffness, friction thresholds, etc.).
   */
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  /// Switch for dithering logic to overcome static friction
  bool dither_switch_;

  //----------------------------------------------------------------------------
  // Hardware interfaces
  //----------------------------------------------------------------------------

  /// Types of state interfaces we want: position, velocity
  std::vector<std::string> state_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
  };

  /// Types of command interfaces we want: effort
  std::vector<std::string> command_interface_types_ = {
    hardware_interface::HW_IF_EFFORT
  };

  /**
   * @brief Number of degrees of freedom for the robot/joints we are controlling.
   * This is typically the size of @c params_.joints.
   */
  size_t dof_;

  /**
   * @brief The names of joints we issue commands for.
   * Could match @c joint_names_ or be a subset, depending on parameter usage.
   */
  std::vector<std::string> command_joint_names_;

  /**
   * @brief Store references to command and state interfaces for each joint and interface type.
   *
   * For each interface type (effort, position, velocity, etc.), we have a vector of references,
   * each corresponding to a particular joint. `joint_command_interface_[i][j]` gives the
   * reference for the j-th joint’s i-th command interface type.
   */
  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  //----------------------------------------------------------------------------
  // Joint information
  //----------------------------------------------------------------------------

  /// Names of the joints managed by this controller (from parameters)
  std::vector<std::string> joint_names_;

  /// Number of joints, should match `joint_names_.size()`
  size_t n_joints_;

  /// Cached vectors for joint positions and velocities read during update()
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
};

}  // namespace spring_actuator_controller

#endif  // SPRING_ACTUATOR_CONTROLLER__SPRING_ACTUATOR_CONTROLLER_HPP_
