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
// Authors: Hye-Jong KIM, Sungho Woo, Woojin Wie

#ifndef FRICTION_DYNAMIXEL_HARDWARE_INTERFACE__FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_HPP_
#define FRICTION_DYNAMIXEL_HARDWARE_INTERFACE__FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <functional>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "friction_dynamixel_hardware_interface/visibility_control.h"
#include "friction_dynamixel_hardware_interface/dynamixel/dynamixel.hpp"

#include "dynamixel_interfaces/msg/dynamixel_state.hpp"
#include "dynamixel_interfaces/srv/get_data_from_dxl.hpp"
#include "dynamixel_interfaces/srv/set_data_to_dxl.hpp"
#include "dynamixel_interfaces/srv/reboot_dxl.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "std_srvs/srv/set_bool.hpp"


namespace friction_dynamixel_hardware_interface
{
/**
 * @brief Constants for hardware state interface names.
 */
constexpr char HW_IF_HARDWARE_STATE[] = "hardware_state";
constexpr char HW_IF_TORQUE_ENABLE[] = "torque_enable";

/**
 * @brief Struct for handling variable types associated with Dynamixel components.
 */
typedef struct HandlerVarType_
{
  uint8_t id;                                /**< ID of the Dynamixel component. */
  uint8_t comm_id;                           /**< ID of the Dynamixel to be communicated. */
  std::string name;                          /**< Name of the component. */
  std::vector<std::string> interface_name_vec; /**< Vector of interface names. */
  std::vector<std::shared_ptr<double>> value_ptr_vec; /**< Vector interface values. */
} HandlerVarType;

/**
 * @brief Enum for Dynamixel status.
 */
typedef enum DxlStatus
{
  DXL_OK = 0,    /**< Normal status. */
  HW_ERROR = 1,  /**< Hardware error status. */
  COMM_ERROR = 2,/**< Communication error status. */
  REBOOTING = 3, /**< Rebooting status. */
} DxlStatus;

/**
 * @brief Enum for Dynamixel torque status.
 */
typedef enum DxlTorqueStatus
{
  TORQUE_ENABLED = 0,        /**< Torque is enabled. */
  TORQUE_DISABLED = 1,       /**< Torque is disabled. */
  REQUESTED_TO_ENABLE = 2,   /**< Torque enable is requested. */
  REQUESTED_TO_DISABLE = 3,  /**< Torque disable is requested. */
} DxlTorqueStatus;

/**
 * @class DynamixelHardware
 * @brief Class for interfacing with Dynamixel hardware using the ROS2 hardware interface.
 *
 * This class is responsible for initializing, reading, and writing to Dynamixel actuators.
 * It also handles errors, resets, and publishes state information using ROS2 services and topics.
 */
class DynamixelHardware : public
  hardware_interface::SystemInterface, rclcpp::Node
{
public:
  /**
   * @brief Constructor for DynamixelHardware.
   */
  DynamixelHardware();

  /**
   * @brief Destructor for DynamixelHardware.
   */
  ~DynamixelHardware();

  /**
   * @brief Initialization callback for hardware interface.
   * @param params Parameters for the hardware component interface.
   * @return Callback return indicating success or error.
   */
  FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  /**
   * @brief Exports state interfaces for ROS2.
   * @return A vector of state interfaces.
   */
  FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Exports command interfaces for ROS2.
   * @return A vector of command interfaces.
   */
  FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Callback for activating the hardware interface.
   * @param previous_state Previous lifecycle state.
   * @return Callback return indicating success or error.
   */
  FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;

  /**
   * @brief Callback for deactivating the hardware interface.
   * @param previous_state Previous lifecycle state.
   * @return Callback return indicating success or error.
   */
  FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;

  /**
   * @brief Reads data from the hardware.
   * @param time Current time.
   * @param period Duration since the last read.
   * @return Hardware interface return type indicating success or error.
   */
  FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type  read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Writes data to the hardware.
   * @param time Current time.
   * @param period Duration since the last write.
   * @return Hardware interface return type indicating success or error.
   */
  FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type  write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  ///// ros
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;

  ///// dxl error
  DxlStatus dxl_status_;
  DxlError dxl_comm_err_;
  std::map<uint8_t /*id*/, uint8_t /*err*/> dxl_hw_err_;
  std::map<uint8_t /*id*/, uint8_t /*error code*/> dxl_error_code_;
  DxlTorqueStatus dxl_torque_status_;
  std::map<std::pair<uint8_t /*comm_id*/, uint8_t /*id*/>, bool /*enable*/> dxl_torque_state_;
  std::vector<std::pair<uint8_t, uint8_t>> torque_enabled_comm_id_id_;
  double err_timeout_ms_;
  rclcpp::Duration read_error_duration_{0, 0};
  rclcpp::Duration write_error_duration_{0, 0};
  bool is_read_in_error_{false};
  bool is_write_in_error_{false};

  bool use_revolute_to_prismatic_{false};
  std::string conversion_dxl_name_{""};
  std::string conversion_joint_name_{""};
  double prismatic_min_{0.0};
  double prismatic_max_{0.0};
  double revolute_min_{0.0};
  double revolute_max_{0.0};
  double conversion_slope_{0.0};
  double conversion_intercept_{0.0};

  /**
   * @brief Starts the hardware interface.
   * @return Callback return indicating success or error.
   */
  hardware_interface::CallbackReturn start();

  /**
   * @brief Stops the hardware interface.
   * @return Callback return indicating success or error.
   */
  hardware_interface::CallbackReturn stop();

  /**
   * @brief Checks for communication errors.
   * @param dxl_comm_err The communication error status.
   * @return The resulting error status.
   */
  DxlError CheckError(DxlError dxl_comm_err);

  /**
   * @brief Resets communication with the hardware.
   * @return True if the reset was successful, false otherwise.
   */
  bool CommReset();

  ///// dxl variable
  std::string port_name_;
  std::string baud_rate_;
  std::vector<std::pair<uint8_t, uint8_t>> dxl_comm_id_id_;
  std::vector<std::pair<uint8_t, uint8_t>> virtual_dxl_comm_id_id_;

  std::vector<std::pair<uint8_t, uint8_t>> sensor_comm_id_id_;
  std::map<uint8_t /*id*/, std::string /*interface_name*/> sensor_item_;

  std::vector<std::pair<uint8_t, uint8_t>> controller_comm_id_id_;
  std::map<uint8_t /*id*/, std::string /*interface_name*/> controller_item_;

  ///// handler variable
  std::vector<HandlerVarType> hdl_trans_states_;
  std::vector<HandlerVarType> hdl_trans_commands_;
  std::vector<HandlerVarType> hdl_joint_states_;
  std::vector<HandlerVarType> hdl_joint_commands_;

  ///// handler sensor variable
  std::vector<HandlerVarType> hdl_gpio_sensor_states_;
  std::vector<HandlerVarType> hdl_sensor_states_;

  ///// handler controller variable
  std::vector<HandlerVarType> hdl_gpio_controller_states_;
  std::vector<HandlerVarType> hdl_gpio_controller_commands_;

  bool is_set_hdl_{false};

  // joint <-> transmission matrix
  size_t num_of_joints_;
  size_t num_of_transmissions_;
  std::vector<std::vector<double>> transmission_to_joint_matrix_;
  std::vector<std::vector<double>> joint_to_transmission_matrix_;

  /**
   * @brief Helper function to initialize items
   * @return True if initialization was successful, false otherwise.
   */
  bool InitItem(const hardware_interface::ComponentInfo & gpio);

  /**
   * @brief Initializes the read items for Dynamixel.
   * @return True if initialization was successful, false otherwise.
   */
  bool InitDxlReadItems();

  /**
   * @brief Initializes the write items for Dynamixel.
   * @return True if initialization was successful, false otherwise.
   */
  bool InitDxlWriteItems();

  /**
   * @brief Reads sensor data.
   * @param sensor The sensor handler variable.
   */
  void ReadSensorData(const HandlerVarType & sensor);

  ///// function
  /**
   * @brief Sets up the joint-to-transmission and transmission-to-joint matrices.
   * @return True if matrices were set up successfully, false otherwise.
   */
  bool SetMatrix();

  /**
   * @brief Calculates the joint states from transmission states.
   */
  void CalcTransmissionToJoint();

  /**
   * @brief Calculates the transmission commands from joint commands.
   */
  void CalcJointToTransmission();

  /**
   * @brief Synchronizes joint commands with the joint states.
   */
  void SyncJointCommandWithStates();

  /**
   * @brief Changes the torque state of the Dynamixel.
   */
  void ChangeDxlTorqueState();

  using DynamixelStateMsg = dynamixel_interfaces::msg::DynamixelState;
  using StatePublisher = realtime_tools::RealtimePublisher<DynamixelStateMsg>;
  rclcpp::Publisher<DynamixelStateMsg>::SharedPtr dxl_state_pub_;
  std::unique_ptr<StatePublisher> dxl_state_pub_uni_ptr_;
  DynamixelStateMsg dxl_state_msg_;

  rclcpp::Service<dynamixel_interfaces::srv::GetDataFromDxl>::SharedPtr get_dxl_data_srv_;
  // Keep service dispatch on the bus-owning read thread, without rebuilding
  // the executor and ROS graph associations on every hardware cycle.
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> service_executor_;
  void get_dxl_data_srv_callback(
    const std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Request> request,
    std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Response> response);

  rclcpp::Service<dynamixel_interfaces::srv::SetDataToDxl>::SharedPtr set_dxl_data_srv_;
  void set_dxl_data_srv_callback(
    const std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Request> request,
    std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Response> response);

  rclcpp::Service<dynamixel_interfaces::srv::RebootDxl>::SharedPtr reboot_dxl_srv_;
  void reboot_dxl_srv_callback(
    const std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Request> request,
    std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Response> response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_dxl_torque_srv_;
  void set_dxl_torque_srv_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  rclcpp::Service<dynamixel_interfaces::srv::GetDataFromDxl>::SharedPtr get_dxl_error_summary_srv_;
  void get_dxl_error_summary_srv_callback(
    const std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Request> request,
    std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Response> response);

  void initRevoluteToPrismaticParam();

  double revoluteToPrismatic(double revolute_value);

  double prismaticToRevolute(double prismatic_value);

  /**
   * @brief Get a formatted error summary for a specific Dynamixel ID
   * @param id The Dynamixel ID
   * @return A string containing the error summary
   */
  std::string getErrorSummary(uint8_t id) const;

  /**
   * @brief Get error summaries for all Dynamixel IDs
   * @return A string containing error summaries for all Dynamixels
   */
  std::string getAllErrorSummaries() const;

  void MapInterfaces(
    size_t outer_size,
    size_t inner_size,
    std::vector<HandlerVarType> & outer_handlers,
    std::vector<HandlerVarType> & inner_handlers,
    const std::vector<std::vector<double>> & matrix,
    const std::unordered_map<std::string, std::vector<std::string>> & iface_map,
    const std::string & conversion_iface = "",
    const std::string & conversion_name = "",
    std::function<double(double)> conversion = nullptr);

  // Move dxl_comm_ to the end for safe destruction order
  std::shared_ptr<Dynamixel> dxl_comm_;
};

// Conversion maps between ROS2 and Dynamixel interface names
inline const std::unordered_map<std::string, std::vector<std::string>> ros2_to_dxl_cmd_map = {
  {hardware_interface::HW_IF_POSITION, {"Goal Position"}},
  {hardware_interface::HW_IF_VELOCITY, {"Goal Velocity"}},
  {hardware_interface::HW_IF_EFFORT, {"Goal Current"}}
};

// Mapping for Dynamixel command interface names to ROS2 state interface names
inline const std::unordered_map<std::string, std::vector<std::string>> dxl_to_ros2_cmd_map = {
  {"Goal Position", {hardware_interface::HW_IF_POSITION}},
  {"Goal Velocity", {hardware_interface::HW_IF_VELOCITY}},
  {"Goal Current", {hardware_interface::HW_IF_EFFORT}}
};

// Mapping for ROS2 state interface names to Dynamixel state interface names
inline const std::unordered_map<std::string, std::vector<std::string>> ros2_to_dxl_state_map = {
  {hardware_interface::HW_IF_POSITION, {"Present Position"}},
  {hardware_interface::HW_IF_VELOCITY, {"Present Velocity"}},
  {hardware_interface::HW_IF_EFFORT, {"Present Current", "Present Load"}}
};

}  // namespace friction_dynamixel_hardware_interface

#endif  // FRICTION_DYNAMIXEL_HARDWARE_INTERFACE__FRICTION_DYNAMIXEL_HARDWARE_INTERFACE_HPP_
