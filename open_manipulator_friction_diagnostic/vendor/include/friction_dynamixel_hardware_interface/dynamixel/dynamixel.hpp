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

#ifndef FRICTION_DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_HPP_
#define FRICTION_DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_HPP_

#include "friction_dynamixel_hardware_interface/dynamixel/dynamixel_info.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <map>
#include <queue>
#include <string>
#include <vector>
#include <iostream>
#include <cstdarg>
#include <memory>
#include <functional>
#include <utility>

namespace friction_dynamixel_hardware_interface
{

/// @brief Control modes for Dynamixel motors.
#define DXL_CURRENT_CTRL_MODE   0  ///< Current control mode.
#define DXL_POSITION_CTRL_MODE  3  ///< Position control mode.
#define DXL_VELOCITY_CTRL_MODE  1  ///< Velocity control mode.

/// @brief Torque states for Dynamixel motors.
#define TORQUE_ON  1  ///< Torque enabled.
#define TORQUE_OFF 0  ///< Torque disabled.

/// @brief Communication types for data transfer.
#define SYNC 0  ///< Synchronous communication.
#define BULK 1  ///< Bulk communication.

/// @brief Maximum number of retries for communication operations
#define MAX_COMM_RETRIES 5  ///< Maximum number of retries for communication operations

/// @brief Error codes for Dynamixel operations.
enum DxlError
{
  OK = 0,                          ///< No error.
  CANNOT_FIND_CONTROL_ITEM = -1,   ///< Control item not found.
  OPEN_PORT_FAIL = -2,             ///< Failed to open port.
  INDIRECT_ADDR_FAIL = -3,         ///< Indirect address setup failed.
  ITEM_WRITE_FAIL = -4,            ///< Failed to write item.
  ITEM_READ_FAIL = -5,             ///< Failed to read item.
  SYNC_WRITE_FAIL = -6,            ///< Sync write failed.
  SYNC_READ_FAIL  = -7,            ///< Sync read failed.
  SET_SYNC_WRITE_FAIL = -8,        ///< Failed to configure sync write.
  SET_SYNC_READ_FAIL = -9,         ///< Failed to configure sync read.
  BULK_WRITE_FAIL = -10,           ///< Bulk write failed.
  BULK_READ_FAIL  = -11,           ///< Bulk read failed.
  SET_BULK_WRITE_FAIL = -12,       ///< Failed to configure bulk write.
  SET_BULK_READ_FAIL = -13,        ///< Failed to configure bulk read.
  SET_READ_ITEM_FAIL = -14,        ///< Failed to set read item.
  SET_WRITE_ITEM_FAIL = -15,       ///< Failed to set write item.
  DXL_HARDWARE_ERROR = -16,        ///< Hardware error detected.
  DXL_REBOOT_FAIL = -17            ///< Reboot failed.
};

// Hardware Error Status bit definitions
struct HardwareErrorStatusBitInfo
{
  int bit;
  const char * label;
  const char * description;
};

static constexpr HardwareErrorStatusBitInfo HardwareErrorStatusTable[] = {
  {0, "Input Voltage Error", "Detects that input voltage exceeds the configured operating voltage"},
  {1, "Motor Hall Sensor Error", "Detects that Motor hall sensor value exceeds normal range"},
  {2, "Overheating Error",
    "Detects that internal temperature exceeds the configured operating temperature"},
  {3, "Motor Encoder Error", "Detects malfunction of the motor encoder"},
  {4,
    "Electrical Shock Error",
    "Detects electric shock on the circuit or insufficient power to operate the motor"},
  {5, "Overload Error", "Detects that persistent load exceeds maximum output"},
  {6, "Not used", "Always 0"},
  {7, "Not used", "Always 0"}
};

inline const HardwareErrorStatusBitInfo * get_hardware_error_status_bit_info(int bit)
{
  for (const auto & entry : HardwareErrorStatusTable) {
    if (entry.bit == bit) {return &entry;}
  }
  return nullptr;
}

// Error Code (153) definitions
struct ErrorCodeInfo
{
  int value;
  const char * label;
  const char * description;
};

static constexpr ErrorCodeInfo ErrorCodeTable[] = {
  {0x00, "No Error", "No error"},
  {0x01, "Over Voltage Error", "Device supply voltage exceeds the Max Voltage Limit(60)"},
  {0x02, "Low Voltage Error", "Device supply voltage exceeds the Min Voltage Limit(62)"},
  {0x03,
    "Inverter Overheating Error",
    "The inverter temperature has exceeded the Inverter Temperature Limit(56)"},
  {0x04,
    "Motor Overheating Error",
    "The motor temperature has exceeded the Motor Temperature Limit(57)"},
  {0x05, "Overload Error", "Operating current exceeding rated current for an extended duration"},
  {0x07, "Inverter Error", "An issue has occurred with the inverter"},
  {0x09, "Battery Warning", "Low Multi-turn battery voltage. Replacement recommended"},
  {0x0A, "Battery Error", "Multi-turn battery voltage is too low, causing issues"},
  {0x0B, "Magnet Error", "Multi-turn position lost. Multi-turn reset required"},
  {0x0C, "Multi-turn Error", "An issue has occurred with the Multi-turn IC"},
  {0x0D, "Encoder Error", "An issue has occurred with the Encoder IC"},
  {0x0E, "Hall Sensor Error", "An issue has occurred with the Hall Sensor"},
  {0x0F, "Calibration Error", "Cannot find calibration Data"},
  {0x11, "Following Error", "Position control error exceeds the Following Error Threshold(44)"},
  {0x12, "Bus Watchdog Error", "An issue has occurred with the Bus Watchdog"},
  {0x13, "Over Speed Error", "Rotates at a speed of 120% or more than the Velocity Limit(72)"},
  {0x14,
    "Position Limit Reached Error",
    "In position control mode, the current position has moved beyond the Max/Min Position Limit"
    " + Position Limit Threshold(38) range."}
};

inline const ErrorCodeInfo * get_error_code_info(int value)
{
  for (const auto & entry : ErrorCodeTable) {
    if (entry.value == value) {return &entry;}
  }
  return nullptr;
}

/**
 * @struct IndirectInfo
 * @brief Structure for storing indirect addressing information for Dynamixel motors.
 */
typedef struct
{
  uint16_t indirect_data_addr;      ///< Base address for indirect data.
  uint16_t cnt;                     ///< Number of control items.
  uint8_t size;                     ///< Total size in bytes.
  std::vector<std::string> item_name;  ///< Names of the control items.
  std::vector<uint8_t> item_size;  ///< Sizes of each control item in bytes.
} IndirectInfo;

/**
 * @struct RWItemBufInfo
 * @brief Buffer structure for read/write operations.
 */
typedef struct
{
  uint8_t comm_id;                  ///< Communication ID used to reach the device.
  uint8_t id;                       ///< ID of the Dynamixel motor.
  ControlItem control_item;         ///< Control item details.
  uint32_t data;                    ///< Data associated with the control item.
  bool read_flag;                   ///< Flag to indicate if the item has been read.
} RWItemBufInfo;

/**
 * @struct RWItemList
 * @brief List structure for managing read/write items for Dynamixel motors.
 */
typedef struct
{
  uint8_t comm_id;                             ///< ID of the Dynamixel to be communicated.
  std::vector<uint8_t> id_arr;                 ///< IDs of the Dynamixel motors.
  std::vector<std::string> item_name;          ///< List of control item names.
  std::vector<uint8_t> item_size;              ///< Sizes of the control items.
  std::vector<uint16_t> item_addr;             ///< Addresses of the control items.
  std::vector<std::shared_ptr<double>> item_data_ptr_vec;  ///< Pointers to the data.
} RWItemList;

class Dynamixel
{
private:
  // dxl communication variable
  dynamixel::PortHandler * port_handler_ = nullptr;
  dynamixel::PacketHandler * packet_handler_ = nullptr;

  // dxl info variable from dxl_model file
  DynamixelInfo dxl_info_;

  // item write variable
  std::vector<RWItemBufInfo> write_item_buf_;
  std::vector<RWItemBufInfo> read_item_buf_;
  std::map<std::pair<uint8_t /*comm_id*/, uint8_t /*id*/>, bool> torque_state_;

  // read item (sync or bulk) variable
  bool read_type_;
  std::vector<RWItemList> read_data_list_;

  // sync read
  dynamixel::GroupSyncRead * group_sync_read_ = nullptr;
  // bulk read
  dynamixel::GroupBulkRead * group_bulk_read_ = nullptr;
  // fast sync read
  dynamixel::GroupFastSyncRead * group_fast_sync_read_ = nullptr;
  // fast bulk read
  dynamixel::GroupFastBulkRead * group_fast_bulk_read_ = nullptr;

  // fast read protocol state (applies to both sync and bulk)
  bool use_fast_read_protocol_ = true;
  bool fast_read_permanent_ = false;
  int fast_read_fail_count_ = 0;

  // indirect inform for sync read
  std::map<uint8_t /*id*/, IndirectInfo> indirect_info_read_;

  // write item (sync or bulk) variable
  bool write_type_;
  std::vector<RWItemList> write_data_list_;

  // sync write
  dynamixel::GroupSyncWrite * group_sync_write_ = nullptr;
  // indirect inform for sync write
  std::map<uint8_t /*id*/, IndirectInfo> indirect_info_write_;

  // bulk write
  dynamixel::GroupBulkWrite * group_bulk_write_ = nullptr;
  // Opt-in diagnostic transport: one SyncWrite per address/length group.
  bool use_grouped_sync_write_ = false;
  std::map<std::pair<uint16_t, uint16_t>, std::unique_ptr<dynamixel::GroupSyncWrite>>
    grouped_sync_writes_;
  // direct inform for bulk write
  std::map<uint8_t /*id*/, IndirectInfo> direct_info_write_;

public:
  explicit Dynamixel(const char * path);
  ~Dynamixel();

  // DXL Communication Setting
  DxlError SetupPort(const std::string & port_name, const std::string & baudrate);
  DxlError InitDxlComm(uint8_t comm_id, uint8_t id);
  DxlError Reboot(uint8_t id);
  void RWDataReset();

  // DXL Read Setting
  DxlError SetDxlReadItems(
    uint8_t comm_id, uint8_t id, std::vector<std::string> item_names,
    std::vector<std::shared_ptr<double>> data_vec_ptr);
  DxlError SetMultiDxlRead();

  // DXL Write Setting
  DxlError SetDxlWriteItems(
    uint8_t comm_id, uint8_t id, std::vector<std::string> item_names,
    std::vector<std::shared_ptr<double>> data_vec_ptr);
  DxlError SetMultiDxlWrite();
  void SetGroupedSyncWrite(bool enabled) {use_grouped_sync_write_ = enabled;}
  DxlError SetupGroupedSyncWrite();
  DxlError SetDxlValueToGroupedSyncWrite();

  // Read Item (sync or bulk)
  DxlError ReadMultiDxlData(double period_ms);
  // Write Item (sync or bulk)
  DxlError WriteMultiDxlData();

  // Set Dxl Option
  // DxlError SetOperatingMode(uint8_t id, uint8_t dynamixel_mode);
  DxlError DynamixelEnable(const std::vector<std::pair<uint8_t, uint8_t>> & comm_id_id_arr);
  DxlError DynamixelDisable(const std::vector<std::pair<uint8_t, uint8_t>> & comm_id_id_arr);

  // DXL Item Write
  DxlError WriteItem(uint8_t comm_id, uint8_t id, std::string item_name, uint32_t data);
  DxlError WriteItem(uint8_t comm_id, uint8_t id, uint16_t addr, uint8_t size, uint32_t data);
  DxlError InsertWriteItemBuf(uint8_t id, std::string item_name, uint32_t data);
  DxlError WriteItemBuf();

  // DXL Item Read
  DxlError ReadItem(uint8_t comm_id, uint8_t id, std::string item_name, uint32_t & data);
  DxlError InsertReadItemBuf(uint8_t id, std::string item_name);
  DxlError ReadItemBuf();
  bool CheckReadItemBuf(uint8_t id, std::string item_name);
  uint32_t GetReadItemDataBuf(uint8_t id, std::string item_name);

  DynamixelInfo GetDxlInfo() {return dxl_info_;}
  std::map<std::pair<uint8_t, uint8_t>, bool> GetDxlTorqueState() {return torque_state_;}

  static std::string DxlErrorToString(DxlError error_num);

  DxlError ReadDxlModelFile(uint8_t comm_id, uint8_t id, uint16_t model_num);
  DxlError ReadDxlModelFile(
    uint8_t comm_id, uint8_t id, uint16_t model_num,
    uint8_t firmware_version);
  DxlError ReadFirmwareVersion(uint8_t comm_id, uint8_t id, uint8_t & firmware_version);

  DxlError InitTorqueStates(
    std::vector<std::pair<uint8_t, uint8_t>> comm_id_id_arr,
    bool disable_torque = false);

  void OverrideUnitInfo(
    uint8_t comm_id,
    uint8_t id,
    const std::string & data_name,
    double unit_multiplier,
    bool is_signed,
    double offset_value);

private:
  bool checkReadType();
  bool checkWriteType();

  // SyncRead
  DxlError SetSyncReadItemAndHandler();
  DxlError SetSyncReadHandler(std::vector<uint8_t> id_arr);
  DxlError GetDxlValueFromSyncRead(double period_ms);
  DxlError SetFastSyncReadHandler(std::vector<uint8_t> id_arr);

  // BulkRead
  DxlError SetBulkReadItemAndHandler();
  DxlError SetBulkReadHandler(std::vector<uint8_t> id_arr);
  DxlError GetDxlValueFromBulkRead(double period_ms);
  DxlError SetFastBulkReadHandler(std::vector<uint8_t> id_arr);

  // DirectRead for BulkRead
  DxlError AddDirectRead(uint8_t id, std::string item_name, uint16_t item_addr, uint8_t item_size);

  // Check Indirect Read
  DxlError CheckIndirectReadAvailable(uint8_t id);

  // Read - Indirect Address
  void ResetIndirectRead(std::vector<uint8_t> id_arr);
  DxlError AddIndirectRead(
    uint8_t id,
    std::string item_name,
    uint16_t item_addr,
    uint8_t item_size);

  // Read - Data Processing
  DxlError ProcessReadData(
    uint8_t comm_id,
    uint16_t indirect_addr,
    const std::vector<uint8_t> & id_arr,
    const std::vector<std::string> & item_names,
    const std::vector<uint8_t> & item_sizes,
    const std::vector<std::shared_ptr<double>> & data_ptrs,
    std::function<uint32_t(uint8_t, uint16_t, uint8_t)> get_data_func);

  DxlError ProcessDirectReadData(
    uint8_t comm_id,
    const std::vector<uint16_t> & item_addrs,
    const std::vector<std::string> & item_names,
    const std::vector<uint8_t> & item_sizes,
    const std::vector<std::shared_ptr<double>> & data_ptrs,
    std::function<uint32_t(uint8_t, uint16_t, uint8_t)> get_data_func);

  // Read - Communication
  DxlError ProcessReadCommunication(
    dynamixel::PortHandler * port_handler,
    double period_ms,
    bool is_sync,
    bool is_fast);

  // SyncWrite
  DxlError SetSyncWriteItemAndHandler();
  DxlError SetSyncWriteHandler(std::vector<uint8_t> id_arr);
  DxlError SetDxlValueToSyncWrite();

  // BulkWrite
  DxlError SetBulkWriteItemAndHandler();
  DxlError SetBulkWriteHandler(std::vector<uint8_t> id_arr);
  DxlError SetDxlValueToBulkWrite();

  // Check Indirect Write
  DxlError CheckIndirectWriteAvailable(uint8_t id);

  // Write - Indirect Address
  void ResetIndirectWrite(std::vector<uint8_t> id_arr);
  void ResetDirectWrite(std::vector<uint8_t> id_arr);
  DxlError AddIndirectWrite(
    uint8_t id,
    std::string item_name,
    uint16_t item_addr,
    uint8_t item_size);

  // Helper function for value conversion with unit info
  double ConvertValueWithUnitInfo(
    uint8_t comm_id,
    uint8_t id,
    std::string item_name,
    uint32_t raw_value,
    uint8_t size,
    bool is_signed);

  // Helper function for converting unit values to raw values
  uint32_t ConvertUnitValueToRawValue(
    uint8_t comm_id,
    uint8_t id,
    std::string item_name,
    double unit_value,
    uint8_t size,
    bool is_signed);

  // Helper function for writing values to buffer
  void WriteValueToBuffer(uint8_t * buffer, uint8_t offset, uint32_t value, uint8_t size);
};

}  // namespace friction_dynamixel_hardware_interface

#endif  // FRICTION_DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_HPP_
