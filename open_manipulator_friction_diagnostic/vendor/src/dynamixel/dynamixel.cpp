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

#include "friction_dynamixel_hardware_interface/dynamixel/dynamixel.hpp"

#include <queue>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <thread>
#include <chrono>
#include <cmath>

namespace friction_dynamixel_hardware_interface
{

Dynamixel::Dynamixel(const char * path)
: port_handler_(nullptr),
  packet_handler_(nullptr),
  group_sync_read_(nullptr),
  group_bulk_read_(nullptr),
  group_fast_sync_read_(nullptr),
  group_fast_bulk_read_(nullptr),
  group_sync_write_(nullptr),
  group_bulk_write_(nullptr)
{
  dxl_info_.SetDxlModelFolderPath(path);
  dxl_info_.InitDxlModelInfo();

  write_item_buf_.clear();
  read_item_buf_.clear();
}

Dynamixel::~Dynamixel()
{
  grouped_sync_writes_.clear();
  fprintf(stderr, "Dynamixel destructor start\n");
  if (group_sync_read_) {
    delete group_sync_read_;
    group_sync_read_ = nullptr;
  }
  if (group_fast_sync_read_) {
    delete group_fast_sync_read_;
    group_fast_sync_read_ = nullptr;
  }
  if (group_bulk_read_) {
    delete group_bulk_read_;
    group_bulk_read_ = nullptr;
  }
  if (group_fast_bulk_read_) {
    delete group_fast_bulk_read_;
    group_fast_bulk_read_ = nullptr;
  }
  if (group_sync_write_) {
    delete group_sync_write_;
    group_sync_write_ = nullptr;
  }
  if (group_bulk_write_) {
    delete group_bulk_write_;
    group_bulk_write_ = nullptr;
  }
  if (port_handler_) {
    port_handler_->closePort();
    delete port_handler_;
    port_handler_ = nullptr;
  }
  packet_handler_ = nullptr;
  fprintf(stderr, "Dynamixel destructor end\n");
}

DxlError Dynamixel::ReadDxlModelFile(uint8_t comm_id, uint8_t id, uint16_t model_num)
{
  try {
    dxl_info_.ReadDxlModelFile(comm_id, id, model_num);
    return DxlError::OK;
  } catch (const std::exception & e) {
    fprintf(
      stderr, "[ReadDxlModelFile][comm_id:%03d][ID:%03d] Error reading model file: %s\n",
      comm_id, id, e.what());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
}

DxlError Dynamixel::ReadDxlModelFile(
  uint8_t comm_id, uint8_t id, uint16_t model_num,
  uint8_t firmware_version)
{
  try {
    dxl_info_.ReadDxlModelFile(comm_id, id, model_num, firmware_version);
    return DxlError::OK;
  } catch (const std::exception & e) {
    fprintf(
      stderr, "[ReadDxlModelFile][comm_id:%03d][ID:%03d] Error reading model file: %s\n",
      comm_id, id, e.what());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
}

DxlError Dynamixel::ReadFirmwareVersion(uint8_t comm_id, uint8_t id, uint8_t & firmware_version)
{
  uint32_t fw_version_data;
  DxlError result = ReadItem(comm_id, id, "Firmware Version", fw_version_data);
  if (result == DxlError::OK) {
    firmware_version = static_cast<uint8_t>(fw_version_data);
  } else {
    fprintf(
      stderr,
      "[ReadFirmwareVersion][comm_id:%03d][ID:%03d] Failed to read firmware version\n", comm_id,
      id);
  }
  return result;
}

DxlError Dynamixel::InitTorqueStates(
  std::vector<std::pair<uint8_t, uint8_t>> comm_id_id_arr,
  bool disable_torque)
{
  for (auto it_pair : comm_id_id_arr) {
    uint8_t it_comm = it_pair.first;
    uint8_t it_id = it_pair.second;
    try {
      if (dxl_info_.CheckDxlControlItem(it_comm, it_id, "Torque Enable")) {
        uint32_t torque_state = 0;
        DxlError result = ReadItem(it_comm, it_id, "Torque Enable", torque_state);
        if (result != DxlError::OK) {
          fprintf(
            stderr, "[InitTorqueStates][comm_id:%03d][ID:%03d] Error reading torque state\n",
            it_comm, it_id);
          return result;
        }

        torque_state_[{it_comm, it_id}] = torque_state;

        if (torque_state_[{it_comm, it_id}] == TORQUE_ON) {
          if (disable_torque) {
            fprintf(
              stderr,
              "[InitTorqueStates][comm_id:%03d][ID:%03d] Torque is enabled, disabling torque\n",
              it_comm, it_id);
            result = WriteItem(it_comm, it_id, "Torque Enable", TORQUE_OFF);
            if (result != DxlError::OK) {
              fprintf(
                stderr, "[InitTorqueStates][comm_id:%03d][ID:%03d] Error disabling torque\n",
                it_comm, it_id);
              return result;
            }
            torque_state_[{it_comm, it_id}] = TORQUE_OFF;
          } else {
            torque_state_[{it_comm, it_id}] = TORQUE_OFF;
            fprintf(
              stderr,
              "[InitTorqueStates][comm_id:%03d][ID:%03d] Torque is enabled, cannot proceed. Set "
              "'disable_torque_at_init' parameter to 'true' to disable torque at initialization "
              "or disable torque manually.\n",
              it_comm, it_id);
            return DxlError::DXL_HARDWARE_ERROR;
          }
        }

        fprintf(
          stderr, "[InitTorqueStates][comm_id:%03d][ID:%03d] Current torque state: %s\n", it_comm,
          it_id,
          torque_state_[{it_comm, it_id}] ? "ON" : "OFF");
      }
    } catch (const std::exception & e) {
      fprintf(
        stderr, "[InitTorqueStates][comm_id:%03d][ID:%03d] Error checking control item: %s\n",
        it_comm, it_id,
        e.what());
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }
  }
  return DxlError::OK;
}

void Dynamixel::OverrideUnitInfo(
  uint8_t comm_id,
  uint8_t id,
  const std::string & data_name,
  double unit_multiplier,
  bool is_signed,
  double offset_value)
{
  // Ensure entry exists
  (void)dxl_info_.dxl_info_by_comm_[comm_id][id];
  dxl_info_.dxl_info_by_comm_[comm_id][id].unit_map[data_name] = unit_multiplier;
  dxl_info_.dxl_info_by_comm_[comm_id][id].sign_type_map[data_name] = is_signed;
  dxl_info_.dxl_info_by_comm_[comm_id][id].offset_map[data_name] = offset_value;
}

DxlError Dynamixel::SetupPort(const std::string & port_name, const std::string & baudrate)
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler();

  bool port_opened = false;
  for (int attempt = 0; attempt < MAX_COMM_RETRIES; ++attempt) {
    if (port_handler_->openPort()) {
      fprintf(
        stderr,
        "Succeeded to open the port [%s]! (attempt %d/%d)\n",
        port_name.c_str(),
        attempt + 1,
        MAX_COMM_RETRIES);
      port_opened = true;
      break;
    }

    fprintf(
      stderr,
      "Failed to open the port [%s]! (attempt %d/%d)\n",
      port_name.c_str(),
      attempt + 1,
      MAX_COMM_RETRIES);

    if (attempt + 1 == MAX_COMM_RETRIES) {
      return DxlError::OPEN_PORT_FAIL;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  if (!port_opened) {
    return DxlError::OPEN_PORT_FAIL;
  }

  if (port_handler_->setBaudRate(stoi(baudrate))) {
    fprintf(stderr, "Succeeded to change the baudrate [%s]!\n", baudrate.c_str());
  } else {
    fprintf(stderr, "Failed to change the baudrate [%s]!\n", baudrate.c_str());
    return DxlError::OPEN_PORT_FAIL;
  }

  read_data_list_.clear();
  write_data_list_.clear();
  return DxlError::OK;
}

DxlError Dynamixel::InitDxlComm(uint8_t comm_id, uint8_t id)
{
  uint16_t dxl_model_number;
  uint8_t dxl_error = 0;

  fprintf(stderr, "[comm_id:%03d][ID:%03d] Request ping\t", comm_id, id);
  int dxl_comm_result = packet_handler_->ping(
    port_handler_, comm_id, &dxl_model_number, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(stderr, " - COMM_ERROR : %s\n", packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  // First, read the model file to get the control table structure
  try {
    dxl_info_.ReadDxlModelFile(comm_id, id, dxl_model_number);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "[InitDxlComm][comm_id:%03d][ID:%03d] Error reading model file: %s\n", comm_id,
      id, e.what());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  if (dxl_error != 0) {
    fprintf(stderr, " - RX_PACKET_ERROR : %s\n", packet_handler_->getRxPacketError(dxl_error));

    // Check if Hardware Error Status control item exists
    uint16_t hw_error_addr, error_code_addr;
    uint8_t hw_error_size, error_code_size;
    bool hw_error_exists = dxl_info_.GetDxlControlItem(
      comm_id, id, "Hardware Error Status", hw_error_addr, hw_error_size);
    bool error_code_exists = dxl_info_.GetDxlControlItem(
      comm_id, id, "Error Code", error_code_addr, error_code_size);

    if (hw_error_exists) {
      uint32_t hw_error_status = 0;
      ReadItem(comm_id, id, "Hardware Error Status", hw_error_status);

      std::string error_string = "";
      uint8_t hw_error_byte = static_cast<uint8_t>(hw_error_status);

      for (int bit = 0; bit < 8; ++bit) {
        if (hw_error_byte & (1 << bit)) {
          const HardwareErrorStatusBitInfo * bit_info = get_hardware_error_status_bit_info(bit);
          if (bit_info) {
            error_string += bit_info->label;
            error_string += " (" + std::string(bit_info->description) + ")/ ";
          } else {
            error_string += "Unknown Error Bit " + std::to_string(bit) + "/ ";
          }
        }
      }

      if (!error_string.empty()) {
        fprintf(
          stderr, "[comm_id:%03d][ID:%03d] Hardware Error Details: 0x%x (%d): %s\n",
          comm_id, id, hw_error_byte, hw_error_byte, error_string.c_str());
      }
    } else if (error_code_exists) {
      uint32_t error_code = 0;
      ReadItem(comm_id, id, "Error Code", error_code);

      uint8_t error_code_byte = static_cast<uint8_t>(error_code);
      if (error_code_byte != 0x00) {
        const ErrorCodeInfo * error_info = get_error_code_info(error_code_byte);
        if (error_info) {
          fprintf(
            stderr, "[comm_id:%03d][ID:%03d] Error Code Details: 0x%x (%s): %s\n",
            comm_id, id, error_code_byte, error_info->label, error_info->description);
        } else {
          fprintf(
            stderr, "[comm_id:%03d][ID:%03d] Error Code Details: 0x%x (Unknown Error Code)\n",
            comm_id, id, error_code_byte);
        }
      }
    } else {
      fprintf(
        stderr,
        "[comm_id:%03d][ID:%03d] Neither Hardware Error Status"
        " nor Error Code control items available.\n",
        comm_id, id
      );
    }

    Reboot(id);
    return DxlError::DXL_HARDWARE_ERROR;
  } else {
    std::string model_name = dxl_info_.GetModelName(dxl_model_number);
    fprintf(
      stderr, " - Ping succeeded. Dynamixel model number : %d (%s)\n", dxl_model_number,
      model_name.c_str());
  }

  try {
    dxl_info_.ReadDxlModelFile(comm_id, id, dxl_model_number);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "[InitDxlComm][comm_id:%03d][ID:%03d] Error reading model file: %s\n", comm_id,
      id, e.what());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  uint8_t firmware_version = 0;
  DxlError fw_result = ReadFirmwareVersion(comm_id, id, firmware_version);
  if (fw_result == DxlError::OK && firmware_version > 0) {
    try {
      dxl_info_.ReadDxlModelFile(comm_id, id, dxl_model_number, firmware_version);
    } catch (const std::exception & e) {
      fprintf(
        stderr,
        "[InitDxlComm][comm_id:%03d][ID:%03d] Error reading firmware-specific model file: %s\n",
        comm_id, id, e.what());
    }
  }

  read_data_list_.clear();
  write_data_list_.clear();
  return DxlError::OK;
}

DxlError Dynamixel::Reboot(uint8_t id)
{
  fprintf(stderr, "[ID:%03d] Rebooting...\n", id);
  uint8_t dxl_error = 0;

  int dxl_comm_result = packet_handler_->reboot(port_handler_, id, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(
      stderr, "[ID:%03d] COMM_ERROR : %s\n",
      id, packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::DXL_REBOOT_FAIL;
  } else if (dxl_error != 0) {
    fprintf(
      stderr, "[ID:%03d] RX_PACKET_ERROR : %s\n",
      id, packet_handler_->getRxPacketError(dxl_error));
    return DxlError::DXL_REBOOT_FAIL;
  }

  fprintf(stderr, "[ID:%03d] Reboot Success!\n", id);

  return DxlError::OK;
}

void Dynamixel::RWDataReset()
{
  grouped_sync_writes_.clear();
  read_data_list_.clear();
  write_data_list_.clear();
}

DxlError Dynamixel::SetDxlReadItems(
  uint8_t comm_id,
  uint8_t id,
  std::vector<std::string> item_names,
  std::vector<std::shared_ptr<double>> data_vec_ptr)
{
  if (item_names.size() == 0) {
    fprintf(stderr, "[comm_id:%03d][ID:%03d] No (Sync or Bulk) Read Item\n", comm_id, id);
    return DxlError::OK;
  }

  if (item_names.size() != data_vec_ptr.size()) {
    fprintf(
      stderr, "Incorrect Read Data Size!!![%zu] [%zu]\n",
      item_names.size(), data_vec_ptr.size());
    return DxlError::SET_READ_ITEM_FAIL;
  }

  // Process items and get their addresses and sizes
  std::vector<uint8_t> item_ids;
  std::vector<uint16_t> item_addrs;
  std::vector<uint8_t> item_sizes;
  for (auto it_name : item_names) {
    uint16_t ITEM_ADDR;
    uint8_t ITEM_SIZE;
    if (dxl_info_.GetDxlControlItem(comm_id, id, it_name, ITEM_ADDR, ITEM_SIZE) == false) {
      fprintf(
        stderr, "[comm_id:%03d][ID:%03d] Cannot find control item in model file : %s\n", comm_id,
        id,
        it_name.c_str());
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }
    item_ids.push_back(id);
    item_addrs.push_back(ITEM_ADDR);
    item_sizes.push_back(ITEM_SIZE);
  }

  // Check if there's already an entry with this comm_id
  for (auto & existing_item : read_data_list_) {
    if (existing_item.comm_id == comm_id) {
      // Found existing entry, append new items
      existing_item.id_arr.insert(existing_item.id_arr.end(), item_ids.begin(), item_ids.end());
      existing_item.item_name.insert(
        existing_item.item_name.end(), item_names.begin(),
        item_names.end());
      existing_item.item_addr.insert(
        existing_item.item_addr.end(), item_addrs.begin(),
        item_addrs.end());
      existing_item.item_size.insert(
        existing_item.item_size.end(), item_sizes.begin(),
        item_sizes.end());
      existing_item.item_data_ptr_vec.insert(
        existing_item.item_data_ptr_vec.end(),
        data_vec_ptr.begin(), data_vec_ptr.end());
      return DxlError::OK;
    }
  }

  // No existing entry found, create new one
  RWItemList read_item;
  read_item.comm_id = comm_id;
  read_item.id_arr = std::move(item_ids);
  read_item.item_name = std::move(item_names);
  read_item.item_addr = std::move(item_addrs);
  read_item.item_size = std::move(item_sizes);
  read_item.item_data_ptr_vec = std::move(data_vec_ptr);

  read_data_list_.push_back(read_item);

  return DxlError::OK;
}

DxlError Dynamixel::SetMultiDxlRead()
{
  read_type_ = checkReadType();

  fprintf(stderr, "Dynamixel Read Type : %s\n", read_type_ ? "bulk read" : "sync read");
  if (read_type_ == SYNC) {
    fprintf(stderr, "ID : ");
    for (auto it_read_data_list : read_data_list_) {
      fprintf(stderr, "%d, ", it_read_data_list.comm_id);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "Read items : ");
    if (!read_data_list_.empty()) {
      for (auto it_read_data_list_item_name : read_data_list_.at(0).item_name) {
        fprintf(stderr, "\t%s", it_read_data_list_item_name.c_str());
      }
    } else {
      fprintf(stderr, "(none)");
    }
    fprintf(stderr, "\n");
  } else {
    for (auto it_read_data_list : read_data_list_) {
      fprintf(stderr, "ID : %d", it_read_data_list.comm_id);
      fprintf(stderr, "\tRead items : ");
      for (auto it_read_data_list_item_name : it_read_data_list.item_name) {
        fprintf(stderr, "\t%s", it_read_data_list_item_name.c_str());
      }
      fprintf(stderr, "\n");
    }
  }

  if (read_type_ == SYNC) {
    return SetSyncReadItemAndHandler();
  } else {
    return SetBulkReadItemAndHandler();
  }
}

DxlError Dynamixel::SetDxlWriteItems(
  uint8_t comm_id,
  uint8_t id,
  std::vector<std::string> item_names,
  std::vector<std::shared_ptr<double>> data_vec_ptr)
{
  if (item_names.size() == 0) {
    fprintf(stderr, "[comm_id:%03d][ID:%03d] No (Sync or Bulk) Write Item\n", comm_id, id);
    return DxlError::OK;
  }

  if (item_names.size() != data_vec_ptr.size()) {
    fprintf(
      stderr, "Incorrect Write Data Size!!![%zu] [%zu]\n",
      item_names.size(), data_vec_ptr.size());
    return DxlError::SET_WRITE_ITEM_FAIL;
  }

  std::vector<uint8_t> item_ids;
  std::vector<uint16_t> item_addrs;
  std::vector<uint8_t> item_sizes;
  for (auto it_name : item_names) {
    uint16_t ITEM_ADDR;
    uint8_t ITEM_SIZE;
    if (dxl_info_.GetDxlControlItem(comm_id, id, it_name, ITEM_ADDR, ITEM_SIZE) == false) {
      fprintf(
        stderr, "[comm_id:%03d][ID:%03d] Cannot find control item in model file : %s\n", comm_id,
        id,
        it_name.c_str());
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }
    item_ids.push_back(id);
    item_addrs.push_back(ITEM_ADDR);
    item_sizes.push_back(ITEM_SIZE);
  }

  for (auto & existing_item : write_data_list_) {
    if (existing_item.comm_id == comm_id) {
      existing_item.id_arr.insert(existing_item.id_arr.end(), item_ids.begin(), item_ids.end());
      existing_item.item_name.insert(
        existing_item.item_name.end(), item_names.begin(),
        item_names.end());
      existing_item.item_addr.insert(
        existing_item.item_addr.end(), item_addrs.begin(),
        item_addrs.end());
      existing_item.item_size.insert(
        existing_item.item_size.end(), item_sizes.begin(),
        item_sizes.end());
      existing_item.item_data_ptr_vec.insert(
        existing_item.item_data_ptr_vec.end(),
        data_vec_ptr.begin(), data_vec_ptr.end());
      return DxlError::OK;
    }
  }

  // No existing entry found, create new one
  RWItemList write_item;
  write_item.comm_id = comm_id;
  write_item.id_arr = std::move(item_ids);
  write_item.item_name = std::move(item_names);
  write_item.item_addr = std::move(item_addrs);
  write_item.item_size = std::move(item_sizes);
  write_item.item_data_ptr_vec = std::move(data_vec_ptr);

  write_data_list_.push_back(write_item);

  return DxlError::OK;
}

DxlError Dynamixel::SetMultiDxlWrite()
{
  write_type_ = checkWriteType();

  fprintf(stderr, "Dynamixel Write Type : %s\n", write_type_ ? "bulk write" : "sync write");
  if (write_type_ == SYNC) {
    fprintf(stderr, "ID : ");
    for (auto it_id : write_data_list_) {
      fprintf(stderr, "%d, ", it_id.comm_id);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "Write items : ");
    if (!write_data_list_.empty()) {
      for (auto it_name : write_data_list_.at(0).item_name) {
        fprintf(stderr, "\t%s", it_name.c_str());
      }
    } else {
      fprintf(stderr, "(none)");
    }
    fprintf(stderr, "\n");
  } else {
    for (auto it_id : write_data_list_) {
      fprintf(stderr, "ID : %d", it_id.comm_id);
      fprintf(stderr, "\tWrite items : ");
      for (auto it_name : it_id.item_name) {
        fprintf(stderr, "\t%s", it_name.c_str());
      }
      fprintf(stderr, "\n");
    }
  }

  if (write_type_ == SYNC) {
    return SetSyncWriteItemAndHandler();
  } else {
    const auto result = SetBulkWriteItemAndHandler();
    if (result != DxlError::OK || !use_grouped_sync_write_) {return result;}
    return SetupGroupedSyncWrite();
  }
}

DxlError Dynamixel::DynamixelEnable(const std::vector<std::pair<uint8_t, uint8_t>> & comm_id_id_arr)
{
  for (auto p : comm_id_id_arr) {
    uint8_t comm_id = p.first;
    uint8_t id = p.second;
    if (dxl_info_.CheckDxlControlItem(comm_id, id, "Torque Enable") == false) {
      continue;
    }
    if (torque_state_[{comm_id, id}] == TORQUE_OFF) {
      if (WriteItem(comm_id, id, "Torque Enable", TORQUE_ON) < 0) {
        fprintf(
          stderr, "[comm_id:%03d][ID:%03d] Cannot write \"Torque On\" command!\n", comm_id,
          id);
        return DxlError::ITEM_WRITE_FAIL;
      }
      torque_state_[{comm_id, id}] = TORQUE_ON;
      fprintf(stderr, "[comm_id:%03d][ID:%03d] Torque ON\n", comm_id, id);
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::DynamixelDisable(
  const std::vector<std::pair<uint8_t,
  uint8_t>> & comm_id_id_arr)
{
  DxlError result = DxlError::OK;
  for (auto p : comm_id_id_arr) {
    uint8_t comm_id = p.first;
    uint8_t id = p.second;
    if (dxl_info_.CheckDxlControlItem(comm_id, id, "Torque Enable") == false) {
      continue;
    }
    if (torque_state_[{comm_id, id}] == TORQUE_ON) {
      if (WriteItem(comm_id, id, "Torque Enable", TORQUE_OFF) < 0) {
        fprintf(
          stderr, "[comm_id:%03d][ID:%03d] Cannot write \"Torque Off\" command!\n", comm_id,
          id);
        result = DxlError::ITEM_WRITE_FAIL;
      } else {
        torque_state_[{comm_id, id}] = TORQUE_OFF;
        fprintf(stderr, "[comm_id:%03d][ID:%03d] Torque OFF\n", comm_id, id);
      }
    }
  }
  return result;
}

// DxlError Dynamixel::SetOperatingMode(uint8_t dxl_id, uint8_t dynamixel_mode)
// {
//   if (WriteItem(dxl_id, "Operating Mode", dynamixel_mode) == false) {
//     return DxlError::ITEM_WRITE_FAIL;
//   }

//   fprintf(stderr, "[ID:%03d] Succeeded to set operating mode(", dxl_id);
//   if (dynamixel_mode == DXL_POSITION_CTRL_MODE) {
//     fprintf(stderr, "Position Control Mode)\n");
//   } else if (dynamixel_mode == DXL_CURRENT_CTRL_MODE) {
//     fprintf(stderr, "Current Control Mode)\n");
//   } else if (dynamixel_mode == DXL_VELOCITY_CTRL_MODE) {
//     fprintf(stderr, "Velocity Control Mode)\n");
//   } else {
//     fprintf(stderr, "Not Defined Control Mode)\n");
//   }

//   return DxlError::OK;
// }

DxlError Dynamixel::WriteItem(uint8_t comm_id, uint8_t id, std::string item_name, uint32_t data)
{
  uint16_t ITEM_ADDR;
  uint8_t ITEM_SIZE;
  if (dxl_info_.GetDxlControlItem(comm_id, id, item_name, ITEM_ADDR, ITEM_SIZE) == false) {
    fprintf(
      stderr, "[WriteItem][comm_id:%03d][ID:%03d] Cannot find control item in model file. : %s\n",
      comm_id, id,
      item_name.c_str());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
  return WriteItem(comm_id, id, ITEM_ADDR, ITEM_SIZE, data);
}

DxlError Dynamixel::WriteItem(
  uint8_t comm_id, uint8_t id, uint16_t addr, uint8_t size,
  uint32_t data)
{
  for (int i = 0; i < MAX_COMM_RETRIES; i++) {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    if (size == 1) {
      dxl_comm_result =
        packet_handler_->write1ByteTxRx(
        port_handler_, comm_id, addr, static_cast<uint8_t>(data),
        &dxl_error);
    } else if (size == 2) {
      dxl_comm_result = packet_handler_->write2ByteTxRx(
        port_handler_, comm_id, addr,
        static_cast<uint16_t>(data), &dxl_error);
    } else if (size == 4) {
      dxl_comm_result = packet_handler_->write4ByteTxRx(
        port_handler_, comm_id, addr,
        static_cast<uint32_t>(data), &dxl_error);
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr,
        "[WriteItem][ID:%03d][comm_id:%03d] COMM_ERROR : %s (retry %d/%d)\n",
        id,
        comm_id,
        packet_handler_->getTxRxResult(dxl_comm_result),
        i + 1,
        MAX_COMM_RETRIES);
      if (i == MAX_COMM_RETRIES - 1) {
        return DxlError::ITEM_WRITE_FAIL;
      }
    // Protocol 2 bit 7 is a hardware ALERT, not an instruction rejection.
    // Keep the device error visible in cyclic readback; allow OFF acknowledgements.
    } else if ((dxl_error & 0x7f) != 0) {
      fprintf(
        stderr,
        "[WriteItem][ID:%03d][comm_id:%03d] RX_PACKET_ERROR : %s\n",
        id,
        comm_id,
        packet_handler_->getRxPacketError(dxl_error));
      return DxlError::ITEM_WRITE_FAIL;
    } else {
      return DxlError::OK;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  fprintf(stderr, "MAX_COMM_RETRIES should be set to 1 or more\n");
  return DxlError::ITEM_WRITE_FAIL;
}

DxlError Dynamixel::InsertWriteItemBuf(uint8_t id, std::string item_name, uint32_t data)
{
  RWItemBufInfo item;

  item.comm_id = id;
  item.id = id;
  item.control_item.item_name = item_name;
  item.data = data;

  if (dxl_info_.GetDxlControlItem(
      id, id, item_name, item.control_item.address,
      item.control_item.size) == false)
  {
    fprintf(stderr, "Cannot find control item in model file.\n");
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  write_item_buf_.push_back(item);
  return DxlError::OK;
}

DxlError Dynamixel::WriteItemBuf()
{
  // Attempt each queued operation once. The supervisor confirms actual drive
  // state and owns OFF retries; replaying an old failed ON is not permitted.
  auto pending = std::move(write_item_buf_);
  write_item_buf_.clear();
  DxlError result = DxlError::OK;
  for (auto it_write_item : pending) {
    // set torque state variable (ON or OFF)
    if (strcmp(it_write_item.control_item.item_name.c_str(), "Torque Enable") == 0) {
      if (WriteItem(
          it_write_item.comm_id, it_write_item.id, it_write_item.control_item.address,
          it_write_item.control_item.size, it_write_item.data) < 0)
      {
        result = DxlError::ITEM_WRITE_FAIL;
        continue;
      }

      torque_state_[{it_write_item.comm_id, it_write_item.id}] = it_write_item.data;
      fprintf(
        stderr, "[comm_id:%03d][ID:%03d] Set Torque %s\n", it_write_item.comm_id, it_write_item.id,
        torque_state_[{it_write_item.comm_id, it_write_item.id}] ? "ON" : "OFF");
    } else {
      if ((dxl_info_.CheckDxlControlItem(
          it_write_item.comm_id, it_write_item.id,
          "Torque Enable")) &&
        torque_state_[{it_write_item.comm_id, it_write_item.id}] == TORQUE_ON)
      {
        if (WriteItem(it_write_item.comm_id, it_write_item.id, "Torque Enable", TORQUE_OFF) < 0) {
          fprintf(
            stderr,
            "[comm_id:%03d][ID:%03d] Cannot write \"Torque Off\" command! Cannot write a Item.\n",
            it_write_item.comm_id, it_write_item.id);
          result = DxlError::ITEM_WRITE_FAIL;
        continue;
        }
        torque_state_[{it_write_item.comm_id, it_write_item.id}] = TORQUE_OFF;
      }

      if (WriteItem(
          it_write_item.comm_id, it_write_item.id, it_write_item.control_item.address,
          it_write_item.control_item.size, it_write_item.data) < 0)
      {
        result = DxlError::ITEM_WRITE_FAIL;
        continue;
      }
    }
  }
  return result;
}

DxlError Dynamixel::ReadItem(uint8_t comm_id, uint8_t id, std::string item_name, uint32_t & data)
{
  uint16_t ITEM_ADDR;
  uint8_t ITEM_SIZE;
  if (dxl_info_.GetDxlControlItem(comm_id, id, item_name, ITEM_ADDR, ITEM_SIZE) == false) {
    fprintf(
      stderr, "[ReadItem][comm_id:%03d][ID:%03d] Cannot find control item in model file. : %s\n",
      comm_id, id,
      item_name.c_str());
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  for (int i = 0; i < MAX_COMM_RETRIES; i++) {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    if (ITEM_SIZE == 1) {
      uint8_t read_data;
      dxl_comm_result = packet_handler_->read1ByteTxRx(
        port_handler_, comm_id, ITEM_ADDR,
        &read_data, &dxl_error);
      data = read_data;
    } else if (ITEM_SIZE == 2) {
      uint16_t read_data;
      dxl_comm_result = packet_handler_->read2ByteTxRx(
        port_handler_, comm_id, ITEM_ADDR,
        &read_data, &dxl_error);
      data = read_data;
    } else if (ITEM_SIZE == 4) {
      uint32_t read_data;
      dxl_comm_result = packet_handler_->read4ByteTxRx(
        port_handler_, comm_id, ITEM_ADDR,
        &read_data, &dxl_error);
      data = read_data;
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr,
        "[ReadItem][ID:%03d][comm_id:%03d] COMM_ERROR : %s (retry %d/%d)\n",
        id,
        comm_id,
        packet_handler_->getTxRxResult(dxl_comm_result),
        i + 1,
        MAX_COMM_RETRIES);
      if (i == MAX_COMM_RETRIES - 1) {
        return DxlError::ITEM_READ_FAIL;
      }
    } else if (dxl_error != 0) {
      bool is_alert = dxl_error == 0x80;
      if (is_alert) {
        fprintf(
          stderr,
          "[ReadItem][ID:%03d][comm_id:%03d] RX_PACKET_ERROR : %s\n",
          id,
          comm_id,
          packet_handler_->getRxPacketError(dxl_error));
        return DxlError::OK;
      }
      fprintf(
        stderr,
        "[ReadItem][ID:%03d][comm_id:%03d] RX_PACKET_ERROR : %s (retry %d/%d)\n",
        id,
        comm_id,
        packet_handler_->getRxPacketError(dxl_error),
        i + 1,
        MAX_COMM_RETRIES);
      if (i == MAX_COMM_RETRIES - 1) {
        return DxlError::ITEM_READ_FAIL;
      }
    } else {
      return DxlError::OK;
    }
  }
  fprintf(stderr, "MAX_COMM_RETRIES should be set to 1 or more\n");
  return DxlError::ITEM_READ_FAIL;
}


DxlError Dynamixel::InsertReadItemBuf(uint8_t id, std::string item_name)
{
  RWItemBufInfo item;

  uint8_t comm_id = id;
  item.comm_id = comm_id;
  item.id = id;
  item.control_item.item_name = item_name;
  item.read_flag = false;

  if (dxl_info_.GetDxlControlItem(
      comm_id, id, item_name, item.control_item.address,
      item.control_item.size) == false)
  {
    fprintf(stderr, "Cannot find control item in model file.\n");
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }

  read_item_buf_.push_back(item);
  return DxlError::OK;
}

DxlError Dynamixel::ReadItemBuf()
{
  for (auto it_read_item = read_item_buf_.begin(); it_read_item < read_item_buf_.end();
    it_read_item++)
  {
    if (it_read_item->read_flag == false) {
      uint8_t id = it_read_item->id;
      uint16_t addr = it_read_item->control_item.address;
      uint16_t size = it_read_item->control_item.size;

      int dxl_comm_result = COMM_TX_FAIL;
      uint8_t dxl_error = 0;

      if (size == 1) {
        uint8_t read_data;
        dxl_comm_result = packet_handler_->read1ByteTxRx(
          port_handler_, id, addr,
          &read_data, &dxl_error);
        it_read_item->data = read_data;
      } else if (size == 2) {
        uint16_t read_data;
        dxl_comm_result = packet_handler_->read2ByteTxRx(
          port_handler_, id, addr,
          &read_data, &dxl_error);
        it_read_item->data = read_data;
      } else if (size == 4) {
        uint32_t read_data;
        dxl_comm_result = packet_handler_->read4ByteTxRx(
          port_handler_, id, addr,
          &read_data, &dxl_error);
        it_read_item->data = read_data;
      }

      if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(
          stderr, "[ID:%03d] COMM_ERROR : %s\n",
          id,
          packet_handler_->getTxRxResult(dxl_comm_result));
        return DxlError::ITEM_READ_FAIL;
      } else if (dxl_error != 0) {
        fprintf(
          stderr, "[ID:%03d] RX_PACKET_ERROR : %s\n",
          id,
          packet_handler_->getRxPacketError(dxl_error));
        return DxlError::ITEM_READ_FAIL;
      } else {
        it_read_item->read_flag = true;
      }
    }
  }
  return DxlError::OK;
}

bool Dynamixel::CheckReadItemBuf(uint8_t id, std::string item_name)
{
  for (auto it_read_item_buf : read_item_buf_) {
    if (it_read_item_buf.id == id && it_read_item_buf.control_item.item_name == item_name) {
      return it_read_item_buf.read_flag;
    }
  }
  return false;
}
uint32_t Dynamixel::GetReadItemDataBuf(uint8_t id, std::string item_name)
{
  for (size_t i = 0; i < read_item_buf_.size(); i++) {
    if (read_item_buf_.at(i).id == id && read_item_buf_.at(i).control_item.item_name == item_name) {
      uint32_t res = read_item_buf_.at(i).data;
      read_item_buf_.erase(read_item_buf_.begin() + static_cast<int64_t>(i));
      return res;
    }
  }
  return 0;
}

std::string Dynamixel::DxlErrorToString(DxlError error_num)
{
  switch (error_num) {
    case OK:
      return "OK";
    case CANNOT_FIND_CONTROL_ITEM:
      return "CANNOT_FIND_CONTROL_ITEM";
    case OPEN_PORT_FAIL:
      return "OPEN_PORT_FAIL";
    case INDIRECT_ADDR_FAIL:
      return "INDIRECT_ADDR_FAIL";
    case ITEM_WRITE_FAIL:
      return "ITEM_WRITE_FAIL";
    case ITEM_READ_FAIL:
      return "ITEM_READ_FAIL";
    case SYNC_WRITE_FAIL:
      return "SYNC_WRITE_FAIL";
    case SYNC_READ_FAIL:
      return "SYNC_READ_FAIL";
    case SET_SYNC_WRITE_FAIL:
      return "SET_SYNC_WRITE_FAIL";
    case SET_SYNC_READ_FAIL:
      return "SET_SYNC_READ_FAIL";
    case BULK_WRITE_FAIL:
      return "BULK_WRITE_FAIL";
    case BULK_READ_FAIL:
      return "BULK_READ_FAIL";
    case SET_BULK_WRITE_FAIL:
      return "SET_BULK_WRITE_FAIL";
    case SET_BULK_READ_FAIL:
      return "SET_BULK_READ_FAIL";
    case SET_READ_ITEM_FAIL:
      return "SET_READ_ITEM_FAIL";
    case SET_WRITE_ITEM_FAIL:
      return "SET_WRITE_ITEM_FAIL";
    case DXL_HARDWARE_ERROR:
      return "DXL_HARDWARE_ERROR";
    case DXL_REBOOT_FAIL:
      return "DXL_REBOOT_FAIL";
    default:
      return "UNKNOWN_ERROR";
  }
}

DxlError Dynamixel::ReadMultiDxlData(double period_ms)
{
  if (read_data_list_.empty()) {
    return DxlError::OK;
  }

  if (read_type_ == SYNC) {
    return GetDxlValueFromSyncRead(period_ms);
  } else {
    return GetDxlValueFromBulkRead(period_ms);
  }
}

DxlError Dynamixel::WriteMultiDxlData()
{
  if (write_data_list_.empty()) {
    return DxlError::OK;
  }

  if (write_type_ == SYNC) {
    return SetDxlValueToSyncWrite();
  } else {
    if (use_grouped_sync_write_) {return SetDxlValueToGroupedSyncWrite();}
    return SetDxlValueToBulkWrite();
  }
}

bool Dynamixel::checkReadType()
{
  if (read_data_list_.size() == 1) {
    if (CheckIndirectReadAvailable(read_data_list_.at(0).comm_id) != DxlError::OK) {
      return BULK;
    }
  }

  for (size_t dxl_index = 1; dxl_index < read_data_list_.size(); dxl_index++) {
    // Check if Indirect Data Read address and size are different
    uint16_t indirect_addr[2];  // [i-1], [i]
    uint8_t indirect_size[2];   // [i-1], [i]

    if (CheckIndirectReadAvailable(read_data_list_.at(dxl_index - 1).comm_id) != DxlError::OK) {
      return BULK;
    }

    if (!dxl_info_.GetDxlControlItem(
        read_data_list_.at(dxl_index).comm_id, read_data_list_.at(dxl_index).comm_id,
        "Indirect Data Read", indirect_addr[1], indirect_size[1]) ||
      !dxl_info_.GetDxlControlItem(
        read_data_list_.at(dxl_index - 1).comm_id, read_data_list_.at(dxl_index - 1).comm_id,
        "Indirect Data Read", indirect_addr[0], indirect_size[0]))
    {
      return BULK;
    }
    if (indirect_addr[1] != indirect_addr[0] || indirect_size[1] != indirect_size[0]) {
      return BULK;
    }

    if (read_data_list_.at(dxl_index).item_size.size() !=
      read_data_list_.at(dxl_index - 1).item_size.size())
    {
      return BULK;
    }
    for (size_t item_index = 0; item_index < read_data_list_.at(dxl_index).item_size.size();
      item_index++)
    {
      if (read_data_list_.at(dxl_index).item_size.at(item_index) !=
        read_data_list_.at(dxl_index - 1).item_size.at(item_index))
      {
        return BULK;
      }
    }
  }
  return SYNC;
}

bool Dynamixel::checkWriteType()
{
  if (write_data_list_.size() == 1) {
    if (CheckIndirectWriteAvailable(write_data_list_.at(0).comm_id) != DxlError::OK) {
      return BULK;
    }
  }

  for (size_t dxl_index = 1; dxl_index < write_data_list_.size(); dxl_index++) {
    // Check if Indirect Data Write address and size are different
    uint16_t indirect_addr[2];  // [i-1], [i]
    uint8_t indirect_size[2];   // [i-1], [i]

    if (CheckIndirectWriteAvailable(write_data_list_.at(dxl_index - 1).comm_id) != DxlError::OK) {
      return BULK;
    }

    if (!dxl_info_.GetDxlControlItem(
        write_data_list_.at(dxl_index).comm_id, write_data_list_.at(dxl_index).comm_id,
        "Indirect Data Write", indirect_addr[1], indirect_size[1]) ||
      !dxl_info_.GetDxlControlItem(
        write_data_list_.at(dxl_index - 1).comm_id, write_data_list_.at(dxl_index - 1).comm_id,
        "Indirect Data Write", indirect_addr[0], indirect_size[0]))
    {
      return BULK;
    }
    if (indirect_addr[1] != indirect_addr[0] || indirect_size[1] != indirect_size[0]) {
      return BULK;
    }

    if (write_data_list_.at(dxl_index).item_size.size() !=
      write_data_list_.at(dxl_index - 1).item_size.size())
    {
      return BULK;
    }
    for (size_t item_index = 0; item_index < write_data_list_.at(dxl_index).item_size.size();
      item_index++)
    {
      if (write_data_list_.at(dxl_index).item_size.at(item_index) !=
        write_data_list_.at(dxl_index - 1).item_size.at(item_index))
      {
        return BULK;
      }
    }
  }
  return SYNC;
}

DxlError Dynamixel::CheckIndirectWriteAvailable(uint8_t id)
{
  uint16_t INDIRECT_ADDR;
  uint8_t INDIRECT_SIZE;
  if (dxl_info_.GetDxlControlItem(
      id, id, "Indirect Address Write",
      INDIRECT_ADDR, INDIRECT_SIZE) == false)
  {
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetSyncReadItemAndHandler()
{
  std::vector<uint8_t> id_arr;
  for (auto it_read_data : read_data_list_) {
    id_arr.push_back(it_read_data.comm_id);
  }

  {
    std::vector<std::pair<uint8_t, uint8_t>> pairs;
    for (auto it_read_data : read_data_list_) {
      pairs.emplace_back(it_read_data.comm_id, it_read_data.comm_id);
    }
    DynamixelDisable(pairs);
  }
  ResetIndirectRead(id_arr);

  for (auto it_read_data : read_data_list_) {
    for (size_t item_index = 0; item_index < it_read_data.item_name.size(); item_index++) {
      auto result = AddIndirectRead(
        it_read_data.comm_id,
        it_read_data.item_name.at(item_index),
        it_read_data.item_addr.at(item_index),
        it_read_data.item_size.at(item_index));

      if (result == DxlError::OK) {
      } else {
        fprintf(
          stderr,
          "[ID:%03d] Failed to Set Indirect Address Read Item: [%s], Addr: %d, Size: %d, "
          "Error code: %d\n",
          it_read_data.comm_id,
          it_read_data.item_name.at(item_index).c_str(),
          it_read_data.item_addr.at(item_index),
          it_read_data.item_size.at(item_index),
          result);
        return DxlError::SET_SYNC_READ_FAIL;
      }
    }
  }

  if (SetSyncReadHandler(id_arr) != DxlError::OK) {
    fprintf(stderr, "Cannot set the SyncRead handler.\n");
    return DxlError::SYNC_READ_FAIL;
  }

  fprintf(stderr, "Success to set SyncRead handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetFastSyncReadHandler(std::vector<uint8_t> id_arr)
{
  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    if (dxl_info_.GetDxlControlItem(
        it_id, it_id, "Indirect Data Read", IN_ADDR,
        IN_SIZE) == false)
    {
      fprintf(
        stderr,
        "Fail to set indirect address fast sync read. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_SYNC_READ_FAIL;
    }
    indirect_info_read_[it_id].indirect_data_addr = IN_ADDR;
  }
  fprintf(
    stderr,
    "set fast sync read (indirect addr) : addr %d, size %d\n",
    IN_ADDR, indirect_info_read_[id_arr.at(0)].size);

  if (group_fast_sync_read_) {
    delete group_fast_sync_read_;
    group_fast_sync_read_ = nullptr;
  }
  group_fast_sync_read_ =
    new dynamixel::GroupFastSyncRead(
    port_handler_, packet_handler_,
    IN_ADDR, indirect_info_read_[id_arr.at(0)].size);

  for (auto it_id : id_arr) {
    if (group_fast_sync_read_->addParam(it_id) != true) {
      fprintf(stderr, "[ID:%03d] groupFastSyncRead addparam failed", it_id);
      return DxlError::SET_SYNC_READ_FAIL;
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetSyncReadHandler(std::vector<uint8_t> id_arr)
{
  if (id_arr.size() == 0) {
    fprintf(stderr, "No Sync Read Item, not setting sync read handler\n");
    return DxlError::OK;
  }

  // Try to set up fast sync read first
  if (use_fast_read_protocol_) {
    DxlError fast_result = SetFastSyncReadHandler(id_arr);
    if (fast_result == DxlError::OK) {
      fprintf(stderr, "FastSyncRead handler set up successfully.\n");
      return DxlError::OK;
    } else {
      fprintf(stderr, "FastSyncRead handler failed, falling back to normal SyncRead.\n");
      use_fast_read_protocol_ = false;
    }
  }
  // Fallback to normal sync read
  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(
        it_id, it_id, "Indirect Data Read", IN_ADDR,
        IN_SIZE) == false)
    {
      fprintf(
        stderr,
        "Fail to set indirect address sync read. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_SYNC_READ_FAIL;
    }
    // Set indirect addr.
    indirect_info_read_[it_id].indirect_data_addr = IN_ADDR;
  }
  fprintf(
    stderr,
    "set sync read (indirect addr) : addr %d, size %d\n",
    IN_ADDR, indirect_info_read_[id_arr.at(0)].size);

  if (group_sync_read_) {
    delete group_sync_read_;
    group_sync_read_ = nullptr;
  }
  group_sync_read_ =
    new dynamixel::GroupSyncRead(
    port_handler_, packet_handler_,
    IN_ADDR, indirect_info_read_[id_arr.at(0)].size);

  for (auto it_id : id_arr) {
    if (group_sync_read_->addParam(it_id) != true) {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", it_id);
      return DxlError::SET_SYNC_READ_FAIL;
    }
  }

  return DxlError::OK;
}

DxlError Dynamixel::GetDxlValueFromSyncRead(double period_ms)
{
  // Try fast sync read for the first 10 attempts after startup/handler setup.
  // If any of the first 10 attempts succeeds, use fast sync read permanently.
  // If all 10 attempts fail, permanently fallback to normal sync read.
  if (use_fast_read_protocol_ && group_fast_sync_read_ &&
    (fast_read_permanent_ || fast_read_fail_count_ < 10))
  {
    DxlError comm_result = ProcessReadCommunication(port_handler_, period_ms, true, true);
    if (comm_result == DxlError::OK) {
      // Success, process data, and use fast sync read permanently
      for (auto it_read_data : read_data_list_) {
        uint8_t id = it_read_data.comm_id;
        uint16_t indirect_addr = indirect_info_read_[id].indirect_data_addr;
        ProcessReadData(
          id,
          indirect_addr,
          it_read_data.id_arr,
          indirect_info_read_[id].item_name,
          indirect_info_read_[id].item_size,
          it_read_data.item_data_ptr_vec,
          [this](uint8_t lambda_id, uint16_t addr, uint8_t size) {
            return group_fast_sync_read_->getData(lambda_id, addr, size);
          });
      }
      // Mark as permanently using fast sync read after first success
      fast_read_permanent_ = true;
      return DxlError::OK;
    } else if (!fast_read_permanent_) {
      // Only increment fail count and fallback if not yet permanent
      ++fast_read_fail_count_;
      fprintf(stderr, "FastSyncRead TxRx failed (attempt %d/10)\n", fast_read_fail_count_);
      if (fast_read_fail_count_ >= 10) {
        // Permanently switch to normal sync read
        fprintf(
          stderr, "FastSyncRead failed 10 times, switching to normal SyncRead "
          "permanently.\n");
        use_fast_read_protocol_ = false;
        // Set up normal sync read handler
        std::vector<uint8_t> id_arr;
        for (auto it_read_data : read_data_list_) {
          id_arr.push_back(it_read_data.comm_id);
        }
        SetSyncReadHandler(id_arr);
      }
      // Return error for this attempt
      return DxlError::SYNC_READ_FAIL;
    } else {
      // If permanent, ignore failures and keep using fast sync read
      return comm_result;
    }
  }
  // Use normal sync read
  DxlError comm_result = ProcessReadCommunication(port_handler_, period_ms, true, false);
  if (comm_result != DxlError::OK) {
    return comm_result;
  }
  for (auto it_read_data : read_data_list_) {
    uint8_t id = it_read_data.comm_id;
    uint16_t indirect_addr = indirect_info_read_[id].indirect_data_addr;
    ProcessReadData(
      id,
      indirect_addr,
      it_read_data.id_arr,
      indirect_info_read_[id].item_name,
      indirect_info_read_[id].item_size,
      it_read_data.item_data_ptr_vec,
      [this](uint8_t lambda_id, uint16_t addr, uint8_t size) {
        return group_sync_read_->getData(lambda_id, addr, size);
      });
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetBulkReadItemAndHandler()
{
  if (group_fast_bulk_read_) {
    delete group_fast_bulk_read_;
    group_fast_bulk_read_ = nullptr;
  }
  group_fast_bulk_read_ = new dynamixel::GroupFastBulkRead(port_handler_, packet_handler_);

  if (group_bulk_read_) {
    delete group_bulk_read_;
    group_bulk_read_ = nullptr;
  }
  group_bulk_read_ = new dynamixel::GroupBulkRead(port_handler_, packet_handler_);

  std::vector<uint8_t> indirect_id_arr;
  std::vector<uint8_t> direct_id_arr;

  std::vector<RWItemList> indirect_read_data_list;
  std::vector<RWItemList> direct_read_data_list;

  for (auto it_read_data : read_data_list_) {
    if (CheckIndirectReadAvailable(it_read_data.comm_id) == DxlError::OK) {
      indirect_id_arr.push_back(it_read_data.comm_id);
      indirect_read_data_list.push_back(it_read_data);
    } else {
      direct_id_arr.push_back(it_read_data.comm_id);
      direct_read_data_list.push_back(it_read_data);
    }
  }

  for (auto it_read_data : direct_read_data_list) {
    if (it_read_data.item_addr.empty() || it_read_data.item_size.empty()) {
      continue;
    }
    // Calculate min address and max end address
    uint16_t min_addr = it_read_data.item_addr[0];
    uint16_t max_end_addr = it_read_data.item_addr[0] + it_read_data.item_size[0];
    for (size_t item_index = 1; item_index < it_read_data.item_addr.size(); ++item_index) {
      uint16_t addr = it_read_data.item_addr[item_index];
      uint16_t size = it_read_data.item_size[item_index];
      if (addr < min_addr) {min_addr = addr;}
      if (addr + size > max_end_addr) {max_end_addr = addr + size;}
    }
    uint8_t total_size = static_cast<uint8_t>(max_end_addr - min_addr);
    // Concatenate all item names with '+'
    std::string group_item_names;
    for (size_t i = 0; i < it_read_data.item_name.size(); ++i) {
      group_item_names += it_read_data.item_name[i];
      if (i + 1 < it_read_data.item_name.size()) {group_item_names += " + ";}
    }
    // Call AddDirectRead once per id
    if (AddDirectRead(
        it_read_data.comm_id,
        group_item_names,
        min_addr,
        total_size) != DxlError::OK)
    {
      fprintf(stderr, "Cannot set the BulkRead handler.\n");
      return DxlError::BULK_READ_FAIL;
    }
    fprintf(
      stderr,
      "set bulk read (direct addr) : addr %d, size %d\n",
      min_addr, total_size);
  }

  {
    std::vector<std::pair<uint8_t, uint8_t>> pairs;
    for (auto cid : indirect_id_arr) {
      pairs.emplace_back(cid, cid);
    }
    DynamixelDisable(pairs);
  }
  ResetIndirectRead(indirect_id_arr);

  for (auto it_read_data : indirect_read_data_list) {
    for (size_t item_index = 0; item_index < it_read_data.item_name.size();
      item_index++)
    {
      auto result = AddIndirectRead(
        it_read_data.comm_id,
        it_read_data.item_name.at(item_index),
        it_read_data.item_addr.at(item_index),
        it_read_data.item_size.at(item_index));

      if (result == DxlError::OK) {
        fprintf(
          stderr, "[ID:%03d] Add Indirect Address Read Item : [%s]\n",
          it_read_data.comm_id,
          it_read_data.item_name.at(item_index).c_str());
      } else if (result == DxlError::INDIRECT_ADDR_FAIL) {
        fprintf(
          stderr,
          "[ID:%03d] Failed to Set Indirect Address Read Item : [%s], Addr: %d, Size: %d, "
          "Error code: %d\n",
          it_read_data.comm_id,
          it_read_data.item_name.at(item_index).c_str(),
          it_read_data.item_addr.at(item_index),
          it_read_data.item_size.at(item_index),
          result);
        return DxlError::INDIRECT_ADDR_FAIL;
      } else if (result == DxlError::CANNOT_FIND_CONTROL_ITEM) {
        fprintf(
          stderr, "[ID:%03d] 'Indirect Address Read' is not defined in control table, "
          "Cannot set Indirect Address Read for : [%s]\n",
          it_read_data.comm_id,
          it_read_data.item_name.at(item_index).c_str());
        return DxlError::CANNOT_FIND_CONTROL_ITEM;
      }
    }
  }

  if (SetBulkReadHandler(indirect_id_arr) != DxlError::OK) {
    fprintf(stderr, "Cannot set the BulkRead handler.\n");
    return DxlError::BULK_READ_FAIL;
  }

  fprintf(stderr, "Success to set BulkRead handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetFastBulkReadHandler(std::vector<uint8_t> id_arr)
{
  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(
        it_id, it_id, "Indirect Data Read", IN_ADDR,
        IN_SIZE) == false)
    {
      fprintf(
        stderr,
        "Fail to set indirect address fast bulk read. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_BULK_READ_FAIL;
    }
    // Set indirect addr.
    indirect_info_read_[it_id].indirect_data_addr = IN_ADDR;

    fprintf(
      stderr,
      "set fast bulk read (indirect addr) : addr %d, size %d\n",
      IN_ADDR, indirect_info_read_[it_id].size);
  }

  for (auto it_id : id_arr) {
    uint8_t ID = it_id;
    uint16_t ADDR = indirect_info_read_[ID].indirect_data_addr;
    uint8_t SIZE = indirect_info_read_[ID].size;
    auto addParamResult = group_fast_bulk_read_->addParam(ID, ADDR, SIZE);
    if (addParamResult) {  // success
      fprintf(
        stderr, "[ID:%03d] Add BulkRead item : [Indirect Item Data][%d][%d]\n", ID, ADDR, SIZE);
    } else {
      fprintf(
        stderr, "[ID:%03d] Failed to BulkRead item : [Indirect Item Data]]\n", ID);
      return DxlError::SET_BULK_READ_FAIL;
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::SetBulkReadHandler(std::vector<uint8_t> id_arr)
{
  // Try to set up fast bulk read first
  if (use_fast_read_protocol_) {
    DxlError fast_result = SetFastBulkReadHandler(id_arr);
    if (fast_result == DxlError::OK) {
      fprintf(stderr, "FastBulkRead handler set up successfully.\n");
      return DxlError::OK;
    } else {
      fprintf(stderr, "FastBulkRead handler failed, falling back to normal BulkRead.\n");
      use_fast_read_protocol_ = false;
    }
  }

  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(
        it_id, it_id, "Indirect Data Read", IN_ADDR,
        IN_SIZE) == false)
    {
      fprintf(
        stderr,
        "Fail to set indirect address bulk read. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_BULK_READ_FAIL;
    }
    // Set indirect addr.
    indirect_info_read_[it_id].indirect_data_addr = IN_ADDR;

    fprintf(
      stderr,
      "set bulk read (indirect addr) : addr %d, size %d\n",
      IN_ADDR, indirect_info_read_[it_id].size);
  }

  for (auto it_id : id_arr) {
    uint8_t ID = it_id;
    uint16_t ADDR = indirect_info_read_[ID].indirect_data_addr;
    uint8_t SIZE = indirect_info_read_[ID].size;
    auto addParamResult = group_bulk_read_->addParam(ID, ADDR, SIZE);
    if (addParamResult) {  // success
      fprintf(
        stderr, "[ID:%03d] Add BulkRead item : [Indirect Item Data][%d][%d]\n", ID, ADDR, SIZE);
    } else {
      fprintf(
        stderr, "[ID:%03d] Failed to BulkRead item : [Indirect Item Data]]\n", ID);
      return DxlError::SET_BULK_READ_FAIL;
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::AddDirectRead(
  uint8_t id, std::string item_name, uint16_t item_addr, uint8_t item_size)
{
  if (group_bulk_read_->addParam(id, item_addr, item_size) == true) {
    fprintf(
      stderr, "[ID:%03d] Add BulkRead item : [%s][%d][%d]\n",
      id, item_name.c_str(), item_addr, item_size);
  } else {
    fprintf(
      stderr, "[ID:%03d] Failed to BulkRead item : [%s][%d][%d]\n",
      id, item_name.c_str(), item_addr, item_size);
    return DxlError::SET_BULK_READ_FAIL;
  }

  if (group_fast_bulk_read_->addParam(id, item_addr, item_size) == true) {
    fprintf(
      stderr, "[ID:%03d] Add FastBulkRead item : [%s][%d][%d]\n",
      id, item_name.c_str(), item_addr, item_size);
  } else {
    fprintf(
      stderr, "[ID:%03d] Failed to FastBulkRead item : [%s][%d][%d]\n",
      id, item_name.c_str(), item_addr, item_size);
    return DxlError::SET_BULK_READ_FAIL;
  }
  return DxlError::OK;
}

DxlError Dynamixel::GetDxlValueFromBulkRead(double period_ms)
{
  // Try fast bulk read for the first 10 attempts after startup/handler setup.
  // If any of the first 10 attempts succeeds, use fast bulk read permanently.
  // If all 10 attempts fail, permanently fallback to normal bulk read.
  if (use_fast_read_protocol_ && group_fast_bulk_read_ &&
    (fast_read_permanent_ || fast_read_fail_count_ < 10))
  {
    DxlError comm_result = ProcessReadCommunication(port_handler_, period_ms, false, true);
    if (comm_result == DxlError::OK) {
      // Success, process data, and use fast bulk read permanently
      if (group_bulk_read_) {
        delete group_bulk_read_;
        group_bulk_read_ = nullptr;
      }

      for (auto it_read_data : read_data_list_) {
        uint8_t id = it_read_data.comm_id;
        uint16_t indirect_addr = indirect_info_read_[id].indirect_data_addr;
        if (CheckIndirectReadAvailable(id) != DxlError::OK) {
          ProcessDirectReadData(
            id,
            it_read_data.item_addr,
            it_read_data.item_name,
            it_read_data.item_size,
            it_read_data.item_data_ptr_vec,
            [this](uint8_t lambda_id, uint16_t addr, uint8_t size) {
              return group_fast_bulk_read_->getData(lambda_id, addr, size);
            });
        } else {
          ProcessReadData(
            id,
            indirect_addr,
            it_read_data.id_arr,
            indirect_info_read_[id].item_name,
            indirect_info_read_[id].item_size,
            it_read_data.item_data_ptr_vec,
            [this](uint8_t lambda_id, uint16_t addr, uint8_t size) {
              return group_fast_bulk_read_->getData(lambda_id, addr, size);
            });
        }
      }
      // Mark as permanently using fast bulk read after first success
      fast_read_permanent_ = true;
      return DxlError::OK;
    } else if (!fast_read_permanent_) {
      // Only increment fail count and fallback if not yet permanent
      ++fast_read_fail_count_;
      fprintf(stderr, "FastBulkRead TxRx failed (attempt %d/10)\n", fast_read_fail_count_);
      if (fast_read_fail_count_ >= 10) {
        if (group_fast_bulk_read_) {
          delete group_fast_bulk_read_;
          group_fast_bulk_read_ = nullptr;
        }
        // Permanently switch to normal bulk read
        fprintf(
          stderr,
          "FastBulkRead failed 10 times, switching to normal BulkRead permanently.\n");
        use_fast_read_protocol_ = false;
        // Set up normal bulk read handler
        std::vector<uint8_t> indirect_id_arr;

        for (auto it_read_data : read_data_list_) {
          if (CheckIndirectReadAvailable(it_read_data.comm_id) == DxlError::OK) {
            indirect_id_arr.push_back(it_read_data.comm_id);
          }
        }

        SetBulkReadHandler(indirect_id_arr);
      }
      // Return error for this attempt
      return DxlError::BULK_READ_FAIL;
    } else {
      // If permanent, ignore failures and keep using fast bulk read
      return comm_result;
    }
  }
  DxlError comm_result = ProcessReadCommunication(port_handler_, period_ms, false, false);
  if (comm_result != DxlError::OK) {
    return comm_result;
  }

  for (auto it_read_data : read_data_list_) {
    uint8_t id = it_read_data.comm_id;
    uint16_t indirect_addr = indirect_info_read_[id].indirect_data_addr;
    if (CheckIndirectReadAvailable(id) != DxlError::OK) {
      ProcessDirectReadData(
        id,
        it_read_data.item_addr,
        it_read_data.item_name,
        it_read_data.item_size,
        it_read_data.item_data_ptr_vec,
        [this](uint8_t lambda_id, uint16_t addr, uint8_t size) {
          return group_bulk_read_->getData(lambda_id, addr, size);
        });
    } else {
      ProcessReadData(
        id,
        indirect_addr,
        it_read_data.id_arr,
        indirect_info_read_[id].item_name,
        indirect_info_read_[id].item_size,
        it_read_data.item_data_ptr_vec,
        [this](uint8_t lambda_id, uint16_t addr, uint8_t size) {
          return group_bulk_read_->getData(lambda_id, addr, size);
        });
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::ProcessReadCommunication(
  dynamixel::PortHandler * port_handler,
  double period_ms,
  bool is_sync,
  bool is_fast)
{
  int dxl_comm_result;

  // Send packet
  if (is_sync) {
    if (is_fast && group_fast_sync_read_) {
      dxl_comm_result = group_fast_sync_read_->txPacket();
    } else if (group_sync_read_) {
      dxl_comm_result = group_sync_read_->txPacket();
    } else {
      return DxlError::SYNC_READ_FAIL;
    }
    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr, "%s Tx Fail [Dxl Size : %ld] [Error code : %d]\n",
        is_fast ? "FastSyncRead" : "SyncRead",
        read_data_list_.size(), dxl_comm_result);
      return DxlError::SYNC_READ_FAIL;
    }
  } else {
    // Bulk read
    if (is_fast && group_fast_bulk_read_) {
      dxl_comm_result = group_fast_bulk_read_->txPacket();
    } else if (group_bulk_read_) {
      dxl_comm_result = group_bulk_read_->txPacket();
    } else {
      return DxlError::BULK_READ_FAIL;
    }
    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr, "%s Tx Fail [Dxl Size : %ld] [Error code : %d]\n",
        is_fast ? "FastBulkRead" : "BulkRead",
        read_data_list_.size(), dxl_comm_result);
      return DxlError::BULK_READ_FAIL;
    }
  }

  // Set timeout if period_ms is specified
  if (period_ms > 0) {
    port_handler->setPacketTimeout(period_ms);
  }

  // Receive packet
  if (is_sync) {
    if (is_fast && group_fast_sync_read_) {
      dxl_comm_result = group_fast_sync_read_->rxPacket();
    } else if (group_sync_read_) {
      dxl_comm_result = group_sync_read_->rxPacket();
    } else {
      return DxlError::SYNC_READ_FAIL;
    }
    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr, "%s Rx Fail [Dxl Size : %ld] [Error code : %d]\n",
        is_fast ? "FastSyncRead" : "SyncRead",
        read_data_list_.size(), dxl_comm_result);
      return DxlError::SYNC_READ_FAIL;
    }
  } else {
    // Bulk read
    if (is_fast && group_fast_bulk_read_) {
      dxl_comm_result = group_fast_bulk_read_->rxPacket();
    } else if (group_bulk_read_) {
      dxl_comm_result = group_bulk_read_->rxPacket();
    } else {
      return DxlError::BULK_READ_FAIL;
    }
    if (dxl_comm_result != COMM_SUCCESS) {
      fprintf(
        stderr, "%s Rx Fail [Dxl Size : %ld] [Error code : %d]\n",
        is_fast ? "FastBulkRead" : "BulkRead",
        read_data_list_.size(), dxl_comm_result);
      return DxlError::BULK_READ_FAIL;
    }
  }

  return DxlError::OK;
}

DxlError Dynamixel::ProcessReadData(
  uint8_t comm_id,
  uint16_t indirect_addr,
  const std::vector<uint8_t> & id_arr,
  const std::vector<std::string> & item_names,
  const std::vector<uint8_t> & item_sizes,
  const std::vector<std::shared_ptr<double>> & data_ptrs,
  std::function<uint32_t(uint8_t, uint16_t, uint8_t)> get_data_func)
{
  uint16_t current_addr = indirect_addr;

  for (size_t item_index = 0; item_index < item_names.size(); item_index++) {
    uint8_t ID = id_arr[item_index];
    uint8_t size = item_sizes[item_index];
    if (item_index > 0) {current_addr += item_sizes[item_index - 1];}

    uint32_t dxl_getdata = get_data_func(comm_id, current_addr, size);

    // Check if there is unit info for this item
    double unit_value;
    bool is_signed;
    if (dxl_info_.GetDxlUnitValue(comm_id, ID, item_names[item_index], unit_value) &&
      dxl_info_.GetDxlSignType(comm_id, ID, item_names[item_index], is_signed))
    {
      // Use unit info and sign type to properly convert the value
      *data_ptrs[item_index] = ConvertValueWithUnitInfo(
        comm_id, ID, item_names[item_index], dxl_getdata,
        size, is_signed);
    } else {
      // Fallback to existing logic for compatibility
      if (item_names[item_index] == "Present Position") {
        *data_ptrs[item_index] = dxl_info_.ConvertValueToRadian(
          comm_id,
          ID,
          static_cast<int32_t>(dxl_getdata));
      } else {
        *data_ptrs[item_index] = static_cast<double>(dxl_getdata);
      }
    }
  }
  return DxlError::OK;
}

DxlError Dynamixel::ProcessDirectReadData(
  uint8_t comm_id,
  const std::vector<uint16_t> & item_addrs,
  const std::vector<std::string> & item_names,
  const std::vector<uint8_t> & item_sizes,
  const std::vector<std::shared_ptr<double>> & data_ptrs,
  std::function<uint32_t(uint8_t, uint16_t, uint8_t)> get_data_func)
{
  for (size_t item_index = 0; item_index < item_addrs.size(); item_index++) {
    uint8_t ID = comm_id;
    uint16_t current_addr = item_addrs[item_index];
    uint8_t size = item_sizes[item_index];

    uint32_t dxl_getdata = get_data_func(comm_id, current_addr, size);

    // Check if there is unit info for this item
    double unit_value;
    bool is_signed;
    if (dxl_info_.GetDxlUnitValue(comm_id, ID, item_names[item_index], unit_value) &&
      dxl_info_.GetDxlSignType(comm_id, ID, item_names[item_index], is_signed))
    {
      // Use unit info and sign type to properly convert the value
      *data_ptrs[item_index] = ConvertValueWithUnitInfo(
        comm_id, ID, item_names[item_index], dxl_getdata,
        size, is_signed);
    } else {
      // Fallback to existing logic for compatibility
      if (item_names[item_index] == "Present Position") {
        *data_ptrs[item_index] = dxl_info_.ConvertValueToRadian(
          comm_id,
          ID,
          static_cast<int32_t>(dxl_getdata));
      } else {
        *data_ptrs[item_index] = static_cast<double>(dxl_getdata);
      }
    }
  }
  return DxlError::OK;
}

void Dynamixel::ResetIndirectRead(std::vector<uint8_t> id_arr)
{
  IndirectInfo temp;
  temp.indirect_data_addr = 0;
  temp.size = 0;
  temp.cnt = 0;
  temp.item_name.clear();
  temp.item_size.clear();
  for (auto it_id : id_arr) {
    indirect_info_read_[it_id] = temp;
  }
}

DxlError Dynamixel::CheckIndirectReadAvailable(uint8_t id)
{
  uint16_t INDIRECT_ADDR;
  uint8_t INDIRECT_SIZE;
  if (dxl_info_.GetDxlControlItem(
      id, id, "Indirect Address Read",
      INDIRECT_ADDR, INDIRECT_SIZE) == false)
  {
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
  return DxlError::OK;
}

DxlError Dynamixel::AddIndirectRead(
  uint8_t id,
  std::string item_name,
  uint16_t item_addr,
  uint8_t item_size)
{
  // write address to indirect addr control item
  uint16_t INDIRECT_ADDR;
  uint8_t INDIRECT_SIZE;
  if (dxl_info_.GetDxlControlItem(
      id, id, "Indirect Address Read",
      INDIRECT_ADDR, INDIRECT_SIZE) == true)
  {
    uint8_t using_size = indirect_info_read_[id].size;

    for (uint16_t i = 0; i < item_size; i++) {
      DxlError write_result = DxlError::INDIRECT_ADDR_FAIL;
      uint16_t addr = static_cast<uint16_t>(INDIRECT_ADDR + (using_size * 2));
      uint16_t item_addr_i = item_addr + i;

      write_result = WriteItem(id, id, addr, 2, item_addr_i);

      if (write_result != DxlError::OK) {
        fprintf(stderr, "[AddIndirectRead][ID:%03d] WriteItem failed\n", id);
        return DxlError::INDIRECT_ADDR_FAIL;
      }
      using_size++;
    }
    indirect_info_read_[id].size = using_size;
    indirect_info_read_[id].cnt += 1;
    indirect_info_read_[id].item_name.push_back(item_name);
    indirect_info_read_[id].item_size.push_back(item_size);

    return DxlError::OK;
  } else {
    return DxlError::CANNOT_FIND_CONTROL_ITEM;
  }
}

DxlError Dynamixel::SetSyncWriteItemAndHandler()
{
  std::vector<uint8_t> id_arr;
  for (auto it_write_data : write_data_list_) {
    id_arr.push_back(it_write_data.comm_id);
  }

  {
    std::vector<std::pair<uint8_t, uint8_t>> pairs;
    for (auto cid : id_arr) {
      pairs.emplace_back(cid, cid);
    }
    DynamixelDisable(pairs);
  }
  ResetIndirectWrite(id_arr);

  for (auto it_write_data : write_data_list_) {
    for (size_t item_index = 0; item_index < it_write_data.item_name.size();
      item_index++)
    {
      if (AddIndirectWrite(
          it_write_data.comm_id,
          it_write_data.item_name.at(item_index),
          it_write_data.item_addr.at(item_index),
          it_write_data.item_size.at(item_index)) != DxlError::OK)
      {
        fprintf(stderr, "Cannot set the SyncWrite handler.\n");
        return DxlError::SYNC_WRITE_FAIL;
      }
    }
  }

  if (SetSyncWriteHandler(id_arr) < 0) {
    fprintf(stderr, "Cannot set the SyncWrite handler.\n");
    return DxlError::SYNC_WRITE_FAIL;
  }

  fprintf(stderr, "Success to set SyncWrite handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetSyncWriteHandler(std::vector<uint8_t> id_arr)
{
  if (id_arr.size() == 0) {
    fprintf(stderr, "No Sync Write Item, not setting sync write handler\n");
    return DxlError::OK;
  }

  uint16_t INDIRECT_ADDR = 0;
  uint8_t INDIRECT_SIZE;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(
        it_id, it_id, "Indirect Data Write", INDIRECT_ADDR,
        INDIRECT_SIZE) == false)
    {
      fprintf(
        stderr,
        "Fail to set indirect address sync write. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_SYNC_WRITE_FAIL;
    }
    // Set indirect addr.
    indirect_info_write_[it_id].indirect_data_addr = INDIRECT_ADDR;
  }
  fprintf(
    stderr,
    "set sync write (indirect addr) : addr %d, size %d\n",
    INDIRECT_ADDR, indirect_info_write_[id_arr.at(0)].size);

  group_sync_write_ =
    new dynamixel::GroupSyncWrite(
    port_handler_, packet_handler_,
    INDIRECT_ADDR, indirect_info_write_[id_arr.at(0)].size);

  return DxlError::OK;
}
DxlError Dynamixel::SetDxlValueToSyncWrite()
{
  for (auto it_write_data : write_data_list_) {
    uint8_t comm_id = it_write_data.comm_id;
    uint8_t * param_write_value = new uint8_t[indirect_info_write_[comm_id].size];
    uint8_t added_byte = 0;

    for (uint16_t item_index = 0; item_index < indirect_info_write_[comm_id].cnt; item_index++) {
      double data = *it_write_data.item_data_ptr_vec.at(item_index);
      uint8_t ID = it_write_data.id_arr.at(item_index);
      std::string item_name = indirect_info_write_[comm_id].item_name.at(item_index);
      uint8_t size = indirect_info_write_[comm_id].item_size.at(item_index);

      // Check if there is unit info for this item
      double unit_value;
      bool is_signed;
      if (dxl_info_.GetDxlUnitValue(comm_id, ID, item_name, unit_value) &&
        dxl_info_.GetDxlSignType(comm_id, ID, item_name, is_signed))
      {
        // Use unit info and sign type to properly convert the value
        uint32_t raw_value = ConvertUnitValueToRawValue(
          comm_id, ID, item_name, data, size,
          is_signed);
        WriteValueToBuffer(param_write_value, added_byte, raw_value, size);
      } else {
        // Fallback to existing logic for compatibility
        if (item_name == "Goal Position") {
          int32_t goal_position = dxl_info_.ConvertRadianToValue(comm_id, ID, data);
          WriteValueToBuffer(
            param_write_value, added_byte, static_cast<uint32_t>(goal_position),
            4);
        } else {
          WriteValueToBuffer(param_write_value, added_byte, static_cast<uint32_t>(data), size);
        }
      }
      added_byte += size;
    }

    if (group_sync_write_->addParam(comm_id, param_write_value) != true) {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed\n", comm_id);
      return DxlError::SYNC_WRITE_FAIL;
    }
  }

  int dxl_comm_result = group_sync_write_->txPacket();
  group_sync_write_->clearParam();

  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(stderr, "%s\n", packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::SYNC_WRITE_FAIL;
  } else {
    return DxlError::OK;
  }
}

DxlError Dynamixel::SetBulkWriteItemAndHandler()
{
  std::vector<uint8_t> id_arr;
  for (auto it_write_data : write_data_list_) {
    id_arr.push_back(it_write_data.comm_id);
  }

  group_bulk_write_ = new dynamixel::GroupBulkWrite(port_handler_, packet_handler_);

  std::vector<uint8_t> indirect_id_arr;
  std::vector<uint8_t> direct_id_arr;

  std::vector<RWItemList> indirect_write_data_list;
  std::vector<RWItemList> direct_write_data_list;

  for (auto it_write_data : write_data_list_) {
    if (CheckIndirectWriteAvailable(it_write_data.comm_id) == DxlError::OK) {
      indirect_id_arr.push_back(it_write_data.comm_id);
      indirect_write_data_list.push_back(it_write_data);
    } else {
      direct_id_arr.push_back(it_write_data.comm_id);
      direct_write_data_list.push_back(it_write_data);
    }
  }

  ResetDirectWrite(direct_id_arr);

  // Handle direct writes
  for (auto it_write_data : direct_write_data_list) {
    if (it_write_data.item_addr.empty() || it_write_data.item_size.empty()) {
      continue;
    }
    // Calculate min address and max end address
    uint16_t min_addr = it_write_data.item_addr[0];
    uint16_t max_end_addr = it_write_data.item_addr[0] + it_write_data.item_size[0];
    for (size_t item_index = 1; item_index < it_write_data.item_addr.size(); ++item_index) {
      uint16_t addr = it_write_data.item_addr[item_index];
      uint16_t size = it_write_data.item_size[item_index];
      if (addr < min_addr) {min_addr = addr;}
      if (addr + size > max_end_addr) {max_end_addr = addr + size;}
    }
    uint8_t total_size = static_cast<uint8_t>(max_end_addr - min_addr);

    // Check for gaps between items
    std::vector<std::pair<uint16_t, uint16_t>> addr_ranges;
    for (size_t item_index = 0; item_index < it_write_data.item_addr.size(); ++item_index) {
      addr_ranges.push_back(
        {
          it_write_data.item_addr[item_index],
          it_write_data.item_addr[item_index] + it_write_data.item_size[item_index]
        });
    }
    std::sort(addr_ranges.begin(), addr_ranges.end());

    for (size_t i = 0; i < addr_ranges.size() - 1; ++i) {
      if (addr_ranges[i].second != addr_ranges[i + 1].first) {
        fprintf(
          stderr, "[ID:%03d] Error: Gap detected between items at addresses %d and %d\n",
          it_write_data.comm_id, addr_ranges[i].second, addr_ranges[i + 1].first);
        return DxlError::BULK_WRITE_FAIL;
      }
    }

    // Store direct write info
    direct_info_write_[it_write_data.comm_id].indirect_data_addr = min_addr;
    direct_info_write_[it_write_data.comm_id].size = total_size;
    direct_info_write_[it_write_data.comm_id].cnt =
      static_cast<uint16_t>(it_write_data.item_name.size());
    direct_info_write_[it_write_data.comm_id].item_name = it_write_data.item_name;
    direct_info_write_[it_write_data.comm_id].item_size = it_write_data.item_size;

    fprintf(
      stderr,
      "set bulk write (direct addr) : addr %d, size %d\n",
      min_addr, total_size);
  }

  // Handle indirect writes
  {
    std::vector<std::pair<uint8_t, uint8_t>> pairs;
    for (auto cid : indirect_id_arr) {
      pairs.emplace_back(cid, cid);
    }
    DynamixelDisable(pairs);
  }
  ResetIndirectWrite(indirect_id_arr);

  for (auto it_write_data : indirect_write_data_list) {
    for (size_t item_index = 0; item_index < it_write_data.item_name.size();
      item_index++)
    {
      if (AddIndirectWrite(
          it_write_data.comm_id,
          it_write_data.item_name.at(item_index),
          it_write_data.item_addr.at(item_index),
          it_write_data.item_size.at(item_index)) != DxlError::OK)
      {
        fprintf(stderr, "Cannot set the BulkWrite handler.\n");
        return DxlError::BULK_WRITE_FAIL;
      }

      fprintf(
        stderr, "[ID:%03d] Add Indirect Address Write Item : [%s]\n",
        it_write_data.comm_id,
        it_write_data.item_name.at(item_index).c_str());
    }
  }

  if (SetBulkWriteHandler(indirect_id_arr) < 0) {
    fprintf(stderr, "Cannot set the BulkWrite handler.\n");
    return DxlError::BULK_WRITE_FAIL;
  }

  fprintf(stderr, "Success to set BulkWrite handler using indirect address\n");
  return DxlError::OK;
}

DxlError Dynamixel::SetBulkWriteHandler(std::vector<uint8_t> id_arr)
{
  uint16_t IN_ADDR = 0;
  uint8_t IN_SIZE = 0;

  for (auto it_id : id_arr) {
    // Get the indirect addr.
    if (dxl_info_.GetDxlControlItem(
        it_id, it_id, "Indirect Data Write", IN_ADDR,
        IN_SIZE) == false)
    {
      fprintf(
        stderr,
        "Fail to set indirect address bulk write. "
        "the dxl unincluding indirect address in control table are being used.\n");
      return DxlError::SET_BULK_WRITE_FAIL;
    }
    // Set indirect addr.
    indirect_info_write_[it_id].indirect_data_addr = IN_ADDR;

    fprintf(
      stderr,
      "set bulk write (indirect addr) : addr %d, size %d\n",
      IN_ADDR, indirect_info_write_[it_id].size);
  }

  return DxlError::OK;
}

DxlError Dynamixel::SetupGroupedSyncWrite()
{
  grouped_sync_writes_.clear();
  for (const auto & item : write_data_list_) {
    const auto & table = direct_info_write_.count(item.comm_id) ?
      direct_info_write_ : indirect_info_write_;
    const auto found = table.find(item.comm_id);
    if (found == table.end() || found->second.size == 0) {
      grouped_sync_writes_.clear();
      return DxlError::SET_BULK_WRITE_FAIL;
    }
    const auto key = std::make_pair(
      static_cast<uint16_t>(found->second.indirect_data_addr),
      static_cast<uint16_t>(found->second.size));
    if (!grouped_sync_writes_.count(key)) {
      grouped_sync_writes_.emplace(key, std::unique_ptr<dynamixel::GroupSyncWrite>(
          new dynamixel::GroupSyncWrite(port_handler_, packet_handler_, key.first, key.second)));
    }
  }
  fprintf(stderr, "Grouped SyncWrite enabled: %zu address/length groups\n",
    grouped_sync_writes_.size());
  return DxlError::OK;
}

DxlError Dynamixel::SetDxlValueToGroupedSyncWrite()
{
  const auto clear = [this]() {
      for (auto & group : grouped_sync_writes_) {group.second->clearParam();}
    };
  clear();
  // Construct every group's payload before transmitting any group. Preserve
  // the existing per-item units/signs and indirect mapping; only framing changes.
  for (const auto & item : write_data_list_) {
    const auto & table = direct_info_write_.count(item.comm_id) ?
      direct_info_write_ : indirect_info_write_;
    const auto found = table.find(item.comm_id);
    if (found == table.end()) {clear(); return DxlError::BULK_WRITE_FAIL;}
    const auto & info = found->second;
    const auto key = std::make_pair(static_cast<uint16_t>(info.indirect_data_addr),
        static_cast<uint16_t>(info.size));
    const auto group = grouped_sync_writes_.find(key);
    if (group == grouped_sync_writes_.end() || info.cnt != item.item_data_ptr_vec.size() ||
      info.cnt != item.id_arr.size() || info.cnt != info.item_name.size() ||
      info.cnt != info.item_size.size())
    {clear(); return DxlError::BULK_WRITE_FAIL;}
    std::vector<uint8_t> data(info.size, 0);
    size_t offset = 0;
    for (size_t i = 0; i < info.cnt; ++i) {
      const auto size = info.item_size.at(i);
      if (!item.item_data_ptr_vec.at(i) || (size != 1 && size != 2 && size != 4) ||
        offset + size > data.size() || offset > 255)
      {clear(); return DxlError::BULK_WRITE_FAIL;}
      const double value = *item.item_data_ptr_vec.at(i);
      if (!std::isfinite(value)) {clear(); return DxlError::BULK_WRITE_FAIL;}
      const auto id = item.id_arr.at(i);
      const auto & name = info.item_name.at(i);
      double unit;
      bool signed_value;
      uint32_t raw;
      if (dxl_info_.GetDxlUnitValue(item.comm_id, id, name, unit) &&
        dxl_info_.GetDxlSignType(item.comm_id, id, name, signed_value))
      {
        raw = ConvertUnitValueToRawValue(item.comm_id, id, name, value, size, signed_value);
      } else if (name == "Goal Position") {
        raw = static_cast<uint32_t>(dxl_info_.ConvertRadianToValue(item.comm_id, id, value));
      } else {
        raw = static_cast<uint32_t>(value);
      }
      WriteValueToBuffer(data.data(), static_cast<uint8_t>(offset), raw, size);
      offset += size;
    }
    if (offset != data.size() || !group->second->addParam(item.comm_id, data.data())) {
      clear(); return DxlError::BULK_WRITE_FAIL;
    }
  }
  for (auto & group : grouped_sync_writes_) {
    const int result = group.second->txPacket();
    if (result != COMM_SUCCESS) {
      fprintf(stderr, "Grouped SyncWrite failed: %s\n", packet_handler_->getTxRxResult(result));
      clear(); return DxlError::BULK_WRITE_FAIL;
    }
  }
  clear();
  return DxlError::OK;
}

DxlError Dynamixel::SetDxlValueToBulkWrite()
{
  for (auto it_write_data : write_data_list_) {
    uint8_t comm_id = it_write_data.comm_id;
    uint8_t * param_write_value;
    uint8_t added_byte = 0;

    // Check if this is a direct write
    if (direct_info_write_.find(comm_id) != direct_info_write_.end()) {
      param_write_value = new uint8_t[direct_info_write_[comm_id].size];

      for (uint16_t item_index = 0; item_index < direct_info_write_[comm_id].cnt; item_index++) {
        double data = *it_write_data.item_data_ptr_vec.at(item_index);
        uint8_t ID = comm_id;
        std::string item_name = direct_info_write_[comm_id].item_name.at(item_index);
        uint8_t size = direct_info_write_[comm_id].item_size.at(item_index);

        // Check if there is unit info for this item
        double unit_value;
        bool is_signed;
        if (dxl_info_.GetDxlUnitValue(comm_id, ID, item_name, unit_value) &&
          dxl_info_.GetDxlSignType(comm_id, ID, item_name, is_signed))
        {
          // Use unit info and sign type to properly convert the value
          uint32_t raw_value = ConvertUnitValueToRawValue(
            comm_id, ID, item_name, data, size,
            is_signed);
          WriteValueToBuffer(param_write_value, added_byte, raw_value, size);
        } else {
          // Fallback to existing logic for compatibility
          if (item_name == "Goal Position") {
            int32_t goal_position = dxl_info_.ConvertRadianToValue(comm_id, ID, data);
            WriteValueToBuffer(
              param_write_value, added_byte, static_cast<uint32_t>(goal_position),
              4);
          } else {
            WriteValueToBuffer(param_write_value, added_byte, static_cast<uint32_t>(data), size);
          }
        }
        added_byte += size;
      }

      if (group_bulk_write_->addParam(
          comm_id,
          direct_info_write_[comm_id].indirect_data_addr,
          direct_info_write_[comm_id].size,
          param_write_value) != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed\n", comm_id);
        return DxlError::BULK_WRITE_FAIL;
      }
    } else {
      // Handle indirect write
      param_write_value = new uint8_t[indirect_info_write_[comm_id].size];

      for (uint16_t item_index = 0; item_index < indirect_info_write_[comm_id].cnt; item_index++) {
        double data = *it_write_data.item_data_ptr_vec.at(item_index);
        uint8_t ID = it_write_data.id_arr.at(item_index);
        std::string item_name = indirect_info_write_[comm_id].item_name.at(item_index);
        uint8_t size = indirect_info_write_[comm_id].item_size.at(item_index);

        // Check if there is unit info for this item
        double unit_value;
        bool is_signed;
        if (dxl_info_.GetDxlUnitValue(comm_id, ID, item_name, unit_value) &&
          dxl_info_.GetDxlSignType(comm_id, ID, item_name, is_signed))
        {
          // Use unit info and sign type to properly convert the value
          uint32_t raw_value = ConvertUnitValueToRawValue(
            comm_id, ID, item_name, data, size,
            is_signed);
          WriteValueToBuffer(param_write_value, added_byte, raw_value, size);
        } else {
          // Fallback to existing logic for compatibility
          if (item_name == "Goal Position") {
            int32_t goal_position = dxl_info_.ConvertRadianToValue(comm_id, ID, data);
            WriteValueToBuffer(
              param_write_value, added_byte, static_cast<uint32_t>(goal_position),
              4);
          } else {
            WriteValueToBuffer(param_write_value, added_byte, static_cast<uint32_t>(data), size);
          }
        }
        added_byte += size;
      }

      if (group_bulk_write_->addParam(
          comm_id,
          indirect_info_write_[comm_id].indirect_data_addr,
          indirect_info_write_[comm_id].size,
          param_write_value) != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed\n", comm_id);
        return DxlError::BULK_WRITE_FAIL;
      }
    }
  }

  int dxl_comm_result = group_bulk_write_->txPacket();
  group_bulk_write_->clearParam();

  if (dxl_comm_result != COMM_SUCCESS) {
    fprintf(stderr, "%s\n", packet_handler_->getTxRxResult(dxl_comm_result));
    return DxlError::BULK_WRITE_FAIL;
  } else {
    return DxlError::OK;
  }
}

void Dynamixel::ResetIndirectWrite(std::vector<uint8_t> id_arr)
{
  IndirectInfo temp;
  temp.indirect_data_addr = 0;
  temp.size = 0;
  temp.cnt = 0;
  temp.item_name.clear();
  temp.item_size.clear();
  for (auto it_id : id_arr) {
    indirect_info_write_[it_id] = temp;
  }
}

DxlError Dynamixel::AddIndirectWrite(
  uint8_t id,
  std::string item_name,
  uint16_t item_addr,
  uint8_t item_size)
{
  // write address to indirect addr control item
  uint16_t INDIRECT_ADDR;
  uint8_t INDIRECT_SIZE;
  dxl_info_.GetDxlControlItem(id, id, "Indirect Address Write", INDIRECT_ADDR, INDIRECT_SIZE);

  uint8_t using_size = indirect_info_write_[id].size;

  for (uint16_t i = 0; i < item_size; i++) {
    if (WriteItem(
        id, id, static_cast<uint16_t>(INDIRECT_ADDR + (using_size * 2)), 2,
        item_addr + i) != DxlError::OK)
    {
      return DxlError::SET_BULK_WRITE_FAIL;
    }
    using_size++;
  }
  indirect_info_write_[id].size = using_size;
  indirect_info_write_[id].cnt += 1;
  indirect_info_write_[id].item_name.push_back(item_name);
  indirect_info_write_[id].item_size.push_back(item_size);

  return DxlError::OK;
}

void Dynamixel::ResetDirectWrite(std::vector<uint8_t> id_arr)
{
  IndirectInfo temp;
  temp.indirect_data_addr = 0;
  temp.size = 0;
  temp.cnt = 0;
  temp.item_name.clear();
  temp.item_size.clear();
  for (auto it_id : id_arr) {
    direct_info_write_[it_id] = temp;
  }
}

double Dynamixel::ConvertValueWithUnitInfo(
  uint8_t comm_id, uint8_t id, std::string item_name, uint32_t raw_value,
  uint8_t size, bool is_signed)
{
  if (size == 1) {
    if (is_signed) {
      int8_t signed_value = static_cast<int8_t>(raw_value);
      return dxl_info_.ConvertValueToUnit<int8_t>(comm_id, id, item_name, signed_value);
    } else {
      uint8_t unsigned_value = static_cast<uint8_t>(raw_value);
      return dxl_info_.ConvertValueToUnit<uint8_t>(comm_id, id, item_name, unsigned_value);
    }
  } else if (size == 2) {
    if (is_signed) {
      int16_t signed_value = static_cast<int16_t>(raw_value);
      return dxl_info_.ConvertValueToUnit<int16_t>(comm_id, id, item_name, signed_value);
    } else {
      uint16_t unsigned_value = static_cast<uint16_t>(raw_value);
      return dxl_info_.ConvertValueToUnit<uint16_t>(comm_id, id, item_name, unsigned_value);
    }
  } else if (size == 4) {
    if (is_signed) {
      int32_t signed_value = static_cast<int32_t>(raw_value);
      return dxl_info_.ConvertValueToUnit<int32_t>(comm_id, id, item_name, signed_value);
    } else {
      uint32_t unsigned_value = static_cast<uint32_t>(raw_value);
      return dxl_info_.ConvertValueToUnit<uint32_t>(comm_id, id, item_name, unsigned_value);
    }
  }

  // Throw error for unknown sizes
  std::string error_msg = "Unknown data size " + std::to_string(size) +
    " for item '" + item_name + "' in ID " + std::to_string(id);
  throw std::runtime_error(error_msg);
}

uint32_t Dynamixel::ConvertUnitValueToRawValue(
  uint8_t comm_id, uint8_t id, std::string item_name, double unit_value,
  uint8_t size, bool is_signed)
{
  if (size == 1) {
    if (is_signed) {
      int8_t signed_value = dxl_info_.ConvertUnitToValue<int8_t>(
        comm_id, id, item_name,
        unit_value);
      return static_cast<uint32_t>(signed_value);
    } else {
      uint8_t unsigned_value = dxl_info_.ConvertUnitToValue<uint8_t>(
        comm_id, id, item_name,
        unit_value);
      return static_cast<uint32_t>(unsigned_value);
    }
  } else if (size == 2) {
    if (is_signed) {
      int16_t signed_value = dxl_info_.ConvertUnitToValue<int16_t>(
        comm_id, id, item_name,
        unit_value);
      return static_cast<uint32_t>(signed_value);
    } else {
      uint16_t unsigned_value = dxl_info_.ConvertUnitToValue<uint16_t>(
        comm_id, id, item_name,
        unit_value);
      return static_cast<uint32_t>(unsigned_value);
    }
  } else if (size == 4) {
    if (is_signed) {
      int32_t signed_value = dxl_info_.ConvertUnitToValue<int32_t>(
        comm_id, id, item_name,
        unit_value);
      return static_cast<uint32_t>(signed_value);
    } else {
      uint32_t unsigned_value = dxl_info_.ConvertUnitToValue<uint32_t>(
        comm_id, id, item_name,
        unit_value);
      return unsigned_value;
    }
  }

  // Throw error for unknown sizes
  std::string error_msg = "Unknown data size " + std::to_string(size) +
    " for item '" + item_name + "' in ID " + std::to_string(id);
  throw std::runtime_error(error_msg);
}

void Dynamixel::WriteValueToBuffer(uint8_t * buffer, uint8_t offset, uint32_t value, uint8_t size)
{
  if (size == 1) {
    buffer[offset] = static_cast<uint8_t>(value);
  } else if (size == 2) {
    uint16_t value_16 = static_cast<uint16_t>(value);
    buffer[offset + 0] = DXL_LOBYTE(value_16);
    buffer[offset + 1] = DXL_HIBYTE(value_16);
  } else if (size == 4) {
    buffer[offset + 0] = DXL_LOBYTE(DXL_LOWORD(value));
    buffer[offset + 1] = DXL_HIBYTE(DXL_LOWORD(value));
    buffer[offset + 2] = DXL_LOBYTE(DXL_HIWORD(value));
    buffer[offset + 3] = DXL_HIBYTE(DXL_HIWORD(value));
  }
}
}  // namespace friction_dynamixel_hardware_interface
