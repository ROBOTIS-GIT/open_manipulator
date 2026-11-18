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

#ifndef FRICTION_DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_INFO_HPP_
#define FRICTION_DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_INFO_HPP_

#include <cmath>
#include <cstring>

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <regex>

#include <boost/algorithm/string.hpp>

namespace friction_dynamixel_hardware_interface
{

typedef struct
{
  uint16_t address;
  uint8_t size;
  std::string item_name;
} ControlItem;

typedef  struct
{
  double min_radian;
  double max_radian;
  int32_t value_of_zero_radian_position;
  int32_t value_of_max_radian_position;
  int32_t value_of_min_radian_position;
  uint16_t model_num;
  uint8_t firmware_version;

  std::vector<ControlItem> item;
  std::map<std::string, double> unit_map;
  std::map<std::string, bool> sign_type_map;
  std::map<std::string, double> offset_map;
} DxlInfo;

class DynamixelInfo
{
private:
  using DxlModelList = std::map<uint16_t, std::string>;
  DxlModelList dxl_model_list_;

  std::string dxl_model_file_dir;

  // Firmware version-aware model file selection
  std::string SelectModelFileByFirmwareVersion(
    const std::string & base_model_name,
    uint8_t firmware_version);
  uint8_t ExtractFirmwareVersionFromFilename(const std::string & filename);

public:
  // comm_id -> (id -> Control table)
  std::map<uint8_t, std::map<uint8_t, DxlInfo>> dxl_info_by_comm_;

  DynamixelInfo() {}
  ~DynamixelInfo() {}

  void SetDxlModelFolderPath(const char * path);
  void InitDxlModelInfo();

  void ReadDxlModelFile(uint8_t comm_id, uint8_t id, uint16_t model_num);
  void ReadDxlModelFile(uint8_t comm_id, uint8_t id, uint16_t model_num, uint8_t firmware_version);
  bool GetDxlControlItem(
    uint8_t comm_id, uint8_t id, std::string item_name, uint16_t & addr,
    uint8_t & size);
  bool CheckDxlControlItem(uint8_t comm_id, uint8_t id, std::string item_name);

  bool GetDxlUnitValue(uint8_t comm_id, uint8_t id, std::string data_name, double & unit_value);
  bool GetDxlSignType(uint8_t comm_id, uint8_t id, std::string data_name, bool & is_signed);
  bool GetDxlOffsetValue(uint8_t comm_id, uint8_t id, std::string data_name, double & offset_value);

  template<typename T>
  double ConvertValueToUnit(uint8_t comm_id, uint8_t id, std::string data_name, T value);

  template<typename T>
  T ConvertUnitToValue(uint8_t comm_id, uint8_t id, std::string data_name, double unit_value);

  // Helper method for internal use
  double GetUnitMultiplier(uint8_t comm_id, uint8_t id, std::string data_name);

  int32_t ConvertRadianToValue(uint8_t comm_id, uint8_t id, double radian);
  double ConvertValueToRadian(uint8_t comm_id, uint8_t id, int32_t value);

  std::string GetModelName(uint16_t model_number) const;
};

// Template implementations
template<typename T>
double DynamixelInfo::ConvertValueToUnit(
  uint8_t comm_id, uint8_t id, std::string data_name,
  T value)
{
  auto & info = dxl_info_by_comm_[comm_id][id];
  auto it = info.unit_map.find(data_name);
  if (it != info.unit_map.end()) {
    double converted_value = static_cast<double>(value) * it->second;
    auto offset_it = info.offset_map.find(data_name);
    if (offset_it != info.offset_map.end()) {
      converted_value += offset_it->second;
    }
    return converted_value;
  }
  return static_cast<double>(value);
}

template<typename T>
T DynamixelInfo::ConvertUnitToValue(
  uint8_t comm_id, uint8_t id, std::string data_name,
  double unit_value)
{
  auto & info = dxl_info_by_comm_[comm_id][id];
  auto it = info.unit_map.find(data_name);
  if (it != info.unit_map.end()) {
    double adjusted_value = unit_value;
    auto offset_it = info.offset_map.find(data_name);
    if (offset_it != info.offset_map.end()) {
      adjusted_value -= offset_it->second;
    }
    return static_cast<T>(adjusted_value / it->second);
  }
  return static_cast<T>(unit_value);
}

}  // namespace friction_dynamixel_hardware_interface

#endif  // FRICTION_DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_INFO_HPP_
