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

#include "friction_dynamixel_hardware_interface/dynamixel/dynamixel_info.hpp"
#include <dirent.h>
#include <string>
#include <utility>
#include <vector>
#include <regex>

namespace friction_dynamixel_hardware_interface
{

void DynamixelInfo::SetDxlModelFolderPath(const char * path)
{
  dxl_model_file_dir = std::string(path);
}

void DynamixelInfo::InitDxlModelInfo()
{
  std::string model_file = dxl_model_file_dir + "/dynamixel.model";
  std::ifstream open_file(model_file.c_str());
  if (open_file.is_open() != 1) {
    fprintf(stderr, "[ERROR] CANNOT FIND DXL MODEL LIST FILE.\n%s\n", model_file.c_str());
    exit(-1);
  }
  std::string line;
  getline(open_file, line);

  fprintf(stderr, "Dynamixel Information File List.\n");
  while (!open_file.eof()) {
    uint16_t model_number;
    std::string file_name;
    open_file >> model_number >> file_name;
    if (open_file.good()) {
      fprintf(stderr, "num: %d, name: %s\n", model_number, file_name.c_str());
      dxl_model_list_.insert(std::make_pair(model_number, file_name));
    }
  }
  open_file.close();
}

void DynamixelInfo::ReadDxlModelFile(uint8_t comm_id, uint8_t id, uint16_t model_num)
{
  ReadDxlModelFile(comm_id, id, model_num, 0);
}

void DynamixelInfo::ReadDxlModelFile(
  uint8_t comm_id, uint8_t id, uint16_t model_num,
  uint8_t firmware_version)
{
  std::string path = dxl_model_file_dir + "/";

  auto it = dxl_model_list_.find(model_num);
  if (it != dxl_model_list_.end()) {
    std::string base_model_name = it->second;
    std::string selected_model_name = SelectModelFileByFirmwareVersion(
      base_model_name,
      firmware_version);
    path += selected_model_name;
  } else {
    fprintf(stderr, "\n");
    std::string error_msg =
      std::string("Cannot find the DXL model from file list (model_num: ") +
      std::to_string(model_num) + ", model_name: " + GetModelName(model_num) + ")";
    throw std::runtime_error(error_msg);
  }

  std::ifstream open_file(path);
  if (open_file.is_open() != 1) {
    fprintf(stderr, "[ERROR] CANNOT FIND DXL [%s] MODEL FILE.\n", path.c_str());
    throw std::runtime_error("Cannot find DXL model file");
  }

  DxlInfo temp_dxl_info;
  std::string line;

  temp_dxl_info.model_num = model_num;
  temp_dxl_info.firmware_version = firmware_version;

  // Check if [type info] section exists
  bool type_info_found = false;
  bool unit_info_found = false;
  bool control_table_found = false;

  while (!open_file.eof() ) {
    getline(open_file, line);
    if (strcmp(line.c_str(), "[type info]") == 0) {
      type_info_found = true;
      break;
    } else if (strcmp(line.c_str(), "[unit info]") == 0) {
      unit_info_found = true;
      break;
    } else if (strcmp(line.c_str(), "[control table]") == 0) {
      control_table_found = true;
      break;
    }
  }
  if (type_info_found) {
    // Parse type info section
    while (!open_file.eof() ) {
      getline(open_file, line);
      if (strcmp(line.c_str(), "[unit info]") == 0) {
        unit_info_found = true;
        break;
      } else if (strcmp(line.c_str(), "[control table]") == 0) {
        control_table_found = true;
        break;
      }

      std::vector<std::string> strs;
      boost::split(strs, line, boost::is_any_of("\t"));

      if (strs.size() < 2) {
        continue;
      }

      try {
        if (strs.at(0) == "value_of_zero_radian_position") {
          temp_dxl_info.value_of_zero_radian_position = static_cast<int32_t>(stoi(strs.at(1)));
        } else if (strs.at(0) == "value_of_max_radian_position") {
          temp_dxl_info.value_of_max_radian_position = static_cast<int32_t>(stoi(strs.at(1)));
        } else if (strs.at(0) == "value_of_min_radian_position") {
          temp_dxl_info.value_of_min_radian_position = static_cast<int32_t>(stoi(strs.at(1)));
        } else if (strs.at(0) == "min_radian") {
          temp_dxl_info.min_radian = static_cast<double>(stod(strs.at(1)));
        } else if (strs.at(0) == "max_radian") {
          temp_dxl_info.max_radian = static_cast<double>(stod(strs.at(1)));
        }
      } catch (const std::exception & e) {
        std::string error_msg = "Error processing line in model file: " + line +
          "\nError: " + e.what();
        throw std::runtime_error(error_msg);
      }
    }
  }

  if (unit_info_found) {
    getline(open_file, line);  // Skip header line
    while (!open_file.eof() ) {
      getline(open_file, line);
      if (strcmp(line.c_str(), "[control table]") == 0) {
        control_table_found = true;
        break;
      }

      std::vector<std::string> strs;
      boost::split(strs, line, boost::is_any_of("\t"));

      if (strs.size() < 4) {
        continue;
      }

      try {
        std::string data_name = strs.at(0);
        double unit_value = static_cast<double>(stod(strs.at(1)));
        std::string sign_type_str = strs.at(3);
        bool is_signed = (sign_type_str == "signed");
        temp_dxl_info.unit_map[data_name] = unit_value;
        temp_dxl_info.sign_type_map[data_name] = is_signed;
        if (strs.size() >= 5) {
          double offset_value = static_cast<double>(stod(strs.at(4)));
          temp_dxl_info.offset_map[data_name] = offset_value;
        } else {
          temp_dxl_info.offset_map[data_name] = 0.0;
        }
      } catch (const std::exception & e) {
        std::string error_msg = "Error processing unit info line: " + line +
          "\nError: " + e.what();
        throw std::runtime_error(error_msg);
      }
    }
  }

  if (!control_table_found) {
    std::string error_msg = "No [control table] section found in model file for ID " +
      std::to_string(id);
    throw std::runtime_error(error_msg);
  }

  getline(open_file, line);
  while (!open_file.eof() ) {
    getline(open_file, line);
    if (!open_file.good()) {
      break;
    }

    std::vector<std::string> strs;
    boost::split(strs, line, boost::is_any_of("\t"));

    if (strs.size() < 3) {
      std::string error_msg = "Malformed control table line: " + line;
      throw std::runtime_error(error_msg);
    }

    try {
      ControlItem temp;
      temp.address = static_cast<uint16_t>(stoi(strs.at(0)));
      temp.size = static_cast<uint8_t>(stoi(strs.at(1)));
      temp.item_name = strs.at(2);
      temp_dxl_info.item.push_back(temp);
    } catch (const std::exception & e) {
      std::string error_msg = "Error processing control table line: " + line +
        "\nError: " + e.what();
      throw std::runtime_error(error_msg);
    }
  }

  if (temp_dxl_info.item.empty()) {
    std::string error_msg = "No control table items found in model file for ID " +
      std::to_string(id);
    throw std::runtime_error(error_msg);
  }

  dxl_info_by_comm_[comm_id][id] = temp_dxl_info;
  open_file.close();
}

std::string DynamixelInfo::SelectModelFileByFirmwareVersion(
  const std::string & base_model_name,
  uint8_t firmware_version)
{
  // If firmware version is 0 (unknown), use the base model file
  if (firmware_version == 0) {
    return base_model_name;
  }

  // Extract base name without extension
  std::string base_name = base_model_name;
  size_t dot_pos = base_name.find_last_of('.');
  if (dot_pos != std::string::npos) {
    base_name = base_name.substr(0, dot_pos);
  }

  // Scan directory for firmware-specific model files
  std::vector<std::string> available_firmware_versions;
  std::string search_path = dxl_model_file_dir + "/";

  DIR * dir = opendir(search_path.c_str());
  if (dir != nullptr) {
    struct dirent * entry;
    std::string prefix = base_name + "_fw";
    std::string suffix = ".model";

    while ((entry = readdir(dir)) != nullptr) {
      std::string filename = entry->d_name;
      if (filename.find(prefix) == 0 &&
        filename.find(suffix) == filename.length() - suffix.length())
      {
        available_firmware_versions.push_back(filename);
      }
    }
    closedir(dir);
  }

  if (available_firmware_versions.empty()) {
    // fprintf(stderr,
    //   "[Firmware Version Selection] No firmware-specific files found for %s, using base model\n",
    //   base_model_name.c_str());
    return base_model_name;
  }

  // fprintf(stderr, "[Firmware Version Selection] Found %zu firmware-specific files for %s\n",
  //         available_firmware_versions.size(), base_model_name.c_str());

  // Find the highest firmware version file that is <= device firmware version
  std::string selected_file = base_model_name;  // Default to base model
  int highest_fw_version = -1;
  std::string highest_fw_file;

  for (const auto & fw_file : available_firmware_versions) {
    uint8_t fw_version = ExtractFirmwareVersionFromFilename(fw_file);
    if (fw_version > highest_fw_version) {
      highest_fw_version = fw_version;
      highest_fw_file = fw_file;
    }
    if (fw_version <= firmware_version &&
      fw_version > ExtractFirmwareVersionFromFilename(selected_file))
    {
      selected_file = fw_file;
    }
  }

  // If device FW is greater than the highest available firmware-specific file, use base model
  if (firmware_version > highest_fw_version) {
    // fprintf(
    //   stderr,
    //   "[Firmware Version Selection] Device FW: %d > "
    //   "highest firmware-specific file FW: %d, using base model.\n",
    //   firmware_version, highest_fw_version);
    return base_model_name;
  }

  fprintf(
    stderr,
    "[NOTICE] Your DYNAMIXEL is not using the latest firmware."
    " For full performance, please download the latest DYNAMIXEL Wizard 2.0"
    " and update your DYNAMIXEL firmware."
    " See: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/\n");

  // Otherwise, use the highest firmware-specific file <= device FW
  fprintf(
    stderr, "[Firmware Version Selection] Device FW: %d, Selected Model: %s (FW: %d)\n",
    firmware_version, selected_file.c_str(),
    ExtractFirmwareVersionFromFilename(selected_file));
  return selected_file;
}

uint8_t DynamixelInfo::ExtractFirmwareVersionFromFilename(const std::string & filename)
{
  std::regex fw_pattern(R"(_fw(\d+)\.model$)");
  std::smatch match;

  if (std::regex_search(filename, match, fw_pattern) && match.size() > 1) {
    return static_cast<uint8_t>(std::stoi(match[1].str()));
  }

  return 0;  // Return 0 if no firmware version found
}

bool DynamixelInfo::GetDxlControlItem(
  uint8_t comm_id, uint8_t id, std::string item_name, uint16_t & addr,
  uint8_t & size)
{
  auto cit = dxl_info_by_comm_.find(comm_id);
  if (cit != dxl_info_by_comm_.end()) {
    auto iit = cit->second.find(id);
    if (iit != cit->second.end()) {
      for (size_t i = 0; i < iit->second.item.size(); i++) {
        if (strcmp(item_name.c_str(), iit->second.item.at(i).item_name.c_str()) == 0) {
          addr = iit->second.item.at(i).address;
          size = iit->second.item.at(i).size;
          return true;
        }
      }
    }
  }
  return false;
}

bool DynamixelInfo::CheckDxlControlItem(uint8_t comm_id, uint8_t id, std::string item_name)
{
  auto cit = dxl_info_by_comm_.find(comm_id);
  if (cit != dxl_info_by_comm_.end()) {
    auto iit = cit->second.find(id);
    if (iit != cit->second.end()) {
      for (size_t i = 0; i < iit->second.item.size(); i++) {
        if (strcmp(item_name.c_str(), iit->second.item.at(i).item_name.c_str()) == 0) {
          return true;
        }
      }
    }
  }
  return false;
}

int32_t DynamixelInfo::ConvertRadianToValue(uint8_t comm_id, uint8_t id, double radian)
{
  const auto & info = dxl_info_by_comm_[comm_id][id];
  if (radian > 0) {
    return static_cast<int32_t>(radian *
           (info.value_of_max_radian_position -
           info.value_of_zero_radian_position) / info.max_radian) +
           info.value_of_zero_radian_position;
  } else if (radian < 0) {
    return static_cast<int32_t>(radian *
           (info.value_of_min_radian_position -
           info.value_of_zero_radian_position) / info.min_radian) +
           info.value_of_zero_radian_position;
  } else {
    return info.value_of_zero_radian_position;
  }
}

double DynamixelInfo::ConvertValueToRadian(uint8_t comm_id, uint8_t id, int32_t value)
{
  const auto & info = dxl_info_by_comm_[comm_id][id];
  if (value > info.value_of_zero_radian_position) {
    return static_cast<double>(value - info.value_of_zero_radian_position) *
           info.max_radian /
           static_cast<double>(info.value_of_max_radian_position -
           info.value_of_zero_radian_position);
  } else if (value < info.value_of_zero_radian_position) {
    return static_cast<double>(value - info.value_of_zero_radian_position) *
           info.min_radian /
           static_cast<double>(info.value_of_min_radian_position -
           info.value_of_zero_radian_position);
  } else {
    return 0.0;
  }
}

bool DynamixelInfo::GetDxlUnitValue(
  uint8_t comm_id, uint8_t id, std::string data_name,
  double & unit_value)
{
  auto cit = dxl_info_by_comm_.find(comm_id);
  if (cit != dxl_info_by_comm_.end()) {
    auto iit = cit->second.find(id);
    if (iit != cit->second.end()) {
      auto it = iit->second.unit_map.find(data_name);
      if (it != iit->second.unit_map.end()) {
        unit_value = it->second;
        return true;
      }
    }
  }
  return false;
}

bool DynamixelInfo::GetDxlSignType(
  uint8_t comm_id, uint8_t id, std::string data_name,
  bool & is_signed)
{
  auto cit = dxl_info_by_comm_.find(comm_id);
  if (cit != dxl_info_by_comm_.end()) {
    auto iit = cit->second.find(id);
    if (iit != cit->second.end()) {
      auto it = iit->second.sign_type_map.find(data_name);
      if (it != iit->second.sign_type_map.end()) {
        is_signed = it->second;
        return true;
      }
    }
  }
  return false;
}

bool DynamixelInfo::GetDxlOffsetValue(
  uint8_t comm_id, uint8_t id, std::string data_name,
  double & offset_value)
{
  auto cit = dxl_info_by_comm_.find(comm_id);
  if (cit != dxl_info_by_comm_.end()) {
    auto iit = cit->second.find(id);
    if (iit != cit->second.end()) {
      auto it = iit->second.offset_map.find(data_name);
      if (it != iit->second.offset_map.end()) {
        offset_value = it->second;
        return true;
      }
    }
  }
  return false;
}

double DynamixelInfo::GetUnitMultiplier(uint8_t comm_id, uint8_t id, std::string data_name)
{
  auto & info = dxl_info_by_comm_[comm_id][id];
  auto it = info.unit_map.find(data_name);
  if (it != info.unit_map.end()) {return it->second;}
  return 1.0;
}

std::string DynamixelInfo::GetModelName(uint16_t model_number) const
{
  auto it = dxl_model_list_.find(model_number);
  if (it != dxl_model_list_.end()) {
    return it->second;
  }
  return "unknown";
}

}  // namespace friction_dynamixel_hardware_interface
