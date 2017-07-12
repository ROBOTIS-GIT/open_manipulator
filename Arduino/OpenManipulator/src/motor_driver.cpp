/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim */

#include "motor_driver.h"
using namespace open_manipulator;

MotorDriver::MotorDriver(float protocol_version, uint32_t baud_rate)
{
  protocol_version_ = protocol_version;
  baud_rate_        = baud_rate;
}

MotorDriver::~MotorDriver()
{
  close();
}

bool MotorDriver::init(Motor* motor, uint8_t motor_num)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud_rate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }

  getMotor(motor, motor_num);

  groupSyncWriteTorque_   = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_TORQUE_ENABLE,    LEN_X_TORQUE_ENABLE);
  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION,    LEN_X_GOAL_POSITION);

  groupSyncReadPosition_  = new dynamixel::GroupSyncRead (portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  return true;
}


void MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
}

void MotorDriver::getMotor(Motor* motor, uint8_t motor_num)
{
  motor_     = motor;
  motor_num_ = motor_num;
}

bool MotorDriver::setTorque(uint8_t onoff)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  uint8_t param_torque[4];

  for (int num = 0; num <= motor_num_; num++)
  {
    if (motor_[num].id == 0)
      continue;

    param_torque[0] = DXL_LOBYTE(DXL_LOWORD(onoff));
    param_torque[1] = DXL_HIBYTE(DXL_LOWORD(onoff));
    param_torque[2] = DXL_LOBYTE(DXL_HIWORD(onoff));
    param_torque[3] = DXL_HIBYTE(DXL_HIWORD(onoff));

    dxl_addparam_result_ = groupSyncWriteTorque_->addParam(motor_[num].id, (uint8_t*)&param_torque);
    if (dxl_addparam_result_ != true)
      return false;
  }

  dxl_comm_result_ = groupSyncWriteTorque_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWriteTorque_->clearParam();
  return true;
}

bool MotorDriver::jointControl(int32_t* value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t param_goal_position[4];

  for (int num = 0; num <= motor_num_; num++)
  {
    if (motor_[num].name.substring(0,5) == "Joint")
    {
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(value[num]));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(value[num]));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(value[num]));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(value[num]));

      dxl_addparam_result = groupSyncWritePosition_->addParam(motor_[num].id, (uint8_t*)&param_goal_position);
      if (dxl_addparam_result != true)
        return false;
    }
  }

  dxl_comm_result = groupSyncWritePosition_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    return false;
  }

  groupSyncWritePosition_->clearParam();
  return true;
}

bool MotorDriver::gripControl(int32_t value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t param_goal_position[4];

  for (int num = 0; num <= motor_num_; num++)
  {
    if (motor_[num].name.substring(0,7) == "Gripper")
    {
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(value));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(value));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(value));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(value));

      dxl_addparam_result = groupSyncWritePosition_->addParam(motor_[num].id, (uint8_t*)&param_goal_position);
      if (dxl_addparam_result != true)
        return false;
    }
  }

  dxl_comm_result = groupSyncWritePosition_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    return false;
  }

  groupSyncWritePosition_->clearParam();
  return true;
}

bool MotorDriver::readPosition(Motor* get_motor)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  int32_t read_value[motor_num_] = {0, };

  for (int num = 0; num <= motor_num_; num++)
  {
    if (get_motor[num].id == 0)
      continue;

    // Set parameter
    dxl_addparam_result = groupSyncReadPosition_->addParam(get_motor[num].id);
    if (dxl_addparam_result != true)
      return false;
  }

  // Syncread present position
  dxl_comm_result = groupSyncReadPosition_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print("comm failed");
    packetHandler_->printTxRxResult(dxl_comm_result);
  }

  for (int num = 0; num <= motor_num_; num++)
  {
    if (get_motor[num].id == 0)
      continue;

    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadPosition_->isAvailable(get_motor[num].id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
      return false;

    // Get data
    read_value[num] = groupSyncReadPosition_->getData(get_motor[num].id,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    get_motor[num].present_position = convertValue2Radian(read_value[num]);
  }

  groupSyncReadPosition_->clearParam();
  return true;
}

int32_t MotorDriver::convertRadian2Value(float radian)
{
  int32_t value = 0;

  if (radian > 0)
  {
    if (VALUE_OF_MAX_RADIAN_POSITION <= VALUE_OF_ZERO_RADIAN_POSITION)
      return VALUE_OF_MAX_RADIAN_POSITION;

    value = (radian * (VALUE_OF_MAX_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION) / MAX_RADIAN)
                + VALUE_OF_ZERO_RADIAN_POSITION;
  }
  else if (radian < 0)
  {
    if (VALUE_OF_MIN_RADIAN_POSITION >= VALUE_OF_ZERO_RADIAN_POSITION)
      return VALUE_OF_MIN_RADIAN_POSITION;

    value = (radian * (VALUE_OF_MIN_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION) / MIN_RADIAN)
                + VALUE_OF_ZERO_RADIAN_POSITION;
  }
  else
  {
    value = VALUE_OF_ZERO_RADIAN_POSITION;
  }
  // if (value[id-1] > VALUE_OF_MAX_RADIAN_POSITION)
  //   value[id-1] =  VALUE_OF_MAX_RADIAN_POSITION;
  // else if (value[id-1] < VALUE_OF_MIN_RADIAN_POSITION)
  //   value[id-1] =  VALUE_OF_MIN_RADIAN_POSITION;

  return value;
}

float MotorDriver::convertValue2Radian(int32_t value)
{
  float radian = 0.0;

  if (value > VALUE_OF_ZERO_RADIAN_POSITION)
  {
    if (MAX_RADIAN <= 0)
      return MAX_RADIAN;

    radian = (float) (value - VALUE_OF_ZERO_RADIAN_POSITION) * MAX_RADIAN
               / (float) (VALUE_OF_MAX_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION);
  }
  else if (value < VALUE_OF_ZERO_RADIAN_POSITION)
  {
    if (MIN_RADIAN >= 0)
      return MIN_RADIAN;

    radian = (float) (value - VALUE_OF_ZERO_RADIAN_POSITION) * MIN_RADIAN
               / (float) (VALUE_OF_MIN_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION);
  }
  //  if (radian[id-1] > MAX_RADIAN)
  //    radian[id-1] =  MAX_RADIAN;
  //  else if (radian[id-1] < MIN_RADIAN)
  //    radian[id-1] =  MIN_RADIAN;

  return radian;
}
