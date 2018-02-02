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

#include "../../include/open_manipulator/OPMDynamixel.h"

OPMDynamixel::OPMDynamixel() : dxl_cnt_(0) {}

OPMDynamixel::~OPMDynamixel() {}

bool OPMDynamixel::begin(const char* device_name, uint32_t baud_rate)
{
  dxl_wb_.begin(device_name, baud_rate);
  
  dxl_wb_.scan(dxl_id_, &dxl_cnt_, 10);
}

void OPMDynamixel::setMode()
{
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_.jointMode(dxl_id_[index]);
}

void OPMDynamixel::setMode(uint8_t id, uint32_t mode)
{
  dxl_wb_.itemWrite(id, "Operating_Mode", mode);
}

void OPMDynamixel::setTorque(bool onoff)
{
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_.itemWrite(dxl_id_[index], "Torque_Enable", onoff);
}

void OPMDynamixel::setSyncWrite(const char* item_name)
{
  dxl_wb_.addSyncWrite(item_name);
}

void OPMDynamixel::setSyncRead(const char* item_name)
{
  dxl_wb_.addSyncRead(item_name);
}

void OPMDynamixel::writeCur(uint8_t id, int16_t data)
{
  dxl_wb_.itemWrite(id, "Goal_Current", (uint32_t)data);
}

void OPMDynamixel::writePos(uint8_t id, int32_t data)
{
  dxl_wb_.itemWrite(id, "Goal_Position", (uint32_t)data);
}

void OPMDynamixel::writePos(int32_t *data)
{
  dxl_wb_.syncWrite("Goal_Position", data);
}

int32_t* OPMDynamixel::readPos()
{
  return dxl_wb_.syncRead("Present_Position");
}

int32_t OPMDynamixel::convertRadian2Value(uint8_t id, float radian)
{  
  return dxl_wb_.convertRadian2Value(id, radian);
}

float OPMDynamixel::convertValue2Radian(uint8_t id, int32_t value)
{
  return dxl_wb_.convertValue2Radian(id, value);
}