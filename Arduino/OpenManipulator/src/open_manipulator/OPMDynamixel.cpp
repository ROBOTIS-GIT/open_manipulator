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

OPMDynamixel::OPMDynamixel()
{
  dxl_cnt_ = 0;
}

OPMDynamixel::~OPMDynamixel()
{
}

bool OPMDynamixel::begin(char* device_name, uint32_t baud_rate)
{
  bool error = 0;

  error = driver_.begin(device_name, baud_rate);
  
  if (error == false)
    dxl_cnt_ = driver_.scan(dxl_, 10);
}

void OPMDynamixel::setMode()
{
  for (int id = 0; id < dxl_cnt_; id++)
    driver_.writeRegister(dxl_[id], "Operating Mode", 3);
}

void OPMDynamixel::setMode(uint8_t id, uint32_t mode)
{
  driver_.writeRegister(id, "Operating Mode", mode);
}

void OPMDynamixel::setTorque(bool onoff)
{
  for (int id = 0; id < dxl_cnt_; id++)
    driver_.writeRegister(dxl_[id], "Torque Enable", (uint32_t)onoff);
}

void OPMDynamixel::setSyncWrite(char* item_name)
{
  driver_.addSyncWrite(item_name);
}

void OPMDynamixel::setSyncRead(char* item_name)
{
  driver_.addSyncRead(item_name);
}

void OPMDynamixel::writeCur(uint8_t id, int16_t data)
{
  driver_.writeRegister(id, "Goal Current", (uint32_t)data);
}

void OPMDynamixel::writePos(uint8_t id, int32_t data)
{
  driver_.writeRegister(id, "Goal Position", (uint32_t)data);
}

void OPMDynamixel::writePos(int32_t *data)
{
  driver_.syncWrite("Goal Position", data);
}

void OPMDynamixel::readPos(int32_t *data)
{
  driver_.syncRead("Present Position", data);
}

int32_t OPMDynamixel::convertRadian2Value(int8_t id, float radian)
{
  int32_t value = 0;
  
  value = driver_.convertRadian2Value(id, radian);

  return value;
}

float OPMDynamixel::convertValue2Radian(int8_t id, int32_t value)
{
  float radian = 0.0;

  radian = driver_.convertValue2Radian(id, value);

  return radian;
}