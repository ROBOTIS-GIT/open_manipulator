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

#ifndef OPMDYNAMIXEL_H_
#define OPMDYNAMIXEL_H_

#include <DynamixelWorkbench.h>

#define DEVICENAME       "/dev/ttyUSB0"
#define BAUDRATE         1000000

class OPMDynamixel
{
 private:
  DynamixelWorkbench dxl_wb_;

 public:
  uint8_t dxl_cnt_;
  uint8_t dxl_id_[10];

 public:
 OPMDynamixel();
 ~OPMDynamixel();

 bool begin(const char* device_name = DEVICENAME, uint32_t baud_rate = BAUDRATE);
 void setMode();
 void setMode(uint8_t id, uint32_t mode);
 void setTorque(bool onoff);
 void setSyncWrite(const char* item_name = "Goal_Position");
 void setSyncRead(const char* item_name = "Present_Position");
 void writeCur(uint8_t id, int16_t data);
 void writePos(uint8_t id, int32_t data);
 void writePos(int32_t *data);
 int32_t* readPos();

 int32_t convertRadian2Value(uint8_t id, float radian);
 float   convertValue2Radian(uint8_t id, int32_t value);
};

#endif // OPMDYNAMIXEL_H_
