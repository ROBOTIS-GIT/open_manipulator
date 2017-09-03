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
  DynamixelDriver driver_;

 public:
  uint8_t dxl_cnt_;
  uint8_t dxl_[10];

 public:
 OPMDynamixel();
 ~OPMDynamixel();

 bool begin(char* device_name = DEVICENAME, uint32_t baud_rate = BAUDRATE);
 void findDynamixel();
 void setMode();
 void setMode(uint8_t id, uint32_t mode);
 void setTorque(bool onoff);
 void setSyncWrite(char* item_name = "Goal Position");
 void setSyncRead(char* item_name = "Present Position");
 void writePos(int32_t *data);
 void readPos(int32_t *data);

 int32_t convertRadian2Value(int8_t id, float radian);
 float   convertValue2Radian(int8_t id, int32_t value);
};

#endif // OPMDYNAMIXEL_H_
