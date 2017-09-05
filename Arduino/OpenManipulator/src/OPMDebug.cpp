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

#include "OPMDebug.h"

void showLedStatus(void)
{
  static uint32_t t_time = millis();

  if ((millis()-t_time) >= 500 )
  {
    t_time = millis();
    digitalWrite(13, !digitalRead(13));
  }

  if (getPowerInVoltage() < 11.1)
  {
    setLedOn(2);
  }
  else
  {
    setLedOff(2);
  }

  if (getUsbConnected() > 0)
  {
    setLedOn(3);
  }
  else
  {
    setLedOff(3);
  }

  updateRxTxLed();
}

void updateRxTxLed(void)
{
  static uint32_t rx_led_update_time;
  static uint32_t tx_led_update_time;
  static uint32_t rx_cnt;
  static uint32_t tx_cnt;


  if ((millis()-tx_led_update_time) > 50)
  {
    tx_led_update_time = millis();

    if (tx_cnt != Serial.getTxCnt())
    {
      setLedToggle(0);
    }
    else
    {
      setLedOff(0);
    }

    tx_cnt = Serial.getTxCnt();
  }

  if( (millis()-rx_led_update_time) > 50 )
  {
    rx_led_update_time = millis();

    if (rx_cnt != Serial.getRxCnt())
    {
      setLedToggle(1);
    }
    else
    {
      setLedOff(1);
    }

    rx_cnt = Serial.getRxCnt();
  }
}

void print_mt3f(const Eigen::Matrix3f& m)
{
  uint8_t i = 0 , j = 0;

  for (i=0; i<3; i++)
  {
      for (j=0; j<3; j++)
      {
          Serial.print(m(i,j), 3);   // print 6 decimal places
          Serial.print(", ");
      }
      Serial.println();
  }
  Serial.println();
}

void print_vt3f(const Eigen::Vector3f& v)
{
  uint8_t i = 0;

  for (i=0; i<3; i++)
  {
    Serial.print(v(i), 3);
    Serial.print(", ");
  }
  Serial.println();
}

void print_mtXf(const Eigen::MatrixXf& m)
{
  uint8_t i = 0 , j = 0;

  for (i=0; i<m.rows(); i++)
  {
      for (j=0; j<m.cols(); j++)
      {
          Serial.print(m(i,j), 3);   // print 6 decimal places
          Serial.print(", ");
      }
      Serial.println();
  }
  Serial.println();
}

void print_vtXf(const Eigen::VectorXf& v)
{
  uint8_t i = 0;

  for (i=0; i<v.size(); i++)
  {
    Serial.print(v(i), 3);
    Serial.print(", ");
  }
  Serial.println();
}

void showJointAngle(String unit, OPMLink* link, int from, int to)
{
  int num = 0;

  if (unit == "rad")
  {
    for (num = from; num <= to; num++)
    {
      Serial.print(link[num].joint_angle_); 
      Serial.print(" ");
    }
    Serial.println("");
  }
  else if (unit == "deg")
  {
    for (num = from; num <= to; num++)
    {
      Serial.print(link[num].joint_angle_*RAD2DEG);
      Serial.print(" ");
    }
    Serial.println("");
  }
}

void showFKResult(OPMLink* link, int from, int to)
{
  int num = 0;

  for (num = from; num <= to; num++)
  {
    Serial.print("link : "); Serial.println(link[num].name_);
    Serial.println("p_ : "); print_vt3f(link[num].p_);
    Serial.println("R_ : "); print_mt3f(link[num].R_);
  }
}

// void showJointProp(float* get_joint_pos, float* get_joint_vel, float* get_joint_acc, int from, int to)
// {
//   int num = 0;

//   Serial.print("pos : ");
//   for (num = from; num <= to; num++)
//   {
//     Serial.print(get_joint_pos[num], 3); 
//     Serial.print(" ");
//   }
//   Serial.println(" ");

//   Serial.print("vel : ");
//   for (num = from; num <= to; num++)
//   {
//     Serial.print(get_joint_vel[num], 3); 
//     Serial.print(" ");
//   }
//   Serial.println(" ");

//   Serial.print("acc : ");
//   for (num = from; num <= to; num++)
//   {
//     Serial.print(get_joint_acc[num], 3); 
//     Serial.print(" ");
//   }
//   Serial.println(" ");  
// }