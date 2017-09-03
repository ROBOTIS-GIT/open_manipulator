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

#include "OPMMinimumJerk.h"

OPMMinimumJerk::OPMMinimumJerk(){}

OPMMinimumJerk::~OPMMinimumJerk(){}

void OPMMinimumJerk::setCoeffi(State* start, State* target, uint8_t link_num, float mov_time, float control_period)
{
  uint16_t step_time = uint16_t(floor(mov_time/control_period) + 1.0);
  mov_time = float(step_time - 1) * control_period;

  coeffi_.resize(6, link_num);

  Eigen::Matrix3f A = Eigen::Matrix3f::Identity(3,3);
  Eigen::Vector3f x = Eigen::Vector3f::Zero();
  Eigen::Vector3f b = Eigen::Vector3f::Zero();

  A <<     pow(mov_time,3),     pow(mov_time,4),     pow(mov_time,5),
       3 * pow(mov_time,2), 4 * pow(mov_time,3), 5 * pow(mov_time,4),
       6 * pow(mov_time,1), 12* pow(mov_time,2), 20* pow(mov_time,3);

  for (int8_t num = 0; num < link_num; num++)
  {
    Eigen::VectorXf single_coeffi(6);

    single_coeffi(0) =       start[num].pos;
    single_coeffi(1) =       start[num].vel;
    single_coeffi(2) = 0.5 * start[num].acc;

    b << (target[num].pos - start[num].pos - (start[num].vel * mov_time + 0.5 * start[num].acc * pow(mov_time,2))),
         (target[num].vel - start[num].vel - (start[num].acc * mov_time)),
         (target[num].acc - start[num].acc);

    Eigen::ColPivHouseholderQR<Eigen::Matrix3f> dec(A);
    x = dec.solve(b);

    single_coeffi(3) = x(0);
    single_coeffi(4) = x(1);
    single_coeffi(5) = x(2);

    coeffi_.col(num) = single_coeffi;
  }  
}

void OPMMinimumJerk::getPosition(State* calc, uint8_t to, float tick)
{
  for (int num = 0; num <= to; num++)
  {
    calc[num].pos = coeffi_(0,num)             +
                    coeffi_(1,num)*pow(tick,1) +
                    coeffi_(2,num)*pow(tick,2) +
                    coeffi_(3,num)*pow(tick,3) +
                    coeffi_(4,num)*pow(tick,4) +
                    coeffi_(5,num)*pow(tick,5);
  }
}

void OPMMinimumJerk::getVelocity(State* calc, uint8_t to, float tick)
{
  for (int num = 0; num <= to; num++)
  {
    calc[num].vel =   coeffi_(1,num)             +
                    2*coeffi_(2,num)*pow(tick,1) +
                    3*coeffi_(3,num)*pow(tick,2) +
                    4*coeffi_(4,num)*pow(tick,3) +
                    5*coeffi_(5,num)*pow(tick,4);
  }
}

void OPMMinimumJerk::getAcceleration(State* calc, uint8_t to, float tick)
{
  for (int num = 0; num <= to; num++)
  {
    calc[num].acc = 2 *coeffi_(2,num)             +
                    6 *coeffi_(3,num)*pow(tick,1) +
                    12*coeffi_(4,num)*pow(tick,2) +
                    20*coeffi_(5,num)*pow(tick,3);
  }
}
