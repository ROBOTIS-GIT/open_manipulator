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

#ifndef OPMMINIMUMJERK_H_
#define OPMMINIMUMJERK_H_

#include "OPMMath.h"
#include "OPMComm.h"

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.
#include <Eigen/Dense>

#include <math.h>

class OPMMinimumJerk
{
 public:
  Eigen::MatrixXf coeffi_;

 public:
  OPMMinimumJerk();
  ~OPMMinimumJerk();

  void setCoeffi(State* start, State* target, uint8_t link_num, float mov_time, float control_period);

  void getPosition(State* calc, uint8_t to, float tick);
  void getVelocity(State* calc, uint8_t to, float tick);
  void getAcceleration(State* calc, uint8_t to, float tick);
};

#endif // OPMMINIMUMJERK_H_
