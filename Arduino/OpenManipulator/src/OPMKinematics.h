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

#ifndef OPMKINEMATICS_H_
#define OPMKINEMATICS_H_

#include "OPMLink.h"
#include "OPMComm.h"
#include "OPMMath.h"

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.
#include <Eigen/Dense>

#include <math.h>

class OPMKinematics
{
 private:
   OPMMath opm_math_;

 public:
  OPMKinematics();
  ~OPMKinematics();

  void forward(OPMLink* link, int8_t from);
  
  void inverse(OPMLink* link, uint8_t to, Pose goal_pose, float lambda = 0.7);
  void sr_inverse(OPMLink* link, uint8_t to, Pose goal_pose);
  void position_only_inverse(OPMLink* link, uint8_t to, Pose goal_pose);

 private:
  void setAngle(OPMLink* link, uint8_t to, Eigen::VectorXf dq);

};

#endif // OPMKINEMATICS_H_
