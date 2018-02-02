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

#ifndef OPMMATH_H_
#define OPMMATH_H_

#include "OPMComm.h"
#include "OPMLink.h"

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.

#include <math.h>

class OPMMath
{
 public:
  OPMMath();
  ~OPMMath();

  Eigen::Matrix3f skew(Eigen::Vector3f v);
  float sign(float num);

  Eigen::Matrix3f Rodrigues(Eigen::Vector3f axis, float angle);
  Eigen::Matrix3f RotationMatrix(String notation, float angle);
  Eigen::Vector3f Verr(Eigen::Vector3f Cref, Eigen::Vector3f Cnow);
  Eigen::Vector3f Werr(Eigen::Matrix3f Cref, Eigen::Matrix3f Cnow);
  Eigen::VectorXf VWerr(Pose goal_pos, Eigen::Vector3f pos, Eigen::Matrix3f rot);
  Eigen::Vector3f AngularVelocity(Eigen::Matrix3f rotation_matrix);
  Eigen::MatrixXf Jacobian(OPMLink* link, Pose goal_pose, int8_t from, int8_t to);
};

#endif // OPMMATH_H_
