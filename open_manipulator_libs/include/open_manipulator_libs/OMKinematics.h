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

/* Authors: Hye-Jong KIM, Darby Lim */

#ifndef OMKINEMATICS_H_
#define OMKINEMATICS_H_

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

#include <math.h>
#include <vector>
#include <map>
#include <stdio.h>
#include "robotis_manipulator/RMAPI.h"

#define PI 3.141592

using namespace Eigen;

namespace OM_KINEMATICS
{
class Chain : public OPEN_MANIPULATOR::Kinematics
{
public:
  Chain(){};
  virtual ~Chain(){};

  virtual MatrixXf jacobian(RM_MANAGER::Manipulator *manipulator, Name tool_name);

  virtual void forward(RM_MANAGER::Manipulator *manipulator, Name component_name);
  virtual void forward(RM_MANAGER::Manipulator *manipulator);

  virtual std::vector<double> inverse(RM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);

  std::vector<double> inverseKinematics(RM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<double> srInverseKinematics(RM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<double> positionOnlyInverseKinematics(RM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);
};

} // namespace OM_KINEMATICS

#endif // OMKINEMATICS_HPP_
