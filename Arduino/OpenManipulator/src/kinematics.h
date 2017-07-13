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

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "calc.h"

namespace open_manipulator
{
#define BASE 0
class Kinematics
{
 private:
   Calc *calc_;

 public:
  Kinematics();
  ~Kinematics();

  void setAngle(Link* link, uint8_t to, Eigen::VectorXf dq);

  void forward(Link* link, int8_t me);
  void inverse(Link* link, uint8_t to, Pose goal_pose, float lambda = 0.7);
  void sr_inverse(Link* link, uint8_t to, Pose goal_pose);
  void position_only_inverse(Link* link, uint8_t to, Pose goal_pose);
};
}

#endif // KINEMATICS_H_
