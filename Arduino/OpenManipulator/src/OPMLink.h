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

#ifndef OPMLINK_H_
#define OPMLINK_H_

#include <unistd.h>
#include <WString.h>
#include <Eigen.h>       

class OPMLink
{
 public:
  String name_;           
  int8_t mother_;
  int8_t sibling_;
  int8_t child_;        
  Eigen::Vector3f p_;           // Position in World Coordinates
  Eigen::Matrix3f R_;           // Attitude in World Coordinates
  float joint_angle_;           // Joint Angle
  float joint_vel_;             // Joint Velocity
  float joint_acc_;             // Joint Acceleration
  Eigen::Vector3f joint_axis_;  // Joint Axis Vector
  Eigen::Vector3f joint_pos_;   // Joint Relative Position (Relative to Parent)

 public:
  OPMLink()
    : name_(""),
      mother_(-1),
      sibling_(-1),
      child_(-1),
      joint_angle_(0.0),
      joint_vel_(0.0),
      joint_acc_(0.0)
  {
    p_ = Eigen::Vector3f::Zero();
    R_ = Eigen::Matrix3f::Identity(3,3);

    joint_axis_ = Eigen::Vector3f::Zero();
    joint_pos_  = Eigen::Vector3f::Zero();
  }
};

#endif // OPMLINK_H_
