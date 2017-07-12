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

#ifndef LINK_H_
#define LINK_H_

#include <Arduino.h>
#include <Eigen.h>        // Calls main Eigen matrix class library

namespace open_manipulator
{

class Link
{
 public:
  String name_;           // Link name
  uint8_t mother_;
  uint8_t sibling_;
  uint8_t child_;
  float mass_;
  Eigen::Vector3f p_;     // Position in World Coordinates
  Eigen::Matrix3f R_;     // Attitue in World Coordinates
  float q_;               // Joint Angle
  float dq_;              // Joint Velocity
  float ddq_;             // Joint Acceleration
  Eigen::Vector3f a_;     // Joint Axis Vector
  Eigen::Vector3f b_;     // Joint Relative Position (Relative to Parent)
  Eigen::Vector3f v_;     // Linear Velocity in World Coordinates
  Eigen::Vector3f w_;     // Angular Velocity in World Coordinate

 public:
  Link()
    : name_(""),
      mother_(0),
      sibling_(0),
      child_(0),
      mass_(0.0),
      q_(0.0),
      dq_(0.0),
      ddq_(0.0)
  {
    p_ = Eigen::Vector3f::Zero();
    R_ = Eigen::Matrix3f::Identity(3,3);

    a_ = Eigen::Vector3f::Zero();
    b_ = Eigen::Vector3f::Zero();
    v_ = Eigen::Vector3f::Zero();
    w_ = Eigen::Vector3f::Zero();
  }
};
}

#endif // LINK_H_
