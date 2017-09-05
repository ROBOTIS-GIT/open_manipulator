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

#include "OPMMath.h"

OPMMath::OPMMath(){}

OPMMath::~OPMMath(){}

Eigen::Matrix3f OPMMath::skew(Eigen::Vector3f v)
{
  Eigen::Matrix3f skew_symmetric_matrix = Eigen::Matrix3f::Zero();

  skew_symmetric_matrix <<    0,  -v(2),  v(1),
                           v(2),      0, -v(0),
                          -v(1),   v(0),     0;

  return skew_symmetric_matrix;
}

float OPMMath::sign(float num)
{
  if (num >= 0.0)
  {
    return 1.0;
  }
  else
  {
    return -1.0;
  }
}

Eigen::Matrix3f OPMMath::Rodrigues(Eigen::Vector3f axis, float angle)
{
  Eigen::Matrix3f skew_symmetric_matrix = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f Identity_matrix = Eigen::Matrix3f::Identity();

  skew_symmetric_matrix = skew(axis);
  rotation_matrix = Identity_matrix +
                    skew_symmetric_matrix * sin(angle) +
                    skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));

  return rotation_matrix;
}

Eigen::Matrix3f OPMMath::RotationMatrix(String notation, float angle)
{
  String roll  = "roll";
  String pitch = "pitch";
  String yaw   = "yaw";

  Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();

  if (notation.equals(roll))
  {
    rotation_matrix << 1.000,  0.000,      0.000,
                       0.000,  cos(angle), sin(angle),
                       0.000, -sin(angle), cos(angle);
  }
  else if (notation.equals(pitch))
  {
    rotation_matrix <<  cos(angle),  0.000, sin(angle),
                        0.000,       1.000, 0.000,
                       -sin(angle),  0.000, cos(angle);
  }
  else if (notation.equals(yaw))
  {
    rotation_matrix << cos(angle), -sin(angle), 0.000,
                       sin(angle),  cos(angle), 0.000,
                       0.000,       0.000,      1.000;
  }
  else
  {
    rotation_matrix = Eigen::Matrix3f::Identity();
  }

  return rotation_matrix;
}

Eigen::Vector3f OPMMath::AngularVelocity(Eigen::Matrix3f rotation_matrix)
{
  Eigen::Matrix3f R = rotation_matrix;
  Eigen::Vector3f l = Eigen::Vector3f::Zero();
  Eigen::Vector3f w = Eigen::Vector3f::Zero();

  float theta = 0.0;
  float diag  = 0.0;
  bool diagonal_matrix = true;

  l << R(2,1) - R(1,2),
       R(0,2) - R(2,0),
       R(1,0) - R(0,1);
  theta = atan2(l.norm(), R(0,0) + R(1,1) + R(2,2) - 1);
  diag  = R(0,0) + R(1,1) + R(2,2);

  for (int8_t i = 0; i < 3; i++)
  {
    for (int8_t j = 0; j < 3; j++)
    {
      if (i != j)
      {
        if (R(i, j) != 0)
        {
          diagonal_matrix = false;
        }
      }
    }
  }

  if (R == Eigen::Matrix3f::Identity())
  {
    w = Eigen::Vector3f::Zero();
  }
  else if (diagonal_matrix == true)
  {
    w << R(0,0) + 1, R(1,1) + 1, R(2,2) + 1;
    w = w * M_PI_2;
  }
  else
  {
    w = theta * (l / l.norm());
  }

  return w;
}

Eigen::Vector3f OPMMath::Verr(Eigen::Vector3f Cref, Eigen::Vector3f Cnow)
{
  Eigen::Vector3f get_Verr;

  get_Verr = Cref - Cnow;

  return get_Verr;
}

Eigen::Vector3f OPMMath::Werr(Eigen::Matrix3f Cref, Eigen::Matrix3f Cnow)
{
  Eigen::Matrix3f get_Rerr;
  Eigen::Vector3f get_Werr;

  get_Rerr = Cnow.transpose() * Cref;
  get_Werr = Cnow * AngularVelocity(get_Rerr);

  return get_Werr;
}

Eigen::VectorXf OPMMath::VWerr(Pose goal_pose, Eigen::Vector3f pos, Eigen::Matrix3f rot)
{
  Eigen::Vector3f get_Verr, get_Werr;
  Eigen::VectorXf get_VWerr(6);

  get_Verr = Verr(goal_pose.position, pos);
  get_Werr = Werr(goal_pose.orientation, rot);
  get_VWerr << get_Verr(0), get_Verr(1), get_Verr(2),
               get_Werr(0), get_Werr(1), get_Werr(2);

  return get_VWerr;
}

Eigen::MatrixXf OPMMath::Jacobian(OPMLink* link, Pose goal_pose, int8_t from, int8_t to)
{
  int8_t size = to-from+1;

  Eigen::MatrixXf J(6,size);
  Eigen::VectorXf pose_changed(6);
  Eigen::Vector3f joint_axis          = Eigen::Vector3f::Zero();
  Eigen::Vector3f position_changed    = Eigen::Vector3f::Zero();
  Eigen::Vector3f orientation_changed = Eigen::Vector3f::Zero();

  for (int8_t id = from; id <= to; id++)
  {
    int8_t mother = link[id].mother_;
    if (mother == -1)
      continue;

    joint_axis          = link[mother].R_ * link[id].joint_axis_;

    position_changed    = skew(joint_axis) * (goal_pose.position - link[id].p_);
    orientation_changed = joint_axis;

    pose_changed <<    position_changed(0),    position_changed(1),    position_changed(2),
                    orientation_changed(0), orientation_changed(1), orientation_changed(2);

    J.col(id-1) = pose_changed;
  }

  return J;
}
