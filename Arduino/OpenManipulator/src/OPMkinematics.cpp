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

#include "OPMKinematics.h"

#define BASE 0

OPMKinematics::OPMKinematics(){}

OPMKinematics::~OPMKinematics(){}

/*******************************************************************************
* Forward kinematics
*******************************************************************************/
void OPMKinematics::forward(OPMLink* link, int8_t from)
{
  int8_t mother = 0;  

  if (from == -1)
  {
    return;
  }

  if (from != 0)
  {
    mother = link[from].mother_;
    link[from].p_ = link[mother].R_ * link[from].joint_pos_ + link[mother].p_;
    link[from].R_ = link[mother].R_ * opm_math_.Rodrigues(link[from].joint_axis_, link[from].joint_angle_);
  }

  forward(link, link[from].sibling_);
  forward(link, link[from].child_);
}

/*******************************************************************************
* Inverse kinematics (Numerical Method)
*******************************************************************************/
void OPMKinematics::inverse(OPMLink* link, uint8_t to, Pose goal_pose, float lambda)
{
  //lambda :  To stabilize the numeric calculation (0 1]
  uint8_t size = to;

  Eigen::MatrixXf J(6,size);
  Eigen::VectorXf VWerr(6);
  Eigen::VectorXf dq(size);

  forward(link, BASE);

  for (int i = 0; i < 10; i++)
  {
    J = opm_math_.Jacobian(link, goal_pose, BASE, to);

    VWerr = opm_math_.VWerr(goal_pose, link[to].p_, link[to].R_);

    if (VWerr.norm() < 1E-6)
      return;

    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(J);
    dq = lambda * dec.solve(VWerr);

    setAngle(link, to, dq);
    forward(link, BASE);
  }
}

void OPMKinematics::sr_inverse(OPMLink* link, uint8_t to, Pose goal_pose)
{
  uint8_t size = to;

  float wn_pos = 1/0.3;
  float wn_ang = 1/(2*M_PI);
  float Ek     = 0.0;
  float Ek2    = 0.0;
  float lambda = 0.0;

  Eigen::MatrixXf J(6,size);
  Eigen::MatrixXf Jh(size,size);
  Eigen::VectorXf VWerr(6);
  Eigen::VectorXf gerr(size);
  Eigen::VectorXf dq(size);

  Eigen::MatrixXf We(6,6);
  We << wn_pos, 0,      0,      0,      0,      0,
        0,      wn_pos, 0,      0,      0,      0,
        0,      0,      wn_pos, 0,      0,      0,
        0,      0,      0,      wn_ang, 0,      0,
        0,      0,      0,      0,      wn_ang, 0,
        0,      0,      0,      0,      0,      wn_ang;

  Eigen::MatrixXf Wn(size, size);
  Wn = Eigen::MatrixXf::Identity(size, size);

  forward(link, BASE);
  VWerr = opm_math_.VWerr(goal_pose, link[to].p_, link[to].R_);
  Ek = VWerr.transpose() * We * VWerr;

  for (int i = 0; i < 10; i++)
  {
    J = opm_math_.Jacobian(link, goal_pose, BASE, to);
    lambda = Ek + 0.002;

    Jh = (J.transpose() * We * J) + (lambda * Wn);
    gerr = J.transpose() * We * VWerr;

    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(Jh);
    dq = dec.solve(gerr);

    setAngle(link, to, dq);
    forward(link, BASE);
    VWerr = opm_math_.VWerr(goal_pose, link[to].p_, link[to].R_);

    Ek2 = VWerr.transpose() * We * VWerr;

    if (Ek2 < 1E-12)
    {
      break;
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      setAngle(link, to, -dq);
      forward(link, BASE);
      break;
    }
  }
}

void OPMKinematics::position_only_inverse(OPMLink* link, uint8_t to, Pose goal_pose)
{
  uint8_t size = to;

  float wn_pos = 1/0.3;
  float wn_ang = 1/(2*M_PI);
  float Ek     = 0.0;
  float Ek2    = 0.0;
  float lambda = 0.0;

  Eigen::MatrixXf J(6,size);
  Eigen::MatrixXf Jpos(3,size);
  Eigen::MatrixXf Jh(3,size);
  Eigen::Vector3f Verr = Eigen::Vector3f::Zero();
  Eigen::VectorXf gerr(size);
  Eigen::VectorXf dq(size);

  Eigen::Matrix3f We;
  We << wn_pos, 0,      0,
        0,      wn_pos, 0,
        0,      0,      wn_pos;

  Eigen::MatrixXf Wn(size, size);
  Wn = Eigen::MatrixXf::Identity(size, size);

  forward(link, BASE);
  Verr = opm_math_.Verr(goal_pose.position, link[to].p_);
  Ek = Verr.transpose() * We * Verr;

  for (int i = 0; i < 10; i++)
  {
    J = opm_math_.Jacobian(link, goal_pose, BASE, to);
    Jpos.row(0) = J.row(0);
    Jpos.row(1) = J.row(1);
    Jpos.row(2) = J.row(2);
    lambda = Ek + 0.002;

    Jh = (Jpos.transpose() * We * Jpos) + (lambda * Wn);
    gerr = Jpos.transpose() * We * Verr;

    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(Jh);
    dq = dec.solve(gerr);

    setAngle(link, to, dq);
    forward(link, BASE);
    Verr = opm_math_.Verr(goal_pose.position, link[to].p_);

    Ek2 = Verr.transpose() * We * Verr;

    if (Ek2 < 1E-12)
    {
      break;
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      setAngle(link, to, -dq);
      forward(link, BASE);
      break;
    }
  }
}

void OPMKinematics::setAngle(OPMLink* link, uint8_t to, Eigen::VectorXf dq)
{
  for (int id = 1; id <= to; id++)
  {
    link[id].joint_angle_ = link[id].joint_angle_ + dq(id-1);
  }
}
