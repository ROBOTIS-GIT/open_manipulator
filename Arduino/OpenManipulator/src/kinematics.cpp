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

#include "kinematics.h"
using namespace open_manipulator;

Kinematics::Kinematics()
{
  calc_ = new Calc();
}

Kinematics::~Kinematics(){}

/*******************************************************************************
* Forward kinematics
*******************************************************************************/
void Kinematics::forward(Link* link, int8_t me)
{
  int8_t mother = 0;

  if (me == -1)
  {
    return;
  }

  if (me != 0)
  {
    mother = link[me].mother_;
    link[me].p_ = link[mother].R_ * link[me].b_ + link[mother].p_;
    link[me].R_ = link[mother].R_ * calc_->Rodrigues(link[me].a_, link[me].q_);
  }

  forward(link, link[me].sibling_);
  forward(link, link[me].child_);
}

/*******************************************************************************
* Inverse kinematics (Numerical Method)
*******************************************************************************/
void Kinematics::inverse(Link* link, uint8_t to, Pose goal_pose, float lambda)
{
  //lambda :  To stabilize the numeric calculation (0 1]
  uint8_t size = to;

  Eigen::MatrixXf J(6,size);
  Eigen::VectorXf VWerr(6);
  Eigen::VectorXf dq(size);

  forward(link, BASE);

  for (int i = 0; i < 10; i++)
  {
    J = calc_->Jacobian(link, size, goal_pose);

    VWerr = calc_->VWerr(goal_pose, link[to].p_, link[to].R_);

    if (VWerr.norm() < 1E-6)
      return;

    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(J);
    dq = lambda * dec.solve(VWerr);

    setAngle(link, to, dq);
    forward(link, BASE);
  }
}

void Kinematics::sr_inverse(Link* link, uint8_t to, Pose goal_pose)
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
  VWerr = calc_->VWerr(goal_pose, link[to].p_, link[to].R_);
  Ek = VWerr.transpose() * We * VWerr;

  for (int i = 0; i < 10; i++)
  {
    J = calc_->Jacobian(link, size, goal_pose);
    lambda = Ek + 0.002;

    Jh = (J.transpose() * We * J) + (lambda * Wn);
    gerr = J.transpose() * We * VWerr;

    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(Jh);
    dq = dec.solve(gerr);

    setAngle(link, to, dq);
    forward(link, BASE);
    VWerr = calc_->VWerr(goal_pose, link[to].p_, link[to].R_);

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

void Kinematics::position_only_inverse(Link* link, uint8_t to, Pose goal_pose)
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
  Verr = calc_->Verr(goal_pose.position, link[to].p_);
  Ek = Verr.transpose() * We * Verr;

  for (int i = 0; i < 10; i++)
  {
    J = calc_->Jacobian(link, size, goal_pose);
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
    Verr = calc_->Verr(goal_pose.position, link[to].p_);

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

void Kinematics::setAngle(Link* link, uint8_t to, Eigen::VectorXf dq)
{
  for (int id = 1; id <= to; id++)
  {
    link[id].q_ = link[id].q_ + dq(id-1);
  }
}
