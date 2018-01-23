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

#ifndef OPENMANIPULATOR_CHAIN_H_
#define OPENMANIPULATOR_CHAIN_H_

#define BASE    0
#define JOINT1  1
#define JOINT2  2
#define JOINT3  3
#define JOINT4  4
#define GRIP    5

#define LINK_NUM 6

OPMLink chain[LINK_NUM];

void initChain()
{
  chain[BASE].name_                      = "Base";
  chain[BASE].me_                        = BASE;
  chain[BASE].mother_                    = -1;
  chain[BASE].sibling_                   = -1;
  chain[BASE].child_                     = JOINT1;
  chain[BASE].p_                         = Eigen::Vector3f::Zero();
  chain[BASE].R_                         = Eigen::Matrix3f::Identity(3,3);
  chain[BASE].joint_angle_               = 0.0;
  chain[BASE].joint_vel_                 = 0.0;
  chain[BASE].joint_acc_                 = 0.0;
  chain[BASE].joint_axis_                = Eigen::Vector3f::Zero();
  chain[BASE].joint_pos_                 = Eigen::Vector3f::Zero();

  chain[JOINT1].name_                    = "Joint1";
  chain[JOINT1].me_                      = JOINT1;
  chain[JOINT1].mother_                  = BASE;
  chain[JOINT1].sibling_                 = -1;
  chain[JOINT1].child_                   = JOINT2;
  chain[JOINT1].p_                       = Eigen::Vector3f::Zero();
  chain[JOINT1].R_                       = Eigen::Matrix3f::Identity(3,3);
  chain[JOINT1].joint_angle_             = 0.0;
  chain[JOINT1].joint_vel_               = 0.0;
  chain[JOINT1].joint_acc_               = 0.0;
  chain[JOINT1].joint_axis_              << 0, 0, 1;
  chain[JOINT1].joint_pos_               << 0.012, 0, 0.036;

  chain[JOINT2].name_                    = "Joint2";
  chain[JOINT2].me_                      = JOINT2;
  chain[JOINT2].mother_                  = JOINT1;
  chain[JOINT2].sibling_                 = -1;
  chain[JOINT2].child_                   = JOINT3;
  chain[JOINT2].p_                       = Eigen::Vector3f::Zero();
  chain[JOINT2].R_                       = Eigen::Matrix3f::Identity(3,3);
  chain[JOINT2].joint_angle_             = 0.0;
  chain[JOINT2].joint_vel_               = 0.0;
  chain[JOINT2].joint_acc_               = 0.0;
  chain[JOINT2].joint_axis_              << 0, -1, 0;
  chain[JOINT2].joint_pos_               << 0, 0, 0.040;

  chain[JOINT3].name_                    = "Joint3";
  chain[JOINT3].me_                      = JOINT3;
  chain[JOINT3].mother_                  = JOINT2;
  chain[JOINT3].sibling_                 = -1;
  chain[JOINT3].child_                   = JOINT4;
  chain[JOINT3].p_                       = Eigen::Vector3f::Zero();
  chain[JOINT3].R_                       = Eigen::Matrix3f::Identity(3,3);
  chain[JOINT3].joint_angle_             = 0.0;
  chain[JOINT3].joint_vel_               = 0.0;
  chain[JOINT3].joint_acc_               = 0.0;
  chain[JOINT3].joint_axis_              << 0, -1, 0;
  chain[JOINT3].joint_pos_               << 0.022, 0, 0.122;

  chain[JOINT4].name_                    = "Joint4";
  chain[JOINT4].me_                      = JOINT4;
  chain[JOINT4].mother_                  = JOINT3;
  chain[JOINT4].sibling_                 = -1;
  chain[JOINT4].child_                   = GRIP;
  chain[JOINT4].p_                       = Eigen::Vector3f::Zero();
  chain[JOINT4].R_                       = Eigen::Matrix3f::Identity(3,3);
  chain[JOINT4].joint_angle_             = 0.0;
  chain[JOINT4].joint_vel_               = 0.0;
  chain[JOINT4].joint_acc_               = 0.0;
  chain[JOINT4].joint_axis_              << 0, -1, 0;
  chain[JOINT4].joint_pos_               << 0.124, 0, 0;

  chain[GRIP].name_                      = "Gripper";
  chain[GRIP].me_                        = GRIP;
  chain[GRIP].mother_                    = JOINT4;
  chain[GRIP].sibling_                   = -1;
  chain[GRIP].child_                     = -1;
  chain[GRIP].p_                         = Eigen::Vector3f::Zero();
  chain[GRIP].R_                         = Eigen::Matrix3f::Identity(3,3);
  chain[GRIP].joint_angle_               = 0.0;
  chain[GRIP].joint_vel_                 = 0.0;
  chain[GRIP].joint_acc_                 = 0.0;
  chain[GRIP].joint_axis_                = Eigen::Vector3f::Zero();
  chain[GRIP].joint_pos_                 << 0.119, 0, 0;
}

#endif //OPENMANIPULATOR_CHAIN_H_