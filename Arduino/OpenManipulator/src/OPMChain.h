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

#ifndef OPMCHAIN_H_
#define OPMCHAIN_H_

#define LINK_NUM  6
#define JOINT_NUM 4
#define GRIP_NUM  1

#define DXL_NUM 5

#define BASE    0
#define JOINT1  1
#define JOINT2  2
#define JOINT3  3
#define JOINT4  4
#define GRIP    5

#define MOV_TIME 2.5

const float grip_on  = 1.3;
const float grip_off = 0.0;

OPMLink chain[LINK_NUM];

static void initChain()
{
  chain[BASE].name_                      = "Base";
  chain[BASE].mother_                    = -1;
  chain[BASE].sibling_                   = -1;
  chain[BASE].child_                     = 1;
  chain[BASE].p_                         = Eigen::Vector3f::Zero();
  chain[BASE].R_                         = Eigen::Matrix3f::Identity(3,3);
  chain[BASE].joint_angle_               = 0.0;
  chain[BASE].joint_vel_                 = 0.0;
  chain[BASE].joint_acc_                 = 0.0;
  chain[BASE].joint_axis_                = Eigen::Vector3f::Zero();
  chain[BASE].joint_pos_                 = Eigen::Vector3f::Zero();

  chain[JOINT1].name_                    = "Joint1";
  chain[JOINT1].mother_                  = 0;
  chain[JOINT1].sibling_                 = -1;
  chain[JOINT1].child_                   = 2;
  chain[JOINT1].p_                       = Eigen::Vector3f::Zero();
  chain[JOINT1].R_                       = Eigen::Matrix3f::Identity(3,3);
  chain[JOINT1].joint_angle_             = 0.0;
  chain[JOINT1].joint_vel_               = 0.0;
  chain[JOINT1].joint_acc_               = 0.0;
  chain[JOINT1].joint_axis_              << 0, 0, 1;
  chain[JOINT1].joint_pos_               << 0.012, 0, 0.036;

  chain[JOINT2].name_                    = "Joint2";
  chain[JOINT2].mother_                  = 1;
  chain[JOINT2].sibling_                 = -1;
  chain[JOINT2].child_                   = 3;
  chain[JOINT2].p_                       = Eigen::Vector3f::Zero();
  chain[JOINT2].R_                       = Eigen::Matrix3f::Identity(3,3);
  chain[JOINT2].joint_angle_             = 0.0;
  chain[JOINT2].joint_vel_               = 0.0;
  chain[JOINT2].joint_acc_               = 0.0;
  chain[JOINT2].joint_axis_              << 0, -1, 0;
  chain[JOINT2].joint_pos_               << 0, 0, 0.040;

  chain[JOINT3].name_                    = "Joint3";
  chain[JOINT3].mother_                  = 2;
  chain[JOINT3].sibling_                 = -1;
  chain[JOINT3].child_                   = 4;
  chain[JOINT3].p_                       = Eigen::Vector3f::Zero();
  chain[JOINT3].R_                       = Eigen::Matrix3f::Identity(3,3);
  chain[JOINT3].joint_angle_             = 0.0;
  chain[JOINT3].joint_vel_               = 0.0;
  chain[JOINT3].joint_acc_               = 0.0;
  chain[JOINT3].joint_axis_              << 0, -1, 0;
  chain[JOINT3].joint_pos_               << 0.022, 0, 0.122;

  chain[JOINT4].name_                    = "Joint4";
  chain[JOINT4].mother_                  = 3;
  chain[JOINT4].sibling_                 = -1;
  chain[JOINT4].child_                   = 5;
  chain[JOINT4].p_                       = Eigen::Vector3f::Zero();
  chain[JOINT4].R_                       = Eigen::Matrix3f::Identity(3,3);
  chain[JOINT4].joint_angle_             = 0.0;
  chain[JOINT4].joint_vel_               = 0.0;
  chain[JOINT4].joint_acc_               = 0.0;
  chain[JOINT4].joint_axis_              << 0, -1, 0;
  chain[JOINT4].joint_pos_               << 0.124, 0, 0;

  chain[GRIP].name_                      = "Gripper";
  chain[GRIP].mother_                    = 4;
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

#endif //OPMCHAIN_H_