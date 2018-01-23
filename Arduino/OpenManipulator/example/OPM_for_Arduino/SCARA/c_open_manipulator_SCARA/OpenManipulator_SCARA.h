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

#ifndef OPENMANIPULATOR_SCARA_H_
#define OPENMANIPULATOR_SCARA_H_

#define BASE    0
#define JOINT1  1
#define JOINT2  2
#define JOINT3  3
#define GRIP    4

#define LINK_NUM 5

OPMLink SCARA[LINK_NUM];

void initSCARA()
{
  SCARA[BASE].name_                      = "Base";
  SCARA[BASE].me_                        = BASE;
  SCARA[BASE].mother_                    = -1;
  SCARA[BASE].sibling_                   = -1;
  SCARA[BASE].child_                     = JOINT1;
  SCARA[BASE].p_                         = Eigen::Vector3f::Zero();
  SCARA[BASE].R_                         = Eigen::Matrix3f::Identity(3,3);
  SCARA[BASE].joint_angle_               = 0.0;
  SCARA[BASE].joint_vel_                 = 0.0;
  SCARA[BASE].joint_acc_                 = 0.0;
  SCARA[BASE].joint_axis_                = Eigen::Vector3f::Zero();
  SCARA[BASE].joint_pos_                 = Eigen::Vector3f::Zero();

  SCARA[JOINT1].name_                    = "Joint1";
  SCARA[JOINT1].me_                      = JOINT1;
  SCARA[JOINT1].mother_                  = BASE;
  SCARA[JOINT1].sibling_                 = -1;
  SCARA[JOINT1].child_                   = JOINT2;
  SCARA[JOINT1].p_                       = Eigen::Vector3f::Zero();
  SCARA[JOINT1].R_                       = Eigen::Matrix3f::Identity(3,3);
  SCARA[JOINT1].joint_angle_             = 0.0;
  SCARA[JOINT1].joint_vel_               = 0.0;
  SCARA[JOINT1].joint_acc_               = 0.0;
  SCARA[JOINT1].joint_axis_              << 0, 0, 1;
  SCARA[JOINT1].joint_pos_               << 0.0, 0.0, 0.0661;

  SCARA[JOINT2].name_                    = "Joint2";
  SCARA[JOINT2].me_                      = JOINT2;
  SCARA[JOINT2].mother_                  = JOINT1;
  SCARA[JOINT2].sibling_                 = -1;
  SCARA[JOINT2].child_                   = JOINT3;
  SCARA[JOINT2].p_                       = Eigen::Vector3f::Zero();
  SCARA[JOINT2].R_                       = Eigen::Matrix3f::Identity(3,3);
  SCARA[JOINT2].joint_angle_             = 0.0;
  SCARA[JOINT2].joint_vel_               = 0.0;
  SCARA[JOINT2].joint_acc_               = 0.0;
  SCARA[JOINT2].joint_axis_              << 0, 0, 1;
  SCARA[JOINT2].joint_pos_               << 0.030, 0.0, 0.0;

  SCARA[JOINT3].name_                    = "Joint3";
  SCARA[JOINT3].me_                      = JOINT3;
  SCARA[JOINT3].mother_                  = JOINT2;
  SCARA[JOINT3].sibling_                 = -1;
  SCARA[JOINT3].child_                   = GRIP;
  SCARA[JOINT3].p_                       = Eigen::Vector3f::Zero();
  SCARA[JOINT3].R_                       = Eigen::Matrix3f::Identity(3,3);
  SCARA[JOINT3].joint_angle_             = 0.0;
  SCARA[JOINT3].joint_vel_               = 0.0;
  SCARA[JOINT3].joint_acc_               = 0.0;
  SCARA[JOINT3].joint_axis_              << 0, 0, 1;
  SCARA[JOINT3].joint_pos_               << 0.09025, 0.0, 0.0;

  SCARA[GRIP].name_                      = "Gripper";
  SCARA[GRIP].me_                        = GRIP;
  SCARA[GRIP].mother_                    = JOINT3;
  SCARA[GRIP].sibling_                   = -1;
  SCARA[GRIP].child_                     = -1;
  SCARA[GRIP].p_                         = Eigen::Vector3f::Zero();
  SCARA[GRIP].R_                         = Eigen::Matrix3f::Identity(3,3);
  SCARA[GRIP].joint_angle_               = 0.0;
  SCARA[GRIP].joint_vel_                 = 0.0;
  SCARA[GRIP].joint_acc_                 = 0.0;
  SCARA[GRIP].joint_axis_                << 1, 0, 0;
  SCARA[GRIP].joint_pos_                 << 0.114, 0.0, 0.0;
}

#endif //OPENMANIPULATOR_SCARA_H_