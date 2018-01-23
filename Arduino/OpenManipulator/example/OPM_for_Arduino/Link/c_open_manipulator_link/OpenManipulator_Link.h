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

#ifndef OPENMANIPULATOR_LINK_H_
#define OPENMANIPULATOR_LINK_H_

#define BASE    0
#define JOINT1  1
#define JOINT2  2
#define JOINT3  3
#define GRIP    4

#define LINK_NUM 5

OPMLink links[LINK_NUM];

void initLink()
{
  links[BASE].name_                      = "Base";
  links[BASE].me_                        = BASE;
  links[BASE].mother_                    = -1;
  links[BASE].sibling_                   = -1;
  links[BASE].child_                     = JOINT1;
  links[BASE].p_                         = Eigen::Vector3f::Zero();
  links[BASE].R_                         = Eigen::Matrix3f::Identity(3,3);
  links[BASE].joint_angle_               = 0.0;
  links[BASE].joint_vel_                 = 0.0;
  links[BASE].joint_acc_                 = 0.0;
  links[BASE].joint_axis_                = Eigen::Vector3f::Zero();
  links[BASE].joint_pos_                 = Eigen::Vector3f::Zero();

  links[JOINT1].name_                    = "Joint1";
  links[JOINT1].me_                      = JOINT1;
  links[JOINT1].mother_                  = 0;
  links[JOINT1].sibling_                 = JOINT3;
  links[JOINT1].child_                   = JOINT2;
  links[JOINT1].p_                       = Eigen::Vector3f::Zero();
  links[JOINT1].R_                       = Eigen::Matrix3f::Identity(3,3);
  links[JOINT1].joint_angle_             = 0.0;
  links[JOINT1].joint_vel_               = 0.0;
  links[JOINT1].joint_acc_               = 0.0;
  links[JOINT1].joint_axis_              << 0, 0, 1;
  links[JOINT1].joint_pos_               << 0.012, 0, 0.036;

  links[JOINT2].name_                    = "Joint2";
  links[JOINT2].me_                      = JOINT2;
  links[JOINT2].mother_                  = JOINT1;
  links[JOINT2].sibling_                 = -1;
  links[JOINT2].child_                   = -1;
  links[JOINT2].p_                       = Eigen::Vector3f::Zero();
  links[JOINT2].R_                       = Eigen::Matrix3f::Identity(3,3);
  links[JOINT2].joint_angle_             = 0.0;
  links[JOINT2].joint_vel_               = 0.0;
  links[JOINT2].joint_acc_               = 0.0;
  links[JOINT2].joint_axis_              << 0, 1, 0;
  links[JOINT2].joint_pos_               << 0, 0, 0.040;

  links[JOINT3].name_                    = "Joint3";
  links[JOINT3].me_                      = JOINT3;
  links[JOINT3].mother_                  = JOINT1;
  links[JOINT3].sibling_                 = -1;
  links[JOINT3].child_                   = GRIP;
  links[JOINT3].p_                       = Eigen::Vector3f::Zero();
  links[JOINT3].R_                       = Eigen::Matrix3f::Identity(3,3);
  links[JOINT3].joint_angle_             = 0.0;
  links[JOINT3].joint_vel_               = 0.0;
  links[JOINT3].joint_acc_               = 0.0;
  links[JOINT3].joint_axis_              << 0, -1, 0;
  links[JOINT3].joint_pos_               << 0.022, 0, 0.122;

  links[GRIP].name_                      = "Gripper";
  links[GRIP].me_                        = GRIP;
  links[GRIP].mother_                    = JOINT3;
  links[GRIP].sibling_                   = -1;
  links[GRIP].child_                     = -1;
  links[GRIP].p_                         = Eigen::Vector3f::Zero();
  links[GRIP].R_                         = Eigen::Matrix3f::Identity(3,3);
  links[GRIP].joint_angle_               = 0.0;
  links[GRIP].joint_vel_                 = 0.0;
  links[GRIP].joint_acc_                 = 0.0;
  links[GRIP].joint_axis_                = Eigen::Vector3f::Zero();
  links[GRIP].joint_pos_                 = Eigen::Vector3f::Zero();
}

#endif //OPENMANIPULATOR_LINK_H_