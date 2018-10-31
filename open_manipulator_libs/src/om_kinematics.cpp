﻿/*******************************************************************************
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

/* Authors: Hye-Jong KIM, Darby Lim */

#include "open_manipulator_libs/om_kinematics.h"

using namespace Eigen;
using namespace ROBOTIS_MANIPULATOR;
using namespace OM_CHAIN_KINEMATICS;

MatrixXd Chain::jacobian(Manipulator *manipulator, Name tool_name)
{
  MatrixXd jacobian = MatrixXd::Identity(6, manipulator->getDOF());

  Vector3f joint_axis = ZERO_VECTOR;

  Vector3f position_changed = ZERO_VECTOR;
  Vector3f orientation_changed = ZERO_VECTOR;
  VectorXf pose_changed = VectorXf::Zero(6);

  int8_t index = 0;
  Name my_name = manipulator->getIteratorBegin()->first;

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getComponentJointAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationToWorld(parent_name) * manipulator->getComponentJointAxis(my_name);
    }

    position_changed = RM_MATH::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionToWorld(tool_name) - manipulator->getComponentPositionToWorld(my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void Chain::forward(Manipulator *manipulator)
{
  forward(manipulator, manipulator->getWorldChildName());
}

void Chain::forward(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Vector3f parent_position_to_world, my_position_to_world;
  Matrix3f parent_orientation_to_world, my_orientation_to_world;

  if (parent_name == manipulator->getWorldName())
  {
    parent_position_to_world = manipulator->getWorldPosition();
    parent_orientation_to_world = manipulator->getWorldOrientation();
  }
  else
  {
    parent_position_to_world = manipulator->getComponentPositionToWorld(parent_name);
    parent_orientation_to_world = manipulator->getComponentOrientationToWorld(parent_name);
  }

  my_position_to_world = parent_orientation_to_world * manipulator->getComponentRelativePositionToParent(my_name) + parent_position_to_world;
  my_orientation_to_world = parent_orientation_to_world * RM_MATH::rodriguesRotationMatrix(manipulator->getComponentJointAxis(my_name), manipulator->getComponentJointAngle(my_name));

  manipulator->setComponentPositionToWorld(my_name, my_position_to_world);
  manipulator->setComponentOrientationToWorld(my_name, my_orientation_to_world);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forward(manipulator, child_name);
  }
}

std::vector<double> Chain::inverse(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  // return positionOnlyInverseKinematics(manipulator, tool_name, target_pose);
  return srInverseKinematics(manipulator, tool_name, target_pose);
}

std::vector<double> Chain::inverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  const double lambda = 0.7;
  const int8_t iteration = 10;

  Manipulator _manipulator = *manipulator;

  MatrixXd jacobian = MatrixXd::Identity(6, _manipulator.getDOF());

  VectorXf pose_changed = VectorXf::Zero(6);
  VectorXf angle_changed = VectorXf::Zero(_manipulator.getDOF());

  for (int8_t count = 0; count < iteration; count++)
  {
    forward(&_manipulator, _manipulator.getIteratorBegin()->first);

    jacobian = this->jacobian(&_manipulator,tool_name);

    pose_changed = RM_MATH::poseDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name),
                                           target_pose.orientation, _manipulator.getComponentOrientationToWorld(tool_name));
    if (pose_changed.norm() < 1E-6)
      return _manipulator.getAllActiveJointAngle();

    ColPivHouseholderQR<MatrixXd> dec(jacobian);
    angle_changed = lambda * dec.solve(pose_changed);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) + angle_changed(index));

    _manipulator.setAllActiveJointAngle(set_angle_changed);
  }

  return _manipulator.getAllActiveJointAngle();
}

std::vector<double> Chain::srInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  double lambda = 0.0;
  const double param = 0.002;
  int8_t iteration = 50;

  Manipulator _manipulator = *manipulator;

  MatrixXd jacobian = MatrixXd::Identity(6, _manipulator.getDOF());
  MatrixXd updated_jacobian = MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());
  VectorXf pose_changed = VectorXf::Zero(_manipulator.getDOF());
  VectorXf angle_changed = VectorXf::Zero(_manipulator.getDOF());
  VectorXf gerr(_manipulator.getDOF());

  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double Ek = 0.0;
  double Ek2 = 0.0;

  MatrixXd We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  MatrixXd Wn = MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  forward(&_manipulator, _manipulator.getIteratorBegin()->first);
  pose_changed = RM_MATH::poseDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name),
                                         target_pose.orientation, _manipulator.getComponentOrientationToWorld(tool_name));
  Ek = pose_changed.transpose() * We * pose_changed;

  for (int8_t count = 0; count < iteration; count++)
  {
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = Ek + param;

    updated_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);
    gerr = jacobian.transpose() * We * pose_changed;

    ColPivHouseholderQR<MatrixXd> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) + angle_changed(index));
    _manipulator.setAllActiveJointAngle(set_angle_changed);


    forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    pose_changed = RM_MATH::poseDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name),
                                           target_pose.orientation, _manipulator.getComponentOrientationToWorld(tool_name));

    Ek2 = pose_changed.transpose() * We * pose_changed;

    if (Ek2 < 1E-12)
    {
      return _manipulator.getAllActiveJointAngle();
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<double> set_angle_changed;
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) - angle_changed(index));

      _manipulator.setAllActiveJointAngle(set_angle_changed);

      forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    }
  }

  return _manipulator.getAllActiveJointAngle();
}

std::vector<double> Chain::positionOnlyInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  Manipulator _manipulator = *manipulator;

  MatrixXd jacobian = MatrixXd::Identity(6, _manipulator.getDOF());
  MatrixXd position_jacobian = MatrixXd::Identity(3, _manipulator.getDOF());
  MatrixXd updated_jacobian = MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());
  VectorXf position_changed = VectorXf::Zero(3);
  VectorXf angle_changed = VectorXf::Zero(_manipulator.getDOF());
  VectorXf gerr(_manipulator.getDOF());

  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double Ek = 0.0;
  double Ek2 = 0.0;

  MatrixXd We(3, 3);
  We << wn_pos, 0, 0,
      0, wn_pos, 0,
      0, 0, wn_pos;

  MatrixXd Wn = MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  forward(&_manipulator, _manipulator.getIteratorBegin()->first);
  position_changed = RM_MATH::positionDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name));
  Ek = position_changed.transpose() * We * position_changed;

  for (int8_t count = 0; count < iteration; count++)
  {
    jacobian = this->jacobian(&_manipulator, tool_name);
    position_jacobian.row(0) = jacobian.row(0);
    position_jacobian.row(1) = jacobian.row(1);
    position_jacobian.row(2) = jacobian.row(2);
    lambda = Ek + param;

    updated_jacobian = (position_jacobian.transpose() * We * jacobian) + (lambda * Wn);
    gerr = position_jacobian.transpose() * We * position_changed;

    ColPivHouseholderQR<MatrixXd> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) + angle_changed(index));

    _manipulator.setAllActiveJointAngle(set_angle_changed);

    forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    position_changed = RM_MATH::positionDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name));

    Ek2 = position_changed.transpose() * We * position_changed;

    if (Ek2 < 1E-12)
    {
      return _manipulator.getAllActiveJointAngle();
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<double> set_angle_changed;
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) - angle_changed(index));

      _manipulator.setAllActiveJointAngle(set_angle_changed);

      forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    }
  }

  return _manipulator.getAllActiveJointAngle();
}