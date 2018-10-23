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

/* Authors: Hye-Jong KIM, Darby Lim */

#include "open_manipulator_libs/OMKinematics.h"

using namespace Eigen;
using namespace OPEN_MANIPULATOR;
using namespace OM_KINEMATICS;

MatrixXf Chain::jacobian(RM_MANAGER::Manipulator *manipulator, Name tool_name)
{
  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(manipulator));

  Vector3f joint_axis = ZERO_VECTOR;

  Vector3f position_changed = ZERO_VECTOR;
  Vector3f orientation_changed = ZERO_VECTOR;
  VectorXf pose_changed = VectorXf::Zero(6);

  int8_t index = 0;
  Name my_name = getIteratorBegin(manipulator)->first;

  for (int8_t size = 0; size < getDOF(manipulator); size++)
  {
    Name parent_name = getComponentParentName(manipulator, my_name);
    if (parent_name == getWorldName(manipulator))
    {
      joint_axis = getWorldOrientation(manipulator) * getComponentJointAxis(manipulator, my_name);
    }
    else
    {
      joint_axis = getComponentOrientationToWorld(manipulator, parent_name) * getComponentJointAxis(manipulator, my_name);
    }

    position_changed = RM_MATH::skewSymmetricMatrix(joint_axis) *
                       (getComponentPositionToWorld(manipulator, tool_name) - getComponentPositionToWorld(manipulator, my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = getComponentChildName(manipulator, my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void Chain::forward(RM_MANAGER::Manipulator *manipulator)
{
  forward(manipulator, manipulator->getWorldChildName());
}

void Chain::forward(RM_MANAGER::Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = getComponentParentName(manipulator, my_name);
  int8_t number_of_child = getComponentChildName(manipulator, my_name).size();

  Vector3f parent_position_to_world, my_position_to_world;
  Matrix3f parent_orientation_to_world, my_orientation_to_world;

  if (parent_name == getWorldName(manipulator))
  {
    parent_position_to_world = getWorldPosition(manipulator);
    parent_orientation_to_world = getWorldOrientation(manipulator);
  }
  else
  {
    parent_position_to_world = getComponentPositionToWorld(manipulator, parent_name);
    parent_orientation_to_world = getComponentOrientationToWorld(manipulator, parent_name);
  }

  my_position_to_world = parent_orientation_to_world * getComponentRelativePositionToParent(manipulator, my_name) + parent_position_to_world;
  my_orientation_to_world = parent_orientation_to_world * RM_MATH::rodriguesRotationMatrix(getComponentJointAxis(manipulator, my_name), getComponentJointAngle(manipulator, my_name));

  setComponentPositionToWorld(manipulator, my_name, my_position_to_world);
  setComponentOrientationToWorld(manipulator, my_name, my_orientation_to_world);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = getComponentChildName(manipulator, my_name).at(index);
    forward(manipulator, child_name);
  }
}

std::vector<double> Chain::inverse(RM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  // return positionOnlyInverseKinematics(manipulator, tool_name, target_pose);
  return srInverseKinematics(manipulator, tool_name, target_pose);
}

std::vector<double> Chain::inverseKinematics(RM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  const double lambda = 0.7;
  const int8_t iteration = 10;

  RM_MANAGER::Manipulator _manipulator = *manipulator;

  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(&_manipulator));

  VectorXf pose_changed = VectorXf::Zero(6);
  VectorXf angle_changed = VectorXf::Zero(getDOF(&_manipulator));

  for (int8_t count = 0; count < iteration; count++)
  {
    forward(&_manipulator, getIteratorBegin(&_manipulator)->first);

    jacobian = this->jacobian(&_manipulator, tool_name);

    pose_changed = RM_MATH::poseDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name),
                                           target_pose.orientation, getComponentOrientationToWorld(&_manipulator, tool_name));
    if (pose_changed.norm() < 1E-6)
      return getAllActiveJointAngle(&_manipulator);

    ColPivHouseholderQR<MatrixXf> dec(jacobian);
    angle_changed = lambda * dec.solve(pose_changed);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < getDOF(&_manipulator); index++)
      set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) + angle_changed(index));

    setAllActiveJointAngle(&_manipulator, set_angle_changed);
  }

  return getAllActiveJointAngle(&_manipulator);
}

std::vector<double> Chain::srInverseKinematics(RM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  double lambda = 0.0;
  const double param = 0.002;
  int8_t iteration = 50;

  RM_MANAGER::Manipulator _manipulator = *manipulator;

  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(&_manipulator));
  MatrixXf updated_jacobian = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));
  VectorXf pose_changed = VectorXf::Zero(getDOF(&_manipulator));
  VectorXf angle_changed = VectorXf::Zero(getDOF(&_manipulator));
  VectorXf gerr(getDOF(&_manipulator));

  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double Ek = 0.0;
  double Ek2 = 0.0;

  MatrixXf We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  MatrixXf Wn = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));

  forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
  pose_changed = RM_MATH::poseDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name),
                                         target_pose.orientation, getComponentOrientationToWorld(&_manipulator, tool_name));
  Ek = pose_changed.transpose() * We * pose_changed;

  for (int8_t count = 0; count < iteration; count++)
  {
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = Ek + param;

    updated_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);
    gerr = jacobian.transpose() * We * pose_changed;

    ColPivHouseholderQR<MatrixXf> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < getDOF(&_manipulator); index++)
      set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) + angle_changed(index));

    setAllActiveJointAngle(&_manipulator, set_angle_changed);

    forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    pose_changed = RM_MATH::poseDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name),
                                           target_pose.orientation, getComponentOrientationToWorld(&_manipulator, tool_name));

    Ek2 = pose_changed.transpose() * We * pose_changed;

    if (Ek2 < 1E-12)
    {
      return getAllActiveJointAngle(&_manipulator);
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<double> set_angle_changed;
      for (int8_t index = 0; index < getDOF(&_manipulator); index++)
        set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) - angle_changed(index));

      setAllActiveJointAngle(&_manipulator, set_angle_changed);

      forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    }
  }

  return getAllActiveJointAngle(&_manipulator);
}

std::vector<double> Chain::positionOnlyInverseKinematics(RM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  RM_MANAGER::Manipulator _manipulator = *manipulator;

  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(&_manipulator));
  MatrixXf position_jacobian = MatrixXf::Identity(3, getDOF(&_manipulator));
  MatrixXf updated_jacobian = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));
  VectorXf position_changed = VectorXf::Zero(3);
  VectorXf angle_changed = VectorXf::Zero(getDOF(&_manipulator));
  VectorXf gerr(getDOF(&_manipulator));

  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double Ek = 0.0;
  double Ek2 = 0.0;

  MatrixXf We(3, 3);
  We << wn_pos, 0, 0,
      0, wn_pos, 0,
      0, 0, wn_pos;

  MatrixXf Wn = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));

  forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
  position_changed = RM_MATH::positionDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name));
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

    ColPivHouseholderQR<MatrixXf> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < getDOF(&_manipulator); index++)
      set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) + angle_changed(index));

    setAllActiveJointAngle(&_manipulator, set_angle_changed);

    forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    position_changed = RM_MATH::positionDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name));

    Ek2 = position_changed.transpose() * We * position_changed;

    if (Ek2 < 1E-12)
    {
      return getAllActiveJointAngle(&_manipulator);
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<double> set_angle_changed;
      for (int8_t index = 0; index < getDOF(&_manipulator); index++)
        set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) - angle_changed(index));

      setAllActiveJointAngle(&_manipulator, set_angle_changed);

      forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    }
  }

  return getAllActiveJointAngle(&_manipulator);
}
