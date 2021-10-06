// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na

#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
#endif

//#define KINEMATICS_DEBUG

using namespace Eigen;
using namespace robotis_manipulator;

namespace kinematics
{
/*****************************************************************************
** Kinematics Solver Using Chain Rule and Jacobian
*****************************************************************************/
class SolverUsingCRAndJacobian : public robotis_manipulator::Kinematics
{
 public:
  SolverUsingCRAndJacobian(){}
  virtual ~SolverUsingCRAndJacobian(){}

  virtual void setOption(const void *arg);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void solveForwardKinematics(Manipulator *manipulator);
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

 private:
  void forward_solver_using_chain_rule(Manipulator *manipulator, Name component_name);
  bool inverse_solver_using_jacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
};

/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Jacobian
*****************************************************************************/
class SolverUsingCRAndSRJacobian : public robotis_manipulator::Kinematics
{
 public:
  SolverUsingCRAndSRJacobian(){}
  virtual ~SolverUsingCRAndSRJacobian(){}

  virtual void setOption(const void *arg);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void solveForwardKinematics(Manipulator *manipulator);
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

 private:
  void forward_solver_using_chain_rule(Manipulator *manipulator, Name component_name);
  bool inverse_solver_using_sr_jacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
};

/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Position Only Jacobian
*****************************************************************************/
class SolverUsingCRAndSRPositionOnlyJacobian : public robotis_manipulator::Kinematics
{
 public:
  SolverUsingCRAndSRPositionOnlyJacobian(){}
  virtual ~SolverUsingCRAndSRPositionOnlyJacobian(){}

  virtual void setOption(const void *arg);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void solveForwardKinematics(Manipulator *manipulator);
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

 private:
  void forward_solver_using_chain_rule(Manipulator *manipulator, Name component_name);
  bool inverse_solver_using_position_only_sr_jacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
};

/*****************************************************************************
** Kinematics Solver Customized for OpenManipulator Chain
*****************************************************************************/
class SolverCustomizedforOMChain : public robotis_manipulator::Kinematics
{
 public:
  SolverCustomizedforOMChain(){}
  virtual ~SolverCustomizedforOMChain(){}

  virtual void setOption(const void *arg);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void solveForwardKinematics(Manipulator *manipulator);
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

 private:
  void forward_solver_using_chain_rule(Manipulator *manipulator, Name component_name);
  bool chain_custom_inverse_kinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
};
}  // namespace KINEMATICS
#endif // KINEMATICS_HPP
