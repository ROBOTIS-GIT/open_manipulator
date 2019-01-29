/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

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
class SolverUsingChainRuleandJacobian : public robotis_manipulator::Kinematics
{
private:
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool inverseSolverUsingJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

public:
  SolverUsingChainRuleandJacobian(){}
  virtual ~SolverUsingChainRuleandJacobian(){}

  virtual void setOption(const void *arg);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void solveForwardKinematics(Manipulator *manipulator);
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
};


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Jacobian
*****************************************************************************/
class SolverUsingChainRuleandSingularityRobustJacobian : public robotis_manipulator::Kinematics
{
private:
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool inverseSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

public:
  SolverUsingChainRuleandSingularityRobustJacobian(){}
  virtual ~SolverUsingChainRuleandSingularityRobustJacobian(){}

  virtual void setOption(const void *arg);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void solveForwardKinematics(Manipulator *manipulator);
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
};


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Position Only Jacobian
*****************************************************************************/
class SolverUsingChainRuleandSingularityRobustPositionOnlyJacobian : public robotis_manipulator::Kinematics
{
private:
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool inverseSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

public:
  SolverUsingChainRuleandSingularityRobustPositionOnlyJacobian(){}
  virtual ~SolverUsingChainRuleandSingularityRobustPositionOnlyJacobian(){}

  virtual void setOption(const void *arg);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void solveForwardKinematics(Manipulator *manipulator);
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
};


/*****************************************************************************
** Kinematics Solver Customized for OpenManipulator Chain
*****************************************************************************/
class SolverCustomizedforOpenManipulatorChain : public robotis_manipulator::Kinematics
{
private:
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool chainCustomInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

public:
  SolverCustomizedforOpenManipulatorChain(){}
  virtual ~SolverCustomizedforOpenManipulatorChain(){}

  virtual void setOption(const void *arg);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void solveForwardKinematics(Manipulator *manipulator);
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
};

} // namespace KINEMATICS


#endif // KINEMATICS_H_
