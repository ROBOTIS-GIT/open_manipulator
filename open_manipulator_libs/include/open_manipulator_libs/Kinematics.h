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
using namespace ROBOTIS_MANIPULATOR;

namespace KINEMATICS
{
class CR_Jacobian_Solver : public ROBOTIS_MANIPULATOR::Kinematics
{
private:
  Eigen::MatrixXd jacobianDot(Manipulator *manipulator, Name tool_name);
  std::vector<JointValue> inverseDynamicSolver(Manipulator manipulator, Name tool_name, PoseValue target_pose);
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool inverseSolverUsingJacobian(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* goal_joint_value);

public:
  CR_Jacobian_Solver(){}
  virtual ~CR_Jacobian_Solver(){}

  virtual void setOption(const void *arg);
  virtual void updatePassiveJointValue(Manipulator *manipulator);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void forwardKinematics(Manipulator *manipulator);
  virtual bool inverseKinematics(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* goal_joint_value);
};

class CR_SRJacobian_Solver : public ROBOTIS_MANIPULATOR::Kinematics
{
private:
  Eigen::MatrixXd jacobianDot(Manipulator *manipulator, Name tool_name);
  std::vector<JointValue> inverseDynamicSolver(Manipulator manipulator, Name tool_name, PoseValue target_pose);
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool inverseSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* goal_joint_value);

public:
  CR_SRJacobian_Solver(){}
  virtual ~CR_SRJacobian_Solver(){}

  virtual void setOption(const void *arg);
  virtual void updatePassiveJointValue(Manipulator *manipulator);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void forwardKinematics(Manipulator *manipulator);
  virtual bool inverseKinematics(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* goal_joint_value);
};

class CR_Position_Only_Jacobian_Solver : public ROBOTIS_MANIPULATOR::Kinematics
{
private:
  Eigen::MatrixXd jacobianDot(Manipulator *manipulator, Name tool_name);
  std::vector<JointValue> inverseDynamicSolver(Manipulator manipulator, Name tool_name, PoseValue target_pose);
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool inverseSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* goal_joint_value);

public:
  CR_Position_Only_Jacobian_Solver(){}
  virtual ~CR_Position_Only_Jacobian_Solver(){}

  virtual void setOption(const void *arg);
  virtual void updatePassiveJointValue(Manipulator *manipulator);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void forwardKinematics(Manipulator *manipulator);
  virtual bool inverseKinematics(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* goal_joint_value);
};

class CR_Custom_Solver : public ROBOTIS_MANIPULATOR::Kinematics
{
private:
  Eigen::MatrixXd jacobianDot(Manipulator *manipulator, Name tool_name);
  std::vector<JointValue> inverseDynamicSolver(Manipulator manipulator, Name tool_name, PoseValue target_pose);
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool chainCustomInverseKinematics(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* goal_joint_value);

public:
  CR_Custom_Solver(){}
  virtual ~CR_Custom_Solver(){}

  virtual void setOption(const void *arg);
  virtual void updatePassiveJointValue(Manipulator *manipulator);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  virtual void forwardKinematics(Manipulator *manipulator);
  virtual bool inverseKinematics(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* goal_joint_value);
};

} // namespace KINEMATICS


#endif // KINEMATICS_H_
