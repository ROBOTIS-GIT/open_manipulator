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

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
#endif

using namespace Eigen;
using namespace ROBOTIS_MANIPULATOR;

namespace KINEMATICS
{
class Chain : public ROBOTIS_MANIPULATOR::Kinematics
{
private:
  int8_t inverse_solver_option_;
public:
  Chain():inverse_solver_option_(0){}
  virtual ~Chain(){}

  virtual void setOption(const void *arg);
  virtual void updatePassiveJointValue(Manipulator *manipulator);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);

  virtual void forward(Manipulator *manipulator);
  virtual void forward(Manipulator *manipulator, Name component_name);
  virtual std::vector<double> inverse(Manipulator *manipulator, Name tool_name, Pose target_pose);

  std::vector<double> inverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<double> srInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<double> positionOnlyInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose);
};

} // namespace KINEMATICS

#endif // KINEMATICS_H_
