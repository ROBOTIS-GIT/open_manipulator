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

#include "../include/open_manipulator_libs/Kinematics.h"

using namespace robotis_manipulator;
using namespace kinematics;


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Jacobian
*****************************************************************************/
void SolverUsingChainRuleandJacobian::setOption(const void *arg){}

Eigen::MatrixXd SolverUsingChainRuleandJacobian::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  //////////////////////////////////////////////////////////////////////////////////

  int8_t index = 0;
  Name my_name =  manipulator->getWorldChildName();

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = robotis_manipulator_math::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
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

void SolverUsingChainRuleandJacobian::solveForwardKinematics(Manipulator *manipulator)
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingChainRuleandJacobian::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseSolverUsingJacobian(manipulator, tool_name, target_pose, goal_joint_value);
}


//private
void SolverUsingChainRuleandJacobian::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  //Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  //position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  //orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * robotis_manipulator_math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
  //linear velocity
  my_pose_value.dynamic.linear.velocity = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //angular velocity
  my_pose_value.dynamic.angular.velocity = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //linear acceleration
  my_pose_value.dynamic.linear.acceleration = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //angular acceleration
  my_pose_value.dynamic.angular.acceleration = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}

bool SolverUsingChainRuleandJacobian::inverseSolverUsingJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  const double lambda = 0.7;
  const int8_t iteration = 10;

  Manipulator _manipulator = *manipulator;

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());

  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd delta_angle = Eigen::VectorXd::Zero(_manipulator.getDOF());

  for (int8_t count = 0; count < iteration; count++)
  {
    //Forward kinematics solve
    solveForwardKinematics(&_manipulator);
    //Get jacobian
    jacobian = this->jacobian(&_manipulator, tool_name);

    //Pose Difference
    pose_changed = robotis_manipulator_math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name),
                                           target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));

    //pose sovler success
    if (pose_changed.norm() < 1E-6)
    {
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      {
        goal_joint_value->at(index).velocity = 0.0;
        goal_joint_value->at(index).acceleration = 0.0;
        goal_joint_value->at(index).effort = 0.0;
      }
      return true;
    }

    //get delta angle
    ColPivHouseholderQR<MatrixXd> dec(jacobian);
    delta_angle = lambda * dec.solve(pose_changed);

    //set changed angle
    std::vector<double> changed_angle;
    for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      changed_angle.push_back(_manipulator.getAllActiveJointPosition().at(index) + delta_angle(index));
    _manipulator.setAllActiveJointPosition(changed_angle);
  }
  *goal_joint_value = {};
  return false;
}


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Jacobian
*****************************************************************************/
void SolverUsingChainRuleandSingularityRobustJacobian::setOption(const void *arg){}

Eigen::MatrixXd SolverUsingChainRuleandSingularityRobustJacobian::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  //////////////////////////////////////////////////////////////////////////////////

  int8_t index = 0;
  Name my_name =  manipulator->getWorldChildName();

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = robotis_manipulator_math::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
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

void SolverUsingChainRuleandSingularityRobustJacobian::solveForwardKinematics(Manipulator *manipulator)
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingChainRuleandSingularityRobustJacobian::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseSolverUsingSRJacobian(manipulator, tool_name, target_pose, goal_joint_value);
}


//private
void SolverUsingChainRuleandSingularityRobustJacobian::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  //Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  //position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  //orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * robotis_manipulator_math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
  //linear velocity
  my_pose_value.dynamic.linear.velocity = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //angular velocity
  my_pose_value.dynamic.angular.velocity = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //linear acceleration
  my_pose_value.dynamic.linear.acceleration = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //angular acceleration
  my_pose_value.dynamic.angular.acceleration = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}

bool SolverUsingChainRuleandSingularityRobustJacobian::inverseSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  //manipulator
  Manipulator _manipulator = *manipulator;

  //solver parameter
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  const double gamma = 0.5;             //rollback delta

  //sr sovler parameter
  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double pre_Ek = 0.0;
  double new_Ek = 0.0;

  Eigen::MatrixXd We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
  Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //delta parameter
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF());    //delta angle (dq)
  Eigen::VectorXd gerr(_manipulator.getDOF());

  //angle parameter
  std::vector<double> present_angle;                                               //angle (q)
  std::vector<double> set_angle;                                                   //set angle (q + dq)

  ////////////////////////////solving//////////////////////////////////

  solveForwardKinematics(&_manipulator);
  //////////////checking dx///////////////
  pose_changed = robotis_manipulator_math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
  pre_Ek = pose_changed.transpose() * We * pose_changed;
  ///////////////////////////////////////

  /////////////////////////////debug/////////////////////////////////
  #if defined(KINEMATICS_DEBUG)
  Eigen::Vector3d target_orientation_rpy = robotis_manipulator_math::convertRotationToRPY(target_pose.orientation);
  Eigen::VectorXd debug_target_pose(6);
  for(int t=0; t<3; t++)
    debug_target_pose(t) = target_pose.position(t);
  for(int t=0; t<3; t++)
    debug_target_pose(t+3) = target_orientation_rpy(t);

  Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
  Eigen::MatrixXd present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
  Eigen::Vector3d present_orientation_rpy = robotis_manipulator_math::convertRotationToRPY(present_orientation);
  Eigen::VectorXd debug_present_pose(6);
  for(int t=0; t<3; t++)
    debug_present_pose(t) = present_position(t);
  for(int t=0; t<3; t++)
    debug_present_pose(t+3) = present_orientation_rpy(t);

  RM_LOG::PRINTLN("------------------------------------");
  RM_LOG::WARN("iter : first");
  RM_LOG::WARN("Ek : ", pre_Ek*1000000000000);
  RM_LOG::PRINTLN("tar_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
  RM_LOG::PRINTLN("pre_pose");
  RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
  RM_LOG::PRINTLN("delta_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
  #endif
  ////////////////////////////debug//////////////////////////////////

  //////////////////////////solving loop///////////////////////////////
  for (int8_t count = 0; count < iteration; count++)
  {
    //////////solve using jacobian//////////
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = pre_Ek + param;

    sr_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
    gerr = jacobian.transpose() * We * pose_changed;                          //calculate gerr (J^T*we) dx

    ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian);                    //solving (get dq)
    angle_changed = dec.solve(gerr);                                          //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

    present_angle = _manipulator.getAllActiveJointPosition();
    set_angle.clear();
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle.push_back(present_angle.at(index) + angle_changed(index));
    _manipulator.setAllActiveJointPosition(set_angle);
    solveForwardKinematics(&_manipulator);
    ////////////////////////////////////////

    //////////////checking dx///////////////
    pose_changed = robotis_manipulator_math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
    new_Ek = pose_changed.transpose() * We * pose_changed;
    ////////////////////////////////////////

    /////////////////////////////debug/////////////////////////////////
    #if defined(KINEMATICS_DEBUG)
    present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    present_orientation_rpy = robotis_manipulator_math::convertRotationToRPY(present_orientation);
    for(int t=0; t<3; t++)
      debug_present_pose(t) = present_position(t);
    for(int t=0; t<3; t++)
      debug_present_pose(t+3) = present_orientation_rpy(t);
    RM_LOG::WARN("iter : ", count,0);
    RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
    RM_LOG::PRINTLN("tar_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
    RM_LOG::PRINTLN("pre_pose");
    RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
    RM_LOG::PRINTLN("delta_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
    #endif
    ////////////////////////////debug//////////////////////////////////

    if (new_Ek < 1E-12)
    {
      /////////////////////////////debug/////////////////////////////////
      #if defined(KINEMATICS_DEBUG)
      RM_LOG::WARN("iter : ", count,0);
      RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
      RM_LOG::ERROR("Success");
      RM_LOG::PRINTLN("------------------------------------");
      #endif
      //////////////////////////debug//////////////////////////////////
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      {
        goal_joint_value->at(index).velocity = 0.0;
        goal_joint_value->at(index).acceleration = 0.0;
        goal_joint_value->at(index).effort = 0.0;
      }
      return true;
    }
    else if (new_Ek < pre_Ek)
    {
      pre_Ek = new_Ek;
    }
    else
    {
      present_angle = _manipulator.getAllActiveJointPosition();
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle.push_back(present_angle.at(index) - (gamma * angle_changed(index)));
      _manipulator.setAllActiveJointPosition(set_angle);

      solveForwardKinematics(&_manipulator);
    }
  }
  robotis_manipulator_log::error("[sr]fail to solve inverse kinematics (please change the solver)");
  *goal_joint_value = {};
  return false;
}


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Position Only Jacobian
*****************************************************************************/
void SolverUsingChainRuleandSingularityRobustPositionOnlyJacobian::setOption(const void *arg){}

Eigen::MatrixXd SolverUsingChainRuleandSingularityRobustPositionOnlyJacobian::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  //////////////////////////////////////////////////////////////////////////////////

  int8_t index = 0;
  Name my_name =  manipulator->getWorldChildName();

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = robotis_manipulator_math::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
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

void SolverUsingChainRuleandSingularityRobustPositionOnlyJacobian::solveForwardKinematics(Manipulator *manipulator)
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingChainRuleandSingularityRobustPositionOnlyJacobian::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseSolverUsingPositionOnlySRJacobian(manipulator, tool_name, target_pose, goal_joint_value);
}


//private
void SolverUsingChainRuleandSingularityRobustPositionOnlyJacobian::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  //Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  //position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  //orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * robotis_manipulator_math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
  //linear velocity
  my_pose_value.dynamic.linear.velocity = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //angular velocity
  my_pose_value.dynamic.angular.velocity = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //linear acceleration
  my_pose_value.dynamic.linear.acceleration = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //angular acceleration
  my_pose_value.dynamic.angular.acceleration = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}

bool SolverUsingChainRuleandSingularityRobustPositionOnlyJacobian::inverseSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  //manipulator
  Manipulator _manipulator = *manipulator;

  //solver parameter
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  const double gamma = 0.5;             //rollback delta

  //jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
  Eigen::MatrixXd position_jacobian = Eigen::MatrixXd::Identity(3, _manipulator.getDOF());
  Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //delta parameter
  Eigen::Vector3d position_changed = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF());    //delta angle (dq)
  Eigen::VectorXd gerr(_manipulator.getDOF());

  //sr sovler parameter
  double wn_pos = 1 / 0.3;
  double pre_Ek = 0.0;
  double new_Ek = 0.0;

  Eigen::MatrixXd We(3, 3);
  We << wn_pos, 0, 0,
      0, wn_pos, 0,
      0, 0, wn_pos;

  Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //angle parameter
  std::vector<double> present_angle;                                               //angle (q)
  std::vector<double> set_angle;                                                   //set angle (q + dq)

  ////////////////////////////solving//////////////////////////////////

  solveForwardKinematics(&_manipulator);
  //////////////checking dx///////////////
  position_changed = robotis_manipulator_math::positionDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name));
  pre_Ek = position_changed.transpose() * We * position_changed;
  ///////////////////////////////////////

  /////////////////////////////debug/////////////////////////////////
  #if defined(KINEMATICS_DEBUG)
  Eigen::Vector3d target_orientation_rpy = robotis_manipulator_math::convertRotationToRPY(target_pose.orientation);
  Eigen::VectorXd debug_target_pose(6);
  for(int t=0; t<3; t++)
    debug_target_pose(t) = target_pose.position(t);
  for(int t=0; t<3; t++)
    debug_target_pose(t+3) = target_orientation_rpy(t);

  Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
  Eigen::MatrixXd present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
  Eigen::Vector3d present_orientation_rpy = robotis_manipulator_math::convertRotationToRPY(present_orientation);
  Eigen::VectorXd debug_present_pose(6);
  for(int t=0; t<3; t++)
    debug_present_pose(t) = present_position(t);
  for(int t=0; t<3; t++)
    debug_present_pose(t+3) = present_orientation_rpy(t);

  RM_LOG::PRINTLN("------------------------------------");
  RM_LOG::WARN("iter : first");
  RM_LOG::WARN("Ek : ", pre_Ek*1000000000000);
  RM_LOG::PRINTLN("tar_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
  RM_LOG::PRINTLN("pre_pose");
  RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
  RM_LOG::PRINTLN("delta_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
  #endif
  ////////////////////////////debug//////////////////////////////////

  //////////////////////////solving loop///////////////////////////////
  for (int8_t count = 0; count < iteration; count++)
  {
    //////////solve using jacobian//////////
    jacobian = this->jacobian(&_manipulator, tool_name);
    position_jacobian.row(0) = jacobian.row(0);
    position_jacobian.row(1) = jacobian.row(1);
    position_jacobian.row(2) = jacobian.row(2);
    lambda = pre_Ek + param;

    sr_jacobian = (position_jacobian.transpose() * We * position_jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
    gerr = position_jacobian.transpose() * We * position_changed;                                //calculate gerr (J^T*we) dx

    ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian);                    //solving (get dq)
    angle_changed = dec.solve(gerr);                                          //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

    present_angle = _manipulator.getAllActiveJointPosition();
    set_angle.clear();
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle.push_back(_manipulator.getAllActiveJointPosition().at(index) + angle_changed(index));
    _manipulator.setAllActiveJointPosition(set_angle);
    solveForwardKinematics(&_manipulator);
    ////////////////////////////////////////

    //////////////checking dx///////////////
    position_changed = robotis_manipulator_math::positionDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name));
    new_Ek = position_changed.transpose() * We * position_changed;
    ////////////////////////////////////////

    /////////////////////////////debug/////////////////////////////////
    #if defined(KINEMATICS_DEBUG)
    present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    present_orientation_rpy = robotis_manipulator_math::convertRotationToRPY(present_orientation);
    for(int t=0; t<3; t++)
      debug_present_pose(t) = present_position(t);
    for(int t=0; t<3; t++)
      debug_present_pose(t+3) = present_orientation_rpy(t);
    RM_LOG::WARN("iter : ", count,0);
    RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
    RM_LOG::PRINTLN("tar_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
    RM_LOG::PRINTLN("pre_pose");
    RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
    RM_LOG::PRINTLN("delta_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
    #endif
    ////////////////////////////debug//////////////////////////////////

    if (new_Ek < 1E-12)
    {
      /////////////////////////////debug/////////////////////////////////
      #if defined(KINEMATICS_DEBUG)
      RM_LOG::WARN("iter : ", count,0);
      RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
      RM_LOG::ERROR("IK Success");
      RM_LOG::PRINTLN("------------------------------------");
      #endif
      //////////////////////////debug//////////////////////////////////
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      {
        goal_joint_value->at(index).velocity = 0.0;
        goal_joint_value->at(index).acceleration = 0.0;
        goal_joint_value->at(index).effort = 0.0;
      }
      return true;
    }
    else if (new_Ek < pre_Ek)
    {
      pre_Ek = new_Ek;
    }
    else
    {
      present_angle = _manipulator.getAllActiveJointPosition();
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle.push_back(_manipulator.getAllActiveJointPosition().at(index) - (gamma * angle_changed(index)));
      _manipulator.setAllActiveJointPosition(set_angle);

      solveForwardKinematics(&_manipulator);
    }
  }
  robotis_manipulator_log::error("[position_only]fail to solve inverse kinematics (please change the solver)");
  *goal_joint_value = {};
  return false;
}


/*****************************************************************************
** Kinematics Solver Customized for OpenManipulator Chain
*****************************************************************************/
void SolverCustomizedforOpenManipulatorChain::setOption(const void *arg){}

Eigen::MatrixXd SolverCustomizedforOpenManipulatorChain::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  //////////////////////////////////////////////////////////////////////////////////

  int8_t index = 0;
  Name my_name =  manipulator->getWorldChildName();

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = robotis_manipulator_math::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
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

void SolverCustomizedforOpenManipulatorChain::solveForwardKinematics(Manipulator *manipulator)
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverCustomizedforOpenManipulatorChain::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return chainCustomInverseKinematics(manipulator, tool_name, target_pose, goal_joint_value);
}


//private
void SolverCustomizedforOpenManipulatorChain::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  //Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  //position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  //orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * robotis_manipulator_math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
  //linear velocity
  my_pose_value.dynamic.linear.velocity = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //angular velocity
  my_pose_value.dynamic.angular.velocity = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //linear acceleration
  my_pose_value.dynamic.linear.acceleration = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);
  //angular acceleration
  my_pose_value.dynamic.angular.acceleration = robotis_manipulator_math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}
bool SolverCustomizedforOpenManipulatorChain::chainCustomInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  //manipulator
  Manipulator _manipulator = *manipulator;

  //solver parameter
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  const double gamma = 0.5;             //rollback delta

  //sr sovler parameter
  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double pre_Ek = 0.0;
  double new_Ek = 0.0;

  Eigen::MatrixXd We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
  Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //delta parameter
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF());    //delta angle (dq)
  Eigen::VectorXd gerr(_manipulator.getDOF());

  //angle parameter
  std::vector<double> present_angle;                                               //angle (q)
  std::vector<double> set_angle;                                                   //set angle (q + dq)

  ////////////////////////////solving//////////////////////////////////

  solveForwardKinematics(&_manipulator);

  //////////////make target ori//////////  //only OpenManipulator Chain
  Eigen::Matrix3d present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
  Eigen::Vector3d present_orientation_rpy = robotis_manipulator_math::convertRotationMatrix2RPYVector(present_orientation);
  Eigen::Matrix3d target_orientation = target_pose.kinematic.orientation;
  Eigen::Vector3d target_orientation_rpy = robotis_manipulator_math::convertRotationMatrix2RPYVector(target_orientation);

  Eigen::Vector3d joint1_rlative_position = _manipulator.getComponentRelativePositionFromParent(_manipulator.getWorldChildName());
  Eigen::Vector3d target_position_from_joint1 = target_pose.kinematic.position - joint1_rlative_position;

  target_orientation_rpy(0) = present_orientation_rpy(0);
  target_orientation_rpy(1) = target_orientation_rpy(1);
  target_orientation_rpy(2) = atan2(target_position_from_joint1(1) ,target_position_from_joint1(0));

  target_pose.kinematic.orientation = robotis_manipulator_math::convertRPY2RotationMatrix(target_orientation_rpy(0), target_orientation_rpy(1), target_orientation_rpy(2));
  ///////////////////////////////////////

  //////////////checking dx///////////////
  pose_changed = robotis_manipulator_math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
  pre_Ek = pose_changed.transpose() * We * pose_changed;
  ///////////////////////////////////////

  /////////////////////////////debug/////////////////////////////////
  #if defined(KINEMATICS_DEBUG)
  Eigen::VectorXd debug_target_pose(6);
  for(int t=0; t<3; t++)
    debug_target_pose(t) = target_pose.position(t);
  for(int t=0; t<3; t++)
    debug_target_pose(t+3) = target_orientation_rpy(t);

  Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
  Eigen::VectorXd debug_present_pose(6);
  for(int t=0; t<3; t++)
    debug_present_pose(t) = present_position(t);
  for(int t=0; t<3; t++)
    debug_present_pose(t+3) = present_orientation_rpy(t);

  RM_LOG::PRINTLN("------------------------------------");
  RM_LOG::WARN("iter : first");
  RM_LOG::WARN("Ek : ", pre_Ek*1000000000000);
  RM_LOG::PRINTLN("tar_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
  RM_LOG::PRINTLN("pre_pose");
  RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
  RM_LOG::PRINTLN("delta_pose");
  RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
  #endif
  ////////////////////////////debug//////////////////////////////////

  //////////////////////////solving loop///////////////////////////////
  for (int8_t count = 0; count < iteration; count++)
  {
    //////////solve using jacobian//////////
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = pre_Ek + param;

    sr_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
    gerr = jacobian.transpose() * We * pose_changed;                          //calculate gerr (J^T*we) dx

    ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian);                    //solving (get dq)
    angle_changed = dec.solve(gerr);                                          //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

    present_angle = _manipulator.getAllActiveJointPosition();
    set_angle.clear();
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle.push_back(present_angle.at(index) + angle_changed(index));
    _manipulator.setAllActiveJointPosition(set_angle);
    solveForwardKinematics(&_manipulator);
    ////////////////////////////////////////

    //////////////checking dx///////////////
    pose_changed = robotis_manipulator_math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
    new_Ek = pose_changed.transpose() * We * pose_changed;
    ////////////////////////////////////////

    /////////////////////////////debug/////////////////////////////////
    #if defined(KINEMATICS_DEBUG)
    present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    present_orientation_rpy = robotis_manipulator_math::convertRotationToRPY(present_orientation);
    for(int t=0; t<3; t++)
      debug_present_pose(t) = present_position(t);
    for(int t=0; t<3; t++)
      debug_present_pose(t+3) = present_orientation_rpy(t);
    RM_LOG::WARN("iter : ", count,0);
    RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
    RM_LOG::PRINTLN("tar_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose,16);
    RM_LOG::PRINTLN("pre_pose");
    RM_LOG::PRINTLN_VECTOR(debug_present_pose,16);
    RM_LOG::PRINTLN("delta_pose");
    RM_LOG::PRINTLN_VECTOR(debug_target_pose-debug_present_pose,16);
    #endif
    ////////////////////////////debug//////////////////////////////////

    if (new_Ek < 1E-12)
    {
      /////////////////////////////debug/////////////////////////////////
      #if defined(KINEMATICS_DEBUG)
      RM_LOG::WARN("iter : ", count,0);
      RM_LOG::WARN("Ek : ", new_Ek*1000000000000);
      RM_LOG::ERROR("Success");
      RM_LOG::PRINTLN("------------------------------------");
      #endif
      //////////////////////////debug//////////////////////////////////
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      {
        goal_joint_value->at(index).velocity = 0.0;
        goal_joint_value->at(index).acceleration = 0.0;
        goal_joint_value->at(index).effort = 0.0;
      }
      return true;
    }
    else if (new_Ek < pre_Ek)
    {
      pre_Ek = new_Ek;
    }
    else
    {
      present_angle = _manipulator.getAllActiveJointPosition();
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle.push_back(present_angle.at(index) - (gamma * angle_changed(index)));
      _manipulator.setAllActiveJointPosition(set_angle);

      solveForwardKinematics(&_manipulator);
    }
  }
  robotis_manipulator_log::error("[OpenManipulator Chain Custom]fail to solve inverse kinematics");
  *goal_joint_value = {};
  return false;
}















































