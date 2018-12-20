#ifndef KINEMATICS_DYNAMICS_H
#define KINEMATICS_DYNAMICS_H

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
#endif

using namespace Eigen;
using namespace ROBOTIS_MANIPULATOR;

namespace KINEMATICS_DYNAMICS
{
class Chain : public ROBOTIS_MANIPULATOR::KinematicsDynamics
{
private:
  STRING inverse_solver_option_;
public:
  Chain():inverse_solver_option_("chain_custom_inverse_kinematics"){}
  virtual ~Chain(){}


  virtual void setOption(const void *arg);
  virtual void updatePassiveJointValue(Manipulator *manipulator);
  virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);

  //kinematics
  virtual void forwardKinematics(Manipulator *manipulator);
  virtual bool inverseKinematics(Manipulator *manipulator, Name tool_name, KinematicPose target_pose, std::vector<double>* goal_joint_position);

  //dynamics
  virtual bool forwardDynamics(Manipulator *manipulator);
  virtual bool inverseDynamics(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* active_joint_value);

  //kinematics solver
  void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool inverseSolverUsingJacobian(Manipulator *manipulator, Name tool_name, KinematicPose target_pose, std::vector<double>* goal_joint_position);
  bool inverseSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, KinematicPose target_pose, std::vector<double>* goal_joint_position);
  bool inverseSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, KinematicPose target_pose, std::vector<double>* goal_joint_position);
  bool chainCustomInverseKinematics(Manipulator *manipulator, Name tool_name, KinematicPose target_pose, std::vector<double>* goal_joint_position);

  //dynamics solver
  bool FDSolverUsingChainRule(Manipulator *manipulator);
  bool IDSolverUsingJacobian(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue>* active_joint_value);
};

}
#endif // KINEMATICS_DYNAMICS_H
