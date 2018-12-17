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
  virtual bool inverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double>* goal_joint_value);

  //dynamics
  virtual bool forwardDynamics(Manipulator *manipulator);
  virtual bool inverseDynamics(Manipulator *manipulator, Name tool_name, std::vector<WayPoint> tool_way_point, std::vector<WayPoint>* active_joint_way_point);


  //kinematics solver
  void FKSolverUsingChainRule(Manipulator *manipulator, Name component_name);
  bool IKSolverUsingJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double>* goal_joint_value);
  bool IKSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double>* goal_joint_value);
  bool IKSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double>* goal_joint_value);
  bool chainCustomInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double>* goal_joint_value);

  //dynamics solver
  bool FDSolverUsingJacobian(Manipulator *manipulator);
  bool IDSolverUsingJacobian(Manipulator *manipulator, Name tool_name, WayPoint tool_way_point, std::vector<WayPoint>* active_joint_way_point);
};

}
#endif // KINEMATICS_DYNAMICS_H
