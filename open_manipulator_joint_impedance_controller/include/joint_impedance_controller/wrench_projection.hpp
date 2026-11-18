#pragma once
#include "joint_impedance_controller/impedance_law.hpp"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <memory>
#include <string>
#include <vector>

namespace omy_joint_impedance
{
// tau = J_tool^T * wrench. This is an equivalent wrench, not a force sensor.
class WrenchProjection
{
public:
  struct Result
  {
    Vector wrench{};  // Fx,Fy,Fz [N], Tx,Ty,Tz [Nm], about/in tip frame.
    double minimum_singular_value_m{0.}, residual_nm{0.};
    bool valid{false};
  };
  bool configure(const KDL::Tree & tree, const std::vector<std::string> & joints,
    const std::string & root, const std::string & tip, double damping_m, double length_m)
  {
    if (!std::isfinite(damping_m) || damping_m <= 0. ||
      !std::isfinite(length_m) || length_m <= 0. || joints.size() != 6 ||
      !tree.getChain(root, tip, chain_) || chain_.getNrOfJoints() != 6) {return false;}
    size_t index=0;
    for (const auto & segment : chain_.segments) {
      if (segment.getJoint().getType() != KDL::Joint::Fixed &&
        segment.getJoint().getName() != joints[index++]) {return false;}
    }
    damping_=damping_m; length_=length_m;
    position_.resize(6);jacobian_.resize(6);
    jacobian_solver_=std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    forward_solver_=std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    return true;
  }
  bool calculate(const Vector & position, const Vector & torque, Result & result)
  {
    result={};
    if (!jacobian_solver_ || !forward_solver_) {return false;}
    for (size_t i=0;i<6;++i) {
      if (!std::isfinite(position[i]) || !std::isfinite(torque[i])) {return false;}
      position_(i)=position[i]; torque_(i)=torque[i];
    }
    KDL::Frame tip;
    if (jacobian_solver_->JntToJac(position_,jacobian_)<0 ||
      forward_solver_->JntToCart(position_,tip)<0) {return false;}
    // KDL already references the tip origin. Rotate axes only, without shifting
    // the moment to the base origin: WrenchStamped.frame_id will be the tip.
    jacobian_.changeBase(tip.M.Inverse());
    transpose_=jacobian_.data.transpose();
    // Normalize force/moment units: solve z=[F,M/length] in newtons. All columns
    // of A then have units of metres, so the damping has a defined length unit.
    normalized_=transpose_;normalized_.rightCols<3>()*=length_;
    svd_.compute(normalized_,Eigen::ComputeFullU|Eigen::ComputeFullV);
    if (svd_.info()!=Eigen::Success) {return false;}
    weights_=svd_.singularValues();
    result.minimum_singular_value_m=weights_[5];
    for (size_t i=0;i<6;++i) {weights_[i]/=weights_[i]*weights_[i]+damping_*damping_;}
    wrench_=svd_.matrixV()*weights_.asDiagonal()*svd_.matrixU().transpose()*torque_;
    wrench_.tail<3>()*=length_;
    result.residual_nm=(transpose_*wrench_-torque_).norm();
    if (!wrench_.allFinite() || !std::isfinite(result.residual_nm)) {return false;}
    for (size_t i=0;i<6;++i) {result.wrench[i]=wrench_[i];}
    result.valid=result.minimum_singular_value_m>=damping_;
    return true;
  }
private:
  using Matrix=Eigen::Matrix<double,6,6>;
  using Column=Eigen::Matrix<double,6,1>;
  KDL::Chain chain_;
  KDL::JntArray position_;
  KDL::Jacobian jacobian_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> forward_solver_;
  Matrix transpose_{},normalized_{};
  Column torque_{},weights_{},wrench_{};
  Eigen::JacobiSVD<Matrix> svd_;
  double damping_{.005},length_{.25};
};
}  // namespace omy_joint_impedance
