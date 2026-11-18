#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace omy_task
{
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
struct Pose
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};
struct Gains
{
  Vector6 stiffness{(Vector6() << 100., 100., 100., 3., 3., 3.).finished()};
  Vector6 damping{(Vector6() << 20., 20., 20., 1., 1., 1.).finished()};
};
struct Result
{
  Vector6 error{Vector6::Zero()}, twist{Vector6::Zero()};
  Vector6 wrench{Vector6::Zero()}, torque{Vector6::Zero()};
};
inline bool valid_pose(const Pose & pose)
{
  const double norm = pose.orientation.norm();
  return pose.position.allFinite() && pose.orientation.coeffs().allFinite() &&
         std::isfinite(norm) && norm > 1e-9;
}
inline bool valid_gains(const Gains & gains)
{
  return gains.stiffness.allFinite() && gains.damping.allFinite() &&
         (gains.stiffness.array() >= 0.).all() && (gains.damping.array() >= 0.).all();
}
// All vectors are expressed in the chain root frame, at the tip origin.
// Jacobian rows: linear xyz, angular xyz. No inverse Jacobian is required.
inline bool evaluate(const Gains & gains, const Pose & actual, const Pose & target,
  const Matrix6 & jacobian, const Vector6 & velocity, Result & result)
{
  result = Result{};
  if (!valid_gains(gains) || !valid_pose(actual) || !valid_pose(target) ||
    !jacobian.allFinite() || !velocity.allFinite()) {return false;}
  Eigen::Quaterniond rotation = target.orientation.normalized() *
    actual.orientation.normalized().conjugate();
  rotation.normalize();
  if (rotation.w() < 0.) {rotation.coeffs() *= -1.;}
  const double sine_half = rotation.vec().norm();
  result.error.head<3>() = target.position - actual.position;
  if (sine_half > 1e-12) {
    result.error.tail<3>() = rotation.vec() * (2. * std::atan2(sine_half, rotation.w()) / sine_half);
  } else {result.error.tail<3>() = 2. * rotation.vec();}
  result.twist = jacobian * velocity;
  result.wrench = gains.stiffness.cwiseProduct(result.error) - gains.damping.cwiseProduct(result.twist);
  result.torque = jacobian.transpose() * result.wrench;
  return result.wrench.allFinite() && result.torque.allFinite();
}
}  // namespace omy_task
