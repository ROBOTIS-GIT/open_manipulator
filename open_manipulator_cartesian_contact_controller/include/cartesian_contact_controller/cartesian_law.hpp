#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

namespace omy_cartesian
{
using Vector = Eigen::Matrix<double, 6, 1>;
using Matrix = Eigen::Matrix<double, 6, 6>;

struct Pose { Eigen::Vector3d position{Eigen::Vector3d::Zero()}; Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()}; };
bool valid_pose(const Pose & pose);
Vector pose_error(const Pose & target, const Pose & actual);

struct Settings
{
  Vector stiffness{(Vector() << 100.,100.,100.,3.,3.,3.).finished()};
  Vector damping{(Vector() << 20.,20.,20.,1.,1.,1.).finished()};
  Vector inner_stiffness{(Vector() << 600.,600.,600.,20.,20.,20.).finished()};
  Vector inner_damping{(Vector() << 60.,60.,60.,3.,3.,3.).finished()};
  Vector task_limit{(Vector() << 10.,10.,10.,1.,1.,1.).finished()};
  Vector correction_limit{(Vector() << 20.,20.,20.,2.,2.,2.).finished()};
  Vector motion_limit{(Vector() << 30.,30.,30.,3.,3.,3.).finished()};
  Vector nominal_error{(Vector() << .03,.03,.03,.1,.1,.1).finished()};
  Vector mass{(Vector() << 2.,2.,2.,.1,.1,.1).finished()};
  double error_slack_weight{1.};
  double max_period{.01};
  double solver_tolerance{1e-5};
  int solver_iterations{400};
};

struct Result
{
  Pose nominal_pose;
  Vector nominal_velocity{Vector::Zero()};
  Vector acceleration{Vector::Zero()}, task{Vector::Zero()}, correction{Vector::Zero()};
  Vector effort{Vector::Zero()}, error_slack{Vector::Zero()};
  int solver_status{0}, iterations{0};
  // 0 OK; 1 invalid state/model/dt; 2 infeasible hard bounds; 3 solver failure.
  int failure{0};
  double solve_time_s{0.};
};

// Root-frame Cartesian virtual-body QP. Units xyz: m,N,kg; rotation: rad,Nm,kg*m^2.
// Only virtual acceleration is optimized; the caller maps wrench with actual J transpose.
// Rotational constraints use a first-order local SO(3) error prediction.
class CartesianLaw
{
public:
  CartesianLaw();
  ~CartesianLaw();
  CartesianLaw(const CartesianLaw &) = delete;
  CartesianLaw & operator=(const CartesianLaw &) = delete;
  bool configure(const Settings & settings);
  bool activate(const Pose & pose, const Vector & velocity);
  bool step(const Pose & pose, const Vector & velocity, const Pose & goal,
    double dt, Result & out);
  const Pose & pose() const {return pose_;}
  const Vector & velocity() const {return velocity_;}
private:
  struct Solver;
  std::unique_ptr<Solver> solver_;
  Settings settings_;
  Pose pose_;
  Vector velocity_{Vector::Zero()};
  bool active_{false};
};
}  // namespace omy_cartesian
