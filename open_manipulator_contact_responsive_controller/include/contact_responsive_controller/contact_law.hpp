#pragma once
#include <Eigen/Core>
#include <memory>

namespace omy_contact
{
using Vector = Eigen::Matrix<double, 6, 1>;
using Matrix = Eigen::Matrix<double, 6, 6>;

struct Settings
{
  Vector stiffness{(Vector() << 5.,5.,4.,2.,1.,.8).finished()};
  Vector damping{(Vector() << 2.,2.,1.5,.5,.3,.2).finished()};
  Vector inner_stiffness{(Vector() << 60.,80.,60.,15.,10.,5.).finished()};
  Vector inner_damping{(Vector() << 8.,10.,8.,2.,1.,.8).finished()};
  Vector task_limit{(Vector() << 2.,3.,2.,1.,.6,.4).finished()};
  Vector correction_limit{(Vector() << 4.,6.,4.,1.5,1.,.8).finished()};
  Vector motion_limit{(Vector() << 6.,8.,6.,2.,1.5,1.).finished()};
  Vector nominal_error{Vector::Constant(.04)};
  double error_slack_weight{1.};
  double max_period{.01};
  double solver_tolerance{1e-5};
  int solver_iterations{400};
};

struct Result
{
  Vector nominal_position{Vector::Zero()}, nominal_velocity{Vector::Zero()};
  Vector acceleration{Vector::Zero()}, task{Vector::Zero()}, correction{Vector::Zero()};
  Vector effort{Vector::Zero()}, error_slack{Vector::Zero()};
  int solver_status{0}, iterations{0};
  // 0 OK; 1 invalid state/model/dt; 2 infeasible hard bounds; 3 solver failure.
  int failure{0};
  double solve_time_s{0.};
};

// Pure torque-control core. Allocations/OSQP setup occur only in configure().
// The real plant is gravity/friction compensated by the caller. The nominal
// plant retains full inertia/Coriolis; residual errors are not labelled friction.
class ContactLaw
{
public:
  ContactLaw();
  ~ContactLaw();
  ContactLaw(const ContactLaw &) = delete;
  ContactLaw & operator=(const ContactLaw &) = delete;
  bool configure(const Settings & settings);
  bool activate(const Vector & q, const Vector & velocity);
  bool step(const Vector & q, const Vector & velocity, const Vector & goal,
    const Matrix & inertia, const Vector & coriolis, double dt, Result & out,
    const Vector & feedforward = Vector::Zero());
  const Vector & position() const {return position_;}
  const Vector & velocity() const {return velocity_;}
private:
  struct Solver;
  std::unique_ptr<Solver> solver_;
  Settings settings_;
  Vector position_{Vector::Zero()}, velocity_{Vector::Zero()};
  bool active_{false};
};
}  // namespace omy_contact
