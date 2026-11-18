#include "cartesian_contact_controller/cartesian_law.hpp"
#include <osqp.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace omy_cartesian
{
bool valid_pose(const Pose & p) {
  return p.position.allFinite() && p.orientation.coeffs().allFinite() &&
    std::isfinite(p.orientation.norm()) && p.orientation.norm()>1e-9;
}
Vector pose_error(const Pose & target,const Pose & actual) {
  Vector e=Vector::Zero();e.head<3>()=target.position-actual.position;
  Eigen::Quaterniond r=target.orientation.normalized()*actual.orientation.normalized().conjugate();
  r.normalize();if(r.w()<0.) r.coeffs()*=-1.;
  const double n=r.vec().norm();
  e.tail<3>()=r.vec()*(n>1e-12?2.*std::atan2(n,r.w())/n:2.);return e;
}

struct CartesianLaw::Solver
{
  OSQPWorkspace * work{nullptr};
  std::array<c_float,27> p{};
  std::array<c_int,27> pi{};
  std::array<c_int,13> pp{};
  std::array<c_float,36> a{};
  std::array<c_int,36> ai{};
  std::array<c_int,13> ap{};
  std::array<c_float,12> gradient{}, primal{};
  std::array<c_float,24> lower{}, upper{}, dual{};
  csc P{}, A{};
  ~Solver() {if (work) {osqp_cleanup(work);}}
  bool setup(const Settings & s)
  {
    int k=0;
    for(int col=0;col<12;++col) {
      pp[col]=k;
      if(col<6) {
        for(int row=0;row<=col;++row) {pi[k]=row;p[k++]=row==col?1.:0.;}
      } else {pi[k]=col;p[k++]=s.error_slack_weight;}
    }
    pp[12]=k; k=0;
    for(int col=0;col<12;++col) {
      ap[col]=k;
      const int j=col%6;
      ai[k]=col; a[k++]=1.;
      ai[k]=12+j; a[k++]=col<6?1.:-1.;
      ai[k]=18+j; a[k++]=-1.;
    }
    ap[12]=k;
    P.m=12;P.n=12;P.nz=-1;P.nzmax=27;P.p=pp.data();P.i=pi.data();P.x=p.data();
    A.m=24;A.n=12;A.nz=-1;A.nzmax=36;A.p=ap.data();A.i=ai.data();A.x=a.data();
    lower.fill(-OSQP_INFTY);upper.fill(OSQP_INFTY);
    for(int i=6;i<12;++i) {lower[i]=0.;}
    OSQPData data{};data.n=12;data.m=24;data.P=&P;data.A=&A;
    data.q=gradient.data();data.l=lower.data();data.u=upper.data();
    OSQPSettings settings;osqp_set_default_settings(&settings);
    settings.verbose=0;settings.polish=0;settings.warm_start=1;
    settings.adaptive_rho=1;settings.adaptive_rho_interval=25;
    settings.max_iter=s.solver_iterations;
    settings.eps_abs=s.solver_tolerance;settings.eps_rel=0.;
    settings.check_termination=5;
    return osqp_setup(&work,&data,&settings)==0;
  }
};

CartesianLaw::CartesianLaw() = default;
CartesianLaw::~CartesianLaw() = default;

bool CartesianLaw::configure(const Settings & s)
{
  active_=false;
  for(const auto * v:{&s.stiffness,&s.damping,&s.inner_stiffness,&s.inner_damping,
      &s.task_limit,&s.correction_limit,&s.motion_limit,
      &s.nominal_error}) {
    if(!v->allFinite() || (v->array()<0.).any()) {return false;}
  }
  if(!s.mass.allFinite() || (s.mass.array()<=0.).any() || (s.inner_stiffness.array()<=0.).any() || (s.inner_damping.array()<=0.).any() ||
    (s.motion_limit.array()<=0.).any() || (s.correction_limit.array()<=0.).any() ||
    (s.nominal_error.array()<=0.).any() || !std::isfinite(s.error_slack_weight) ||
    s.error_slack_weight<=0. || !std::isfinite(s.max_period) || s.max_period<=0. ||
    s.max_period>.05 || !std::isfinite(s.solver_tolerance) || s.solver_tolerance<=0. ||
    s.solver_tolerance>.001 || s.solver_iterations<1 || s.solver_iterations>10000) {return false;}
  auto candidate=std::make_unique<Solver>();
  if(!candidate->setup(s)) {return false;}
  settings_=s;solver_=std::move(candidate);return true;
}

bool CartesianLaw::activate(const Pose & q,const Vector & v)
{
  active_=false;
  if(!solver_ || !valid_pose(q) || !v.allFinite()) {return false;}
  pose_=q;pose_.orientation.normalize();velocity_=v;
  solver_->primal.fill(0.);solver_->dual.fill(0.);
  if(osqp_warm_start(solver_->work,solver_->primal.data(),solver_->dual.data())!=0) {return false;}
  active_=true;return true;
}

bool CartesianLaw::step(const Pose & q,const Vector & v,const Pose & goal,
  double dt,Result & out)
{
  out=Result{};
  if(!active_ || !valid_pose(q) || !v.allFinite() || !valid_pose(goal) ||
    !std::isfinite(dt) || dt<1e-5 || dt>settings_.max_period) {
    out.failure=1;return false;
  }
  const auto & s=settings_;
  const Vector raw_task=s.stiffness.cwiseProduct(pose_error(goal,pose_))-s.damping.cwiseProduct(velocity_);
  const Vector task=raw_task.cwiseMax(-s.task_limit).cwiseMin(s.task_limit);
  const Vector common=task;
  const Vector free_acceleration=task.cwiseQuotient(s.mass);
  const Matrix weight=(s.mass/s.mass.mean()).asDiagonal();
  const Vector gradient=-weight*free_acceleration;
  const Vector error0=pose_error(pose_,q)+dt*(velocity_-v);
  const Vector b=s.inner_stiffness.cwiseProduct(error0)+s.inner_damping.cwiseProduct(velocity_-v);
  const Vector coefficient=dt*dt*s.inner_stiffness+dt*s.inner_damping;
  Vector lo,hi;
  for(int i=0;i<6;++i) {
    lo[i]=std::max({
      (-s.correction_limit[i]-b[i])/coefficient[i],
      (-s.motion_limit[i]-common[i]-b[i])/coefficient[i]});
    hi[i]=std::min({
      (s.correction_limit[i]-b[i])/coefficient[i],
      (s.motion_limit[i]-common[i]-b[i])/coefficient[i]});
    if(!std::isfinite(lo[i]) || !std::isfinite(hi[i]) || lo[i]>hi[i]) {out.failure=2;return false;}
    solver_->lower[i]=lo[i];solver_->upper[i]=hi[i];
    solver_->upper[12+i]=(s.nominal_error[i]-error0[i])/(dt*dt);
    solver_->upper[18+i]=(s.nominal_error[i]+error0[i])/(dt*dt);
    solver_->gradient[i]=gradient[i];
  }
  int k=0;
  for(int col=0;col<6;++col) {for(int row=0;row<=col;++row) {solver_->p[k++]=weight(row,col);}}
  const auto start=std::chrono::steady_clock::now();
  if(osqp_update_P(solver_->work,solver_->p.data(),nullptr,27)!=0 ||
    osqp_update_lin_cost(solver_->work,solver_->gradient.data())!=0 ||
    osqp_update_bounds(solver_->work,solver_->lower.data(),solver_->upper.data())!=0) {
    out.failure=3;return false;
  }
  // A contact reversal can make the previous dual iterate a poor warm start.
  // Retry at most once from a feasible point of THIS QP, never from an old torque.
  int total_iterations=0;
  for(int attempt=0;attempt<2;++attempt) {
    if(osqp_solve(solver_->work)!=0) {out.failure=3;return false;}
    total_iterations+=solver_->work->info->iter;
    out.solver_status=solver_->work->info->status_val;
    if(out.solver_status!=OSQP_MAX_ITER_REACHED || attempt==1) {break;}
    solver_->dual.fill(0.);
    for(int i=0;i<6;++i) {
      const double initial=std::clamp(free_acceleration[i],lo[i],hi[i]);
      solver_->primal[i]=initial;
      solver_->primal[6+i]=std::max({0.,initial-solver_->upper[12+i],
                                    -initial-solver_->upper[18+i]});
    }
    if(osqp_update_rho(solver_->work,0.1)!=0 ||
      osqp_warm_start(solver_->work,solver_->primal.data(),solver_->dual.data())!=0) {
      out.failure=3;return false;
    }
  }
  out.solve_time_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  out.iterations=total_iterations;
  const double residual_tolerance=10.*s.solver_tolerance;
  if((out.solver_status!=OSQP_SOLVED && out.solver_status!=OSQP_SOLVED_INACCURATE) ||
    !std::isfinite(solver_->work->info->pri_res) ||
    !std::isfinite(solver_->work->info->dua_res) ||
    solver_->work->info->pri_res>residual_tolerance ||
    solver_->work->info->dua_res>residual_tolerance) {out.failure=3;return false;}
  Vector a;
  for(int i=0;i<6;++i) {
    a[i]=solver_->work->solution->x[i];
    out.error_slack[i]=dt*dt*solver_->work->solution->x[6+i];
    const double slack=solver_->work->solution->x[6+i];
    if(slack < -residual_tolerance ||
      a[i]-slack > solver_->upper[12+i]+residual_tolerance ||
      -a[i]-slack > solver_->upper[18+i]+residual_tolerance) {out.failure=3;return false;}
  }
  if(!a.allFinite() || !out.error_slack.allFinite() ||
    (a.array()<lo.array()-10.*s.solver_tolerance).any() ||
    (a.array()>hi.array()+10.*s.solver_tolerance).any()) {out.failure=3;return false;}
  // Remove only solver-level roundoff, not an after-the-fact torque saturation.
  a=a.cwiseMax(lo).cwiseMin(hi);
  out.acceleration=a;out.task=common;out.correction=b+coefficient.cwiseProduct(a);
  out.effort=out.task+out.correction;
  const Vector next_velocity=velocity_+dt*a;
  Pose next_pose=pose_;
  next_pose.position+=dt*next_velocity.head<3>();
  const Eigen::Vector3d rotation=dt*next_velocity.tail<3>();
  const double angle=rotation.norm();
  if(angle>1e-12) next_pose.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(angle,rotation/angle))*pose_.orientation;
  next_pose.orientation.normalize();
  if(!out.effort.allFinite() || !valid_pose(next_pose) || !next_velocity.allFinite()) {
    out.failure=3;return false;
  }
  pose_=next_pose;velocity_=next_velocity;
  out.nominal_pose=pose_;out.nominal_velocity=velocity_;
  return true;
}
}  // namespace omy_cartesian
