#pragma once
#include "joint_impedance_controller/impedance_law.hpp"
#include "friction_compensation_controller/stateful_friction.hpp"
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <memory>
#include <optional>
#include <vector>

namespace omy_joint_impedance
{
// Diagnostic motor-current residual. No command torque or compensation ratio is
// treated as a measurement; the independent friction bank uses its raw model.
class ExternalTorqueObserver
{
public:
  bool configure(const KDL::Tree & tree, const std::vector<double> & scale,
    const std::vector<std::optional<friction_compensation_controller::StatefulFriction>> & bank,
    double model_period, double filter_time, double acceleration_filter_time)
  {
    if (tree.getNrOfJoints()<6 || scale.size()!=6 || bank.size()!=6 ||
      !std::isfinite(model_period) || model_period<=0. ||
      !std::isfinite(filter_time) || filter_time<=0. ||
      !std::isfinite(acceleration_filter_time) || acceleration_filter_time<=0.) {return false;}
    for (double x:scale) {if (!std::isfinite(x)) {return false;}}
    tree_=tree;bank_=bank;
    for (auto & joint:bank_) {if (joint) {joint->reset();}}
    const auto size=tree_.getNrOfJoints();
    q_.resize(size);v_.resize(size);a_.resize(size);zero_.resize(size);
    gravity_.resize(size);dynamics_.resize(size);
    KDL::SetToZero(q_);KDL::SetToZero(v_);KDL::SetToZero(a_);KDL::SetToZero(zero_);
    solver_=std::make_unique<KDL::TreeIdSolver_RNE>(tree_,KDL::Vector(0,0,-9.81));
    std::copy(scale.begin(),scale.end(),scale_.begin());
    period_=model_period;filter_=filter_time;acceleration_filter_=acceleration_filter_time;
    reset();return true;
  }
  void reset()
  {
    previous_velocity_={};acceleration_={};external_={};measured_={};
    for (auto & joint:bank_) {if (joint) {joint->reset();}}
    primed_=false;model_elapsed_=0.;
  }
  bool update(const Vector & position,const Vector & velocity,const Vector & measured,
    double dt,bool ready)
  {
    if (!solver_ || !std::isfinite(dt) || dt<=0. || dt>.02 || !ready) {reset();return false;}
    for (size_t i=0;i<6;++i) {
      if (!std::isfinite(position[i]) || !std::isfinite(velocity[i]) || !std::isfinite(measured[i])) {
        reset();return false;
      }
    }
    const bool first=!primed_;
    const double alpha=dt/(acceleration_filter_+dt);
    for (size_t i=0;i<6;++i) {
      acceleration_[i]=first?0.:acceleration_[i]+alpha*((velocity[i]-previous_velocity_[i])/dt-acceleration_[i]);
      previous_velocity_[i]=velocity[i];q_(i)=position[i];v_(i)=velocity[i];a_(i)=acceleration_[i];
    }
    model_elapsed_+=dt;
    if (first || model_elapsed_>=period_) {
      if (solver_->CartToJnt(q_,zero_,zero_,no_external_,gravity_)<0 ||
        solver_->CartToJnt(q_,v_,a_,no_external_,dynamics_)<0) {reset();return false;}
      model_elapsed_=std::fmod(model_elapsed_,period_);
    }
    const double beta=dt/(filter_+dt);
    for (size_t i=0;i<6;++i) {
      double passive_friction=0.;
      if (bank_[i]) {
        bank_[i]->step(position[i],velocity[i],dt,true,true,gravity_(i)*scale_[i]);
        if (bank_[i]->fault()) {reset();return false;}
        // requested is before compensation fraction; remove the commanded
        // dither waveform, which is not passive transmission friction.
        passive_friction=bank_[i]->diagnostics().requested-bank_[i]->dither_components().torque;
      }
      const double residual=dynamics_(i)*scale_[i]+passive_friction-measured[i];
      if (!std::isfinite(residual)) {reset();return false;}
      external_[i]=first?residual:external_[i]+beta*(residual-external_[i]);
      measured_[i]=measured[i];
    }
    primed_=true;return true;
  }
  const Vector & external() const {return external_;}
  const Vector & measured() const {return measured_;}
private:
  KDL::Tree tree_;
  KDL::JntArray q_,v_,a_,zero_,gravity_,dynamics_;
  KDL::WrenchMap no_external_;
  std::unique_ptr<KDL::TreeIdSolver_RNE> solver_;
  std::vector<std::optional<friction_compensation_controller::StatefulFriction>> bank_;
  Vector scale_{},previous_velocity_{},acceleration_{},external_{},measured_{};
  double period_{.02},filter_{.1},acceleration_filter_{.05},model_elapsed_{0.};
  bool primed_{false};
};
}  // namespace omy_joint_impedance
