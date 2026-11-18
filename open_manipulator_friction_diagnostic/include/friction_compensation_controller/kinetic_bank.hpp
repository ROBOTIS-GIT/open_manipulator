// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__KINETIC_BANK_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__KINETIC_BANK_HPP_
#include "friction_compensation_controller/stateful_friction.hpp"
namespace friction_compensation_controller
{
// Constructed during configuration only. No parameter access or allocations in update().
struct KineticBankJoint
{
  int64_t kind=0;  // smooth, periodic, signed table
  double width=1.,load_scale=1.,frequency=0.,fraction=1.;
  std::vector<int64_t> orders{1,2};
  std::vector<double> speeds,loads,positive,negative;
  bool load_hat_mean_extension=false;
  StatefulParameters state{};
  double dither_amplitude=0.,dither_frequency=1.,dither_full_velocity=.004,
    dither_zero_velocity=.012,dither_ramp=.5;
};
inline StatefulFriction make_kinetic_bank_joint(const KineticBankJoint & c)
{
  std::optional<DirectionalLoadLaw> law;
  if(c.kind==0) {
    law.emplace(DirectionalLoadLaw::SmoothLoad{c.width,c.load_scale,c.speeds,c.positive,c.negative});
  } else if(c.kind==1) {
    law.emplace(DirectionalLoadLaw::PeriodicLoad{c.width,c.load_scale,c.frequency,c.orders,c.speeds,c.positive,c.negative,
      c.loads.size()>1?c.loads:std::vector<double>{},c.load_hat_mean_extension});
  } else if(c.kind==2) {
    law.emplace(c.loads,c.speeds,c.positive,c.negative);
  } else {throw std::invalid_argument("Unsupported per-axis kinetic law");}
  if(!std::isfinite(c.dither_amplitude)||c.dither_amplitude<0.) {
    throw std::invalid_argument("Invalid per-axis rest amplitude");
  }
  std::optional<RestDither> rest;
  if(c.dither_amplitude>0.) {
    rest.emplace(RestDitherParameters{c.dither_amplitude,c.dither_frequency,
      c.dither_full_velocity,c.dither_zero_velocity,c.dither_ramp});
  }
  return StatefulFriction(c.state,std::nullopt,c.fraction,std::move(rest),c.dither_amplitude>0.,std::move(law));
}
// Stage every axis before committing commands: a later-axis fault suppresses
// all assistance in this same update. Storage is allocated by configure().
template<class Position, class Velocity, class Gravity>
unsigned int step_kinetic_bank(std::vector<std::optional<StatefulFriction>> & bank,
  Position position, Velocity velocity, Gravity gravity, double dt,
  bool observation_ready, bool enabled, std::vector<FrictionDiagnostics> & details)
{
  unsigned int reasons=0;
  for(size_t i=0;i<bank.size();++i) {
    details[i]={};details[i].flags=Disabled;
    if(!bank[i]) {continue;}
    bank[i]->step(position(i),velocity(i),dt,observation_ready,enabled,gravity(i));
    details[i]=bank[i]->diagnostics();
    if(bank[i]->fault()) {reasons|=bank[i]->fault_reason();}
  }
  if(reasons) {
    for(auto & d:details) {d.applied=0.;d.flags|=Fault;}
  }
  return reasons;
}
}
#endif
