// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__ELASTOPLASTIC_REPLAY_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__ELASTOPLASTIC_REPLAY_HPP_
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace friction_compensation_controller
{
// Research candidate: offline stress tests and bounded J6 diagnostic trials only.
// Iwatani & Kikuuwe (2017), eq.18, without dither.
// https://home.hiroshima-u.ac.jp/kikuuwe/pub/17_jcmsi.pdf
// The directional yield levels and rate term are model parameters. K requires
// presliding evidence; alpha and external damping are controller design values.
// The production freedrive loader does not select this research candidate.
// CurrentProbeController may exercise it in a logged, bounded OFF/ON experiment.
class ElastoplasticReplay
{
public:
  struct Output
  {
    double elastic = 0.;
    double rate_term = 0.;
    double damping = 0.;
    double total = 0.;
    double plastic_velocity = 0.;
  };
  ElastoplasticReplay(double stiffness, double decay_rate, double damping)
  : stiffness_(stiffness), decay_(decay_rate), damping_(damping)
  {
    if (!std::isfinite(stiffness) || stiffness <= 0. ||
      !std::isfinite(decay_rate) || decay_rate <= 0. ||
      !std::isfinite(damping) || damping < 0.)
    {throw std::invalid_argument("Invalid replay controller parameters");}
  }
  void reset() {primed_ = false; plastic_ = 0.;}

  // Phi must be the signed, continuous rate-dependent term with Phi(0)=0.
  // This boundary accepts separate positive/negative levels, so a paired mean
  // cannot silently become a claim of identified directional friction.
  template<class RateLaw>
  Output step(double position, double observed_velocity, double dt,
    double positive_yield, double negative_yield, RateLaw phi, bool enabled = true,
    double stiffness_override = 0.)
  {
    if (!std::isfinite(position) || !std::isfinite(observed_velocity) ||
      !std::isfinite(dt) || dt <= 0. || dt > .02 ||
      !std::isfinite(positive_yield) || positive_yield <= 0. ||
      !std::isfinite(negative_yield) || negative_yield <= 0. ||
      !std::isfinite(stiffness_override) || stiffness_override < 0.)
    {reset(); throw std::invalid_argument("Invalid replay sample/model");}
    if (!enabled) {reset(); return {};}
    const double stiffness=stiffness_override>0. ? stiffness_override : stiffness_;
    if (!primed_) {
      plastic_=previous_position_=position;last_stiffness_=stiffness;primed_=true;return {};
    }
    if (stiffness != last_stiffness_) {
      // Preserve the previous elastic torque when nominal load changes. This
      // coordinate change must not enter the plastic-velocity/rate term.
      plastic_=previous_position_-(previous_position_-plastic_)*last_stiffness_/stiffness;
    }
    Output out;
    const double trial = stiffness*(position-plastic_)/(1.+dt*decay_);
    out.elastic = std::clamp(trial, -negative_yield, positive_yield);
    const double next = position-out.elastic/stiffness;
    out.plastic_velocity = (next-plastic_)/dt;
    out.rate_term = phi(out.plastic_velocity);
    // Additional hand-guiding damping, separate from the fitted friction law.
    // With delayed velocity, this term alone is NOT a sampled passivity proof.
    out.damping = -damping_*observed_velocity;
    out.total = out.elastic+out.rate_term+out.damping;
    if (!std::isfinite(out.total)) {
      reset(); throw std::invalid_argument("Nonfinite rate-law output");
    }
    plastic_ = next;
    previous_position_=position;last_stiffness_=stiffness;
    return out;
  }
private:
  double stiffness_, decay_, damping_, plastic_ = 0.;
  double previous_position_=0.,last_stiffness_=0.;
  bool primed_ = false;
};
}  // namespace friction_compensation_controller
#endif
