// Known-input experiment or manual observation without automatic excitation.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__COMMON_RELEASE_SEQUENCE_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__COMMON_RELEASE_SEQUENCE_HPP_
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "friction_compensation_controller/stateful_friction.hpp"
namespace friction_compensation_controller {
inline bool common_release_start_allowed(size_t,double,double angle,bool)
{
  // A preloaded Goal Current does not support a loaded arm during the drive's
  // brake-release/goal-update delay. Loaded current-mode startup is unqualified.
  return angle==0.;
}
inline bool common_release_speed_allowed(size_t target,double speed,bool manual)
{
  return target<6 && (speed==.02 || (!manual &&
    ((target==0 && (speed==.3 || speed==.6)) || (target==1 && speed==.3))));
}
// Manual observations use the full periodic gravity table, with no position
// window. Known-input diagnostics retain their original local table and limits.
inline double probe_gravity_at(const std::vector<double> & angles,
  const std::vector<double> & values,double q,bool manual)
{
  if(!std::isfinite(q))return NAN;
  if(manual) {
    const double period=angles.back()-angles.front();
    q=angles.front()+std::fmod(std::fmod(q-angles.front(),period)+period,period);
  }
  const auto upper=std::upper_bound(angles.begin(),angles.end(),q);
  if(upper==angles.begin() || upper==angles.end())return NAN;
  const size_t i=static_cast<size_t>(upper-angles.begin());
  return values[i-1]+(values[i]-values[i-1])*(q-angles[i-1])/(angles[i]-angles[i-1]);
}
inline int probe_motion_fault(const std::array<double,18> & state,
  const std::array<double,6> & hold,size_t target,double excursion,double hold_error,
  double speed,bool manual)
{
  if(manual)return 0;
  int error=0;
  for(size_t j=0;j<6;++j) {
    if(std::abs(state[3*j]-hold[j])>(j==target?excursion:hold_error))error=3;
    if(std::abs(state[3*j+1])>speed)error=4;
  }
  return error;
}
// Component isolation for finite diagnostics only. The frozen model remains
// archived, but a dither-only result must not qualify kinetic compensation.
inline StatefulFriction make_common_probe_friction(StatefulParameters parameters,
  const DirectionalLoadLaw & law, std::optional<RestDither> rest, bool dither_only,
  double output_fraction=1.)
{
  if(!std::isfinite(output_fraction) || output_fraction<=0. || output_fraction>1.)
    throw std::invalid_argument("Invalid common kinetic output fraction");
  if(dither_only) {
    if(!rest)throw std::invalid_argument("Dither isolation requires a measured waveform");
    parameters.load_transition_distance=0.;
    return StatefulFriction(parameters,std::nullopt,1.,rest,false);
  }
  // Scale the kinetic/elastic assistance, never the independent rest waveform.
  return StatefulFriction(parameters,std::nullopt,output_fraction,rest,rest.has_value(),law);
}
// Diagnostic-only rest classification: retain strict harmonic results and allow
// small bounded cycle-centered vibration for explicit soft_start_v1 trials.
// This never changes the compensation waveform or growth check.
class DitherRestSettling {
public:
  void reset() {*this=DitherRestSettling{};}
  void add(double time,double q,double frequency,double velocity=NAN) {
    if(!count_){start_=time;origin_=q;minimum_=maximum_=0.;}
    const double t=time-start_,y=q-origin_,phase=6.2831853071795864769*frequency*t;
    if(count_<times_.size()) {
      times_[count_]=t;
      if(count_) {
        const double dt=t-times_[count_-1];
        integral_q_[count_]=integral_q_[count_-1]+.5*(previous_q_+y)*dt;
        integral_v_[count_]=integral_v_[count_-1]+.5*(previous_v_+velocity)*dt;
        cycles_valid_=cycles_valid_ && dt>0. && dt<=.02;
      }
      cycles_valid_=cycles_valid_ && std::isfinite(velocity);
      previous_q_=y;previous_v_=velocity;
    } else {cycles_valid_=false;}
    const std::array<double,4> x{{std::sin(phase),std::cos(phase),1.,t}};
    for(size_t i=0;i<4;++i){
      rhs_[i]+=x[i]*y;
      for(size_t j=0;j<4;++j)normal_[i][j]+=x[i]*x[j];
    }
    sum_square_+=y*y;minimum_=std::min(minimum_,y);maximum_=std::max(maximum_,y);
    duration_=t;++count_;
  }
  bool passed(double frequency) const {
    constexpr double encoder=0.000011984224905356572;
    constexpr double pi=3.14159265358979323846;
    if(count_<100 || duration_<10./frequency || maximum_-minimum_>.12*pi/180.)return false;
    if(bounded_cycle_centers(frequency))return true;
    std::array<std::array<double,5>,4> a{};
    for(size_t i=0;i<4;++i){
      for(size_t j=0;j<4;++j)a[i][j]=normal_[i][j];
      a[i][4]=rhs_[i];
    }
    for(size_t i=0;i<4;++i){
      size_t pivot=i;
      for(size_t j=i+1;j<4;++j)if(std::abs(a[j][i])>std::abs(a[pivot][i]))pivot=j;
      if(std::abs(a[pivot][i])<1e-12)return false;
      std::swap(a[i],a[pivot]);const double divisor=a[i][i];
      for(size_t j=i;j<5;++j)a[i][j]/=divisor;
      for(size_t k=0;k<4;++k)if(k!=i){
        const double factor=a[k][i];for(size_t j=i;j<5;++j)a[k][j]-=factor*a[i][j];
      }
    }
    const double peak_to_peak=2.*std::hypot(a[0][4],a[1][4]);
    double residual=sum_square_;
    for(size_t i=0;i<4;++i)residual-=a[i][4]*rhs_[i];
    const double rms=std::sqrt(std::max(0.,residual)/count_);
    return std::isfinite(peak_to_peak) && std::isfinite(rms) &&
      peak_to_peak>=2.*encoder && peak_to_peak<=.1*pi/180. &&
      std::abs(a[3][4])*duration_<=2.*encoder && rms<=std::max(2.*encoder,.1*peak_to_peak);
  }
private:
  bool bounded_cycle_centers(double frequency) const {
    // Friction need not turn a sine torque into a pure sine displacement.
    // Keep the existing 0.12-degree raw dither envelope; require its two-period
    // center to stay within 0.001 rad, the existing zero-input rest envelope.
    // Cycle-mean speed must remain below 0.002 rad/s.
    // No dynamic allocation, output filtering, damping or waveform change.
    if(!cycles_valid_ || count_>times_.size())return false;
    const double width=2./frequency;
    if(duration_<=width)return false;
    size_t end=1,windows=0;double lo=0.,hi=0.;
    for(size_t i=0;i<count_ && times_[i]+width<=duration_;++i) {
      const double target=times_[i]+width;
      while(end<count_ && times_[end]<target)++end;
      if(end==count_)break;
      const double ratio=(target-times_[end-1])/(times_[end]-times_[end-1]);
      const auto mean=[&](const auto & integral) {
        return (integral[end-1]+ratio*(integral[end]-integral[end-1])-integral[i])/width;
      };
      const double center=mean(integral_q_),speed=mean(integral_v_);
      if(!std::isfinite(center) || !std::isfinite(speed) || std::abs(speed)>.002)return false;
      if(!windows)lo=hi=center;
      else {lo=std::min(lo,center);hi=std::max(hi,center);}
      ++windows;
    }
    // Do not shorten the effective drift interval by the averaging width.
    return windows>0 && (hi-lo)*duration_/(duration_-width)<=.001;
  }
  std::array<double,2048> times_{},integral_q_{},integral_v_{};
  double previous_q_=0.,previous_v_=0.;bool cycles_valid_=true;
  std::array<std::array<double,4>,4> normal_{};std::array<double,4> rhs_{};
  size_t count_=0;double start_=0.,origin_=0.,minimum_=0.,maximum_=0.,duration_=0.,sum_square_=0.;
};
class CommonReleaseSequence {
public:
  struct Sample {double input=0.; int phase=0,trial=0; bool assistance=false,done=false,failed=false;};
  CommonReleaseSequence(std::vector<int64_t> directions,std::vector<int64_t> assistance,double hand_seconds=0.,
    double ramp_rate=.5,double ramp_max=12.,double release_speed=.02,double minimum_travel=0.,double zero_input_seconds=0.,
    double soft_start_rest_frequency=0.)
  : directions_(std::move(directions)), assistance_(std::move(assistance)),hand_seconds_(hand_seconds),
    ramp_rate_(ramp_rate),ramp_max_(ramp_max),release_speed_(release_speed),minimum_travel_(minimum_travel),
    zero_input_seconds_(zero_input_seconds),soft_start_rest_frequency_(soft_start_rest_frequency) {
    if(!std::isfinite(soft_start_rest_frequency_) || (soft_start_rest_frequency_!=0. &&
        (soft_start_rest_frequency_<5. || soft_start_rest_frequency_>20.)))
      throw std::invalid_argument("Unsupported soft-start rest diagnostic frequency");
    if(!std::isfinite(zero_input_seconds_) || (zero_input_seconds_!=0. &&
        (zero_input_seconds_!=8. || hand_seconds_!=0. || directions_.size()!=1 ||
         release_speed_!=.02 || minimum_travel_!=0.)))
      throw std::invalid_argument("Zero-input isolation requires one finite eight-second observation");
    if(release_speed_!=.02 && release_speed_!=.3 && release_speed_!=.6)
      throw std::invalid_argument("Unsupported diagnostic release speed");
    if(release_speed_!=.02 && (hand_seconds_!=0. || directions_.size()!=1))
      throw std::invalid_argument("Higher-speed coast is a separate single finite trial");
    if(directions_.empty() || directions_.size()>8 || directions_.size()!=assistance_.size())
      throw std::invalid_argument("Invalid release sequence");
    for(auto v:directions_)if(v!=1 && v!=-1)throw std::invalid_argument("Invalid direction");
    for(auto v:assistance_)if(v!=0 && v!=1)throw std::invalid_argument("Invalid assistance mask");
    if(minimum_travel_!=0. && ((minimum_travel_!=.4 && minimum_travel_!=.6) ||
        release_speed_!=.3 || hand_seconds_!=0. || directions_.size()!=1 || assistance_[0]!=0))
      throw std::invalid_argument("Held-input travel requires one finite OFF diagnostic");
    if(!std::isfinite(ramp_rate_) || !std::isfinite(ramp_max_) ||
       !(ramp_rate_>0. && ramp_rate_<=.5 && ramp_max_>0. && ramp_max_<=12.))
      throw std::invalid_argument("Invalid known-input ramp");
    if(!std::isfinite(hand_seconds_) || (hand_seconds_!=0. &&
        ((hand_seconds_!=-1. && hand_seconds_!=30. && hand_seconds_!=60.) || directions_.size()!=1 || assistance_[0]!=1)))
      throw std::invalid_argument("Hand observation requires one enabled trial: -1 (manual stop), 30 or 60 seconds");
  }
  Sample step(double q,double v,double dt) {
    if(!std::isfinite(q) || !std::isfinite(v) || !(dt>0. && dt<=.02))
      throw std::invalid_argument("Invalid release sample");
    if(out_.done)return out_;
    if(!primed_){origin_=rest_min_=rest_max_=q;primed_=true;}
    out_.assistance=assistance_[out_.trial]!=0;
    out_.input=0.; elapsed_+=dt;
    if(zero_input_seconds_!=0.) {
      // Component-isolation evidence only: never proceed to a torque ramp.
      out_.phase=51;
      rest_min_=std::min(rest_min_,q);rest_max_=std::max(rest_max_,q);
      out_.failed=rest_max_-rest_min_>.001;
      out_.done=out_.failed || elapsed_>=zero_input_seconds_;
      out_.assistance=out_.assistance && !out_.done;
      return out_;
    }
    if(hand_seconds_!=0.) {
      // No automatic excitation during a manual observation. The owner keeps
      // the same operator shutdown and device-error handling.
      out_.phase=50;out_.done=hand_seconds_>0. && elapsed_>=hand_seconds_;
      out_.assistance=!out_.done;return out_;
    }
    if(out_.phase==0){
      rest_min_=std::min(rest_min_,q);rest_max_=std::max(rest_max_,q);
      if(rest_max_-rest_min_>.001){out_.failed=out_.done=true;return out_;}
      if(elapsed_>=4.){origin_=q;elapsed_=0.;out_.phase=1;}
    } else if(out_.phase==1){
      const bool reached_speed=directions_[out_.trial]*v>=release_speed_;
      if(minimum_travel_!=0. && reached_speed && !input_frozen_) {
        frozen_input_=previous_input_;input_frozen_=true;
      }
      const bool release=minimum_travel_!=0. ?
        directions_[out_.trial]*(q-origin_)>=minimum_travel_ :
        release_speed_==.02 ? (std::abs(q-origin_)>=.003 || std::abs(v)>=.02) : reached_speed;
      if(release){
        elapsed_=0.;out_.phase=2;release_origin_=rest_min_=rest_max_=q;
        release_initial_speed_=std::abs(v);growth_time_=0.;dither_rest_.reset();
      } else if(elapsed_>=24.){out_.failed=out_.done=true;}
      else out_.input=input_frozen_?frozen_input_:
        directions_[out_.trial]*std::min(ramp_max_,ramp_rate_*elapsed_);
      previous_input_=out_.input;
    } else {
      const bool soft_rest=out_.assistance && soft_start_rest_frequency_>0.;
      // Brief elastic return may occupy the accepted 0.12-degree dither
      // envelope. No-growth and final bounded rest checks still apply.
      const double reverse_limit=soft_rest ? .12*3.14159265358979323846/180. : .001;
      if(directions_[out_.trial]*(q-release_origin_)<-reverse_limit){out_.failed=out_.done=true;return out_;}
      if(elapsed_<=.15)release_initial_speed_=std::max(release_initial_speed_,std::abs(v));
      if(elapsed_>=.25){
        growth_time_=std::abs(v)-release_initial_speed_>std::max(.01,.5*release_initial_speed_) ? growth_time_+dt : 0.;
        if(growth_time_>=.1){out_.failed=out_.done=true;return out_;}
      }
      // The final two seconds must settle. The known motor-side input is zero
      // throughout this phase; it is not a measurement of external hand force.
      if(elapsed_<6.)rest_min_=rest_max_=q;
      else {rest_min_=std::min(rest_min_,q);rest_max_=std::max(rest_max_,q);}
      if(soft_rest && elapsed_>=5.9)dither_rest_.add(elapsed_,q,soft_start_rest_frequency_,v);
      if(elapsed_>=8.){
        const bool stationary=rest_max_-rest_min_<=.0002 && std::abs(v)<=.002;
        if(!stationary && !(soft_rest && dither_rest_.passed(soft_start_rest_frequency_))){out_.failed=out_.done=true;}
        else if(out_.trial+1==static_cast<int>(directions_.size()))out_.done=true;
        else {++out_.trial;out_.phase=0;elapsed_=0.;origin_=rest_min_=rest_max_=q;}
      }
    }
    return out_;
  }
private:
  std::vector<int64_t> directions_,assistance_;
  Sample out_{};double elapsed_=0.,origin_=0.,release_origin_=0.,rest_min_=0.,rest_max_=0.;bool primed_=false;
  double release_initial_speed_=0.,growth_time_=0.;
  double hand_seconds_=0.;
  double ramp_rate_=.5,ramp_max_=12.,release_speed_=.02;
  double minimum_travel_=0.,previous_input_=0.,frozen_input_=0.;bool input_frozen_=false;
  double zero_input_seconds_=0.;
  double soft_start_rest_frequency_=0.;DitherRestSettling dither_rest_;
};
}
#endif
