// Finite position/current mode comparison; never a freedrive controller.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__CURRENT_TRACKING_SEQUENCE_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__CURRENT_TRACKING_SEQUENCE_HPP_
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace friction_compensation_controller {
class CurrentTrackingSequence {
public:
  struct Output {double command=0.,reference_position=0.;bool failed=false,done=false;};
  CurrentTrackingSequence(std::vector<double> t,std::vector<double> q,
    std::vector<double> v,std::vector<double> ff)
  : t_(std::move(t)),q_(std::move(q)),v_(std::move(v)),ff_(std::move(ff)) {
    if(t_.size()<3 || t_.size()>2000 || q_.size()!=t_.size() || v_.size()!=t_.size() ||
      ff_.size()!=t_.size() || t_.front()!=0. || t_.back()>30. ||
      q_.front()!=0. || v_.front()!=0. || v_.back()!=0. || std::abs(ff_.front())>.15)
      throw std::invalid_argument("Invalid neutral-start tracking table");
    for(size_t k=0;k<t_.size();++k)
      if(!std::isfinite(t_[k]) || !std::isfinite(q_[k]) || !std::isfinite(v_[k]) || !std::isfinite(ff_[k]) ||
        (k && t_[k]<=t_[k-1]) || std::abs(q_[k])>.901 || std::abs(v_[k])>.201 || std::abs(ff_[k])>50.)
        throw std::invalid_argument("Invalid finite tracking sample");
  }
  Output step(double q,double v,double dt) {
    Output out;
    if(!std::isfinite(q) || !std::isfinite(v) || !(dt>0. && dt<=.02))
      throw std::invalid_argument("Invalid tracking observation");
    const size_t hi=std::min<size_t>(t_.size()-1,std::max<size_t>(1,
      std::upper_bound(t_.begin(),t_.end(),elapsed_)-t_.begin()));
    const double f=std::clamp((elapsed_-t_[hi-1])/(t_[hi]-t_[hi-1]),0.,1.);
    const auto at=[&](const std::vector<double> &a){return a[hi-1]+f*(a[hi]-a[hi-1]);};
    out.reference_position=at(q_);
    const double error=out.reference_position-q;
    out.failed=std::abs(error)>.06;
    integral_=std::clamp(integral_+20.*error*dt,-12.,12.);
    out.command=at(ff_)+80.*error+10.*(at(v_)-v)+integral_;
    out.done=elapsed_>=t_.back();
    if(out.done && std::abs(v)>.01)out.failed=true;
    elapsed_+=dt;
    return out;
  }
private:
  std::vector<double> t_,q_,v_,ff_;
  double elapsed_=0.,integral_=0.;
};
}
#endif
