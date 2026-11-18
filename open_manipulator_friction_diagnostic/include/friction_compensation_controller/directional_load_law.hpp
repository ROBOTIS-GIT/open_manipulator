// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__DIRECTIONAL_LOAD_LAW_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__DIRECTIONAL_LOAD_LAW_HPP_
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <optional>
#include <vector>

namespace friction_compensation_controller
{
// Signed nominal gravity is a model feature, not measured output torque.
// Supports parametric load coefficients or a directional load/speed map.
// Four coefficients: F = C + B|v| + L+ max(g,0) + L- max(-g,0).
// No live-current input: an external push must not become the load estimate.
class DirectionalLoadLaw
{
public:
  using Coefficients = std::array<double, 4>;
  using HighLoad = std::array<double, 2>;
  struct SmoothLoad
  {
    double width_nm, feature_scale_nm;
    std::vector<double> speeds, positive, negative;
  };
  struct PeriodicLoad
  {
    double width_nm,feature_scale_nm,frequency;
    std::vector<int64_t> orders;
    std::vector<double> speeds,positive,negative;
    std::vector<double> load_hat_knots{};
    bool load_hat_mean_extension=false;
  };
  explicit DirectionalLoadLaw(PeriodicLoad model)
  : positive_{},negative_{},speed_endpoint_(0.),speeds_(std::move(model.speeds)),
    positive_table_(std::move(model.positive)),negative_table_(std::move(model.negative)),
    smooth_width_(model.width_nm),smooth_scale_(model.feature_scale_nm),
    periodic_frequency_(model.frequency),orders_(std::move(model.orders)),
    load_hat_knots_(std::move(model.load_hat_knots)),load_hat_mean_extension_(model.load_hat_mean_extension)
  {
    if(!std::isfinite(smooth_width_) || smooth_width_<=0. || !std::isfinite(smooth_scale_) || smooth_scale_<=0. ||
      !std::isfinite(periodic_frequency_) || periodic_frequency_<=0. || orders_.empty() || orders_.size()>4 ||
      speeds_.size()<2 || speeds_.size()>32 || positive_table_.size()!=(3+load_hat_knots_.size())*speeds_.size()*(1+4*orders_.size()) ||
      negative_table_.size()!=positive_table_.size())throw std::invalid_argument("Invalid periodic load law");
    if(!load_hat_knots_.empty() && (load_hat_knots_.size()<3 || load_hat_knots_.size()>16))
      throw std::invalid_argument("Invalid additional load coordinates");
    for(size_t k=0;k<load_hat_knots_.size();++k)
      if(!std::isfinite(load_hat_knots_[k]) || (k && load_hat_knots_[k]<=load_hat_knots_[k-1]))
        throw std::invalid_argument("Invalid additional load coordinates");
    for(size_t k=0;k<orders_.size();++k)
      if(orders_[k]<=0 || (k && orders_[k]<=orders_[k-1]))throw std::invalid_argument("Invalid periodic orders");
    for(size_t k=0;k<speeds_.size();++k)
      if(!std::isfinite(speeds_[k]) || speeds_[k]<=0. || (k && speeds_[k]<=speeds_[k-1]))throw std::invalid_argument("Invalid periodic speeds");
    for(const auto *row:{&positive_table_,&negative_table_})for(double x:*row)
      if(!std::isfinite(x) || x<0.)throw std::invalid_argument("Nonnegative periodic weights required");
    if(load_hat_mean_extension_) {
      if(load_hat_knots_.empty())throw std::invalid_argument("Mean extension requires load knots");
      for(const auto *row:{&positive_table_,&negative_table_})
        for(size_t k=0;k<3*speeds_.size()*(1+4*orders_.size());++k)
          if((*row)[k]!=0.)throw std::invalid_argument("Mean extension requires pure signed-load coefficients");
    }
  }
  explicit DirectionalLoadLaw(SmoothLoad model)
  : positive_{}, negative_{}, speed_endpoint_(0.), speeds_(std::move(model.speeds)),
    positive_table_(std::move(model.positive)), negative_table_(std::move(model.negative)),
    smooth_width_(model.width_nm), smooth_scale_(model.feature_scale_nm)
  {
    if (!std::isfinite(smooth_width_) || smooth_width_<=0. ||
      !std::isfinite(smooth_scale_) || smooth_scale_<=0. || speeds_.size()<2 || speeds_.size()>32 ||
      positive_table_.size()!=3*speeds_.size() || negative_table_.size()!=positive_table_.size()) {
      throw std::invalid_argument("Invalid smooth-load dimensions or scale");
    }
    for (size_t i=0;i<speeds_.size();++i) {
      if (!std::isfinite(speeds_[i]) || speeds_[i]<=0. || (i && speeds_[i]<=speeds_[i-1])) {
        throw std::invalid_argument("Invalid smooth-load speed knots");
      }
    }
    for (const auto * row : {&positive_table_,&negative_table_}) {
      for (double v:*row) {
        if (!std::isfinite(v) || v<0.) {throw std::invalid_argument("Nonnegative finite load coefficients required");}
      }
      for (size_t i=0;i<speeds_.size();++i) {
        if ((*row)[i]+(*row)[speeds_.size()+i]+(*row)[2*speeds_.size()+i]<=0.) {
          throw std::invalid_argument("Positive load loss required");
        }
      }
    }
  }
  DirectionalLoadLaw(std::vector<double> gravity, std::vector<double> speeds,
    std::vector<double> positive, std::vector<double> negative)
  : positive_{}, negative_{}, speed_endpoint_(0.), gravity_(std::move(gravity)),
    speeds_(std::move(speeds)), positive_table_(std::move(positive)),
    negative_table_(std::move(negative))
  {
    if (gravity_.size() < 3 || gravity_.size() > 16 || speeds_.size() < 2 ||
      speeds_.size() > 32 || positive_table_.size() != gravity_.size()*speeds_.size() ||
      negative_table_.size() != positive_table_.size()) {
      throw std::invalid_argument("Invalid loaded map dimensions");
    }
    for (const auto * knots : {&gravity_, &speeds_}) {
      for (size_t i=0; i<knots->size(); ++i) {
        if (!std::isfinite((*knots)[i]) || (i && (*knots)[i] <= (*knots)[i-1])) {
          throw std::invalid_argument("Invalid loaded map knots");
        }
      }
    }
    if (speeds_.front() <= 0.) {throw std::invalid_argument("Measured speeds must be positive");}
    for (const auto * table : {&positive_table_, &negative_table_}) {
      for (size_t i=0; i<table->size(); ++i) {
        const double value=(*table)[i];
        if (!std::isfinite(value) || value < 0. || (i%speeds_.size()==0 && value<=0.)) {
          throw std::invalid_argument("Nonnegative kinetic losses and positive rest yields required");
        }
      }
    }
  }
  DirectionalLoadLaw(Coefficients positive, Coefficients negative, double speed_endpoint = 0.,
    std::optional<HighLoad> positive_high = std::nullopt,
    std::optional<HighLoad> negative_high = std::nullopt)
  : positive_(positive), negative_(negative), speed_endpoint_(speed_endpoint),
    positive_high_(positive_high), negative_high_(negative_high)
  {
    if (!std::isfinite(speed_endpoint_) || speed_endpoint_ < 0.) {
      throw std::invalid_argument("Invalid measured speed endpoint");
    }
    if (positive_high_.has_value() != negative_high_.has_value() ||
      (positive_high_ && speed_endpoint_ <= 0.)) {
      throw std::invalid_argument("Speed-dependent loads need two endpoints");
    }
    for (const auto & row : {positive_high_, negative_high_}) {
      if (row) {
        for (double value : *row) {
          if (!std::isfinite(value) || value < 0.) {
            throw std::invalid_argument("Invalid high-speed load coefficient");
          }
        }
      }
    }
    for (const auto & row : {positive_, negative_}) {
      if (row[0] <= 0.) {throw std::invalid_argument("Positive loaded yield required");}
      for (double value : row) {
        if (!std::isfinite(value) || value < 0.) {
          throw std::invalid_argument("Invalid directional load coefficient");
        }
      }
    }
  }
  double yield(bool positive, double gravity, double position=0.) const
  {
    if (!std::isfinite(gravity)) {throw std::invalid_argument("Invalid nominal gravity");}
    if (!gravity_.empty() || smooth_width_>0.) {return lookup(positive, 0., gravity,position);}
    const auto & c = positive ? positive_ : negative_;
    return c[0]+c[2]*std::max(gravity, 0.)+c[3]*std::max(-gravity, 0.);
  }
  double rate(double velocity, double gravity = 0.,double position=0.) const
  {
    if (!gravity_.empty() || smooth_width_>0.) {
      if (velocity == 0.) {return 0.;}
      return (velocity > 0. ? 1. : -1.) *
        (lookup(velocity > 0., std::abs(velocity), gravity,position)-yield(velocity > 0., gravity,position));
    }
    // Same endpoint policy as the J6 curve: do not invent an increasing loss
    // outside measured speeds. It does not stop or restrict robot velocity.
    const double v = speed_endpoint_ > 0. ?
      std::clamp(velocity, -speed_endpoint_, speed_endpoint_) : velocity;
    const auto & c = v >= 0. ? positive_ : negative_;
    const auto & high = v >= 0. ? positive_high_ : negative_high_;
    double slope = c[1];
    if (high) {
      // A decreasing load loss is identified velocity weakening, not an
      // independently added damper. Interpolated load gains stay nonnegative.
      slope += (((*high)[0]-c[2])*std::max(gravity, 0.)+
        ((*high)[1]-c[3])*std::max(-gravity, 0.))/speed_endpoint_;
    }
    return slope*v;
  }
  double magnitude(double velocity, double gravity,double position=0.) const
  {return yield(velocity >= 0., gravity,position)+(velocity >= 0. ? 1. : -1.)*rate(velocity, gravity,position);}
private:
  double lookup(bool positive, double speed, double gravity,double position=0.) const
  {
    if (!std::isfinite(speed) || !std::isfinite(gravity)) {throw std::invalid_argument("Invalid map input");}
    const auto & table=positive ? positive_table_ : negative_table_;
    size_t s=1; while (s+1<speeds_.size() && speed>speeds_[s]) {++s;}
    const double fraction=std::clamp((speed-speeds_[s-1])/(speeds_[s]-speeds_[s-1]),0.,1.);
    if(periodic_frequency_>0.) {
      if(!std::isfinite(position))throw std::invalid_argument("Invalid periodic position");
      std::array<double,17> h{};h[0]=1.;size_t n=1;
      for(auto order:orders_)for(double x:{std::sin(periodic_frequency_*order*position),std::cos(periodic_frequency_*order*position)}){
        h[n++]=1.+x;h[n++]=1.-x;
      }
      const double r=std::hypot(gravity,smooth_width_);
      std::array<double,19> load{};
      load[0]=1.;load[1]=(r+gravity)/(2*smooth_scale_);load[2]=(r-gravity)/(2*smooth_scale_);
      if(!load_hat_knots_.empty()) {
        if(gravity<=load_hat_knots_.front())load[3]=1.;
        else if(gravity>=load_hat_knots_.back())load[2+load_hat_knots_.size()]=1.;
        else {
          const size_t hi=std::upper_bound(load_hat_knots_.begin(),load_hat_knots_.end(),gravity)-load_hat_knots_.begin();
          const double f=(gravity-load_hat_knots_[hi-1])/(load_hat_knots_[hi]-load_hat_knots_[hi-1]);
          load[3+hi-1]=1.-f;load[3+hi]=f;
        }
      }
      double result=0.;
      for(size_t g=0;g<3+load_hat_knots_.size();++g)for(size_t k=0;k<n;++k){
        const size_t lo=(g*speeds_.size()+s-1)*n+k,hi=lo+n;
        result+=load[g]*h[k]*(table[lo]+fraction*(table[hi]-table[lo]));
      }
      if(load_hat_mean_extension_ &&
        (gravity<load_hat_knots_.front() || gravity>load_hat_knots_.back())) {
        const bool lower=gravity<load_hat_knots_.front();
        const size_t edge=lower?0:load_hat_knots_.size()-1,near=lower?1:edge-1;
        const auto mean=[&](size_t g){
          double value=0.;
          for(size_t k=0;k<n;++k){
            const size_t lo=((3+g)*speeds_.size()+s-1)*n+k;
            value+=table[lo]+fraction*(table[lo+n]-table[lo]);
          }
          return value;
        };
        const double slope=std::max(0.,(mean(edge)-mean(near))/std::abs(load_hat_knots_[edge]-load_hat_knots_[near]));
        result+=std::abs(gravity-load_hat_knots_[edge])*slope;
      }
      return result;
    }
    const auto value=[&](size_t g) {
      size_t base=g*speeds_.size();
      return table[base+s-1]+fraction*(table[base+s]-table[base+s-1]);
    };
    if (smooth_width_>0.) {
      const double radius=std::hypot(gravity,smooth_width_);
      return value(0)+value(1)*(radius+gravity)/(2*smooth_scale_)+
        value(2)*(radius-gravity)/(2*smooth_scale_);
    }
    size_t g=1; while (g+1<gravity_.size() && gravity>gravity_[g]) {++g;}
    double low=value(g-1), high=value(g), slope=(high-low)/(gravity_[g]-gravity_[g-1]);
    // Explicit extension hypothesis: no unmeasured decrease outside the table.
    // This changes neither motion limits nor the measured in-domain values.
    if (gravity<gravity_.front()) {slope=std::min(0.,slope);}
    if (gravity>gravity_.back()) {return high+std::max(0.,slope)*(gravity-gravity_.back());}
    return low+slope*(gravity-gravity_[g-1]);
  }
  Coefficients positive_, negative_;
  double speed_endpoint_;
  std::optional<HighLoad> positive_high_, negative_high_;
  std::vector<double> gravity_, speeds_, positive_table_, negative_table_;
  double smooth_width_=0., smooth_scale_=1.;
  double periodic_frequency_=0.;
  std::vector<int64_t> orders_;
  std::vector<double> load_hat_knots_;
  bool load_hat_mean_extension_=false;
};
}  // namespace friction_compensation_controller
#endif
