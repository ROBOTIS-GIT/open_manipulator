// Copyright 2026 ROBOTIS. Licensed under Apache-2.0.
#ifndef GRAVITY_COMPENSATION_CONTROLLER__MODIFIED_LUGRE_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__MODIFIED_LUGRE_HPP_
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace friction_compensation_controller
{
// Research implementation of KR102479904B1 eqs.5--9,12. The parameter inequality
// alone is NOT proof of passivity of the sampled robot with uncertain friction.
class ModifiedLuGre
{
public:
  ModifiedLuGre(double fc, double fs, double vs, double viscous, double sigma0,
    double sigma1, double smoothing_k)
  : fc_(fc), fs_(fs), vs_(vs), viscous_(viscous), sigma0_(sigma0), sigma1_(sigma1), k_(smoothing_k)
  {
    for (double x : {fc, fs, vs, viscous, sigma0, sigma1, smoothing_k}) {
      if (!std::isfinite(x) || x < 0.) {throw std::invalid_argument("Invalid LuGre parameter");}
    }
    if (fc <= 0. || fs < fc || vs <= 0. || sigma0 <= 0. || smoothing_k <= 0.) {
      throw std::invalid_argument("LuGre requires identified dynamic parameters");
    }
  }
  void reset() {z_ = 0.; sigma1_used_ = 0.;}
  double bristle() const {return z_;}
  double damping() const {return sigma1_used_;}
  double step(double v, double dt)
  {
    if (!std::isfinite(v) || !std::isfinite(dt) || dt <= 0. || dt > .02) {
      throw std::invalid_argument("Invalid LuGre sample");
    }
    const double g = fc_+(fs_-fc_)*std::exp(-std::pow(v/vs_, 2));
    const double s1 = v*(2./std::acos(-1.))*std::atan(k_*v);
    sigma1_used_ = s1 > 0. ? std::min(sigma1_, 4.*sigma0_*g/s1) : sigma1_;
    const double x = sigma0_*s1/g*dt;
    const double phi = x > 1e-6 ? -std::expm1(-x)/x : 1.-x/2.+x*x/6.;
    const double psi = x > 1e-6 ? (1.-phi)/x : .5-x/6.+x*x/24.;
    const double average_z = z_*phi+v*dt*psi;
    const double next = z_*std::exp(-x)+v*dt*phi;
    const double torque = sigma0_*average_z+sigma1_used_*(next-z_)/dt+viscous_*v;
    z_ = next;
    return torque;
  }
private:
  double fc_, fs_, vs_, viscous_, sigma0_, sigma1_, k_;
  double z_ = 0., sigma1_used_ = 0.;
};
}  // namespace friction_compensation_controller
#endif
