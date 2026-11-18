"""Offline LuGre reference using KR102479904B1 equations 5--9 and 12.

Positive output denotes the *resisting torque magnitude with velocity sign* in
our M*qdd + C*qd + g + friction = motor + external convention. Adding it as
motor assistance is not automatically passive. Equation 12 is a model-parameter
condition, not a certificate for an uncertain, sampled robot/drive assembly.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import least_squares


@dataclass(frozen=True)
class Parameters:
    fc: float
    fs: float
    vs: float
    viscous: float
    sigma0: float
    sigma1: float
    smoothing_k: float

    def __post_init__(self):
        if (not np.isfinite(list(vars(self).values())).all() or
                min(self.fc, self.vs, self.sigma0, self.smoothing_k) <= 0 or
                self.fs < self.fc or min(self.viscous, self.sigma1) < 0):
            raise ValueError('Invalid LuGre parameters; dynamic coefficients must be identified')


class LuGre:
    """Exact constant-input state step; bounded sigma1, zero initial bristle state.

    No static intent, current inverse filter, angle feature or output gain is
    inferred. The returned sample-average torque is suitable for offline work
    accounting; the real sampled motor system still needs separate validation.
    """
    def __init__(self, parameters):
        self.p = parameters
        self.reset()

    def reset(self):
        self.z = 0.
        self.sigma1_used = 0.

    def step(self, velocity, dt):
        if not math.isfinite(velocity) or not math.isfinite(dt) or not 0 < dt <= .02:
            raise ValueError('Invalid or stale LuGre input')
        p = self.p
        g = p.fc+(p.fs-p.fc)*math.exp(-(velocity/p.vs)**2)
        s0 = 2/math.pi*math.atan(p.smoothing_k*velocity)
        s1 = velocity*s0
        rate = p.sigma0*s1/g
        # Patent eq.12. Do not replace Fc/Fs with a hand-selected percentage.
        self.sigma1_used = min(p.sigma1, 4*p.sigma0*g/s1) if s1 > 0 else p.sigma1
        x = rate*dt
        old = self.z
        # Integral form avoids v/rate and cancellation as v tends to zero.
        phi = -math.expm1(-x)/x if x > 1e-6 else 1-x/2+x*x/6
        psi = (1-phi)/x if x > 1e-6 else .5-x/6+x*x/24
        average_z = old*phi+velocity*dt*psi
        self.z = old*math.exp(-x)+velocity*dt*phi
        return p.sigma0*average_z+self.sigma1_used*(self.z-old)/dt+p.viscous*velocity


def replay(t, velocity, parameters, initial_bristle_torque=0.):
    t, velocity = np.asarray(t), np.asarray(velocity)
    if (t.ndim != 1 or len(t) < 3 or velocity.shape != t.shape or
            not np.isfinite([t, velocity]).all() or
            np.any(np.diff(t) <= 0) or np.max(np.diff(t)) > .02):
        raise ValueError('Need a finite, ordered high-rate transient trace')
    model = LuGre(parameters)
    model.z = initial_bristle_torque/parameters.sigma0
    output = np.zeros(len(t)); output[0] = initial_bristle_torque
    for i in range(1, len(t)):
        output[i] = model.step(float((velocity[i-1]+velocity[i])/2), float(t[i]-t[i-1]))
    return output


def fit_transient(t, velocity, friction_torque, steady, smoothing_k, *,
                  sensor_timing_verified, rigid_residual_verified):
    """Fit sigma0/1 plus initial bristle torque, never from steady means alone.

    Input must already have verified sensor alignment and fixed-model rigid
    torque subtraction. The caller supplies an independent validation trace.
    Unverified OMY timing cannot be promoted by assuming zero delay here.
    """
    if not sensor_timing_verified or not rigid_residual_verified:
        raise ValueError('Transient identification needs verified timing and rigid residual')
    t, v, y = map(np.asarray, (t, velocity, friction_torque))
    if y.shape != t.shape or not np.isfinite(y).all() or len(t) < 100:
        raise ValueError('Invalid transient torque trace')
    if np.count_nonzero(np.diff(np.sign(v))) < 2 or np.ptp(v) < .01:
        raise ValueError('Transient trace needs starts and reversals')
    fc, fs, vs, b = steady[:4]
    def residual(x):
        p = Parameters(fc, fs, vs, b, math.exp(x[0]), math.exp(x[1]), smoothing_k)
        return replay(t, v, p, x[2])-y
    solutions = [least_squares(residual, [math.log(k), math.log(.1), 0.],
        bounds=([math.log(.1), math.log(1e-6), -fs],
                [math.log(1e6), math.log(1e3), fs]), max_nfev=300)
        for k in (10., 100., 1000.)]
    fit = min(solutions, key=lambda x: float(x.fun@x.fun))
    singular = np.linalg.svd(fit.jac, compute_uv=False)
    condition = float(singular[0]/max(singular[-1], 1e-15))
    identifiable = bool(fit.success and condition < 1e6 and not np.any(fit.active_mask))
    p = Parameters(fc, fs, vs, b, math.exp(fit.x[0]), math.exp(fit.x[1]), smoothing_k)
    return dict(parameters=vars(p), initial_bristle_torque_nm=float(fit.x[2]),
                rmse_nm=float(np.sqrt(np.mean(fit.fun**2))), identifiable=identifiable,
                jacobian_condition=condition, passivity_proven=False,
                hardware_deployable=False, independent_validation_required=True)


def simulation_profile(parameters, config, urdf, baseline):
    """Explicit synthetic fixture; never upgrades a measured candidate."""
    from .integration import fingerprint
    from .protocol import sha
    if len(parameters) != 6:
        raise ValueError('Need six separately specified LuGre parameter sets')
    for p in parameters:
        Parameters(**p)
    return dict(schema=7, source='synthetic', qualification='lugre_simulation_only_v1',
                config=config, robot_fingerprint=fingerprint(urdf), baseline_hash=sha(baseline),
                parameters=parameters, hardware_deployable=False, passivity_proven=False)


def controller_parameters(profile, c, urdf, baseline):
    from .integration import fingerprint, original_gravity
    from .protocol import sha
    if (profile.get('schema') != 7 or profile.get('source') != 'synthetic' or
            profile.get('qualification') != 'lugre_simulation_only_v1' or
            profile.get('config') != c or profile.get('robot_fingerprint') != fingerprint(urdf) or
            profile.get('baseline_hash') != sha(baseline) or
            profile.get('hardware_deployable') is not False or profile.get('passivity_proven') is not False):
        raise ValueError('Invalid simulation-only LuGre profile')
    parameters = [Parameters(**x) for x in profile['parameters']]
    if len(parameters) != 6:
        raise ValueError('Need six LuGre parameter sets')
    result = original_gravity(baseline)
    result.update(calibrated_friction_enabled=True, calibrated_unscaled=True,
        calibrated_lugre=True, calibrated_gain=1., calibrated_max_torque=[0.]*6,
        calibrated_slew_rate=[0.]*6, calibrated_deadband=0.,
        calibrated_min_velocity=min(c['speeds_rad_s']),
        calibrated_max_velocity=c['max_velocity_rad_s'],
        calibrated_max_acceleration=c['max_acceleration_rad_s2'],
        calibrated_enforce_load_bounds=False, calibrated_load_gain=[0.]*6,
        calibrated_load_min=[0.]*6, calibrated_load_max=[1.]*6)
    for name in ('fc', 'fs', 'vs', 'viscous', 'sigma0', 'sigma1', 'smoothing_k'):
        result['calibrated_'+name] = [getattr(p, name) for p in parameters]
    return result
