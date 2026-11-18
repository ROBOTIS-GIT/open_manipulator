"""Training-only selection of nested steady friction curves.

Eight cycle means cannot in general identify all four Stribeck parameters.
AICc counts one residual-variance parameter as well as the curve coefficients.
The nonlinear transition must lie inside the observed speed interval; a fitted
transition at a search boundary is not evidence for an unseen static peak.
"""
import numpy as np
from scipy.optimize import least_squares, lsq_linear


def select_curve(speed, torque, absolute_tolerance, relative_tolerance):
    v, y = np.asarray(speed, dtype=float), np.asarray(torque, dtype=float)
    if v.ndim != 1 or y.shape != v.shape or len(v) < 8 or np.any(v <= 0) or not np.isfinite([v, y]).all():
        raise ValueError('Need at least eight finite positive-speed cycle means')
    candidates = []
    def add(name, p, count, identifiable=True):
        prediction = p[0]+(p[1]-p[0])*np.exp(-(v/p[2])**2)+p[3]*v
        rss = float(np.sum((prediction-y)**2))
        k, n = count+1, len(v)  # include the estimated residual variance
        score = n*np.log(max(rss/n, np.finfo(float).tiny))+2*k+2*k*(k+1)/(n-k-1)
        candidates.append(dict(model=name, parameters=list(map(float,p)), fitted_coefficients=count,
            residual_sum_squares=rss, aicc=float(score), identifiable=bool(identifiable),
            training_adequate=bool(np.all(abs(prediction-y) <= absolute_tolerance+relative_tolerance*y))))
    constant = max(0., float(np.mean(y)))
    add('coulomb', [constant, constant, min(v), 0., 0.], 1)
    line = lsq_linear(np.column_stack([np.ones(len(v)), v]), y, bounds=(0., np.inf)).x
    add('coulomb_viscous', [line[0], line[0], min(v), line[1], 0.], 2)
    solutions = []
    for vs in np.geomspace(min(v)*1.01, max(v)/1.01, 7):
        fit = least_squares(lambda z: z[0]+z[1]*np.exp(-(v/z[2])**2)+z[3]*v-y,
            [min(y)*.8, max(np.ptp(y), .01), vs, .1],
            bounds=([0.,0.,min(v),0.],[np.inf,np.inf,max(v),np.inf]), max_nfev=3000)
        solutions.append(fit)
    fit = min(solutions, key=lambda x: float(x.fun@x.fun))
    fc, delta, vs, b = fit.x
    interior = min(v)*(1+1e-4) < vs < max(v)*(1-1e-4)
    add('stribeck_viscous', [fc, fc+delta, vs, b, 0.], 4, interior)
    eligible = [x for x in candidates if x['identifiable'] and x['training_adequate']]
    if not eligible:
        raise ValueError('No identifiable steady model meets the unchanged training-error requirement')
    selected = min(eligible, key=lambda x: x['aicc'])
    return np.asarray(selected['parameters']), dict(criterion='AICc; residual variance counted',
        observations=len(v), independent_observations='one mean per speed in training pass only',
        selected_model=selected['model'], candidates=candidates,
        training_adequacy=dict(absolute_nm=absolute_tolerance, relative=relative_tolerance),
        static_threshold_measured=False, validation_used_for_selection=False)
