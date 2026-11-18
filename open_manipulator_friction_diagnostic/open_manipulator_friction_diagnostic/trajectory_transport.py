"""Bounded cubic-Hermite knot reduction for JointTrajectoryController transport.

The measurement reference and experiment timing remain at their original rate.
Only transmitted position/velocity knots are reduced; JTC interpolates cubics.
"""
import numpy as np
from scipy.interpolate import CubicHermiteSpline


def compact_points(data, anchors=(), position_tolerance=1e-5, velocity_tolerance=1e-4,
                   acceleration_limit=None):
    t, q, v = (np.asarray(data[k]) for k in ('t', 'q', 'v'))
    keep = set([0, len(t)-1, *map(int, anchors)])
    if len(t) < 2 or any(i < 0 or i >= len(t) for i in keep) or np.any(np.diff(t) <= 0):
        raise ValueError('Invalid trajectory/anchors for transport')
    if acceleration_limit is not None and (not np.isfinite(acceleration_limit) or acceleration_limit <= 0):
        raise ValueError('Invalid acceleration limit')
    ordered = sorted(keep)
    pending = list(zip(ordered[:-1], ordered[1:]))
    while pending:
        left, right = pending.pop()
        spline = CubicHermiteSpline(t[[left, right]], q[[left, right]], v[[left, right]])
        # A cubic's acceleration is linear. Its endpoint extrema bound the whole
        # interval, including overshoot missed by checking sampled velocities.
        acceleration = float(abs(spline(t[[left, right]], 2)).max())
        acceleration_bad = acceleration_limit is not None and acceleration > acceleration_limit*(1+1e-10)
        if right-left <= 1:
            if acceleration_bad:
                raise ValueError('Original trajectory interpolation exceeds acceleration limit')
            continue
        sample = np.arange(left+1, right)
        errors = np.maximum(np.max(abs(spline(t[sample])-q[sample]), axis=1)/position_tolerance,
                            np.max(abs(spline(t[sample], 1)-v[sample]), axis=1)/velocity_tolerance)
        k = int(np.argmax(errors))
        if errors[k] > 1 or acceleration_bad:
            mid = (left+right)//2 if acceleration_bad else int(sample[k])
            keep.add(mid)
            pending.extend([(left, mid), (mid, right)])
    ids = np.array(sorted(keep))
    spline = CubicHermiteSpline(t[ids], q[ids], v[ids])
    # Include left AND right limits at knots; the second derivative can jump.
    acceleration_max = max(float(abs(2*spline.c[1]).max()),
                           float(abs(6*spline.c[0]*np.diff(t[ids])[:,None]+2*spline.c[1]).max()))
    report = dict(original_points=len(t), transmitted_points=len(ids),
                  max_grid_position_error_rad=float(abs(spline(t)-q).max()),
                  max_grid_velocity_error_rad_s=float(abs(spline(t, 1)-v).max()),
                  position_tolerance_rad=position_tolerance,
                  velocity_tolerance_rad_s=velocity_tolerance,
                  acceleration_limit_rad_s2=acceleration_limit,
                  max_continuous_acceleration_rad_s2=acceleration_max)
    return dict(t=t[ids], q=q[ids], v=v[ids]), report
