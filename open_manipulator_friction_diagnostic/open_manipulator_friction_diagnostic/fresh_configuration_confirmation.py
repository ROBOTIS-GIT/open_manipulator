"""Prospective prediction check of a frozen model on unused configurations.

This is separate from development cross validation and from controller release
tests. It never fits coefficients, relaxes error thresholds, or enables a robot.
Passing means are evidence for supervised evaluation, not spatial qualification.
"""
from collections import Counter
import numpy as np

from .automatic_calibration import checked_cycles
from .kinetic_selection import validate


def review_confirmation(model, cycles, evidence, conditions, speeds, development,
                        fraction=.95, mock=False):
    checked_cycles(development)
    checked_cycles(cycles)
    joint=model['joint']-1
    if (not model.get('training_review',{}).get('means_passed')
            or any(c['joint']!=joint for c in development)
            or len(conditions)<3 or len({c['name'] for c in conditions})!=len(conditions)
            or any(c['joint']!=joint for c in conditions)):
        raise ValueError('Complete development and three new configurations required')
    speeds=np.asarray(speeds,float)
    if (speeds.ndim!=1 or len(speeds)<4 or not np.isfinite(speeds).all()
            or min(speeds)<=0 or np.any(np.diff(speeds)<=0)
            or speeds[0]>model['speed_range_rad_s'][0]+1e-9
            or speeds[-1]<model['speed_range_rad_s'][1]-1e-9):
        raise ValueError('Fresh configuration speeds must cover the frozen speed endpoints')
    # Other-joint configurations must be unseen, not a renamed repetition of a
    # fit pose. The target angle can change independently of load configuration.
    others=[j for j in range(6) if j!=joint]
    previous=np.array([np.mean(c['q'],axis=0)[others] for c in development])
    centers={}
    for condition in conditions:
        q=np.asarray(condition['q_rad'],float)
        if q.shape!=(6,) or not np.isfinite(q).all():
            raise ValueError('Invalid planned configuration')
        z=q[others]
        if (np.any(np.max(abs(previous-z),axis=1)<=.01)
                or any(np.max(abs(v-z))<=.01 for v in centers.values())):
            raise ValueError('Validation configuration is repeated or already developed')
        centers[condition['name']]=z
    expected=Counter({(name,float(v)):2 for name in centers for v in speeds})
    actual=Counter((c['group'],float(c['speed'])) for c in cycles)
    if actual!=expected:
        raise ValueError('Every declared configuration/speed needs both ordered repetitions')
    for c in cycles:
        if np.max(abs(np.asarray(c['q'])[:,others]-centers[c['group']]))>.003:
            raise ValueError('Measured configuration differs from the frozen plan')
    result=validate(model,cycles,evidence,mock=mock,fraction=fraction)
    confirmed=bool(result['independent'] and result['means_passed']
                   and result['support']['within_development_bounds'])
    return dict(result, confirmed_means=confirmed, new_configurations=len(centers),
        development_configuration_means_passed=model.get('posture_review',{}).get('means_passed'),
        mean_failures=[row for row in result['rows'] if not row['mean_passed']],
        automatic_deployment_allowed=False,hardware_qualified=False,
        additional_physical_release_evidence_required=True,
        confirmation_scope='unseen configurations, both directions and ordered repeats; current-derived means')
