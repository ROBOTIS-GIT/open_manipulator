"""Compose frozen directional laws after development experiments, without qualification.

Positive and negative motion need not prefer the same model complexity. This
operation changes neither fitted coefficients nor gravity/current conversion.
The resulting candidate needs its own physical validation, including reversal.
"""
from copy import deepcopy
import numpy as np
from .automatic_calibration import content_hash
from .kinetic_runtime import model_parameters


def combine(positive, negative):
    models={'positive':positive,'negative':negative}
    for key in ('identity','joint','speed_knots_rad_s','speed_range_rad_s',
                'gravity_range_nm','development_sha256','development_payload_hashes'):
        if key not in positive or positive[key]!=negative.get(key):
            raise ValueError('Directional candidates differ in '+key)
    for m in models.values():model_parameters(m,m['joint'])
    periodic=[m for m in models.values() if m['schema']=='common_periodic_friction_v1']
    out=deepcopy(periodic[0] if periodic else positive)
    if not periodic:
        if any(m['family']!='velocity' for m in models.values()):
            raise ValueError('Only shared-coordinate periodic or velocity candidates are supported')
        for d,m in models.items():out['tables'][d]=deepcopy(m['tables'][d])
    else:
        width=1+4*len(out['orders']);n=len(out['speed_knots_rad_s'])
        for d,m in models.items():
            if m['schema']=='common_periodic_friction_v1':
                for key in ('frequency_rad_per_rad','orders','load_width_nm','load_feature_scale_nm'):
                    if m[key]!=out[key]:raise ValueError('Periodic coordinate mismatch: '+key)
                for key,default in (('load_hat_knots_nm',[]),('load_hat_mean_extension',False)):
                    if m.get(key,default)!=out.get(key,default):
                        raise ValueError('Periodic coordinate mismatch: '+key)
                out['coefficients'][d]=deepcopy(m['coefficients'][d])
            elif m['family']=='velocity':
                if out.get('load_hat_knots_nm'):
                    raise ValueError('Velocity embedding in additional load coordinates is not supported')
                table=np.asarray(m['tables'][d])
                if table.shape!=(3,n) or not np.all(table==table[0]):
                    raise ValueError('Velocity candidate unexpectedly depends on load')
                coeff=np.zeros(3*n*width);coeff[np.arange(n)*width]=table[0]
                out['coefficients'][d]=coeff.tolist()
            else:raise ValueError('Cannot exactly embed this load law in the periodic coordinates')
    for key in ('selection','training_review','posture_review','observability','angular_max_speed_rad_s',
                'macro_model','macro_model_sha256','macro_family','macro_smoothing'):
        out.pop(key,None)
    out.update(directional_sources={d:dict(candidate_sha256=content_hash(m),family=m['family'])
                                    for d,m in models.items()},
        fit_method='directional_frozen_composition',
        selection_stage='directional_development_experiments_require_fresh_validation',
        automatic_deployment_allowed=False,hardware_qualified=False,
        missing=sorted(set(positive.get('missing',[])+negative.get('missing',[])+
                           ['fresh_validation','kinetic_release_coverage'])))
    model_parameters(out,out['joint'])
    return out
