"""One exact kinetic-law adapter for all axes. No dithering identification here."""
import numpy as np
from .single_joint_model import parameters as base_parameters


def model_parameters(model, joint):
    if joint not in range(1,7) or model.get('joint')!=joint:
        raise ValueError('Model joint differs from selected joint')
    speeds=np.asarray(model['speed_knots_rad_s'],float)
    if (speeds.ndim!=1 or not 2<=len(speeds)<=32 or not np.isfinite(speeds).all()
            or speeds[0]<=0 or np.any(np.diff(speeds)<=0)):
        raise ValueError('Invalid measured speed coordinates')
    periodic=model.get('schema')=='common_periodic_friction_v1'
    signed=False;orders=[1,2];frequency=0.;hat_knots=[]
    if periodic and model.get('family')=='positive_periodic_load':
        coefficients=model['coefficients'];width=model['load_width_nm'];scale=model['load_feature_scale_nm']
        orders=model['orders'];frequency=model['frequency_rad_per_rad']
        hat_knots=model.get('load_hat_knots_nm',[])
        if hat_knots:
            a=np.asarray(hat_knots,float)
            if a.ndim!=1 or not 3<=len(a)<=16 or not np.isfinite(a).all() or np.any(np.diff(a)<=0):
                raise ValueError('Invalid additional load coordinates')
        if (not 1<=len(orders)<=4 or any(type(k)!=int or k<=0 for k in orders)
                or orders!=sorted(set(orders)) or not np.isfinite(frequency) or frequency<=0):
            raise ValueError('Invalid periodic coordinates')
    elif model.get('schema')=='common_friction_v1' and model['family'] in ('smooth_load','shared_smooth_load'):
        coefficients=model['soft_load_coefficients'];width=model['load_width_nm'];scale=model['load_feature_scale_nm']
    elif model.get('schema')=='common_friction_v1' and model['family'] in ('velocity','signed_load'):
        signed=model['family']=='signed_load';coefficients={};width=scale=1.
        knots=np.asarray(model.get('gravity_knots_nm',[-1.,0.,1.]),float)
        if knots.ndim!=1 or not 3<=len(knots)<=16 or not np.isfinite(knots).all() or np.any(np.diff(knots)<=0):
            raise ValueError('Invalid load coordinates')
        for d in ('positive','negative'):
            table=np.asarray(model['tables'][d],float)
            if (table.shape!=(len(knots),len(speeds)) or not np.isfinite(table).all()
                    or np.min(table)<0 or np.min(table[:,0])<=0
                    or (not signed and np.min(table)<=0)):
                raise ValueError('Invalid load table: nonnegative kinetic loss and positive rest yield required')
            if not signed and not np.all(table==table[0]):
                raise ValueError('Velocity family contains nonzero load dependence')
            coefficients[d]=table if signed else np.vstack([table[0],np.zeros_like(table[0]),np.zeros_like(table[0])])
    else:raise ValueError('Unsupported common kinetic model family')
    if not np.isfinite([width,scale]).all() or min(width,scale)<=0:
        raise ValueError('Invalid load scale/width')
    n=(len(knots) if signed else 3+len(hat_knots))*len(speeds)*(1+4*len(orders) if periodic else 1)
    for d in ('positive','negative'):
        row=np.asarray(coefficients[d],float).ravel()
        if row.size!=n or not np.isfinite(row).all() or np.min(row)<0 or np.max(row)<=0:
            raise ValueError('Invalid directional kinetic coefficients')
    extension=model.get('load_hat_mean_extension',False)
    if type(extension)!=bool or (extension and (not periodic or not hat_knots or any(
            np.any(np.asarray(coefficients[d]).reshape(3+len(hat_knots),len(speeds),-1)[:3]!=0)
            for d in ('positive','negative')))):
        raise ValueError('Mean load extension requires a pure signed-load periodic basis')
    p=dict(common_load_width=width,common_load_scale=scale,common_speeds=speeds.tolist(),
        common_positive=np.asarray(coefficients['positive']).ravel().tolist(),
        common_negative=np.asarray(coefficients['negative']).ravel().tolist(),
        common_periodic_frequency=frequency,common_periodic_orders=orders)
    if signed:p['common_gravity_knots']=knots.tolist()
    if hat_knots:p['common_gravity_knots']=list(hat_knots)
    if extension:p['common_load_hat_mean_extension']=True
    return p


def parameters(model, design, baseline, fraction):
    """Preserve the exact model and original gravity; dither is composed separately."""
    expected=model_parameters(model,model['joint'])
    optional_defaults={'common_load_hat_mean_extension':False,'common_gravity_knots':[]}
    if (any(design.get(k)!=v for k,v in expected.items()) or
            any(design.get(k,default)!=expected.get(k,default) for k,default in optional_defaults.items())):
        raise ValueError('Probe model coefficients differ')
    if not np.isfinite([design['transition_decay'],design['common_transition_distance']]).all() or design['transition_decay']<=0 or design['common_transition_distance']<0:
        raise ValueError('Invalid kinetic transition design')
    if model['schema']=='common_periodic_friction_v1':
        from .periodic_friction import evaluate
        adapter=dict(model,models={d:[float(evaluate(model,0.,0.,0.,d)[0]),0.,0.,0.] for d in ('positive','negative')})
    else:
        from .common_friction import runtime_model
        adapter=runtime_model(model)
    p=base_parameters(adapter,baseline,design['transition_stiffness'],fraction)
    p.update(stateful_load_periodic_frequency=expected['common_periodic_frequency'],
        stateful_load_periodic_orders=expected['common_periodic_orders'],
        stateful_load_transition_distance=design['common_transition_distance'],
        stateful_decay=[design['transition_decay'] if j==model['joint']-1 else 0. for j in range(6)])
    if 'common_gravity_knots' not in expected or model['schema']=='common_periodic_friction_v1':
        p.update(stateful_load_gravity_knots=[0.],stateful_load_smooth_width_nm=expected['common_load_width'],
            stateful_load_feature_scale_nm=expected['common_load_scale'],stateful_load_speed_knots=expected['common_speeds'],
            stateful_load_smooth_positive=expected['common_positive'],stateful_load_smooth_negative=expected['common_negative'])
        if model.get('load_hat_knots_nm'):
            p['stateful_load_gravity_knots']=expected['common_gravity_knots']
    if model.get('load_hat_mean_extension',False):
        p['stateful_load_hat_mean_extension']=True
    return p
