"""One loaded joint: directional residual identification against unchanged gravity.

Small parametric candidates and a load-interpolated affine-speed map share the
same directional-cycle validation. Current residuals are NOT independent output
torque measurements. Spatial failures are retained; scaling cannot pass them.
"""
from copy import deepcopy
import numpy as np
from scipy.optimize import nnls
from .automatic_calibration import checked_cycles, content_hash, payload_hash


def features(cycle, speed_endpoint=None):
    g = np.asarray(cycle['signed_gravity_ros_nm'])
    if speed_endpoint is not None:
        t=min(cycle['speed']/speed_endpoint,1.)
        return np.column_stack([np.ones(len(g)),np.full(len(g),min(cycle['speed'],speed_endpoint)),
            np.maximum(g,0.)*(1-t),np.maximum(-g,0.)*(1-t),
            np.maximum(g,0.)*t,np.maximum(-g,0.)*t])
    return np.column_stack([np.ones(len(g)), np.full(len(g), cycle['speed']),
                            np.maximum(g, 0.), np.maximum(-g, 0.)])


def table_values(model, speed, gravity, direction):
    """Bilinear load/speed map; endpoint speed and outward nonnegative load slopes.

    Extrapolation is an explicit hypothesis, never a qualification of that load.
    Only nominal gravity is used, so user-applied force cannot enter this map.
    """
    if model.get('family') in ('smooth_load', 'shared_smooth_load'):
        coef=[np.interp(abs(speed),model['speed_knots_rad_s'],row)
              for row in model['soft_load_coefficients'][direction]]
        g=np.asarray(gravity); r=np.hypot(g,model['load_width_nm'])
        return coef[0]+coef[1]*(r+g)/(2*model['load_feature_scale_nm'])+coef[2]*(r-g)/(2*model['load_feature_scale_nm'])
    knots=np.asarray(model['gravity_knots_nm']);speeds=model['speed_knots_rad_s']
    table=np.asarray(model['tables'][direction])
    values=np.array([np.interp(abs(speed),speeds,row) for row in table])
    g=np.asarray(gravity)
    out=np.interp(g,knots,values)
    left=min(0.,(values[1]-values[0])/(knots[1]-knots[0]))
    right=max(0.,(values[-1]-values[-2])/(knots[-1]-knots[-2]))
    return out+left*np.minimum(g-knots[0],0.)+right*np.maximum(g-knots[-1],0.)


def fit(cycles, joint, speed_dependent_load=True, load_table=False):
    checked_cycles(cycles)
    if joint not in range(6) or any(c['joint'] != joint for c in cycles):
        raise ValueError('Exactly one selected joint required')
    if len({c['speed'] for c in cycles}) < 3 or len({c['group'] for c in cycles}) < 2:
        raise ValueError('Need multiple speeds and load conditions')
    if load_table:
        # Pool nearby nominal loads rather than fitting a steep slope between
        # almost identical postures. One Nm is fixed before fitting any losses.
        groups=sorted({c['group'] for c in cycles},key=lambda group:np.mean([
            np.mean(c['signed_gravity_ros_nm']) for c in cycles if c['group']==group]))
        clusters=[];centres=[]
        for group in groups:
            centre=float(np.mean([np.mean(c['signed_gravity_ros_nm']) for c in cycles if c['group']==group]))
            if not clusters or centre-centres[-1]>1.:
                clusters.append([group]);centres.append(centre)
            else:clusters[-1].append(group)
        speeds=sorted({c['speed'] for c in cycles});knots=[];tables={d:[] for d in ('positive','negative')}
        if len(clusters)<3:raise ValueError('Load table needs three separated contemporary load levels')
        for cluster in clusters:
            rows=[c for c in cycles if c['group'] in cluster]
            knots.append(float(np.mean([np.mean(c['signed_gravity_ros_nm']) for c in rows])))
            for d in tables:
                for v in speeds:
                    selected=[np.mean(c[d+'_resistance_proxy_nm']) for c in rows if c['speed']==v]
                    if not selected or min(selected)<=0:raise ValueError('Positive measurements at every load/speed required')
                # Two endpoint coefficients per load avoid memorising the noise
                # in individual speed bins; weakening remains permitted.
                t=np.array([(c['speed']-speeds[0])/(speeds[-1]-speeds[0]) for c in rows])
                endpoints=nnls(np.column_stack([1-t,t]),
                    [np.mean(c[d+'_resistance_proxy_nm']) for c in rows])[0]
                if min(endpoints)<=0:raise ValueError('Positive loss endpoints required')
                values=np.interp(speeds,[speeds[0],speeds[-1]],endpoints).tolist()
                tables[d].append(values)
        return dict(schema='single_joint_signed_load_v3',joint=joint+1,tables=tables,
            gravity_knots_nm=knots,speed_knots_rad_s=speeds,load_clusters=clusters,
            load_clustering_width_nm=1.,load_extension='outward_nonnegative_slope',
            per_load_speed_model='positive_affine_endpoints',
            speed_extension='constant_endpoint',fitting_target='complete_directional_cycle_mean',
            speed_range_rad_s=[speeds[0],speeds[-1]],gravity_range_nm=[knots[0],knots[-1]],
            development_payload_hashes=[payload_hash(c) for c in cycles],development_sha256=content_hash(cycles),
            gravity_or_current_conversion_changed=False,physical_output_torque_identified=False,
            static_breakaway_identified=False,hardware_qualified=False)
    endpoint=max(c['speed'] for c in cycles) if speed_dependent_load else None
    # Identify the mean transmission loss. Spatial current ripple is evaluated
    # separately, not used as artificial load excitation within one posture.
    x = np.array([np.mean(features(c,endpoint),axis=0) for c in cycles])
    weight = np.ones(len(cycles))
    norm = np.sqrt(np.average(x*x, axis=0, weights=weight))
    active = norm > 1e-10
    a = x[:, active]/norm[active]*np.sqrt(weight[:, None])
    condition=float(np.linalg.cond(a))
    if np.linalg.matrix_rank(a) != sum(active) or condition>1000.:
        raise ValueError('Unobservable/ill-conditioned mean-load model; add a contemporary zero-load condition')
    models = {}
    for direction in ('positive', 'negative'):
        y = np.array([np.mean(c[direction+'_resistance_proxy_nm']) for c in cycles])
        if np.min(y) <= 0.:
            raise ValueError('Nonpositive directional residual: gravity/current error must be separated')
        coefficients = np.zeros(x.shape[1])
        coefficients[active] = nnls(a, y*np.sqrt(weight))[0]/norm[active]
        if coefficients[0] <= 0.:
            raise ValueError('No positive zero-load yield')
        models[direction] = coefficients.tolist()
    gravity=np.concatenate([c['signed_gravity_ros_nm'] for c in cycles])
    return dict(schema='single_joint_signed_load_v2' if speed_dependent_load else 'single_joint_signed_load_v1', joint=joint+1,
        models=models, feature_names=['coulomb','viscous','positive_nominal_gravity','negative_nominal_gravity']+
            (['positive_gravity_at_max_speed','negative_gravity_at_max_speed'] if speed_dependent_load else []),
        unobserved_columns=np.flatnonzero(~active).tolist(),
        mean_design_condition_number=condition, fitting_target='complete_directional_cycle_mean',
        speed_range_rad_s=[min(c['speed'] for c in cycles),max(c['speed'] for c in cycles)],
        gravity_range_nm=[float(np.min(gravity)),float(np.max(gravity))],
        development_payload_hashes=[payload_hash(c) for c in cycles],
        development_sha256=content_hash(cycles), gravity_or_current_conversion_changed=False,
        physical_output_torque_identified=False, static_breakaway_identified=False,
        hardware_qualified=False)


def evaluate(model, cycles, independent=False):
    checked_cycles(cycles)
    if model['schema'] not in ('single_joint_signed_load_v1','single_joint_signed_load_v2','single_joint_signed_load_v3') or any(c['joint']+1!=model['joint'] for c in cycles):
        raise ValueError('Wrong model/joint')
    if independent and set(payload_hash(c) for c in cycles)&set(model['development_payload_hashes']):
        raise ValueError('Validation duplicates development data')
    rows=[]
    for c in cycles:
        x=features(c,model['speed_range_rad_s'][1] if model['schema'].endswith('v2') else None)
        for direction in ('positive','negative'):
            y=np.asarray(c[direction+'_resistance_proxy_nm'])
            predicted=(table_values(model,c['speed'],c['signed_gravity_ros_nm'],direction)
                if model['schema'].endswith('v3') else x@model['models'][direction])
            error=predicted-y; allowance=.08+.20*np.maximum(y,0.)
            mean_passed=bool(abs(error.mean())<=.08+.20*max(0.,y.mean()))
            spatial=float(np.mean(abs(error)<=allowance))
            rows.append(dict(cycle_id=c['cycle_id'],group=c['group'],direction=direction,
                speed_rad_s=c['speed'], gravity_mean_nm=float(np.mean(c['signed_gravity_ros_nm'])),
                measured_mean_nm=float(y.mean()),predicted_mean_nm=float(predicted.mean()),
                mean_passed=mean_passed,spatial_pass_fraction=spatial,
                maximum_overprediction_nm=float(max(0.,max(error))),
                maximum_underprediction_nm=float(max(0.,max(-error))),
                passed=mean_passed and spatial>=.95))
    return dict(independent=independent,rows=rows,means_passed=all(r['mean_passed'] for r in rows),
        passed=all(r['passed'] for r in rows),model_sha256=content_hash(model),
        validation_sha256=content_hash(cycles),physical_output_torque_validated=False)


def freeze(cycles, identity, joint, load_table=False):
    model=fit(cycles,joint,load_table=load_table)
    training=evaluate(model,cycles)
    # Reserve the opposite direction-order repetitions before fitting.
    # Repeat is the full-cycle index, not necessarily order 0/1. Match each
    # group/speed chronologically and hold its second complete paired cycle out.
    groups=sorted({(c['group'],c['speed']) for c in cycles})
    train=[];held=[]
    for group,speed in groups:
        rows=[c for c in cycles if (c['group'],c['speed'])==(group,speed)]
        if len(rows)!=2:
            raise ValueError('Two complete, counterbalanced pairs per condition required')
        rows.sort(key=lambda c:c['repeat']);train.append(rows[0]);held.append(rows[1])
    cross=evaluate(fit(train,joint,load_table=load_table),held,True)
    model.update(identity=deepcopy(identity),joints=[joint+1],
        missing=[] if training['passed'] and cross['passed'] else ['steady_development_prediction_failed'],
        training_review=training,opposite_order_review=cross,
        automatic_deployment_allowed=False)
    return model


def validate(frozen, cycles, evidence, allow_synthetic=False):
    if (evidence.get('candidate_sha256')!=content_hash(frozen) or
        evidence.get('identity')!=frozen.get('identity') or
        evidence.get('role')!='fresh_final_validation' or
        evidence.get('source') not in (('hardware','synthetic') if allow_synthetic else ('hardware',)) or
        evidence.get('completed') is not True or
        evidence.get('contact_free') is not True):
        raise ValueError('Fresh collection is not reserved for this exact model and robot')
    result=evaluate(frozen,cycles,True)
    result.update(stage='STEADY_VALIDATED' if result['passed'] and not frozen['missing'] else 'NEEDS_REVIEW',
        missing=[] if result['passed'] and not frozen['missing'] else ['independent_loaded_prediction_failed'],
        hardware_deployable=False)
    return result


def parameters(model, baseline, stiffness, output_scale=1.):
    """Generate additive parameters only; the original gravity settings survive."""
    j=model['joint']-1
    if j not in range(6) or baseline['joints']!=[f'joint{k}' for k in range(1,7)]:
        raise ValueError('Original six-axis gravity configuration required')
    if not np.isfinite(stiffness) or stiffness<=0 or not 0<output_scale<=1:
        raise ValueError('Invalid explicit transition design')
    p=deepcopy(baseline)
    p.update(calibrated_friction_enabled=False,stateful_friction_enabled=True,
        stateful_joint_mask=[int(k==j) for k in range(6)],stateful_load_joint=j+1,
        stateful_load_speed_endpoint=model['speed_range_rad_s'][1],
        stateful_curve_j6_enabled=False,stateful_dither_j6_only=False,stateful_dither_with_elastic=False,
        stateful_curve_output_scale=output_scale,stateful_observer_window=.02,friction_diagnostics_rate=50.)
    for key,value in [('stiffness',stiffness),('decay',5.),('damping',0.),('maximum_velocity',0.)]:
        p['stateful_'+key]=[value if k==j else 0. for k in range(6)]
    for d in ('positive','negative'):
        c=(np.array([float(table_values(model,0.,0.,d)),0.,0.,0.])
           if model['schema'].endswith('v3') else np.asarray(model['models'][d]))
        if c.shape not in ((4,),(6,)) or not np.isfinite(c).all() or min(c)<0 or c[0]<=0:
            raise ValueError('Invalid directional coefficients')
        p['stateful_load_'+d]=c.tolist()
        p['stateful_'+d+'_yield']=[float(c[0]) if k==j else 0. for k in range(6)]
        p['stateful_'+d+'_viscous']=[0.]*6
    if model['schema'].endswith('v3'):
        p.update(stateful_load_gravity_knots=model['gravity_knots_nm'],
                 stateful_load_speed_knots=model['speed_knots_rad_s'])
        for d in ('positive','negative'):p['stateful_load_'+d+'_table']=np.asarray(model['tables'][d]).ravel().tolist()
    if model.get('family') in ('smooth_load', 'shared_smooth_load'):
        p.update(stateful_load_gravity_knots=[0.],stateful_load_positive_table=[0.],stateful_load_negative_table=[0.],
                 stateful_load_smooth_width_nm=model['load_width_nm'],
                 stateful_load_feature_scale_nm=model['load_feature_scale_nm'])
        for d in ('positive','negative'):
            p['stateful_load_smooth_'+d]=np.asarray(model['soft_load_coefficients'][d]).ravel().tolist()
    return p


def transition_stiffness(model):
    if model['schema'].endswith('v3'):
        yields=[row[0] for table in model['tables'].values() for row in table]
    else:
        yields=[c[0]+c[2]*max(g,0.)+c[3]*max(-g,0.)
                for c in model['models'].values() for g in model['gravity_range_nm']]
    return max(yields)*5./model['speed_range_rad_s'][0]
