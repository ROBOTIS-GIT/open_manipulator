"""Shared spatial kinetic review, separate from rest and release qualification."""
import numpy as np
from .automatic_calibration import checked_cycles
from .kinetic_runtime import model_parameters


def angular_speed_support(cycles,joint,position,velocity):
    """Measured target-angle support at both neighbouring speed coordinates.

    This is an analysis result, never a robot motion limit or a certificate of
    load/history coverage. A periodic model may extrapolate beyond these angles,
    but that extrapolation must not be described as a measured operating domain.
    """
    q,v=np.broadcast_arrays(np.asarray(position,float),np.asarray(velocity,float))
    if (joint not in range(6) or q.ndim!=1 or not np.isfinite([q,v]).all()
            or not cycles or any(c['joint']!=joint for c in cycles)):
        raise ValueError('Invalid angular/speed coverage inputs')
    speeds=np.array(sorted({c['speed'] for c in cycles}))
    if not np.isfinite(speeds).all() or np.any(speeds<=0):raise ValueError('Invalid measured speeds')
    supported=[]
    for speed in speeds:
        covered=np.zeros(len(q),bool)
        for c in cycles:
            if c['speed']!=speed:continue
            angles=np.asarray(c['q'],float)[:,joint]
            if not np.isfinite(angles).all() or len(angles)<2:raise ValueError('Invalid measured angles')
            covered|=(q>=min(angles))&(q<=max(angles))
        supported.append(covered)
    high=np.searchsorted(speeds,abs(v),side='left').clip(0,len(speeds)-1)
    low=np.maximum(high-1,0)
    # Exact knots need only their own data; below the first knot uses the same
    # constant endpoint as runtime, with extrapolation flagged separately.
    low=np.where(np.isclose(abs(v),speeds[high],rtol=1e-10,atol=1e-12),high,low)
    low=np.where((abs(v)<=speeds[0])|(abs(v)>=speeds[-1]),high,low)
    support=np.asarray(supported);index=np.arange(len(q))
    inside=support[low,index]&support[high,index]
    measured_speed=(abs(v)>=speeds[0])&(abs(v)<=speeds[-1])
    return dict(angle_supported=inside.tolist(),speed_supported=measured_speed.tolist(),
        lower_speed_rad_s=speeds[low].tolist(),upper_speed_rad_s=speeds[high].tolist(),
        angular_extrapolation_fraction=float(np.mean(~inside)) if len(q) else None,
        measured_domain_fraction=float(np.mean(inside&measured_speed)) if len(q) else None,
        full_pose_load_history_coverage_proven=False,motion_restricted=False)


def prediction(model,cycle,direction):
    if model['schema']=='common_periodic_friction_v1':
        from .periodic_friction import cycle_prediction
        return cycle_prediction(model,cycle,direction)
    from .single_joint_model import table_values
    return table_values(model,cycle['speed'],cycle['signed_gravity_ros_nm'],direction)


def review(model,cycles,fraction=.95):
    model_parameters(model,model['joint']);checked_cycles(cycles)
    if not np.isfinite(fraction) or not 0<fraction<=1 or any(c['joint']+1!=model['joint'] for c in cycles):
        raise ValueError('Invalid kinetic fraction or joint data')
    rows=[];scores=[]
    for c in cycles:
        for d in ('positive','negative'):
            y=np.asarray(c[d+'_resistance_proxy_nm']);pred=prediction(model,c,d)
            error=pred-y;excess=fraction*pred-y;allowance=.08+.2*np.maximum(y,0.)
            scores.append(float(np.mean((error/allowance)**2)))
            rows.append(dict(cycle_id=c['cycle_id'],group=c['group'],speed_rad_s=c['speed'],direction=d,
                measured_mean_nm=float(y.mean()),predicted_mean_nm=float(pred.mean()),
                mean_passed=bool(abs(error.mean())<=.08+.2*max(0.,y.mean())),
                spatial_pass_fraction=float(np.mean(abs(error)<=allowance)),
                maximum_compensation_excess_nm=float(max(0.,excess.max())),
                compensation_excess_fraction=float(np.mean(excess>0)),
                q_interval_rad=[float(np.min(np.asarray(c['q'])[:,c['joint']])),float(np.max(np.asarray(c['q'])[:,c['joint']]))]))
    return dict(rows=rows,score=float(np.sqrt(np.mean(scores))),score_kind='equal_cycle_direction_local_normalized_rmse',
        means_passed=all(r['mean_passed'] for r in rows),
        spatial_passed=all(r['mean_passed'] and r['spatial_pass_fraction']>=.95 for r in rows),
        kinetic_fraction=fraction,local_excess_detected=any(r['maximum_compensation_excess_nm']>0 for r in rows),
        coverage=dict(joint=model['joint'],speed_rad_s=sorted({c['speed'] for c in cycles}),
                      intervals=[dict(cycle_id=r['cycle_id'],speed_rad_s=r['speed_rad_s'],direction=r['direction'],q_interval_rad=r['q_interval_rad']) for r in rows]),
        comparison_is_current_derived_proxy=True,coast_deceleration_verified=False,
        independent_output_torque_measured=False,hardware_deployable=False)
