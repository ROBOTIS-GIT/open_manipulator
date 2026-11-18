"""Offline, grouped development of load-aware steady friction models.

No ROS publishers, launch, profile export, motor writes, or deployment approval.
Existing posture holdouts are explicitly reclassified as DEVELOPMENT data.
The resulting leave-one-posture-out scores select a candidate; they are not a
fresh final validation. No temperature or rigid-body parameters are fitted.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import nnls

from .campaign import read_acquisition
from .fit import pair_samples
from .poses import Robot
from .protocol import JOINTS, sha

# Prespecified dictionary; no per-posture identifiers or joint-angle harmonics.
SPEED_SCALES = (.004, .012, .04, .12, .30)
FAMILIES = ('velocity', 'gravity', 'wrench')
FEATURES = ('abs_gravity_ros_nm', 'bending_moment_nm', 'radial_force_n')
RIDGE = 1e-4


def interval_means(x, y, edges):
    """Spatial integrals, retaining all input knots, including encoder ripple."""
    x, y, edges = map(np.asarray, (x, y, edges))
    if (len(x) < 2 or np.any(np.diff(x) <= 0) or np.any(np.diff(edges) <= 0)
            or edges[0] < x[0] or edges[-1] > x[-1] or not np.isfinite(y).all()):
        raise ValueError('Invalid spatial integration grid')
    means = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        knots = np.r_[lo, x[(x > lo) & (x < hi)], hi]
        values = np.interp(knots, x, y)
        means.append(np.sum(np.diff(knots)*(values[:-1]+values[1:])/2)/(hi-lo))
    return np.array(means)


def bin_cycles(rows, robot, scaling, bins=24):
    if type(bins) is not int or bins < 8 or bins > 4096:
        raise ValueError('Use 8..4096 integer spatial bins per complete cycle')
    cycles = []
    for row in rows:
        x = np.asarray(row['position_rad']); j = row['joint']
        edges = np.linspace(x[0], x[-1], bins+1); centers = (edges[:-1]+edges[1:])/2
        qp, qn = [np.column_stack([np.interp(centers, x, np.asarray(row[key])[:, k])
                                  for k in range(6)]) for key in ('positive_q_rad','negative_q_rad')]
        q = (qp+qn)/2
        loads = robot.static_load_batch(q)
        g = loads['gravity_nm'][:, j]*scaling[j]
        features = np.column_stack([abs(g), loads['bending_moment_nm'][:, j],
                                    loads['radial_force_n'][:, j]])
        positive, negative = [interval_means(x, row[key], edges) for key in
                             ('positive_effort_samples_nm','negative_effort_samples_nm')]
        y = (positive-negative)/2
        group = row['validation_pose'] or 'base'
        cycles.append(dict(joint=j, repeat=row['repeat'], group=group, speed=row['speed'],
            spatial_bins=bins, raw_angular_support=row.get('raw_angular_support'),
            cycle_id=f"{group}/r{row['repeat']}/j{j+1}/v{row['speed']:.12g}",
            q=q.tolist(), features=features.tolist(), target=y.tolist(),
            signed_gravity_ros_nm=g.tolist(),
            directional_bias_ros_nm=((positive+negative)/2-g).tolist(),
            positive_resistance_proxy_nm=(positive-g).tolist(),
            negative_resistance_proxy_nm=(g-negative).tolist(),
            max_direction_configuration_difference_rad=float(np.max(abs(qp-qn))),
            angle_interval_rad=[float(x[0]),float(x[-1])]))
    return cycles


def design(speed, features, family, vs):
    v=np.asarray(speed); z=np.asarray(features)
    if family not in FAMILIES or vs <= 0 or z.shape != (len(v),3):
        raise ValueError('Invalid load model design')
    if not np.isfinite(v).all() or not np.isfinite(z).all() or np.any(v<0) or np.any(z<0):
        raise ValueError('Nonfinite/negative model inputs')
    peak=np.exp(-(v/vs)**2)
    columns=[np.ones_like(v),v,peak]; names=['coulomb','viscous','low_speed_peak']
    for i in range({'velocity':0,'gravity':1,'wrench':3}[family]):
        columns.extend([z[:,i],z[:,i]*peak]);names.extend([FEATURES[i],FEATURES[i]+'_low_speed'])
    return np.column_stack(columns), names


def flatten(cycles):
    if not cycles: raise ValueError('Empty complete-cycle set')
    v=np.concatenate([np.full(len(c['target']),c['speed']) for c in cycles])
    f=np.concatenate([c['features'] for c in cycles])
    y=np.concatenate([c['target'] for c in cycles])
    # One cycle has total weight one, regardless of sensor sample count or bins.
    w=np.concatenate([np.full(len(c['target']),1/len(c['target'])) for c in cycles])
    return v,f,y,w


def fit(cycles, family, vs):
    v,z,y,w=flatten(cycles)
    if not np.isfinite(y).all() or np.any(y<=0): raise ValueError('Invalid paired resistance')
    x,names=design(v,z,family,vs)
    # A posture-invariant load contributes only another intercept/peak. It
    # cannot identify a load coefficient, regardless of sample count. Remove
    # those exact positive multiples, not arbitrary correlated columns.
    active=list(range(3)); pruned=[]
    for i in range((x.shape[1]-3)//2):
        if np.ptp(z[:,i]) <= 1e-10*max(1.,float(np.max(z[:,i]))):
            pruned.extend(names[3+2*i:5+2*i])
        else:
            active.extend([3+2*i,4+2*i])
    full_columns=x.shape[1];x=x[:,active]
    scale=np.sqrt(np.sum(w[:,None]*x*x,axis=0)/np.sum(w)); scale[scale<1e-10]=1.
    a=x/scale*np.sqrt(w[:,None]/np.sum(w));b=y*np.sqrt(w/np.sum(w))
    coef,_=nnls(np.vstack([a,np.sqrt(RIDGE)*np.eye(x.shape[1])]),np.r_[b,np.zeros(x.shape[1])],maxiter=1000)
    sv=np.linalg.svd(a,compute_uv=False)
    rank=int(np.sum(sv>max(sv[0]*1e-7,1e-12)))
    full_coef=np.zeros(full_columns);full_coef[active]=coef/scale
    return dict(family=family,vs_rad_s=vs,coefficients=full_coef.tolist(),feature_names=names,
                unregularized_rank=rank,columns=x.shape[1],singular_values=sv.tolist(),
                pruned_constant_features=pruned,
                coefficients_physically_identified=False,regularization=RIDGE,
                cycle_ids=[c['cycle_id'] for c in cycles],
                feature_min=np.min(z,axis=0).tolist(),feature_max=np.max(z,axis=0).tolist(),
                speed_range=[float(min(v)),float(max(v))])


def predict(model, cycle):
    x,_=design(np.full(len(cycle['target']),cycle['speed']),cycle['features'],
               model['family'],model['vs_rad_s'])
    return x@model['coefficients']


def score(model, cycles, absolute, relative):
    rows=[]
    for c in cycles:
        y=np.asarray(c['target']);p=predict(model,c);limit=absolute+relative*y
        z=np.asarray(c['features']);lo=np.asarray(model['feature_min']);hi=np.asarray(model['feature_max'])
        # Marginal bounds are diagnostic only, not a support/safety certificate.
        outside=(z<lo-1e-7)|(z>hi+1e-7)
        rows.append(dict(cycle_id=c['cycle_id'],group=c['group'],speed_rad_s=c['speed'],
            measured_mean_nm=float(y.mean()),predicted_mean_nm=float(p.mean()),
            mean_passed=bool(abs(p.mean()-y.mean())<=absolute+relative*y.mean()),
            normalized_mse=float(np.mean(((p-y)/limit)**2)),
            spatial_pass_fraction=float(np.mean(abs(p-y)<=limit)),
            max_overprediction_nm=float(max(0.,np.max(p-y))),
            outside_feature_box_fraction=float(np.mean(np.any(outside,axis=1)))))
    return dict(cycles=len(rows),mean_failed=sum(not r['mean_passed'] for r in rows),
                normalized_rmse=float(np.sqrt(np.mean([r['normalized_mse'] for r in rows]))),
                spatial_pass_fraction=float(np.mean([r['spatial_pass_fraction'] for r in rows])),rows=rows)


def choose(cycles, absolute, relative):
    groups=sorted({c['group'] for c in cycles})
    if len(groups)<3:raise ValueError('Need at least three posture groups, not randomly split samples')
    candidates=[]
    for family in FAMILIES:
        for vs in SPEED_SCALES:
            folds=[]
            for heldout in groups:
                train=[c for c in cycles if c['group']!=heldout]
                test=[c for c in cycles if c['group']==heldout]
                model=fit(train,family,vs)
                folds.append(dict(heldout=heldout,training_cycle_ids=model['cycle_ids'],
                                  **score(model,test,absolute,relative)))
            mse=np.mean([f['normalized_rmse']**2 for f in folds])
            candidates.append(dict(family=family,vs_rad_s=vs,score=float(np.sqrt(mse)),folds=folds))
    selected=min(candidates,key=lambda r:(r['score'],FAMILIES.index(r['family']),r['vs_rad_s']))
    model=fit(cycles,selected['family'],selected['vs_rad_s'])
    return model, dict(selected_family=selected['family'],selected_vs=selected['vs_rad_s'],
                       selection_score=selected['score'],candidates=candidates,
                       interpretation='Development model selection by whole-posture exclusion; NOT fresh final validation')


def reject_contact(campaign):
    marker=Path(campaign)/'human_contact_review.json'
    if marker.exists():
        raise ValueError('Human-contact run is quarantined; use a separately collected campaign')


def run(campaign, output, baseline, bins=24, propose_poses=False):
    from .integration import original_gravity
    campaign=Path(campaign).resolve();output=Path(output).resolve()
    reject_contact(campaign)
    if output.exists():raise ValueError('Use a new output folder; evidence is never overwritten')
    bm,bd,be=read_acquisition(campaign/'identification',False)
    pm,pd,pe=read_acquisition(campaign/'postures',True)
    if bm['config']!=pm['config'] or be['hashes']['robot.urdf']!=pe['hashes']['robot.urdf']:
        raise ValueError('Inconsistent configuration or robot geometry')
    old=json.loads((campaign/'candidate/identification.json').read_text())
    if old['baseline_hash']!=sha(baseline):raise ValueError('Original gravity baseline changed')
    for expected,actual in zip(old['acquisitions'],(be,pe)):
        if expected['hashes']!=actual['hashes']:raise ValueError('Acquisition hashes changed')
    scaling=original_gravity(baseline).get('torque_scaling_factors',[1.]*6)
    robot=Robot(campaign/'identification/robot.urdf',JOINTS)
    rows=pair_samples(bm,bd,include_directional=True)[0]+pair_samples(pm,pd,include_directional=True)[0]
    cycles=bin_cycles(rows,robot,scaling,bins)
    c=bm['config'];absolute=c['validation_absolute_error_nm'];relative=c['validation_relative_error']
    joints=[]
    for j in range(6):
        all_rows=[x for x in cycles if x['joint']==j]
        dev=[x for x in all_rows if x['group']!='base' or x['repeat']==0]
        repeat=[x for x in all_rows if x['group']=='base' and x['repeat']==1]
        model,cv=choose(dev,absolute,relative)
        y=np.concatenate([x['target'] for x in all_rows]);bias=np.concatenate([x['directional_bias_ros_nm'] for x in all_rows])
        joints.append(dict(joint=JOINTS[j],model=model,posture_group_selection=cv,
            development_fit=score(model,dev,absolute,relative),
            repeat_diagnostic=score(model,repeat,absolute,relative),
            directional_diagnostic=dict(bias_abs_95_nm=float(np.quantile(abs(bias),.95)),
                apparent_asymmetry_ratio_95=float(np.quantile(abs(bias)/y,.95)),
                gravity_or_directional_friction_separable=False),
            required_next='Collect fresh matched speed x signed-load x bearing-load conditions and unseen posture combinations'))
        print(f'{JOINTS[j]}: {model["family"]}, development posture CV={cv["selection_score"]:.3f}',flush=True)
    result=dict(schema=1,method='whole_cycle_physical_load_development_v1',campaign=str(campaign),
        input_hashes=[be,pe],baseline_hash=sha(baseline),bins_per_cycle=bins,
        feature_definition=list(FEATURES),speed_scales=list(SPEED_SCALES),joints=joints,
        prior_posture_holdouts_reclassified_as_development=True,
        fresh_external_validation=False,temperature_model=False,rigid_parameters_modified=False,
        current_units_modified=False,measured_effort_as_runtime_input=False,
        sensor_timing_verified=c['sensor_timing_verified'],hardware_deployable=False,
        dynamic_parameters_identified=False,compensation_parameters=None,
        scope='Steady symmetric paired resistance under nominal payload; direction bias is diagnostic only',
        sources=['https://ieeexplore.ieee.org/document/9417160/','https://arxiv.org/abs/2410.12685'])
    result['analysis_source_hashes']={name:sha(Path(__file__).with_name(name)) for name in
                                     ('generalization.py','poses.py','fit.py')}
    output.mkdir(parents=True)
    (output/'development.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (output/'cycles.json').write_text(json.dumps(cycles,allow_nan=False)+'\n')
    if propose_poses:
        from .coverage_plan import propose
        proposal=propose(cycles,campaign/'identification/robot.urdf',scaling)
        proposal['source_hashes']={name:sha(Path(__file__).with_name(name)) for name in
                                 ('coverage_plan.py','poses.py')}
        (output/'pose_proposals.json').write_text(json.dumps(proposal,indent=2,allow_nan=False)+'\n')
    return result


def main():
    from .integration import baseline_path
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--campaign',required=True);p.add_argument('--output',required=True)
    p.add_argument('--bins',type=int,default=24)
    p.add_argument('--propose-poses',action='store_true',help='Static-clearance suggestions only; never move hardware')
    a=p.parse_args();run(a.campaign,a.output,baseline_path(),a.bins,a.propose_poses)


if __name__=='__main__':main()
