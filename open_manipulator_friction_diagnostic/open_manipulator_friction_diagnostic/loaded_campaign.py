"""Freeze training-only models, then independently assess the loaded repeat.

No hardware command, controller activation, latest link, or gain fitting lives
here. Qualification is experimental steady-mean prediction, not passivity.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import numpy as np
import yaml

from .campaign import read_acquisition
from .curve_selection import select_curve
from .fit import pair_samples, spatial_weights
from .integration import fingerprint, original_gravity
from .loaded_model import attach_nominal_load, fit_candidates, fit_position_candidates
from .protocol import JOINTS, sha

QUALIFICATION='loaded_steady_mean_unscaled_experiment_v1'
UNQUALIFIED='unqualified_loaded_candidate_v1'
ORDER=['fc','fs_curve','vs','viscous','load_gain','load_peak','load_vs',
       'position_peak','position_order','position_phase']
CONTEXT_ORDER=['context_peak','context_vs','context_joint_1based','context_power']
POLICY=dict(gain=1.,torque_cap=None,slew_limit=None,low_speed_fade=False,
            high_speed_fade=False,speed_cutoff=False,zero_velocity_output=0.)


def predict(row,p):
    fc,fs,vs,b,lc,lp,lvs,qp,n,phase=p[:10]
    v=row['speed'];q=np.asarray(row['position_rad']);load=np.asarray(row['nominal_load_samples_nm'])
    result=fc+(fs-fc+qp*np.sin(n*q+phase)**2)*np.exp(-(v/vs)**2)+b*v+(lc+lp*np.exp(-(v/lvs)**2))*load
    if len(p)==14 and p[10]>0:
        peak,cvs,joint,power=p[10:]
        context=np.asarray(row['joint_position_samples_rad'])[:,int(joint)-1]
        result+=peak*load*np.abs(np.sin(context))**power*np.exp(-(v/cvs)**2)
    return result


def observations(folder,posture=False,role=None,mock=False,scaling=None,probe=False,expected_joints=None):
    meta,data,evidence=read_acquisition(folder,posture,mock,loaded_role=role,position_probe=probe)
    rows,starts=pair_samples(meta,data)
    if role:
        joints=[1,2,3] if expected_joints is None else expected_joints
        if meta.get('loaded_study_joints')!=joints:raise ValueError('Unexpected loaded-study joint scope')
        expected={(j,pose,s) for j in joints for pose in ('follower_ready','working')
                  for s in meta['config']['speeds_rad_s']}
        actual=[(r['joint'],r['validation_pose'],r['speed']) for r in rows]
        if len(actual)!=len(expected) or set(actual)!=expected:
            raise ValueError('Incomplete loaded study')
    if probe:
        axis='q3' if meta.get('configuration_probe') else 'q2'
        expected={(1,p,s) for p in ('working_'+axis+'_lower','working_'+axis+'_upper') for s in meta['config']['speeds_rad_s'][:3]}
        actual=[(r['joint'],r['validation_pose'],r['speed']) for r in rows]
        if len(actual)!=6 or set(actual)!=expected:raise ValueError('Incomplete held-out position probe')
    rows=attach_nominal_load(rows,meta,Path(folder)/'robot.urdf',scaling)
    return meta,rows,evidence


def freeze(base_folder,training_folder,output,baseline,mock=False,development_folder=None,development_joints=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    scaling=original_gravity(baseline).get('torque_scaling_factors',[1.]*6)
    bm,br,be=observations(base_folder,mock=mock,scaling=scaling)
    lm,lr,le=observations(training_folder,role='training',mock=mock,scaling=scaling)
    c=bm['config']
    if c!=lm['config'] or fingerprint((Path(base_folder)/'robot.urdf').read_text())!=fingerprint((Path(training_folder)/'robot.urdf').read_text()):
        raise ValueError('Acquisition identity/configuration mismatch')
    if not c['sensor_timing_verified'] and (any(c['current_filter_time_constant_s']) or any(c['current_measurement_delay_s'])):
        raise ValueError('Unknown sensor timing requires explicit zero placeholders')
    train=[r for r in br if r['repeat']==0]+lr
    development_joints=[] if development_folder is None else (development_joints or [2])
    development_evidence=[]
    if development_folder:
        dm,dr,de=observations(development_folder,role='validation',mock=mock,scaling=scaling)
        if dm['config']!=c or fingerprint((Path(development_folder)/'robot.urdf').read_text())!=fingerprint((Path(base_folder)/'robot.urdf').read_text()):
            raise ValueError('Development acquisition identity mismatch')
        if any(j not in (1,2,3) for j in development_joints):raise ValueError('Invalid refit scope')
        train += [r for r in dr if r['joint'] in development_joints]
        development_evidence.append(de)
    parameters=[];selections=[]
    for j in range(6):
        rows=[r for r in train if r['joint']==j]
        if j in (1,2,3):
            selection=fit_candidates(rows,c['validation_absolute_error_nm'],c['validation_relative_error'])
            selected=selection['selected']
            if selected:
                fc,delta,vs,b,lc,lp,lvs=selected['parameters']
                p=[fc,fc+delta,vs,b,lc,lp,lvs,0.,1.,0.]
            else:
                # An explicit empirical extension, selected on training only.
                position=fit_position_candidates(rows,c['validation_absolute_error_nm'],c['validation_relative_error'])
                selection['position_extension']=position;selected=position['selected']
                if not selected:
                    (output/'training_failure.json').write_text(json.dumps(selection,indent=2))
                    raise ValueError(f'No adequate training model for {JOINTS[j]}')
                fc,b,lc,s0,sq,vs,phase,n=selected['parameters']
                p=[fc,fc+s0,vs,b,lc,0.,.01,sq,n,phase]
        else:
            p0,selection=select_curve([r['speed'] for r in rows],[r['friction_nm'] for r in rows],
                                      c['validation_absolute_error_nm'],c['validation_relative_error'])
            p=list(map(float,p0))+[0.,.01,0.,1.,0.]
        parameters.append(p);selections.append(selection)
    candidate=dict(schema=1,created_utc=datetime.now(timezone.utc).isoformat(),parameters=parameters,
        parameter_order=ORDER,selections=selections,training_rows=train,config=c,
        source='synthetic' if mock else 'hardware',joints=JOINTS,robot_unit=c['robot_unit'],
        robot_fingerprint=fingerprint((Path(base_folder)/'robot.urdf').read_text()),
        baseline_hash=sha(baseline),training_acquisitions=[be,le]+development_evidence,runtime_policy=POLICY,
        development_joints=development_joints,
        qualification=QUALIFICATION,validation_used_for_fit=False,
        implementation_sha256={n:sha(Path(__file__).with_name(n)) for n in
            ('loaded_campaign.py','loaded_model.py','curve_selection.py','fit.py')})
    (output/'frozen.json').write_text(json.dumps(candidate,indent=2))
    return candidate


def check_rows(rows,parameters,c,training):
    checks=[]
    for r in rows:
        pred=predict(r,parameters[r['joint']]);w=spatial_weights(r['position_rad'])
        mean=float(w@pred);measured=r['friction_nm'];tolerance=c['validation_absolute_error_nm']+c['validation_relative_error']*measured
        references=[t for t in training if (t['joint'],t['validation_pose'],t['speed'])==
                        (r['joint'],r['validation_pose'],r['speed'])]
        repeat_ok=True;repeat_error=None
        if references:
            reference=float(np.mean([t['friction_nm'] for t in references]))
            repeat_error=measured-reference
            repeat_ok=abs(repeat_error)<=c['validation_absolute_error_nm']+c['validation_relative_error']*min(measured,reference)
        pointwise=np.abs(pred-r['friction_samples_nm'])
        checks.append(dict(joint=JOINTS[r['joint']],pose=r['validation_pose'],repeat=r['repeat'],speed=r['speed'],
            predicted_mean_nm=mean,measured_mean_nm=measured,error_nm=mean-measured,tolerance_nm=tolerance,
            mean_passed=bool(abs(mean-measured)<=tolerance),repeat_passed=bool(repeat_ok),repeat_difference_nm=repeat_error,
            training_reference_cycles=len(references),training_reference='arithmetic mean of training cycle means',
            passed=bool(abs(mean-measured)<=tolerance and repeat_ok),
            pointwise_passed=bool(np.all(pointwise<=c['validation_absolute_error_nm']+c['validation_relative_error']*np.array(r['friction_samples_nm']))),
            max_pointwise_error_nm=float(max(pointwise)),samples_used_for_fit=False))
    return checks


def validate(frozen,posture_folder,validation_folder,output,baseline,mock=False,position_probe_folder=None,fresh_repeat_folder=None,configuration_probe_folder=None):
    frozen=Path(frozen);candidate=json.loads(frozen.read_text());c=candidate['config']
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    if candidate['baseline_hash']!=sha(baseline) or candidate['validation_used_for_fit']:
        raise ValueError('Changed baseline or contaminated candidate')
    for e in candidate['training_acquisitions']:
        for file,digest in e['hashes'].items():
            if sha(Path(e['folder'])/file)!=digest:raise ValueError('Training evidence changed')
    scaling=original_gravity(baseline).get('torque_scaling_factors',[1.]*6)
    bm,br,be=observations(candidate['training_acquisitions'][0]['folder'],mock=mock,scaling=scaling)
    pm,pr,pe=observations(posture_folder,posture=True,mock=mock,scaling=scaling)
    vm,vr,ve=observations(validation_folder,role='validation',mock=mock,scaling=scaling)
    required=any(p[7]>0 for p in candidate['parameters'])
    context_required=any(len(p)==14 and p[10]>0 for p in candidate['parameters'])
    if required and position_probe_folder is None:raise ValueError('Empirical position model requires fresh shifted-center validation')
    if context_required and configuration_probe_folder is None:raise ValueError('Configuration model requires a fresh context-angle probe')
    extras=[];extra_evidence=[];identity=[(pm,pe),(vm,ve)]
    development_joints=candidate.get('development_joints',[])
    if development_joints:
        if fresh_repeat_folder is None:raise ValueError('Refitted joints require a fresh independent acquisition')
        folders=fresh_repeat_folder if isinstance(fresh_repeat_folder,(list,tuple)) else [fresh_repeat_folder]
        fresh_rows=[];seen=[]
        for folder in folders:
            if Path(folder).resolve() in [Path(e['folder']).resolve() for e in candidate['training_acquisitions']]:
                raise ValueError('Fresh validation cannot reuse training data')
            scope=json.loads((Path(folder)/'plan.json').read_text())['loaded_study_joints']
            fm,fr,fe=observations(folder,role='validation',mock=mock,scaling=scaling,expected_joints=scope)
            if any(fe['hashes']['measurements.npz']==e['hashes']['measurements.npz'] for e in candidate['training_acquisitions']):
                raise ValueError('Fresh validation cannot reuse training measurements')
            seen+=scope;fresh_rows+=fr;identity.append((fm,fe));extra_evidence.append(fe)
        if sorted(seen)!=sorted(development_joints):raise ValueError('Fresh repeat joint coverage mismatch')
        vr=[r for r in vr if r['joint'] not in development_joints]+fresh_rows
    for folder,is_context in ((position_probe_folder,False),(configuration_probe_folder,True)):
        if folder:
            if Path(folder).resolve() in [Path(e['folder']).resolve() for e in candidate['training_acquisitions']]:
                raise ValueError('Validation probe cannot reuse training data')
            xm,xr,xe=observations(folder,mock=mock,scaling=scaling,probe=True)
            if any(xe['hashes']['measurements.npz']==e['hashes']['measurements.npz'] for e in candidate['training_acquisitions']):
                raise ValueError('Validation probe cannot reuse training measurements')
            if bool(xm.get('configuration_probe'))!=is_context:raise ValueError('Wrong probe kind')
            extras+=xr;identity.append((xm,xe));extra_evidence.append(xe)
    for meta,e in identity:
        if meta['config']!=c or fingerprint((Path(e['folder'])/'robot.urdf').read_text())!=candidate['robot_fingerprint']:
            raise ValueError('Validation identity/configuration mismatch')
    # The original posture failure informed J2-J4 model development; those old
    # rows are excluded. A fresh reversed-speed acquisition tests those axes.
    rows=[r for r in br if r['repeat']>0]+[r for r in pr if r['joint'] in (0,4,5)]+vr+extras
    checks=check_rows(rows,candidate['parameters'],c,candidate['training_rows'])
    report=dict(schema=1,qualification=QUALIFICATION,passed=all(x['passed'] for x in checks),
        acceptance_policy='independent_cycle_mean_and_repeat_v1',checks=checks,
        pointwise_model_passed=all(x['pointwise_passed'] for x in checks),
        parameters=candidate['parameters'],parameter_order=candidate['parameter_order'],runtime_policy=POLICY,
        frozen_sha256=sha(frozen),frozen_path=str(frozen.resolve()),
        acquisitions=candidate['training_acquisitions']+[pe,ve]+extra_evidence,
        position_probe_required=required,position_probe_passed=bool(position_probe_folder) and all(x['passed'] for x in checks[-len(extras):]),
        configuration_probe_required=context_required,configuration_probe_passed=bool(configuration_probe_folder) and all(x['passed'] for x in checks[-6:]),
        validation_probe_count=len(extras),
        refitted_joints=development_joints,fresh_repeat_after_refit=bool(development_joints),
        static_breakaway_identified=False,passivity_proven=False,hardware_freedrive_validated=False,
        unseen_pose_generalization_validated=False,temperature_model=False,
        rigid_parameters_modified=False,torque_conversion_modified=False,
        sensor_timing_verified=c['sensor_timing_verified'],sensor_timing_assumption='zero_delay' if not c['sensor_timing_verified'] else 'verified_configuration',
        effort_unit='joint_equivalent_Nm',validation_tolerances=dict(absolute_nm=c['validation_absolute_error_nm'],relative=c['validation_relative_error'],changed=False),
        scope='Symmetric steady means; independent repeat at measured pose regions, not arbitrary-pose or startup qualification')
    path=output/'identification.json';path.write_text(json.dumps(report,indent=2))
    (output/'config.yaml').write_text(yaml.safe_dump(c))
    if not report['passed']:return report
    profile=dict(schema=5,qualification=QUALIFICATION,source=candidate['source'],config=c,joints=JOINTS,
        robot_unit=c['robot_unit'],robot_fingerprint=candidate['robot_fingerprint'],baseline_hash=sha(baseline),
        report_sha256=sha(path),parameters=candidate['parameters'],runtime_policy=POLICY,passed=True,
        effort_unit='joint_equivalent_Nm',hardware_freedrive_validated=False,passivity_proven=False,
        static_breakaway_identified=False,sensor_timing_verified=c['sensor_timing_verified'])
    (output/'profile.json').write_text(json.dumps(profile,indent=2))
    return report


def export_unqualified(report_path,baseline):
    """Explicit research export; never relabel a failed report as passed."""
    report_path=Path(report_path);r=json.loads(report_path.read_text())
    reject_unsafe_trial(report_path.parent)
    if r.get('passed') is not False or r.get('qualification')!=QUALIFICATION:
        raise ValueError('Requires a completed, failed loaded-model validation report')
    fpath=Path(r['frozen_path']);f=json.loads(fpath.read_text())
    if sha(fpath)!=r['frozen_sha256'] or f['parameters']!=r['parameters'] or f['baseline_hash']!=sha(baseline):
        raise ValueError('Frozen candidate provenance mismatch')
    p=dict(schema=6,qualification=UNQUALIFIED,passed=False,source=f['source'],config=f['config'],
        joints=JOINTS,robot_unit=f['robot_unit'],robot_fingerprint=f['robot_fingerprint'],baseline_hash=sha(baseline),
        report_sha256=sha(report_path),parameters=f['parameters'],runtime_policy=POLICY,effort_unit='joint_equivalent_Nm',
        hardware_freedrive_validated=False,passivity_proven=False,static_breakaway_identified=False,
        sensor_timing_verified=f['config']['sensor_timing_verified'],failed_checks=[x for x in r['checks'] if not x['passed']])
    path=report_path.parent/'unqualified_profile.json'
    path.write_text(json.dumps(p,indent=2));return path


def reject_unsafe_trial(folder):
    marker=Path(folder)/'hardware_trial.json'
    if marker.exists() and json.loads(marker.read_text()).get('outcome')=='unsafe_motion':
        raise ValueError('Hardware trial rejected this candidate after uncontrolled motion; no automatic retry')


def controller_parameters(path,c,urdf,baseline,allow_experiment,mock=False,allow_unqualified=False):
    path=Path(path);p=json.loads(path.read_text());report_path=path.parent/'identification.json'
    reject_unsafe_trial(path.parent)
    if not allow_experiment:raise ValueError('Unscaled steady-mean model requires explicit experimental launch')
    unqualified=p.get('schema')==6
    if unqualified and not allow_unqualified:raise ValueError('Unqualified candidate requires explicit experimental authorization')
    if (p.get('schema') not in (5,6) or p.get('qualification')!=(UNQUALIFIED if unqualified else QUALIFICATION) or
        p.get('passed') is not (False if unqualified else True) or
        p.get('source') not in (('hardware','synthetic') if mock else ('hardware',)) or p.get('config')!=c or
        p.get('joints')!=JOINTS or p.get('robot_unit')!=c['robot_unit'] or p.get('robot_fingerprint')!=fingerprint(urdf) or
        p.get('baseline_hash')!=sha(baseline) or p.get('report_sha256')!=sha(report_path) or p.get('runtime_policy')!=POLICY):
        raise ValueError('Loaded profile provenance mismatch')
    r=json.loads(report_path.read_text())
    if ((not unqualified and (not r.get('passed') or not all(x['passed'] for x in r['checks']))) or
        (unqualified and (r.get('passed') is not False or p.get('failed_checks')!=[x for x in r['checks'] if not x['passed']])) or
        r['parameters']!=p['parameters'] or
        r['qualification']!=QUALIFICATION or r['runtime_policy']!=POLICY or sha(r['frozen_path'])!=r['frozen_sha256']):
        raise ValueError('Missing independent validation')
    frozen=json.loads(Path(r['frozen_path']).read_text())
    if (frozen['parameters']!=p['parameters'] or frozen['config']!=c or
        len(r['checks'])!=114+r.get('validation_probe_count',6 if r.get('position_probe_passed') else 0) or
        (not unqualified and any(v[7]>0 for v in p['parameters']) and not r.get('position_probe_passed')) or
        (not unqualified and any(len(v)==14 and v[10]>0 for v in p['parameters']) and not r.get('configuration_probe_passed')) or
        p.get('effort_unit')!='joint_equivalent_Nm' or
        p.get('sensor_timing_verified')!=c['sensor_timing_verified'] or
        any(p.get(k) is not False or r.get(k) is not False for k in
            ('static_breakaway_identified','passivity_proven','hardware_freedrive_validated'))):
        raise ValueError('Profile differs from its frozen model or qualification')
    for e in r['acquisitions']:
        for file,digest in e['hashes'].items():
            if sha(Path(e['folder'])/file)!=digest:raise ValueError('Acquisition evidence changed')
    a=np.asarray(p['parameters'],dtype=float)
    if (a.shape not in ((6,10),(6,14)) or not np.isfinite(a).all() or np.any(a[:,:9]<0) or np.any(a[:,1]<a[:,0]) or
        np.any(a[:,2]<=0) or np.any(a[:,6]<=0) or np.any(a[:,8]<1)):
        raise ValueError('Invalid loaded model coefficients')
    if a.shape[1]==14 and (np.any(a[:,10:]<0) or np.any(a[:,11]<=0) or
            np.any(a[:,12]>6) or np.any(a[:,12]!=np.floor(a[:,12])) or
            np.any((a[:,10]>0)&(a[:,12]<1)) or np.any(~np.isin(a[:,13],[2,4,6]))):
        raise ValueError('Invalid configuration model coefficients')
    result=original_gravity(baseline)
    result.update(calibrated_friction_enabled=True,calibrated_unscaled=True,calibrated_gain=1.,
        calibrated_max_torque=[0.]*6,calibrated_slew_rate=[0.]*6,calibrated_deadband=0.,
        calibrated_min_velocity=min(c['speeds_rad_s']),calibrated_max_velocity=max(c['speeds_rad_s']),
        calibrated_enforce_load_bounds=False,calibrated_load_min=[0.]*6,calibrated_load_max=[1.]*6)
    names=['fc','fs','vs','viscous','load_gain','load_peak','load_vs','position_peak','position_order','position_phase']
    if a.shape[1]==14:names+=CONTEXT_ORDER
    result.update({'calibrated_'+n:a[:,i].tolist() for i,n in enumerate(names)})
    return result


def main():
    from .integration import baseline_path
    from rclpy.utilities import remove_ros_args
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['freeze','validate','export-unqualified'])
    p.add_argument('--base-run');p.add_argument('--training-run');p.add_argument('--frozen')
    p.add_argument('--posture-run');p.add_argument('--validation-run');p.add_argument('--output',required=True)
    p.add_argument('--position-probe-run')
    p.add_argument('--development-run');p.add_argument('--development-joints',type=int,nargs='+',default=[3])
    p.add_argument('--fresh-repeat-run',action='append')
    p.add_argument('--configuration-probe-run')
    a=p.parse_args(remove_ros_args()[1:])
    if a.stage=='export-unqualified':
        print(export_unqualified(Path(a.output)/'identification.json',baseline_path()));return
    if a.stage=='freeze':freeze(a.base_run,a.training_run,a.output,baseline_path(),
        development_folder=a.development_run,development_joints=[j-1 for j in a.development_joints])
    else:
        r=validate(a.frozen,a.posture_run,a.validation_run,a.output,baseline_path(),position_probe_folder=a.position_probe_run,
            fresh_repeat_folder=a.fresh_repeat_run,configuration_probe_folder=a.configuration_probe_run)
        print(json.dumps({k:r[k] for k in ('passed','pointwise_model_passed','static_breakaway_identified','hardware_freedrive_validated')},indent=2))
        if not r['passed']:raise SystemExit('Independent validation failed; report saved, no profile exported')
