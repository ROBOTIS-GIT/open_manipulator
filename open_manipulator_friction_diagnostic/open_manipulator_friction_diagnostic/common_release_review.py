"""Known motor-input OFF/ON comparison; never an output-force measurement."""
import argparse
import json
from pathlib import Path
import numpy as np


def reverse_travel_limit(row):
    # A brief elastic return inside the already accepted dither envelope is
    # distinct from sustained reverse motion. Final settling is still required.
    return float(np.deg2rad(.12)) if row.get('soft_start_dither') is True else .001


def require_known_input_release(execution):
    if (execution.get('purpose')=='common_zero_input_isolation' or
            execution.get('common_parameters',{}).get('common_zero_input_seconds')):
        raise ValueError('Zero-input isolation cannot qualify a known-input release')


def modulated_command_review(tail,frequency,amplitude):
    """Check the sinusoid before its recorded elastic/velocity envelope.

    Controller phase advances by this row's dt before producing the output.
    A free phase offset accounts for the beginning of the selected tail; no DC,
    drift, or additional harmonics are fitted. Full-state replay remains a
    separate handoff requirement in coast_handoff.check_components.
    """
    fields=('time_s','dt_s','dither_envelope','dither_nm')
    if (not all(n in tail.dtype.names and np.isfinite(tail[n]).all() for n in fields)
            or not np.isfinite([frequency,amplitude]).all() or min(frequency,amplitude)<=0):
        return dict(passed=False)
    envelope=tail['dither_envelope']
    if (np.any(envelope<0) or np.any(envelope>1+1e-9)
            or np.any(tail['dt_s']<=0) or np.any(tail['dt_s']>.02)):
        return dict(passed=False)
    phase=2*np.pi*frequency*(tail['time_s']+tail['dt_s'])
    x=envelope[:,None]*np.column_stack([np.sin(phase),np.cos(phase)])
    coefficients,_,rank,_=np.linalg.lstsq(x,tail['dither_nm'],rcond=None)
    fitted_amplitude=float(np.linalg.norm(coefficients))
    maximum_error=float(np.max(abs(tail['dither_nm']-x@coefficients)))
    return dict(passed=bool(rank==2 and abs(fitted_amplitude-amplitude)<=1e-6
                           and maximum_error<=1e-6),
                amplitude_nm=fitted_amplitude,maximum_waveform_error_nm=maximum_error)


def rest_dither_review(release,rest,joint=2):
    from .automatic_calibration import harmonic,POLICY
    from .rest_response import ENCODER_STEP_RAD
    if rest is None or 'dither_nm' not in release.dtype.names:return dict(passed=False)
    f=rest['dither_frequency'];amplitude=rest['dither_amplitude']
    tail=release[release['time_s']>=release['time_s'][-1]-2.1]
    t=tail['time_s']-tail['time_s'][0]
    if len(t)<100 or t[-1]<10/f:return dict(passed=False)
    position=harmonic(t,tail[f'j{joint}_q'],f);command=harmonic(t,tail['dither_nm'],f)
    pp=2*position['amplitude']/ENCODER_STEP_RAD
    drift=abs(position['drift_per_s'])*t[-1]/ENCODER_STEP_RAD
    lo,hi=POLICY['dither_peak_to_peak_counts']
    profile=rest.get('rest_response_profile')
    if profile not in (None,'soft_start_v1'):raise ValueError('Unknown rest response profile')
    extent=.0002;residual_limit=2.
    if profile=='soft_start_v1':
        hi=np.deg2rad(.1)/ENCODER_STEP_RAD
        extent=np.deg2rad(.12)  # allow the explicitly accepted small harmonics
        residual_limit=max(2.,.1*pp)
    modulated=None
    command_matches=abs(command['amplitude']-amplitude)<=max(.02,.05*amplitude)
    if 'dither_envelope' in tail.dtype.names or 'dt_s' in tail.dtype.names:
        modulated=modulated_command_review(tail,f,amplitude)
        command_matches=modulated['passed']
    passed=(np.ptp(tail[f'j{joint}_q'])<=extent and lo<=pp<=hi and
        drift<=POLICY['dither_max_drift_counts'] and position['residual_rms']/ENCODER_STEP_RAD<=residual_limit and
        command_matches)
    return dict(passed=bool(passed),coherent_peak_to_peak_counts=float(pp),drift_counts=float(drift),
                position_fit=position,command_fit=command,modulated_command=modulated,
                policy=profile or 'existing measured rest-response policy')


def small_envelope_dither_review(release,rest,joint=2):
    """Retain the ordinary rest envelope while separating commanded vibration.

    The ordinary classifier permits 0.0002 rad of total position extent, but
    its instantaneous velocity bound also includes the deliberate sine motion.
    Only for a verified soft-start waveform, average signed velocity over two
    periods before applying the same 0.002 rad/s bound. Every overlapping window
    must pass. This original small-envelope result stays unchanged. A separate
    bounded-cycle result permits the accepted 0.12-degree vibration envelope
    with center movement below 0.001 rad. Neither trace is detrended.
    """
    if not rest or rest.get('rest_response_profile')!='soft_start_v1':
        return dict(passed=False)
    qname=f'j{joint}_q';vname=f'j{joint}_position_velocity'
    if not all(n in release.dtype.names for n in ('time_s',qname,vname)) or len(release)<100:
        return dict(passed=False)
    tail=release[release['time_s']>=release['time_s'][-1]-2.1]
    if len(tail)<100 or not all(np.isfinite(tail[n]).all() for n in ('time_s',qname,vname)):
        return dict(passed=False)
    t=tail['time_s']-tail['time_s'][0];frequency=rest['dither_frequency']
    if not np.isfinite(frequency) or not 5<=frequency<=20 or t[-1]<2. or np.any(np.diff(t)<=0):
        return dict(passed=False)
    command=modulated_command_review(tail,frequency,rest['dither_amplitude'])
    width=2./frequency
    # Interpolate integral endpoints so scheduler jitter cannot leave a partial
    # sine period in the mean. Test every recorded window start, not selected bins.
    starts=t[t<=t[-1]-width]
    def averages(values):
        integral=np.r_[0.,np.cumsum(.5*(values[1:]+values[:-1])*np.diff(t))]
        return (np.interp(starts+width,t,integral)-np.interp(starts,t,integral))/width
    means=averages(tail[vname]);centers=averages(tail[qname])
    extent=float(np.ptp(tail[qname]));speed=float(np.max(abs(means)))
    # Correct for the endpoints lost to averaging: a constant drift is judged
    # over the full observed duration, rather than just the shorter center trace.
    center_extent=float(np.ptp(centers)*t[-1]/(t[-1]-width))
    bounded_cycles=bool(command['passed'] and extent<=np.deg2rad(.12)
                        and center_extent<=.001 and speed<=.002)
    return dict(passed=bool(command['passed'] and extent<=.0002 and speed<=.002),
                position_peak_to_peak_rad=extent,maximum_cycle_mean_speed_rad_s=speed,
                cycle_center_extent_rad=center_extent,bounded_cycle_centers=bounded_cycles,
                cycle_center_limit_rad=.001,raw_dither_extent_limit_rad=float(np.deg2rad(.12)),
                averaging_periods=2,modulated_command=command,
                policy='original_rest_extent_with_verified_periodic_velocity')


def review_rows(a,mask,directions=None,rest=None,joint=2):
    if joint not in range(1,7):raise ValueError('Unsupported release-review joint')
    directions=([1,-1]*4)[:len(mask)] if directions is None else directions
    if len(directions)!=len(mask) or any(d not in (-1,1) for d in directions):
        raise ValueError('Direction/assistance mismatch')
    trials=[]
    for k,enabled in enumerate(mask):
        ramp=a[a['phase']==10*k+1];release=a[a['phase']==10*k+2]
        if len(ramp)<10 or len(release)<10:continue
        tail=release[release['time_s']>=release['time_s'][-1]-1.9]
        settled=(np.ptp(tail[f'j{joint}_q'])<=.0002 and np.max(abs(tail[f'j{joint}_position_velocity']))<=.002)
        dither=rest_dither_review(release,rest if enabled else None,joint)
        small_dither=small_envelope_dither_review(release,rest if enabled else None,joint)
        reverse=max(0.,float(-np.min(directions[k]*(release[f'j{joint}_q']-release[f'j{joint}_q'][0]))))
        elapsed=release['time_s']-release['time_s'][0]
        finite=all(np.isfinite(release[n]).all() for n in
                   ('time_s','test_input_nm','assistance_nm',f'j{joint}_q',f'j{joint}_position_velocity'))
        gaps=np.diff(release['time_s'])
        temporal_coverage=finite and bool(np.all(gaps>0) and np.max(gaps)<=.03)
        input_removed=finite and bool(np.all(abs(release['test_input_nm'])<=1e-9))
        initial_speed=float(np.max(abs(release[f'j{joint}_position_velocity'][elapsed<=.15])))
        # Window medians reject isolated differentiation spikes. Compare every
        # later window against the preceding minimum, not only the initial peak.
        soft=bool(enabled and rest and rest.get('rest_response_profile')=='soft_start_v1')
        width=max(.1,2./rest['dither_frequency']) if soft else .1
        # For explicit larger dither, average signed velocity over at least
        # two commanded periods before taking its magnitude. Rectifying first
        # would mistake a zero-mean rest vibration for unidirectional coasting.
        statistic=(lambda v: abs(float(np.mean(v)))) if soft else (lambda v: float(np.median(abs(v))))
        windows=[statistic(release[f'j{joint}_position_velocity'][(elapsed>=t)&(elapsed<t+width)])
                 for t in np.arange(.25,max(.25,elapsed[-1]-width),.1)
                 if np.count_nonzero((elapsed>=t)&(elapsed<t+width))>=3]
        sequence=np.r_[initial_speed,windows]
        speed_gain=float(max(0.,np.max(sequence-np.minimum.accumulate(sequence))))
        allowance=max(.01,.05*initial_speed)
        trials.append(dict(trial=k,direction=directions[k],enabled=bool(enabled),
            soft_start_dither=soft,
            start_position_rad=float(ramp[f'j{joint}_q'][0]),
            motor_input_at_release_nm=float(np.max(abs(ramp['test_input_nm']))),
            release_duration_s=float(np.ptp(release['time_s'])),
            release_peak_velocity_rad_s=float(np.max(abs(release[f'j{joint}_position_velocity']))),
            release_travel_rad=float(release[f'j{joint}_q'][-1]-release[f'j{joint}_q'][0]),
            reverse_travel_rad=reverse,
            post_release_speed_gain_rad_s=speed_gain,
            initial_release_speed_rad_s=initial_speed,
            release_position_interval_rad=[float(np.min(release[f'j{joint}_q'])),float(np.max(release[f'j{joint}_q']))],
            speed_growth_allowance_rad_s=allowance,external_test_input_removed=bool(input_removed),
            temporal_coverage_sufficient=temporal_coverage,
            no_speed_growth=bool(temporal_coverage and windows and speed_gain<=allowance),
            stopped=bool(settled),bounded_rest_dither=dither['passed'],rest_response=dither,
            small_envelope_dither=small_dither,
            bounded_cycle_dither=small_dither.get('bounded_cycle_centers',False),
            rest_settled=bool(settled or dither['passed'] or small_dither['passed']
                              or small_dither.get('bounded_cycle_centers',False)),
            assistance_peak_nm=float(np.max(abs(release['assistance_nm'])))))
    return summarize(trials,joint)


def summarize(trials,joint=2):
    comparisons=[]
    for d in (1,-1):
        off=[r['motor_input_at_release_nm'] for r in trials if r['direction']==d and not r['enabled']]
        on=[r['motor_input_at_release_nm'] for r in trials if r['direction']==d and r['enabled']]
        if len(off)!=2 or len(on)!=2:continue
        before,after=float(np.median(off)),float(np.median(on))
        starts=[r['start_position_rad'] for r in trials if r['direction']==d]
        comparisons.append(dict(direction=d,off_motor_input_nm=before,on_motor_input_nm=after,
            start_position_spread_rad=float(np.ptp(starts)),matched_starts=bool(np.ptp(starts)<=.003),
            motor_input_reduction_fraction=float((before-after)/before) if before>0 else None,
            improved=bool(before-after>.1)))
    passed=(len(trials)==8 and len(comparisons)==2 and all(r.get('external_test_input_removed',False) and r['rest_settled'] and r['no_speed_growth'] and r['release_duration_s']>=7.9 and r['reverse_travel_rad']<=reverse_travel_limit(r) for r in trials)
            and all(r['improved'] and r['matched_starts'] for r in comparisons))
    return dict(local_comparison_passed=passed,trials=trials,comparisons=comparisons,
        independent_output_torque_measured=False,all_postures_qualified=False,operating_domain_qualified=False,
        kinetic_release_coverage=[dict(direction=r['direction'],enabled=r['enabled'],initial_speed_rad_s=r.get('initial_release_speed_rad_s'),position_interval_rad=r.get('release_position_interval_rad')) for r in trials],
        automatic_deployment_allowed=False,
        joint=joint, scope=f'J{joint} finite local test at recorded held pose; known injected motor input and eight-second release, not hand force')


def review_segments(segments,rest=None,joint=2):
    """Keep each acquisition's own time base; no synthetic concatenated trace."""
    if len(segments)!=8:raise ValueError('Eight complete separate trials required')
    trials=[]
    for k,a in enumerate(segments):
        result=review_rows(a,[[0,0,1,1,0,0,1,1][k]],[[1,-1][k%2]],rest,joint)
        if len(result['trials'])!=1:raise ValueError('Missing ramp/release in separate trial')
        row=result['trials'][0];row['trial']=k;trials.append(row)
    return dict(summarize(trials,joint),acquisition='eight_separate_same_start_trials')


def review_approach(folder,direction,target=None,joint=2):
    """Verify a recorded final 0.01-rad approach, not just its requested label."""
    from .automatic_calibration import file_hash
    folder=Path(folder);ev=json.loads((folder/'execution.json').read_text())
    target=np.zeros(6) if target is None else np.asarray(target,float)
    recorded_joint=ev.get('approach_joint',2)
    recorded_direction=ev.get('approach_direction',ev.get('j2_approach_direction'))
    if (joint not in range(1,7) or direction not in (-1,1) or not ev.get('completed') or
            recorded_joint!=joint or recorded_direction!=direction or target.shape!=(6,) or
            not np.allclose(ev.get('target_rad',[]),target,rtol=0,atol=1e-5)):
        raise ValueError('Preparation history mismatch')
    with np.load(folder/'trajectory.npz') as plan: end=float(plan['t'][-1])
    with np.load(folder/'measurements.npz') as a:
        t,q=a['t'],a['q'][:,joint-1]-target[joint-1]
        if np.max(abs(a['q'][-1]-target))>.001:raise ValueError('Measured target pose mismatch')
    if not np.isfinite([t,q]).all() or np.any(np.diff(t)<=0):
        raise ValueError('Invalid measured preparation trace')
    selected=q[(t>=end-5.) & (abs(q)>=.001) & (abs(q)<=.009)]
    progress=direction*float(selected[-1]-selected[0]) if len(selected)>20 else 0.
    if progress<.005 or abs(q[-1])>.001:
        raise ValueError('Measured approach does not confirm the requested direction')
    return dict(passed=True,joint=joint,direction=direction,measured_progress_rad=progress,
                files_sha256={n:file_hash(folder/n) for n in ('execution.json','trajectory.npz','measurements.npz')})


def review_acquisitions(folders,approach=None):
    from .automatic_calibration import file_hash
    if len(folders)!=8:raise ValueError('Eight acquisitions required')
    evidence=[];segments=[];bindings=[]
    joint=json.loads((Path(folders[0])/'execution.json').read_text())['target_joint']
    if joint not in range(1,7):raise ValueError('Unsupported release-review joint')
    for k,folder in enumerate(map(Path,folders)):
        ev=json.loads((folder/'execution.json').read_text())
        require_known_input_release(ev)
        if (ev.get('common_trial_index')!=k or ev.get('kind')!='common_ab' or ev.get('target_joint')!=joint
                or ev.get('directions')!=[[1,-1][k%2]]
                or ev.get('assistance_mask')!=[[0,0,1,1,0,0,1,1][k]]):
            raise ValueError('Acquisition order/direction/mask mismatch')
        params=dict(ev['common_parameters']);params.pop('assistance_mask')
        if params.get('common_dither_only'):
            raise ValueError('Dither isolation cannot qualify the common kinetic model')
        if params.get('common_hand_seconds'):
            raise ValueError('Manual observation is not a known-input release experiment')
        binding=(ev['model_sha256'],ev['implementation_sha256'],params)
        if bindings and binding!=bindings[0]:raise ValueError('Changed model or controller between trials')
        if file_hash(folder/'candidate.json')!=ev['model_sha256']:raise ValueError('Changed model evidence')
        bindings.append(binding);evidence.append(ev)
        segments.append(np.genfromtxt(folder/'samples.csv',delimiter=',',names=True))
    params=evidence[0]['common_parameters']
    result=review_segments(segments,params if params.get('common_rest_dither') else None,joint)
    if approach not in (None,'same','opposite'):raise ValueError('Unknown preparation history')
    histories=[]
    if approach is not None:
        for k,folder in enumerate(map(Path,folders)):
            direction=(1 if k%2==0 else -1)*(1 if approach=='same' else -1)
            preparation=folder.parent/f'zero_{k}'
            target=json.loads((preparation/'execution.json').read_text())['target_rad']
            histories.append(review_approach(preparation,direction,target,joint))
    result.update(preparation_history=approach,preparation_history_verified=bool(histories),preparations=histories)
    physical=all(not e['mock'] and e.get('passed') and e.get('finished') and e.get('register_restored') for e in evidence)
    if physical:
        for folder,a in zip(map(Path,folders),segments):
            restore=json.loads((folder/'register_restore.json').read_text())
            physical=physical and restore.get('passed') and restore.get('torque_enable_verified_off')
            physical=physical and all(np.all(a[f'j{j}_drive_error']==0) for j in range(1,7))
    result.update(physical_local_comparison_passed=bool(physical and result['local_comparison_passed']),
                  experiment_passed=bool(physical),model_sha256=evidence[0]['model_sha256'],
                  implementation_sha256=evidence[0]['implementation_sha256'],
                  acquisitions=[dict(path=str(p),samples_sha256=file_hash(Path(p)/'samples.csv'),
                                     execution_sha256=file_hash(Path(p)/'execution.json')) for p in folders])
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    p.add_argument('--separate',type=Path,nargs=8,help='Eight original per-trial folders; folder is the report destination')
    p.add_argument('--approach',choices=('same','opposite'),help='Verify actual position preparation history as well')
    a=p.parse_args()
    if a.separate:
        result=review_acquisitions(a.separate,a.approach)
        a.folder.mkdir(parents=True,exist_ok=True)
        (a.folder/'comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));return
    ev=json.loads((a.folder/'execution.json').read_text())
    require_known_input_release(ev)
    params=ev.get('common_parameters',{})
    result=review_rows(np.genfromtxt(a.folder/'samples.csv',delimiter=',',names=True),ev['assistance_mask'],
                      ev.get('directions'),params if params.get('common_rest_dither') else None,ev['target_joint'])
    result.update(experiment_passed=bool(ev.get('passed') and ev.get('finished') and ev.get('register_restored')),
        mock=ev['mock'],model_sha256=ev['model_sha256'],implementation_sha256=ev['implementation_sha256'])
    result['physical_local_comparison_passed']=bool(not ev['mock'] and result['experiment_passed'] and result['local_comparison_passed'])
    (a.folder/'comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':main()
