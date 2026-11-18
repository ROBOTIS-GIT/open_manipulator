"""Physical-probe handoff for supervised evaluation, not automatic qualification.

Extends the existing local-probe hand-trial path to repeated, bidirectional
evidence and independently measured rest waveforms. Spatial prediction failures
remain visible in the frozen candidate and still block automatic calibration.
"""
import json
from pathlib import Path
import numpy as np
from .automatic_calibration import file_hash,content_hash
from .common_release_review import review_rows, reverse_travel_limit
from .common_probe_hand import parameters_from_comparison
from .integration import fingerprint,original_gravity
from .rest_response import load_archive


def check_series(rows,model_hash):
    if (len(rows)!=4 or len({r['acquisition'] for r in rows})!=len(rows)
            or [r['direction'] for r in rows]!=[1,-1,1,-1]):
        raise ValueError('Two independent physical releases per direction required')
    for r in rows:
        if (r['model_sha256']!=model_hash or r['direction'] not in (-1,1)
                or not all(r[k] for k in ('enabled','external_test_input_removed',
                    'temporal_coverage_sufficient','no_speed_growth','rest_settled'))
                or not np.isfinite([r['initial_release_speed_rad_s'],r['release_duration_s'],r['reverse_travel_rad']]).all()
                or r['initial_release_speed_rad_s']<.02 or r['release_duration_s']<7.9
                or r['reverse_travel_rad']>reverse_travel_limit(r)):
            raise ValueError('Physical release evidence failed; handoff is unavailable')


def check_components(a,c,model):
    """Bind the recorded waveform components to the intended hand settings."""
    names=('time_s','assistance_nm','elastic_nm','rate_term_nm','damping_nm',
           'dither_nm','dither_envelope')
    if not all(n in a.dtype.names and np.isfinite(a[n]).all() for n in names):
        raise ValueError('Incomplete or nonfinite compensation components')
    if c.get('additional_damping')!=0. or np.any(a['damping_nm']!=0.):
        raise ValueError('Physical trial contains unrequested damping')
    if np.max(abs(a['assistance_nm']-a['elastic_nm']-a['rate_term_nm']-a['dither_nm']))>1e-8:
        raise ValueError('Recorded compensation components do not sum to output')
    if not c.get('common_rest_dither'):
        if np.any(a['dither_nm']!=0.) or np.any(a['dither_envelope']!=0.):
            raise ValueError('Physical trial contains unrequested dither')
        return
    amplitude=c['dither_amplitude'];frequency=c['dither_frequency']
    if not np.isfinite([amplitude,frequency]).all() or min(amplitude,frequency)<=0:
        raise ValueError('Invalid measured dither settings')
    if (np.min(a['dither_envelope'])<0 or np.max(a['dither_envelope'])>1+1e-9 or
            np.any(abs(a['dither_nm'])>amplitude*a['dither_envelope']+1e-8)):
        raise ValueError('Recorded dither exceeds its measured amplitude/envelope')
    # Replay every update of the configured waveform and elastic crossfade.
    # Final completed-sequence row skips the controller step and writes zero.
    from .periodic_friction import evaluate
    from .single_joint_model import table_values
    joint=model['joint'];vname=f'j{joint}_position_velocity';qname=f'j{joint}_q'
    for n in ('phase','dt_s','gravity_nm',qname):
        if n not in a.dtype.names or not np.isfinite(a[n]).all():
            raise ValueError('Missing full-rate waveform replay data')
    if vname not in a.dtype.names:raise ValueError('Missing waveform velocity')
    if (np.any(a['dt_s']<=0) or np.any(a['dt_s']>.02) or
            np.max(abs(np.diff(a['time_s'])-a['dt_s'][:-1]))>1e-7):
        raise ValueError('Waveform replay requires continuous controller updates')
    full,zero,ramp=c['dither_full_velocity'],c['dither_zero_velocity'],c['dither_ramp']
    fraction=c['common_output_fraction']
    if not np.isfinite([full,zero,ramp,fraction]).all() or not 0<=full<zero or ramp<=0 or not 0<fraction<=1:
        raise ValueError('Invalid waveform velocity/ramp/fraction settings')
    yields={d:(evaluate(model,a[qname],0.,a['gravity_nm'],d) if model['schema']=='common_periodic_friction_v1'
               else table_values(model,0.,a['gravity_nm'],d)) for d in ('positive','negative')}
    if any(not np.isfinite(loss).all() or np.any(loss<=0) for loss in yields.values()):
        raise ValueError('Nonpositive or nonfinite yield cannot enter physical handoff')
    active=np.flatnonzero(a['phase']>=0)
    if not len(active) or a['phase'][active[-1]]!=2:
        raise ValueError('Missing completed release waveform')
    last_active=active[-1]
    phase=0.;envelope=0.;expected_torque=np.zeros(len(a));expected_envelope=np.zeros(len(a))
    for i,row in enumerate(a):
        if row['phase']<0 or not np.isfinite(row[vname]) or i==last_active:continue
        if row['phase'] not in (0,1,2):raise ValueError('Unexpected release phase')
        x=np.clip((abs(row[vname])-full)/(zero-full),0.,1.)
        envelope=min(1-x*x*(3-2*x),envelope+row['dt_s']/ramp)
        phase=(phase+2*np.pi*frequency*row['dt_s'])%(2*np.pi)
        elastic=row['elastic_nm']/fraction
        loss=yields['positive' if elastic>=0 else 'negative'][i]
        rest_fraction=np.clip(1-abs(elastic)/loss,0.,1.)
        expected_envelope[i]=envelope*rest_fraction
        expected_torque[i]=amplitude*expected_envelope[i]*np.sin(phase)
    if (not np.isfinite([expected_envelope,expected_torque]).all() or
            np.max(abs(expected_envelope-a['dither_envelope']))>1e-6 or
            np.max(abs(expected_torque-a['dither_nm']))>1e-6):
        raise ValueError('Full recorded dither waveform/envelope differs from hand settings')


def require_configuration_means(model,manifest,fraction):
    """Accept development CV or a prospective check of the final frozen model.

    The latter must cover three unused configurations, both ordered repeats and
    the frozen speed endpoints. Its raw measurements are re-extracted; the old
    CV failure remains in the candidate. Neither route replaces four physical
    releases below or grants automatic/spatial qualification.
    """
    if model.get('posture_review',{}).get('means_passed'):
        return
    binding=manifest.get('fresh_configuration_confirmation')
    if not isinstance(binding,dict) or set(binding)!={'path','sha256'}:
        raise ValueError('Whole-configuration means or prospective confirmation required')
    if file_hash(binding['path'])!=binding['sha256']:
        raise ValueError('Changed prospective configuration confirmation')
    from .confirmation_archive import load_confirmation
    result=load_confirmation(binding['path'],model,fraction)
    if not result['confirmed_means']:
        raise ValueError('Prospective configuration confirmation failed')



def load_parameters(folder,urdf,baseline,fraction):
    folder=Path(folder);model=json.loads((folder/'candidate.json').read_text())
    manifest=json.loads((folder/'coast_series.json').read_text())
    identity=model['identity'];model_hash=file_hash(folder/'candidate.json')
    if (manifest.get('schema')!='supervised_coast_series_v1'
            or manifest.get('automatic_deployment_allowed') is not False
            or manifest.get('model_sha256')!=model_hash
            or identity['robot_fingerprint']!=fingerprint(urdf)
            or identity['baseline_sha256']!=file_hash(baseline)
            or not model.get('training_review',{}).get('means_passed')):
        raise ValueError('Changed identity/model or unsuitable development means')
    if model.get('directional_sources'):
        from .directional_candidate import combine
        from .kinetic_runtime import model_parameters
        sources={d:json.loads(Path(p).read_text()) for d,p in manifest['source_models'].items()}
        for d,m in sources.items():
            if (content_hash(m)!=model['directional_sources'][d]['candidate_sha256']
                    or not m['posture_review']['means_passed']):
                raise ValueError('Directional development evidence changed or failed')
        reconstructed=combine(sources['positive'],sources['negative'])
        if model_parameters(reconstructed,model['joint'])!=model_parameters(model,model['joint']):
            raise ValueError('Directional runtime is not the frozen composition')
    else:
        require_configuration_means(model,manifest,fraction)
    rows=[];parameters=[];root=Path(__file__).resolve().parents[2]
    sequence_path=Path(manifest['sequence_path'])
    if file_hash(sequence_path)!=manifest['sequence_sha256']:
        raise ValueError('Physical sequence record changed')
    sequence=json.loads(sequence_path.read_text())
    expected_paths=[str(sequence_path.parent/f'trial_{k}'/'hardware') for k in range(4)]
    if (sequence.get('schema')!='frozen_four_release_sequence_v1'
            or sequence.get('model_sha256')!=model_hash or sequence.get('completed') is not True
            or sequence.get('failure') or sequence.get('planned_directions')!=[1,-1,1,-1]
            or sequence.get('completed_trial_indices')!=[0,1,2,3]
            or [str(Path(e['path'])) for e in manifest['trials']]!=expected_paths):
        raise ValueError('A complete prespecified four-trial sequence is required; no successful subsets')
    for entry in manifest['trials']:
        path=Path(entry['path'])
        for name,digest in entry['files_sha256'].items():
            if file_hash(path/name)!=digest:raise ValueError('Changed physical evidence: '+name)
        if not {'execution.json','samples.csv','register_restore.json','candidate.json'}<=entry['files_sha256'].keys():
            raise ValueError('Incomplete physical evidence binding')
        e=json.loads((path/'execution.json').read_text());restore=json.loads((path/'register_restore.json').read_text())
        from .common_release_review import require_known_input_release
        require_known_input_release(e)
        if (e.get('mock') is not False or e.get('kind')!='common_ab' or e.get('target_joint')!=model['joint']
                or not all(e.get(k) for k in ('passed','finished','register_restored'))
                or e.get('failure') or e.get('probe',{}).get('fault')!=0
                or e.get('model_sha256')!=model_hash or file_hash(path/'candidate.json')!=model_hash
                or not restore.get('passed') or not restore.get('torque_enable_verified_off')
                or any(restore.get('error_codes',{}).get(str(j))!=0 for j in range(1,7))):
            raise ValueError('Incomplete, failed or unrestored physical trial')
        from .implementation_compatibility import verify
        verify(e['implementation_sha256'], model, root)
        c=e['common_parameters']
        if c.get('common_hand_seconds') or c.get('common_dither_only') or e['assistance_mask']!=[1]:
            raise ValueError('Hand/dither-only observation is not a known-input coast')
        a=np.genfromtxt(path/'samples.csv',delimiter=',',names=True)
        check_components(a,c,model)
        if any(np.any(a[f'j{j}_drive_error']!=0) for j in range(1,7)):
            raise ValueError('Device error in physical trace')
        result=review_rows(a,e['assistance_mask'],e['directions'],c if c.get('common_rest_dither') else None,model['joint'])
        if len(result['trials'])!=1:raise ValueError('Exactly one original release per acquisition required')
        rows.append(dict(result['trials'][0],acquisition=file_hash(path/'samples.csv'),model_sha256=e['model_sha256']))
        rest=load_archive(path/'rest_evidence',mock=False) if c.get('common_rest_dither') else None
        parameters.append(parameters_from_comparison(model,c,original_gravity(baseline),rest,fraction))
    check_series(rows,model_hash)
    if any(p!=parameters[0] for p in parameters[1:]):
        raise ValueError('Different runtime settings across physical trials')
    return parameters[0]
