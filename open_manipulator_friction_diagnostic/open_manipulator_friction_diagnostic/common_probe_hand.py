"""Operator-supervised kinetic-only hand trial after a finite physical release.

This is deliberately separate from automatic calibration qualification. A local
release does not certify other postures, direction histories or passivity.
"""
import json
from pathlib import Path
import numpy as np
from .automatic_calibration import file_hash
from .integration import fingerprint,original_gravity
from .periodic_friction import evaluate
from .kinetic_runtime import parameters as kinetic_parameters


def parameters_from_probe(model,c,baseline):
    if (model.get('schema')!='common_periodic_friction_v1' or model.get('joint') not in range(1,7) or
            c.get('common_rest_dither') or c.get('common_dither_only') or
            c.get('common_hand_seconds') or c['additional_damping']!=0.):
        raise ValueError('Only the J2 kinetic-only physical probe can enter this supervised trial')
    return _periodic_parameters(model,c,baseline)


def _periodic_parameters(model,c,baseline):
    return kinetic_parameters(model,c,baseline,1.)


def parameters_from_hand_session(model,c,baseline,rest,output_fraction):
    """Same observed common law, with all-axis live gravity and explicit kinetic fraction.

    This adapter does not qualify the model for other loads or postures.
    Dither amplitude and gravity are independent of the kinetic fraction.
    """
    if (model.get('schema') not in ('common_periodic_friction_v1','common_friction_v1') or model.get('joint') not in range(1,7) or
            c.get('common_dither_only') or not c.get('common_rest_dither') or
            c.get('common_hand_seconds')!=-1 or c['additional_damping']!=0. or
            not np.isfinite(output_fraction) or not 0.<output_fraction<=1.):
        raise ValueError('Expected a completed common kinetic+dither manual observation')
    selected=rest.get('selected')
    if (rest.get('joint')!=model.get('joint') or rest.get('identity')!=model.get('identity') or
            selected is None or selected['amplitude_nm']!=c['dither_amplitude'] or
            selected['frequency_hz']!=c['dither_frequency'] or
            rest.get('response_profile')!=c.get('rest_response_profile') or
            rest.get('selection_policy')!=c.get('rest_selection_policy')):
        raise ValueError('Measured rest selection differs from the hand session')
    p=_periodic_parameters(model,c,baseline)
    p.update(stateful_curve_output_scale=float(output_fraction),stateful_dither_with_elastic=True,
        stateful_dither_amplitude_nm=c['dither_amplitude'],stateful_dither_frequency_hz=c['dither_frequency'],
        stateful_dither_full_velocity=c['dither_full_velocity'],
        stateful_dither_zero_velocity=c['dither_zero_velocity'],stateful_dither_ramp_seconds=c['dither_ramp'])
    return p


def load_completed_hand_parameters(folder,urdf,baseline,output_fraction):
    """Reuse recorded manual settings, never promote completion to qualification."""
    from .rest_response import load_archive
    folder=Path(folder);read=lambda n:json.loads((folder/n).read_text())
    e=read('execution.json');model=read('candidate.json');restore=read('register_restore.json')
    if (e.get('mock') is not False or e.get('kind')!='common_ab' or e.get('target_joint')!=model.get('joint') or model.get('joint') not in range(1,7) or
            not e.get('supervised_hand_observation') or not e.get('finished') or
            not e.get('register_restored') or e.get('probe',{}).get('fault')!=0 or
            e.get('failure') or not restore.get('passed') or not restore.get('torque_enable_verified_off') or
            any(restore.get('error_codes',{}).get(str(j))!=0 for j in range(1,7))):
        raise ValueError('Require a finished, restored, fault-free physical selected-axis hand session')
    if file_hash(folder/'candidate.json')!=e['model_sha256']:
        raise ValueError('Changed hand-session model evidence')
    identity=model['identity']
    if identity['robot_fingerprint']!=fingerprint(urdf) or identity['baseline_sha256']!=file_hash(baseline):
        raise ValueError('Robot or original gravity baseline changed')
    root=Path(__file__).resolve().parents[2]
    for name in ('stateful_friction.hpp','elastoplastic_replay.hpp','directional_load_law.hpp',
                 'kinetic_curve.hpp','rest_dither.hpp','position_velocity_observer.hpp'):
        path='open_manipulator_friction_diagnostic/include/friction_compensation_controller/'+name
        recorded=e['implementation_sha256'].get(path);current=file_hash(root/path)
        # This exact revision only adds a default-false mean-extension branch.
        # Its disabled path preserves the old law; retain original evidence hashes.
        disabled_mean_compatible=(name=='directional_load_law.hpp' and
            recorded=='523ff2f31422a80ade7e9654daaa9919e739e81c5475aa699b01540f4930a12d' and
            current=='b48f2fbf0324265bc00e993a0cfc77f65e68053b06b80250334af0e25b51ecd5' and
            model.get('load_hat_mean_extension',False) is False)
        if recorded!=current and not disabled_mean_compatible:
            raise ValueError('Physical compensation implementation changed: '+name)
    rest=load_archive(folder/'rest_evidence',mock=False)
    return parameters_from_hand_session(model,e['common_parameters'],original_gravity(baseline),rest,output_fraction)


def load_probe_hand_parameters(folder,urdf,baseline):
    from .common_release_review import review_rows
    folder=Path(folder);read=lambda n:json.loads((folder/n).read_text())
    e=read('execution.json');model=read('candidate.json');restore=read('register_restore.json')
    if (e.get('mock') is not False or e.get('kind')!='common_ab' or e.get('target_joint')!=2 or
            not e.get('finished') or not e.get('passed') or not e.get('register_restored') or
            e.get('assistance_mask')!=[1] or e.get('output_fraction')!=1. or
            not restore.get('passed') or not restore.get('torque_enable_verified_off') or
            any(restore.get('error_codes',{}).get(str(j))!=0 for j in range(1,7))):
        raise ValueError('A completed physical kinetic-only release and restoration are required')
    if file_hash(folder/'candidate.json')!=e['model_sha256']:
        raise ValueError('Changed physical model evidence')
    identity=model['identity']
    if identity['robot_fingerprint']!=fingerprint(urdf) or identity['baseline_sha256']!=file_hash(baseline):
        raise ValueError('Robot or original gravity baseline changed')
    root=Path(__file__).resolve().parents[2]
    for name in ('stateful_friction.hpp','elastoplastic_replay.hpp','directional_load_law.hpp',
                 'kinetic_curve.hpp','rest_dither.hpp','position_velocity_observer.hpp'):
        path='open_manipulator_friction_diagnostic/include/friction_compensation_controller/'+name
        if e['implementation_sha256'].get(path)!=file_hash(root/path):
            raise ValueError('Physical compensation implementation changed: '+name)
    a=np.genfromtxt(folder/'samples.csv',delimiter=',',names=True)
    result=review_rows(a,[1],e['directions'])
    if len(result['trials'])!=1:raise ValueError('Missing physical release data')
    t=result['trials'][0]
    if not (t['stopped'] and t['no_speed_growth'] and t['release_duration_s']>=7.9 and t['reverse_travel_rad']<=.001):
        raise ValueError('Physical release did not stop')
    if any(np.any(a[f'j{j}_drive_error']!=0) for j in range(1,7)):
        raise ValueError('Drive error in physical evidence')
    if np.any(a['dither_nm']!=0) or np.any(a['damping_nm']!=0):
        raise ValueError('Physical trial contains a different compensation component')
    return parameters_from_probe(model,e['common_parameters'],original_gravity(baseline))


def parameters_from_comparison(model,c,baseline,rest,output_fraction):
    """Exact kinetic law for any axis; compose an independent rest artifact only when requested."""
    if (c.get('common_hand_seconds') or c.get('common_dither_only') or
            c.get('additional_damping')!=0. or not np.isfinite(output_fraction) or
            not 0.<output_fraction<=1. or c.get('common_output_fraction')!=output_fraction):
        raise ValueError('Require the exact finite kinetic comparison and fraction')
    p=kinetic_parameters(model,c,baseline,output_fraction)
    if not c.get('common_rest_dither'):
        if rest is not None:raise ValueError('Unexpected rest evidence for kinetic-only comparison')
        return p
    selected=rest.get('selected') if rest else None
    if (not rest or rest.get('joint')!=model['joint'] or rest.get('identity')!=model.get('identity') or
            not selected or not (selected.get('passed') or
                (rest.get('selection_policy')=='fixed_compromise_v1' and selected.get('fixed_compromise_eligible'))) or
            selected['amplitude_nm']!=c['dither_amplitude'] or selected['frequency_hz']!=c['dither_frequency'] or
            rest.get('response_profile')!=c.get('rest_response_profile') or
            rest.get('selection_policy')!=c.get('rest_selection_policy')):
        raise ValueError('Measured rest selection differs from comparison')
    p.update(stateful_dither_with_elastic=True,stateful_dither_amplitude_nm=c['dither_amplitude'],
        stateful_dither_frequency_hz=c['dither_frequency'],stateful_dither_full_velocity=c['dither_full_velocity'],
        stateful_dither_zero_velocity=c['dither_zero_velocity'],stateful_dither_ramp_seconds=c['dither_ramp'])
    return p


def check_steady_comparison(model,check,reference):
    if not check['means_passed'] or not reference['passed']:
        raise ValueError('Independent steady means/reference repeat failed')
    if model.get('kinetic_protocol')=='common_kinetic_v2' and not (
            check.get('stage')=='STEADY_VALIDATED' and check.get('independent')
            and check.get('spatial_passed') and check['support']['within_development_bounds']
            and model.get('posture_review',{}).get('passed')):
        raise ValueError('Common kinetic spatial/support/posture validation is incomplete')
    # Legacy mean-only hand trials remain explicit supervised compatibility,
    # not fresh all-domain qualification of the canonical kinetic protocol.


def load_comparison_parameters(folder,urdf,baseline,output_fraction):
    """Physical finite comparisons plus independent steady data; supervised use only."""
    from .common_release_review import review_acquisitions
    from .kinetic_selection import validate
    from .rest_response import load_archive
    folder=Path(folder);model=json.loads((folder/'candidate.json').read_text())
    identity=model['identity']
    if identity['robot_fingerprint']!=fingerprint(urdf) or identity['baseline_sha256']!=file_hash(baseline):
        raise ValueError('Robot or original gravity baseline changed')
    trials=[folder/f'trial_{k}' for k in range(8)]
    result=review_acquisitions(trials,approach='same')
    if not result['physical_local_comparison_passed'] or file_hash(folder/'candidate.json')!=result['model_sha256']:
        raise ValueError('Complete matching physical OFF/ON comparison required')
    root=Path(__file__).resolve().parents[2]
    for name in ('stateful_friction.hpp','elastoplastic_replay.hpp','directional_load_law.hpp',
                 'kinetic_curve.hpp','rest_dither.hpp','position_velocity_observer.hpp'):
        path='open_manipulator_friction_diagnostic/include/friction_compensation_controller/'+name
        if result['implementation_sha256'].get(path)!=file_hash(root/path):
            raise ValueError('Physical compensation implementation changed: '+name)
    v=folder/'steady_validation'
    check=validate(model,json.loads((v/'cycles.json').read_text()),json.loads((v/'evidence.json').read_text()))
    reference=json.loads((v/'reference_check.json').read_text())
    check_steady_comparison(model,check,reference)
    e=json.loads((trials[0]/'execution.json').read_text())
    if any(json.loads((t/'execution.json').read_text())['common_parameters'].get('common_release_speed',.02)!=.02 for t in trials):
        raise ValueError('Higher-speed diagnostic alone is not a standard hand comparison')
    rest=load_archive(trials[0]/'rest_evidence',mock=False) if e['common_parameters'].get('common_rest_dither') else None
    return parameters_from_comparison(model,e['common_parameters'],original_gravity(baseline),rest,output_fraction)
