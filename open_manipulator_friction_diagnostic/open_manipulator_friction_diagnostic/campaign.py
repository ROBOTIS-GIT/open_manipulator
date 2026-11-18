"""Fresh assembled-robot identification with separate, untouched posture validation.

A steady mean fit is not instantaneous friction cancellation or a passivity proof.
Old pointwise diagnostics remain in the report; they are never relabelled passed.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
import yaml

from .fit import identify, magnitude, pair_samples, validate_log
from .integration import fingerprint, original_gravity
from .protocol import JOINTS, sha, validation_poses

QUALIFICATION = 'steady_mean_unscaled_experiment_v1'
POLICY = 'independent_cycle_mean_v1'


def read_acquisition(folder, posture_only, mock=False, loaded_role=None, position_probe=False):
    folder = Path(folder).resolve()
    meta = json.loads((folder/'plan.json').read_text())
    collection = json.loads((folder/'collection.json').read_text())
    if (not collection.get('completed') or collection.get('failure') or
            collection.get('source') != ('synthetic' if mock else 'hardware') or not collection.get('record_only') or
            not meta.get('record_only') or
            (not posture_only and loaded_role is None and not position_probe and bool(meta.get('posture_validation_poses'))) or
            (position_probe and (not meta.get('position_probe') or not collection.get('position_probe'))) or
            (loaded_role is not None and (not meta.get('loaded_study') or
                meta.get('loaded_study_role') != loaded_role or
                collection.get('loaded_study_role') != loaded_role)) or
            bool(meta.get('posture_validation_only')) != posture_only or
            bool(collection.get('posture_validation_only')) != posture_only or
            meta['sha256'] != sha(folder/'trajectory.npz') or
            collection.get('plan_sha256') != meta['sha256'] or
            collection.get('robot_fingerprint') != fingerprint((folder/'robot.urdf').read_text())):
        raise ValueError('Incomplete, synthetic, mismatched or wrong-scope acquisition')
    if (yaml.safe_load((folder/'config.yaml').read_text()) != meta['config'] or
            collection.get('sensor_timing_verified') != meta['config']['sensor_timing_verified']):
        raise ValueError('Acquisition configuration or timing assumption changed')
    measured = dict(np.load(folder/'measurements.npz'))
    validate_log(meta, measured)
    return meta, measured, dict(folder=str(folder), hashes={name: sha(folder/name) for name in
        ('plan.json', 'trajectory.npz', 'measurements.npz', 'collection.json', 'robot.urdf', 'config.yaml')})


def mean_check(row, parameters, c):
    prediction = float(magnitude(row['speed'], parameters, row['mean_load_nm']))
    residual = prediction-row['friction_nm']
    tolerance = c['validation_absolute_error_nm']+c['validation_relative_error']*row['friction_nm']
    return dict(joint=JOINTS[row['joint']], repeat=row['repeat'],
                validation_pose=row['validation_pose'], speed_rad_s=row['speed'],
                measured_mean_nm=row['friction_nm'], predicted_mean_nm=prediction,
                error_nm=residual, tolerance_nm=tolerance,
                passed=bool(abs(residual) <= tolerance), samples_used_for_fit=False)


def assess(base_meta, base_data, posture_meta, posture_data):
    c = base_meta['config']
    if c != posture_meta['config'] or c['friction_model'] != 'velocity_only':
        raise ValueError('Campaign requires the same velocity-only configuration in both acquisitions')
    if not c['sensor_timing_verified'] and (
            any(c['current_filter_time_constant_s']) or any(c['current_measurement_delay_s'])):
        raise ValueError('Unknown timing experiment requires explicit zero-delay placeholders')
    # Freeze coefficients using the first pass ONLY. The second pass and posture
    # file do not enter the optimization, model selection, gain or cap selection.
    original = identify(base_meta, base_data, select_velocity_model=True)
    validate_log(posture_meta, posture_data)
    posture_pairs, posture_starts = pair_samples(posture_meta, posture_data)
    expected = {p['name'] for p in validation_poses(c)}
    if {p['name'] for p in posture_meta['posture_validation_poses']} != expected:
        raise ValueError('Missing planned held-out postures')
    checks = []
    for j, parameters in enumerate(original['parameters']):
        for name in expected:
            rows = [r for r in posture_pairs if r['joint'] == j and r['validation_pose'] == name]
            if sorted(r['speed'] for r in rows) != sorted(c['posture_validation_speeds_rad_s']):
                raise ValueError(f'Incomplete posture observations for {JOINTS[j]} / {name}')
        checks.extend(mean_check(r, parameters, c) for r in posture_pairs if r['joint'] == j)
    repeated = all(x['mean_model_passed'] for x in original['validation'])
    posture_passed = all(x['passed'] for x in checks)
    from .matched_analysis import diagnostics
    matched = diagnostics(original['pairs'], posture_pairs, c)
    settling = None
    if c.get('sweep_protocol') == 'matched_v2':
        from .settling import replay
        settling = dict(base=replay(base_meta, base_data), postures=replay(posture_meta, posture_data))
    rests_passed = settling is None or all(r['passed'] for r in settling.values())
    return dict(schema=1, qualification=QUALIFICATION, acceptance_policy=POLICY,
                effort_unit='joint_equivalent_Nm',
                scope='Steady symmetric mean friction at measured speeds and tested postures only',
                passed=repeated and posture_passed and rests_passed, independent_repeat_passed=repeated,
                matched_condition_diagnostics=matched, mechanical_settling=settling,
                independent_posture_passed=posture_passed, posture_checks=checks,
                parameters=original['parameters'], parameter_order=original['parameter_order'],
                pointwise_model_passed=original['passed'], original_pointwise_report=original,
                posture_pairs=posture_pairs, posture_start_events=posture_starts,
                sensor_timing_verified=c['sensor_timing_verified'],
                sensor_timing_assumption='zero_delay' if not c['sensor_timing_verified'] else 'configured_verified',
                static_breakaway_identified=False, passivity_proven=False,
                hardware_freedrive_validated=False, rigid_parameters_modified=False,
                torque_conversion_modified=False,
                runtime_policy=dict(gain=1., torque_cap=None, slew_limit=None,
                    low_speed_fade=False, high_speed_fade=False, speed_cutoff=False,
                    zero_velocity_output=0., out_of_range='diagnostic; extrapolation is unvalidated'),
                validation_tolerances=dict(absolute_nm=c['validation_absolute_error_nm'],
                    relative=c['validation_relative_error'], changed_for_this_run=False),
                acceptance_change='Mean-model acceptance is separate from legacy instantaneous acceptance; '
                    'a pointwise failure is retained and is not a passivity qualification')


def export(base_folder, posture_folder, output, baseline, mock=False):
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    base_meta, base_data, base_evidence = read_acquisition(base_folder, False, mock)
    posture_meta, posture_data, posture_evidence = read_acquisition(posture_folder, True, mock)
    if base_evidence['folder'] == posture_evidence['folder']:
        raise ValueError('Separate held-out acquisition required')
    if fingerprint((Path(base_folder)/'robot.urdf').read_text()) != fingerprint(
            (Path(posture_folder)/'robot.urdf').read_text()):
        raise ValueError('Robot changed between acquisitions')
    report = assess(base_meta, base_data, posture_meta, posture_data)
    report['acquisitions'] = [base_evidence, posture_evidence]
    report['created_utc'] = datetime.now(timezone.utc).isoformat()
    report['implementation_sha256'] = {name: sha(Path(__file__).with_name(name))
        for name in ('campaign.py', 'curve_selection.py', 'fit.py')}
    path = output/'identification.json'; path.write_text(json.dumps(report, indent=2))
    (output/'config.yaml').write_text((Path(base_folder)/'config.yaml').read_text())
    if not report['passed']:
        raise ValueError('Steady mean repeat/posture validation failed; report saved, no profile exported')
    profile = dict(schema=4, qualification=QUALIFICATION, source='synthetic' if mock else 'hardware',
        effort_unit=report['effort_unit'],
        robot_unit=base_meta['config']['robot_unit'], joints=JOINTS,
        robot_fingerprint=fingerprint((Path(base_folder)/'robot.urdf').read_text()),
        baseline_hash=sha(baseline), report_sha256=sha(path), config=base_meta['config'],
        parameters=report['parameters'], passed=True, sensor_timing_verified=report['sensor_timing_verified'],
        sensor_timing_assumption=report['sensor_timing_assumption'],
        pointwise_model_passed=report['pointwise_model_passed'],
        static_breakaway_identified=False, passivity_proven=False, hardware_freedrive_validated=False,
        runtime_policy=report['runtime_policy'])
    (output/'profile.json').write_text(json.dumps(profile, indent=2))
    # Deliberately no latest symlink or controller activation.
    return report


def controller_parameters(path, c, urdf, baseline, allow_experiment, mock=False, allow_unqualified=False):
    path = Path(path).resolve(); p = json.loads(path.read_text())
    if p.get('schema') in (5,6):
        from .loaded_campaign import controller_parameters as load_parameters
        return load_parameters(path,c,urdf,baseline,allow_experiment,mock,allow_unqualified)
    if not allow_experiment:
        raise ValueError('Unscaled candidate is an explicit experiment, not certified passive compensation')
    if (p.get('schema') != 4 or p.get('qualification') != QUALIFICATION or
            p.get('effort_unit') != 'joint_equivalent_Nm' or
            p.get('source') not in (('hardware', 'synthetic') if mock else ('hardware',)) or p.get('passed') is not True or
            p.get('robot_unit') != c['robot_unit'] or p.get('joints') != JOINTS or p['config'] != c or
            p.get('robot_fingerprint') != fingerprint(urdf) or p.get('baseline_hash') != sha(baseline) or
            p.get('report_sha256') != sha(path.parent/'identification.json')):
        raise ValueError('Unscaled profile provenance mismatch')
    report = json.loads((path.parent/'identification.json').read_text())
    if (report.get('acceptance_policy') != POLICY or report.get('passed') is not True or
            not report.get('independent_repeat_passed') or not report.get('independent_posture_passed') or
            report['parameters'] != p['parameters'] or report['runtime_policy'] != p['runtime_policy']):
        raise ValueError('Missing or mismatched independent validation')
    for field in ('effort_unit', 'sensor_timing_verified', 'sensor_timing_assumption',
                  'pointwise_model_passed', 'static_breakaway_identified',
                  'passivity_proven', 'hardware_freedrive_validated'):
        if p.get(field) != report.get(field):
            raise ValueError('Profile qualification differs from its evidence: '+field)
    for evidence in report['acquisitions']:
        for file, digest in evidence['hashes'].items():
            if sha(Path(evidence['folder'])/file) != digest:
                raise ValueError('Acquisition evidence changed')
    parameters = np.asarray(p['parameters'])
    if (parameters.shape != (6, 5) or not np.isfinite(parameters).all() or np.any(parameters < 0) or
            np.any(parameters[:, 1] < parameters[:, 0]) or np.any(parameters[:, 2] <= 0) or
            np.any(parameters[:, 4] != 0)):
        raise ValueError('Invalid identified velocity-only parameters')
    result = original_gravity(baseline)
    result.update(calibrated_friction_enabled=True, calibrated_unscaled=True,
        calibrated_gain=1., calibrated_max_torque=[0.]*6, calibrated_slew_rate=[0.]*6,
        calibrated_deadband=0., calibrated_min_velocity=min(c['speeds_rad_s']),
        calibrated_max_velocity=max(c['speeds_rad_s']), calibrated_enforce_load_bounds=False,
        calibrated_load_gain=[0.]*6, calibrated_load_min=[0.]*6, calibrated_load_max=[1.]*6,
        calibrated_fc=parameters[:, 0].tolist(), calibrated_fs=parameters[:, 1].tolist(),
        calibrated_vs=parameters[:, 2].tolist(), calibrated_viscous=parameters[:, 3].tolist())
    return result


def main():
    from .integration import baseline_path
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--identification-run', required=True)
    parser.add_argument('--posture-run', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--assume-zero-delay', action='store_true', required=True)
    args = parser.parse_args(remove_ros_args()[1:])
    report = export(args.identification_run, args.posture_run, args.output, baseline_path())
    print(json.dumps({key: report[key] for key in ('passed', 'independent_repeat_passed',
        'independent_posture_passed', 'pointwise_model_passed', 'static_breakaway_identified',
        'hardware_freedrive_validated', 'runtime_policy')}, indent=2))
