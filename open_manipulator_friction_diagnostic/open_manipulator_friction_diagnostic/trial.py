"""Explicit supervised experiment: assume zero current delay, never mark it verified."""
import argparse
import json
from pathlib import Path

import numpy as np
from .protocol import JOINTS, sha
from .fit import identify
from .integration import fingerprint, baseline_path
from .trial_limits import QUALIFICATION, qualify_bounded_trial


def validate_trial_evidence(folder, c, profile=None):
    folder = Path(folder)
    meta = json.loads((folder/'plan.json').read_text())
    collection = json.loads((folder/'collection.json').read_text())
    if (not collection.get('completed') or collection.get('failure') or
            collection.get('source') != 'hardware' or not collection.get('record_only') or
            collection.get('sensor_timing_verified') is not False or
            collection.get('identification_eligible') is not False):
        raise ValueError('Trial requires completed hardware record-only acquisition; mock/partial data rejected')
    if (not meta.get('record_only') or meta.get('posture_validation_poses') or
            meta['config'] != c or c['friction_model'] != 'velocity_only' or
            c.get('sensor_timing_verified') is not False or
            any(c['current_filter_time_constant_s']) or any(c['current_measurement_delay_s'])):
        raise ValueError('Trial requires unchanged velocity-only config with explicit unknown/zero timing placeholders')
    if (sha(folder/'trajectory.npz') != meta['sha256'] or
            meta['sha256'] != collection['plan_sha256'] or
            fingerprint((folder/'robot.urdf').read_text()) != collection['robot_fingerprint']):
        raise ValueError('Trial acquisition evidence mismatch')
    if profile:
        if (profile.get('qualification') != QUALIFICATION or
                profile.get('sensor_timing_verified') is not False or
                profile.get('sensor_timing_assumption') != 'zero_delay' or
                profile.get('posture_validation_performed') is not False or
                profile.get('source') != 'hardware' or
                profile.get('collection_sha256') != sha(folder/'collection.json') or
                profile.get('plan_metadata_sha256') != sha(folder/'plan.json')):
            raise ValueError('Trial profile does not match recorded assumptions/evidence')
    return meta, collection


def export_trial(folder, baseline):
    folder = Path(folder).expanduser().resolve()
    meta = json.loads((folder/'plan.json').read_text()); c = meta['config']
    validate_trial_evidence(folder, c)
    profile_path = folder/'trial_profile.json'
    if profile_path.exists():
        raise ValueError('Trial profile already exists; preserve the existing experiment')
    data = dict(np.load(folder/'measurements.npz'))
    try:
        result = qualify_bounded_trial(identify(meta, data), c)
    except ValueError as exc:
        (folder/'trial_identification.json').write_text(json.dumps(dict(
            passed=False, qualification=QUALIFICATION, failure=str(exc),
            sensor_timing_verified=False, sensor_timing_assumption='zero_delay',
            posture_validation_performed=False), indent=2))
        raise
    result.update(qualification=QUALIFICATION, sensor_timing_verified=False,
                  sensor_timing_assumption='zero_delay', posture_validation_performed=False,
                  hardware_freedrive_validated=False,
                  trial_gain=result['trial_runtime_limits']['gain'])
    report_path = folder/'trial_identification.json'
    report_path.write_text(json.dumps(result, indent=2))
    if not result['passed']:
        failed = [v['joint'] for v in result['validation'] if not v['passed']]
        raise ValueError(f'Repeat validation failed: {failed}; no trial profile exported')
    profile = dict(schema=3, source='hardware', qualification=QUALIFICATION,
                   sensor_timing_verified=False, sensor_timing_assumption='zero_delay',
                   posture_validation_performed=False, robot_unit=c['robot_unit'], joints=JOINTS,
                   robot_fingerprint=fingerprint((folder/'robot.urdf').read_text()),
                   baseline_hash=sha(baseline), plan_sha256=meta['sha256'],
                   plan_metadata_sha256=sha(folder/'plan.json'),
                   collection_sha256=sha(folder/'collection.json'),
                   measurements_sha256=sha(folder/'measurements.npz'),
                   identification_sha256=sha(report_path), effort_unit='joint_equivalent_Nm',
                   estimator=result['estimator'], pointwise_model_passed=result['pointwise_model_passed'],
                   acceptance_policy=result['acceptance_policy'],
                   trial_runtime_limits=result['trial_runtime_limits'],
                   method=result['method'], hardware_freedrive_validated=False,
                   static_breakaway_identified=False, parameters=result['parameters'], config=c,
                   load_bounds_nm=result['load_bounds_nm'], load_compensation_enabled=False,
                   passed=True)
    temporary = folder/'trial_profile.json.tmp'
    temporary.write_text(json.dumps(profile, indent=2)); temporary.replace(profile_path)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--assume-zero-delay', action='store_true', required=True)
    args = parser.parse_args()
    result = export_trial(args.run, baseline_path())
    print(json.dumps(dict(passed=result['passed'], qualification=QUALIFICATION,
                         pointwise_model_passed=result['pointwise_model_passed'],
                         sensor_timing_verified=False, trial_gain=result['trial_gain'],
                         trial_max_compensation_nm=result['trial_runtime_limits']['max_compensation_nm'],
                         profile=str(Path(args.run).resolve()/'trial_profile.json')), indent=2))
