"""Independent finite current-probe collection and measured fixed-wave selection.

The ROS orchestrator moves to each reviewed pose in position mode, turns all
motors OFF, and starts the mixed current/position probe runtime. This module
never starts a device, substitutes a trace, or consumes kinetic fit residuals.
"""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import yaml

from .automatic_calibration import assess_rest_trace, content_hash, file_hash, select_rest
from .rest_selection import make_plan, review_geometry, acquisition_metadata, validate_acquisition


def make_plans(config_path, identity, *, previous=None, joints=None):
    settings = yaml.safe_load(Path(config_path).read_text())
    if settings.get('schema') != 'omy_independent_dither_recipe_v1':
        raise ValueError('Invalid independent dither recipe')
    from .joint_selection import parse_joints
    joints = parse_joints(settings['joints'] if joints is None else joints)
    settings['joints'] = joints
    if (not joints or len(set(joints)) != len(joints) or
            any(type(j) is not int or j not in range(1, 7) for j in joints)):
        raise ValueError('Select distinct dither joints 1 through 6')
    conditions = settings['conditions']
    if len(conditions) < 2 or len({tuple(q) for q in conditions.values()}) != len(conditions):
        raise ValueError('Dither needs distinct physical conditions')
    if (settings['response_profile'] != 'soft_start_v1' or
            settings['selection_policy'] != 'fixed_compromise_v1'):
        raise ValueError('Use the explicit fixed-wave comfort selection policy')
    if not 0 <= settings['full_velocity'] < settings['zero_velocity'] or not settings['ramp'] > 0:
        raise ValueError('Invalid fixed dither fade settings')
    prior = (previous or {}).get('dither', {}) if (previous or {}).get('identity') == identity else {}
    plans = []
    for joint in joints:
        old = prior.get(f'joint{joint}', {})
        amplitudes = sorted(set(settings['amplitudes_nm'] + ([old['amplitude']] if old.get('amplitude', 0) > 0 else [])))
        frequencies = sorted(set(settings['frequencies_hz'] + ([old['frequency']] if old.get('amplitude', 0) > 0 else [])))
        for condition, pose in conditions.items():
            plans.append(make_plan(joint, identity, pose, amplitudes, frequencies, condition))
    return settings, plans


def probe_parameters(plan, output_csv):
    return dict(plan['controller_parameters'], probe_kind='rest_response',
        target_joint=plan['target_joint'], expected_pose=list(plan['center_rad']),
        sine_amplitudes_nm=list(plan['amplitudes_nm']), sine_frequencies_hz=list(plan['frequencies_hz']),
        sine_repeats=2, defer_probe_start=True, stationary_gravity_compensation=False,
        neutral_current_response=False, monitor_drive_errors=True,
        output_csv=str(Path(output_csv).resolve()))


def prepare_dither(config_path, output_dir, urdf_text, identity, *, previous=None, joints=None):
    """Publish reviewed per-pose jobs. Transit review belongs to the collector."""
    settings, plans = make_plans(config_path, identity, previous=previous, joints=joints)
    out = Path(output_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out / 'manifest.json').exists():
        raise ValueError('Use a new independent dither output directory')
    urdf = out / 'robot.urdf'
    urdf.write_text(urdf_text)
    jobs = []
    for index, plan in enumerate(plans):
        folder = out / f'probe_{index:02d}_joint{plan["target_joint"]}_{plan["condition"]}'
        folder.mkdir(exist_ok=False)
        review = review_geometry(plan, urdf)
        params = probe_parameters(plan, folder / 'samples.csv')
        (folder / 'plan.json').write_text(json.dumps(plan, indent=2))
        (folder / 'geometry_review.json').write_text(json.dumps(review, indent=2))
        (folder / 'parameters.yaml').write_text(yaml.safe_dump({'current_probe': {'ros__parameters': params}}))
        jobs.append(dict(directory=str(folder), plan=plan, parameters=params,
            target_joint=plan['target_joint'], center_rad=plan['center_rad'],
            duration_s=plan['sine_rest_s']+len(plan['sine_blocks'])*(plan['sine_active_s']+plan['sine_rest_s'])))
    manifest = dict(schema='omy_independent_dither_collection_v1', settings=settings,
        identity=identity, jobs=jobs, total_probe_seconds=sum(j['duration_s'] for j in jobs),
        transit_and_motor_transitions_additional=True, kinetic_fit_used=False,
        all_motors_off_before_each_probe=True, initial_pose_must_be_measured=True)
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    return manifest


def validate_completion(status, proof):
    """Accept only complete saved probe data and read-back shutdown evidence."""
    if (status.get('state') != 2 or status.get('fault') != 0 or status.get('saved') is not True or
            status.get('journal_ok') is not True or status.get('rows', 0) < 100 or
            status.get('saved_rows') != status.get('rows') or
            proof.get('source') not in ('hardware', 'gazebo') or
            proof.get('all_motors_off') is not True or proof.get('torque_enable') != [0]*6 or
            proof.get('started_after_all_running') is not True):
        raise ValueError('Need completed saved probe, verified RUNNING start and all-six OFF shutdown')


def record_run(directory, status, proof):
    """Bind actual controller CSV to a finished runtime receipt, after shutdown."""
    validate_completion(status, proof)
    folder = Path(directory)
    plan = json.loads((folder / 'plan.json').read_text())
    data = np.atleast_1d(np.genfromtxt(folder / 'samples.csv', delimiter=',', names=True))
    if len(data) != status['saved_rows']:
        raise ValueError('Probe CSV row count differs from controller saved status')
    execution = dict(acquisition_metadata(plan, folder / 'geometry_review.json'),
        schema='omy_independent_dither_run_v1', kind='rest_response', finished=True,
        passed=True, probe=status, source=proof['source'], lifecycle=deepcopy(proof),
        samples_sha256=file_hash(folder / 'samples.csv'), synthetic_effort=False)
    validate_acquisition(execution, folder, data)
    if not set(['time_s', 'phase', 'command_nm', f'j{plan["target_joint"]}_effort']).issubset(data.dtype.names or ()):
        raise ValueError('Probe CSV lacks actual command/current samples')
    (folder / 'execution.json').write_text(json.dumps(execution, indent=2))
    return execution


def _traces(job):
    folder = Path(job['directory'])
    e = json.loads((folder / 'execution.json').read_text())
    if (e.get('schema') != 'omy_independent_dither_run_v1' or e.get('synthetic_effort') is not False or
            e.get('samples_sha256') != file_hash(folder / 'samples.csv') or
            e.get('rest_selection_plan') != job['plan']):
        raise ValueError('Independent dither acquisition changed or has wrong source')
    validate_completion(e['probe'], e['lifecycle'])
    if e['source'] != e['lifecycle']['source']:
        raise ValueError('Dither source differs from runtime receipt')
    data = np.atleast_1d(np.genfromtxt(folder / 'samples.csv', delimiter=',', names=True))
    validate_acquisition(e, folder, data)
    joint = e['target_joint']
    traces = []
    for index, block in enumerate(e['sine_blocks']):
        rows = data[data['phase'] == index]
        if len(rows) < 100:
            raise ValueError('Missing measured sine block')
        if np.ptp(rows[f'j{joint}_effort']) < 1e-5:
            raise ValueError('Measured effort is zero or constant; no dither response identification')
        traces.append(dict(block, trace_id=e['samples_sha256'][:16]+f'/block{index}',
            source='hardware' if e['source'] == 'hardware' else 'synthetic',
            acquisition_source=e['source'], joint=joint, identity=e['identity'], contact_free=True,
            contact_assumption='No reported contact during the finite commanded probe',
            input_kind='known_sine', condition=e['condition'], ramp_s=e['sine_ramp_s'],
            t=(rows['time_s']-rows['time_s'][0]).tolist(), q=rows[f'j{joint}_q'].tolist(),
            command_nm=rows['command_nm'].tolist(), current_proxy_nm=rows[f'j{joint}_effort'].tolist()))
    return traces


def finish_dither(manifest):
    """Select one measured fixed waveform per joint, separately from friction."""
    settings = manifest['settings']
    selections, failures, values, sources = {}, {}, {}, set()
    for joint in settings['joints']:
        key = f'joint{joint}'
        try:
            traces = [t for job in manifest['jobs'] if job['target_joint'] == joint for t in _traces(job)]
            sources.update(t['acquisition_source'] for t in traces)
            result = select_rest(traces, settings['encoder_step_rad'], settings['conditions'],
                                 settings['response_profile'], settings['selection_policy'])
            selections[key] = result
            if result['selected'] is None:
                raise ValueError('No measured waveform meets the fixed-wave selection criteria')
            selected = result['selected']
            values[key] = dict(amplitude=selected['amplitude_nm'], frequency=selected['frequency_hz'],
                **{k: settings[k] for k in ('full_velocity', 'zero_velocity', 'ramp')})
        except (OSError, ValueError, KeyError) as exc:
            failures[key] = str(exc)
    if len(sources) > 1:
        failures['source'] = 'Mixed hardware and Gazebo measurements are not one calibration'
    source = next(iter(sources)) if len(sources) == 1 else None
    return dict(schema='omy_independent_dither_result_v1', collected=True, independent=True,
        source=source, usable=not failures and len(values) == len(settings['joints']),
        values=values, failures=failures, selections=selections,
        recipe_sha256=content_hash(settings), kinetic_fit_used=False,
        hardware_qualified=False, fixed_waveform=True,
        selection_scope='Fixed measured waveform; no automatic combined-controller release qualification')


def apply_result(artifact, result):
    """Update only independent waveform fields after all requested axes pass."""
    if (result.get('usable') is not True or result.get('collected') is not True or
            result.get('source') != artifact.get('source', 'hardware')):
        raise ValueError('Dither result is incomplete or belongs to a different source')
    output = deepcopy(artifact)
    for joint, values in result['values'].items():
        output.setdefault('dither', {})[joint] = deepcopy(values)
        for name, value in values.items():
            output['controller_parameters'][f'stateful_bank.{joint}.dither_{name}'] = value
    output['dither_collection'] = deepcopy(result)
    return output


def collect(output_dir, identification, previous=None):
    """Run physical acquisition; imported lazily to keep pure APIs ROS-free."""
    from .dither_collection import collect as acquire
    return acquire(output_dir, identification, previous)
