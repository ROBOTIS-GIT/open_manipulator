"""Purpose-bound, sampled geometry review for the common six-axis rest experiment.

This offline path supplies no kinetic parameters or hardware qualification. A
current-reporting diagnostic is a different experiment and cannot use this gate.
"""
import json
from pathlib import Path

import numpy as np

from .automatic_calibration import content_hash, file_hash
from .generalization_motion import check_geometry
from .integration import fingerprint
from .poses import Robot
from .protocol import JOINTS, validate_urdf_plan


def source_hashes():
    return {name: file_hash(Path(__file__).with_name(name)) for name in
            ('rest_selection.py', 'generalization_motion.py', 'poses.py', 'protocol.py', 'integration.py')}


def make_plan(joint, identity, center, amplitudes, frequencies, condition):
    """Use the existing <=3 Nm rest sine recipe with a small neutral-load envelope."""
    center, amplitudes, frequencies = (np.asarray(x, dtype=float) for x in
                                        (center, amplitudes, frequencies))
    if (type(joint) is not int or joint not in range(1, 7) or center.shape != (6,) or not np.isfinite(center).all() or
            amplitudes.ndim != 1 or not 1 <= len(amplitudes) <= 10 or
            not np.all(np.isfinite(amplitudes) & (amplitudes > 0) & (amplitudes <= 3.)) or
            not np.all(np.diff(amplitudes) > 0) or frequencies.ndim != 1 or
            not 1 <= len(frequencies) <= 3 or len(set(frequencies)) != len(frequencies) or
            not np.all(np.isfinite(frequencies) & (frequencies >= 5) & (frequencies <= 20)) or
            not isinstance(condition, str) or not condition or
            not isinstance(identity, dict) or any(not identity.get(k) for k in
                ('robot_unit', 'robot_fingerprint', 'baseline_sha256'))):
        raise ValueError('Invalid rest-selection identity, pose or sine schedule')
    return dict(schema='common_rest_selection_v1', purpose='rest_selection', target_joint=joint,
        identity=dict(identity), center_rad=center.tolist(), condition=condition,
        amplitudes_nm=amplitudes.tolist(), frequencies_hz=frequencies.tolist(),
        sine_blocks=[dict(amplitude_nm=float(a), frequency_hz=float(f), repeat=r)
                     for a in amplitudes for f in frequencies for r in range(2)],
        sine_active_s=6., sine_rest_s=2., sine_ramp_s=.5,
        controller_parameters=dict(rest_selection=True, max_excursion_rad=.002,
                                   max_speed_rad_s=.1, held_joint_error_rad=.001))


def _validate_plan(plan):
    try:
        expected = make_plan(plan['target_joint'], plan['identity'], plan['center_rad'],
                             plan['amplitudes_nm'], plan['frequencies_hz'], plan['condition'])
        if expected != plan:
            raise ValueError('Rest-selection plan differs from the common sine protocol')
    except (KeyError, TypeError) as exc:
        raise ValueError('Missing rest-selection plan') from exc


def sample_positions(plan):
    _validate_plan(plan)
    positions = np.tile(plan['center_rad'], (401, 1))
    positions[:, plan['target_joint']-1] += np.linspace(-.003, .003, len(positions))
    return positions


def review_geometry(plan, urdf):
    """Review only the small probe at its held pose; this is not a transit plan."""
    positions = sample_positions(plan)
    urdf = Path(urdf)
    robot_fingerprint = fingerprint(urdf.read_text())
    if robot_fingerprint != plan['identity']['robot_fingerprint']:
        raise ValueError('Rest-selection robot fingerprint mismatch')
    validate_urdf_plan({}, dict(q=positions), urdf.read_text())
    geometry = check_geometry(positions, urdf)
    load = float(np.max(np.abs(Robot(urdf, JOINTS).gravity_batch(positions)[:, plan['target_joint']-1])))
    result = dict(schema='rest_selection_geometry_v1', purpose='rest_selection',
        plan_sha256=content_hash(plan), robot_fingerprint=robot_fingerprint,
        source_hashes=source_hashes(), positions_sha256=content_hash(positions.tolist()),
        reviewed_half_range_rad=.003, geometry=geometry,
        maximum_abs_nominal_gravity_nm=load,
        passed=bool(geometry['passed'] and np.isfinite(load) and load < .15),
        hardware_started=False, compensation_qualified=False)
    validate_review(plan, result)
    return result


def validate_review(plan, review):
    """Bind the purpose, pose, waveform, robot and sampled neutral-load geometry."""
    positions = sample_positions(plan)
    geometry = review.get('geometry', {})
    load = review.get('maximum_abs_nominal_gravity_nm', float('nan'))
    if (review.get('schema') != 'rest_selection_geometry_v1' or
            review.get('purpose') != 'rest_selection' or review.get('passed') is not True or
            review.get('plan_sha256') != content_hash(plan) or
            review.get('robot_fingerprint') != plan['identity']['robot_fingerprint'] or
            review.get('source_hashes') != source_hashes() or
            review.get('positions_sha256') != content_hash(positions.tolist()) or
            review.get('reviewed_half_range_rad') != .003 or
            geometry.get('passed') is not True or geometry.get('samples') != len(positions) or
            geometry.get('collision_samples') != 0 or
            not geometry.get('minimum_distance_m', -1) >= .01 or
            not geometry.get('floor_minimum_m', -1) >= .02 or
            not np.isfinite(load) or not 0 <= load < .15):
        raise ValueError('Rest-selection geometry/purpose evidence is missing, stale or mismatched')


def acquisition_metadata(plan, geometry_path):
    try:
        review = json.loads(Path(geometry_path).read_text())
    except (OSError, ValueError) as exc:
        raise ValueError('Missing rest-selection geometry review') from exc
    validate_review(plan, review)
    return dict(purpose='rest_selection', rest_selection_plan=plan,
        rest_selection_geometry_sha256=file_hash(geometry_path),
        rest_selection_parameters=dict(plan['controller_parameters']),
        held_positions_rad=list(plan['center_rad']), compensation_enabled=False,
        **{k: plan[k] for k in ('target_joint', 'identity', 'condition', 'sine_blocks',
                              'sine_active_s', 'sine_rest_s', 'sine_ramp_s')})


def validate_acquisition(execution, folder, samples):
    plan = execution.get('rest_selection_plan')
    _validate_plan(plan)
    expected = acquisition_metadata(plan, Path(folder)/'geometry_review.json')
    if (execution.get('neutral_current_response') or execution.get('stationary_parameters') or
            any(execution.get(k) != value for k, value in expected.items())):
        raise ValueError('Rest-selection acquisition purpose or reviewed settings mismatch')
    names = [f'j{j}_q' for j in range(1, 7)]
    if not set(names) <= set(samples.dtype.names or ()):
        raise ValueError('Rest-selection samples lack held-axis geometry evidence')
    positions = np.column_stack([samples[n] for n in names])
    limits = np.full(6, plan['controller_parameters']['held_joint_error_rad'])
    limits[plan['target_joint']-1] = plan['controller_parameters']['max_excursion_rad']
    if (not len(positions) or not np.isfinite(positions).all() or
            np.any(np.abs(positions-plan['center_rad']) > limits)):
        raise ValueError('Rest-selection samples exceed the reviewed controller envelope')
