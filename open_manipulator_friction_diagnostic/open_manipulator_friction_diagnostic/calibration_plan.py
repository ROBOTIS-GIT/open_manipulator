"""Portable position-collection plans; no device access or historical paths."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import yaml

from . import automatic_motion
from .automatic_calibration import content_hash
from .generalization_motion import check_geometry, sources
from .geometry_cache import review as review_geometry
from .integration import fingerprint
from .joint_selection import parse_joints, check_partial_identity
from .protocol import config, sha, validate_urdf_plan


def load_schedule(config_path, *, previous=None, validate=False, joints=None, device_config=None):
    """Resolve package-relative inputs and report fresh acquisition coverage."""
    path = Path(config_path).expanduser().resolve()
    settings = yaml.safe_load(path.read_text())
    if settings.get('schema') != 'omy_portable_calibration_recipe_v1':
        raise ValueError('Invalid portable calibration recipe')
    joints = parse_joints(settings.get('joints', 'all') if joints is None else joints)
    settings['joints'] = joints
    for name in ('collection_config', 'conditions', 'fitting'):
        value = Path(settings[name]).expanduser()
        settings[name] = str((path.parent / value).resolve() if not value.is_absolute() else value.resolve())
    collection = config(settings['collection_config'], record_only=True,
                        device_settings=device_config if device_config else True)
    collection.update(common_friction_identification=True, automatic_sparse_collection=True)
    declared = yaml.safe_load(Path(settings['conditions']).read_text())
    if declared.get('schema') != 'omy_sparse_conditions_v1':
        raise ValueError('Invalid explicit collection conditions')
    conditions = [deepcopy(p) for p in declared['conditions'] if p['joint'] + 1 in joints
                  and (validate or p['role'] == 'development')]
    if len({p['name'] for p in conditions}) != len(conditions):
        raise ValueError('Collection conditions need unique names')
    prior_models = (previous or {}).get('models', {})
    coverage = {}
    for joint in joints:
        rows = [p for p in conditions if p['joint'] == joint - 1 and p['role'] == 'development']
        if not rows:
            raise ValueError(f'No development measurements for joint{joint}')
        configurations = {tuple(round(v, 4) for k, v in enumerate(p['q_rad']) if k != joint - 1) for p in rows}
        if len(configurations) < 3:
            raise ValueError(f'joint{joint} needs three independent held configurations')
        planned = sorted({v for p in rows for v in automatic_motion.condition_speeds(collection, p, ())})
        old = prior_models.get(f'joint{joint}', {}).get('speed_knots_rad_s', [])
        coverage[f'joint{joint}'] = dict(planned_speeds_rad_s=planned,
            unmeasured_reference_speeds_rad_s=[v for v in old if v not in planned],
            measured_coverage_verified=False, full_speed_load_rectangle_measured=False,
            support_policy='Fit only fresh measurements; old coefficients and support are never reused')
    automatic_motion.check_sparse_holdouts(collection, conditions, [])
    return settings, collection, conditions, coverage


def _trajectory(collection, conditions, validate, robot_fingerprint=None):
    """Append reserved configurations without allowing them into development."""
    def generate(c, role):
        if robot_fingerprint is None:
            return automatic_motion.generate(c, conditions, role)
        from .trajectory_assets import generate as precomputed
        return precomputed(c, conditions, role, robot_fingerprint)
    meta, data = generate(collection, 'development')
    if validate:
        held, extra = generate(dict(collection, automatic_repeats=2), 'fresh_final_validation')
        offset = len(data['t']) - 1
        cycle_offset = len(meta['automatic_cycles'])
        for segment in held['segments']:
            segment['start'] += offset
            segment['stop'] += offset
            if segment['repeat'] >= 0:
                segment['repeat'] += cycle_offset
            if 'full_cycle' in segment:
                segment['full_cycle'] += cycle_offset
        for cycle in held['automatic_cycles']:
            for key in ('motion_start_index', 'motion_stop_index'):
                cycle[key] += offset
            cycle['cycle_id'] += cycle_offset
        for key in ('q', 'v', 'a'):
            data[key] = np.vstack([data[key][:-1], extra[key]])
        data['t'] = np.arange(len(data['q'])) * collection['sample_period_s']
        meta['segments'].extend(held['segments'])
        meta['automatic_cycles'].extend(held['automatic_cycles'])
        meta['expected_paired_cycles'] += held['expected_paired_cycles']
        meta['duration_s'] = float(data['t'][-1])
        meta['motion_limits_rad'] = np.maximum(meta['motion_limits_rad'], np.max(abs(data['q']), axis=0)).tolist()
    meta['validation_groups'] = [p['name'] for p in conditions if p['role'] == 'fresh_final_validation']
    meta['validation_requested'] = bool(validate)
    meta['validation_scope'] = 'Reserved configuration prediction; no motor release or passive stability qualification'
    return meta, data


def prepare_session(config_path, output_dir, urdf_text, baseline_path, *, validate=False, previous=None, joints=None,
                    device_config=None):
    """Return ``(metadata, trajectory)`` and publish a reviewed ingestable plan.

    The collector separately checks the actual starting pose and slowly homes to
    zero. The fixed trajectory needs an exact cached or fresh geometry review.
    Existing empty output directories are accepted; existing plans are immutable.
    """
    settings, collection, conditions, coverage = load_schedule(config_path, previous=previous,
        validate=validate, joints=joints, device_config=device_config)
    identity = dict(robot_unit=collection['robot_unit'], robot_fingerprint=fingerprint(urdf_text),
                    baseline_sha256=sha(baseline_path))
    check_partial_identity(previous, identity, settings['joints'])
    out = Path(output_dir).expanduser().resolve()
    if (out / 'plan.json').exists() or (out / 'trajectory.npz').exists():
        raise ValueError('Use a new calibration output directory')
    out.mkdir(parents=True, exist_ok=True)
    urdf_path = out / 'robot.urdf'
    urdf_path.write_text(urdf_text)
    print(f"PREPARING joints={settings['joints']}: fixed collection trajectory (motors not started)", flush=True)
    meta, data = _trajectory(collection, conditions, validate, fingerprint(urdf_text))
    budget = settings['max_collection_minutes']
    if not np.isfinite(budget) or budget <= 0 or meta['duration_s'] > budget * 60:
        raise ValueError(f"Collection needs {meta['duration_s'] / 60:.1f} minutes; no requested measurements were removed")
    validate_urdf_plan(meta, data, urdf_text)
    geometry, cache = review_geometry(data, urdf_path, checker=check_geometry)
    if not geometry['passed']:
        (out / 'geometry_failure.json').write_text(json.dumps(geometry, indent=2))
        raise ValueError('Collection geometry review failed; no executable plan published')
    np.savez_compressed(out / 'trajectory.npz', **data)
    meta.update(sha256=sha(out / 'trajectory.npz'), source_hashes=sources(),
        automatic_generator_sha256=sha(automatic_motion.__file__),
        portable_generator_sha256=sha(__file__), robot_fingerprint=fingerprint(urdf_text),
        baseline_sha256=sha(baseline_path), reuse_cycles_sha256=content_hash([]),
        automatic_candidate_sha256=None, portable_calibration=True,
        planned_support=coverage, selected_joints=settings['joints'], hardware_started=False)
    (out / 'plan.json').write_text(json.dumps(meta, indent=2, allow_nan=False))
    review = dict(plan_sha256=sha(out / 'plan.json'), trajectory_sha256=meta['sha256'],
        source_hashes=meta['source_hashes'], automatic_generator_sha256=meta['automatic_generator_sha256'],
        portable_generator_sha256=meta['portable_generator_sha256'], robot_fingerprint=meta['robot_fingerprint'],
        geometry=geometry, geometry_cache=cache, duration_s=meta['duration_s'], includes_ready_homing_and_completion=False,
        actual_start_homing_review_required=True, hardware_started=False, compensation_qualified=False)
    for name, value in [('motion_review.json', review), ('conditions.json', conditions),
                        ('preparation.json', dict(duration_minutes=meta['duration_s'] / 60,
                            planned_support=coverage, selected_joints=settings['joints'], validation_requested=bool(validate), geometry_cache=cache,
                            homing_and_fitting_additional=True)), ('cycles.json', [])]:
        (out / name).write_text(json.dumps(value, indent=2, allow_nan=False))
    (out / 'config.yaml').write_text(yaml.safe_dump(collection, sort_keys=False))
    (out / 'fitting.yaml').write_text(Path(settings['fitting']).read_text())
    return meta, data
