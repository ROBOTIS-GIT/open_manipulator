"""Fresh profile-based identification; old coefficients never enter a new fit.

The standard selection remains available on every axis. The configured loaded
joint group shares one load-table/angular recipe. Missing measurements remain
missing; neither old coefficients nor support are copied.
"""
from copy import deepcopy
from pathlib import Path
import time
from itertools import product

import numpy as np
import yaml

from .automatic_calibration import checked_cycles, content_hash, immutable_cycle_hashes
from .integration import original_gravity
from .kinetic_bank import compose
from .kinetic_review import review
from .kinetic_runtime import model_parameters, parameters
from .kinetic_selection import select, selection_folds
from .kinetic_support import measured_support


def _positive_measurements(cycles, minimum):
    checked_cycles(cycles)
    for c in cycles:
        for direction in ('positive', 'negative'):
            values = np.asarray(c[direction + '_resistance_proxy_nm'], dtype=float)
            if not np.isfinite(values).all() or float(np.mean(values)) <= minimum:
                raise ValueError('No positive measured directional resistance; effort cannot identify this physical model')


def _settings(fitting):
    if fitting is None:
        fitting = Path(__file__).resolve().parents[1] / 'config/fitting.yaml'
        if not fitting.is_file():
            from ament_index_python.packages import get_package_share_directory
            fitting = Path(get_package_share_directory('open_manipulator_friction_diagnostic')) / 'config/fitting.yaml'
    result = yaml.safe_load(Path(fitting).read_text()) if isinstance(fitting, (str, Path)) else deepcopy(fitting)
    for key in ('kinetic_fraction', 'transition_distance_rad', 'transition_stiffness_nm_rad',
                'transition_decay_per_s', 'minimum_positive_resistance_nm'):
        result[key] = float(result[key])
        if not np.isfinite(result[key]) or result[key] < 0:
            raise ValueError('Invalid fitting setting: ' + key)
    return result


def select_fresh(cycles, joint, settings):
    """Compare observable fresh macro laws and shared angular templates.

    A shared angular template needs a resolved full period in each training
    fold, rather than pretending to identify independent harmonics at every
    sparse speed/load combination. Its invariance assumption stays in the model.
    """
    from . import shared_periodic_friction as shared
    fraction = settings['kinetic_fraction']
    max_folds = settings['development_cv_folds']
    base = select(cycles, joint, (), fraction, max_folds=max_folds)
    candidates = [base]
    rejections = []
    groups, coverage = selection_folds(cycles, joint, max_folds)
    families = [('velocity', None)] if base['family'] == 'velocity' else []
    for family, width in families:
        for angular, smoothing, objective in product(settings['periodic_candidates'],
                settings['macro_smoothing_candidates'], settings['fit_objectives']):
            recipe = dict(angular, macro_family=family, macro_smoothing=smoothing,
                          load_width_nm=width, objective=objective)
            try:
                model = shared.fit(cycles, joint, recipe)
                training = review(model, cycles, fraction)
                folds = []
                for ids in groups:
                    train = [c for c in cycles if c['cycle_id'] not in ids]
                    held = [c for c in cycles if c['cycle_id'] in ids]
                    candidate = shared.fit(train, joint, recipe)
                    folds.append(dict(training_cycle_ids=[c['cycle_id'] for c in train], held_cycle_ids=ids,
                                      **review(candidate, held, fraction)))
                model.update(training_review=training,
                    posture_review=dict(means_passed=all(f['means_passed'] for f in folds),
                        passed=all(f['spatial_passed'] for f in folds), checks=sum(len(f['rows']) for f in folds)),
                    selection=dict(method='fresh_shared_template_configuration_cv', folds=folds,
                        score=float(np.sqrt(np.mean([f['score']**2 for f in folds]))), fresh_validation=False),
                    kinetic_protocol='common_kinetic_v2', measurement_support=measured_support(cycles, joint),
                    missing=['fresh_validation', 'kinetic_release_coverage'])
                candidates.append(model)
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                rejections.append(dict(recipe=recipe, reason=str(exc)))
    if base['family'] != 'velocity':
        gravity = np.concatenate([c['signed_gravity_ros_nm'] for c in cycles])
        # Dense near zero and coarser at large loads. These are interpolation
        # coordinates, never an expansion of measured gravity support.
        knots = .5 * np.sinh(np.linspace(np.arcsinh(min(gravity)/.5),
                                         np.arcsinh(max(gravity)/.5), 16))
        knots[0], knots[-1] = min(gravity), max(gravity)
        for frequency in sorted({a['frequency'] for a in settings['periodic_candidates']}):
            recipes = [dict(a, macro_family='shared_smooth_load', macro_smoothing=smoothing,
                            load_width_nm=width, objective=objective)
                       for a in settings['periodic_candidates'] if a['frequency'] == frequency
                       for width in settings['shared_template_load_widths_nm']
                       for smoothing in settings['macro_smoothing_candidates']
                       for objective in settings['fit_objectives']]
            try:
                model = shared.select(cycles, joint, recipes, knots.tolist(),
                    settings['shared_projection_error_nm'], fraction, max_folds=max_folds)
                model['selection']['score'] = float(np.sqrt(np.mean([
                    f['score']**2 for f in model['selection']['folds']])))
                candidates.append(model)
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                rejections.append(dict(method='independent_direction_shared_template',
                    frequency=frequency, reason=str(exc)))
    training_ok = [m for m in candidates if m['training_review']['means_passed']]
    held_ok = [m for m in training_ok if m['posture_review']['means_passed']]
    eligible = held_ok or training_ok or candidates
    best = min(m['selection']['score'] for m in eligible)
    # Base macro laws win an effectively tied comparison. The template must
    # improve held-configuration local prediction to justify its extra terms.
    chosen = next(m for m in eligible if m['selection']['score'] <= best + .01)
    chosen['fresh_selection'] = dict(method='fresh_macro_and_shared_angular_configuration_cv',
        previous_coefficients_used=False, previous_support_used=False,
        shared_template_rejections=rejections,
        candidates=[dict(family=m['family'], fit_method=m.get('fit_method'),
            score=m['selection']['score'], training_means_passed=m['training_review']['means_passed'],
            training_spatial_passed=m['training_review']['spatial_passed']) for m in candidates])
    chosen['fit_source'] = 'fresh_measurements_only'
    chosen['measurement_support'] = measured_support(cycles, joint)
    chosen['configuration_cv_coverage'] = coverage
    if not coverage['all_configurations_assessed'] and 'incomplete_configuration_cv' not in chosen['missing']:
        chosen['missing'].append('incomplete_configuration_cv')
    return chosen


def identify(cycles, identity, baseline, *, source='hardware', previous=None, fitting=None, validate=False, joints=None, progress=None):
    """Return an artifact fit only from this collection, for every requested axis.

    ``previous`` is accepted only for a comparison hash. No prior numeric value,
    basis, identity, controller setting or measurement support enters fitting.
    Reserved validation groups never participate in fitting.
    """
    fit_started = time.monotonic()
    if source not in ('hardware', 'gazebo'):
        raise ValueError('Calibration source must be hardware or gazebo')
    if set(identity) != {'robot_unit', 'robot_fingerprint', 'baseline_sha256'} or not all(identity.values()):
        raise ValueError('Robot and baseline identity required')
    held = [c for c in cycles if c['group'].startswith('validation_')]
    development = [c for c in cycles if not c['group'].startswith('validation_')]
    if validate and not held:
        raise ValueError('Validation requested without reserved measured configurations')
    settings = _settings(fitting)
    from .joint_selection import parse_joints
    selected = parse_joints('all' if joints is None else joints)
    original = original_gravity(baseline) if isinstance(baseline, (str, Path)) else deepcopy(baseline)
    models, failures, updated = {}, {}, []
    for index, joint in enumerate(selected, 1):
        name = f'joint{joint}'
        if progress:
            progress('FITTING_JOINT', joint=joint, index=index, total=len(selected))
        local = deepcopy([c for c in development if c['joint'] == joint - 1])
        try:
            _positive_measurements(local, settings['minimum_positive_resistance_nm'])
            with immutable_cycle_hashes(local):
                loaded = settings.get('loaded_joints', {})
                if joint in loaded.get('joints', []):
                    if loaded['method'] != 'load_table_periodic_v1':
                        raise ValueError('Unknown loaded-joint identification method')
                    from .loaded_joint_identification import select as select_loaded
                    model = select_loaded(local, joint - 1, loaded['recipe'],
                        settings['kinetic_fraction'], settings['development_cv_folds'])
                else:
                    model = select_fresh(local, joint - 1, settings)
            model['identity'] = deepcopy(identity)
            models[name] = model
            if not model['training_review']['means_passed']:
                failures[name] = 'Fresh mean resistance is not explained by an observable candidate; collect additional conditions'
            else:
                updated.append(joint)
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            failures[name] = str(exc)
        if progress:
            progress('JOINT_FIT_FINISHED', joint=joint, index=index, total=len(selected),
                     passed=name not in failures, failure=failures.get(name))
    validation = dict(requested=bool(validate), performed=False, passed=None,
                      scope='Reserved measured configuration prediction only')
    if validate:
        if progress:
            progress('VALIDATING', joints=selected)
        checks = {}
        for name, model in models.items():
            local = [c for c in held if c['joint'] + 1 == model['joint']]
            if not local:
                failures[name] = 'Missing reserved measurements for this joint'
                continue
            checks[name] = review(model, local, settings['kinetic_fraction'])
        validation.update(performed=bool(checks), per_joint=checks,
            passed=len(checks) == len(selected) and all(r['means_passed'] and r['spatial_passed'] for r in checks.values()),
            candidate_frozen_before_collection=False, automatic_release_test_performed=False)
    usable = len(updated) == len(selected) and not failures
    if validate:
        usable = usable and validation['passed']
    controller = {}
    if usable:
        independent = []
        for model in models.values():
            design = dict(model_parameters(model, model['joint']),
                transition_stiffness=settings['transition_stiffness_nm_rad'],
                transition_decay=settings['transition_decay_per_s'],
                common_transition_distance=settings['transition_distance_rad'])
            independent.append(parameters(model, design, original, settings['kinetic_fraction']))
        controller = compose(independent)
    return dict(schema='omy_friction_calibration_v1', source=source, robot_unit=identity['robot_unit'],
        identity=deepcopy(identity), joint_names=list(original['joints']), models=models,
        controller_parameters=controller, usable=bool(usable), failures=failures,
        selected_joints=selected, reidentified_joints=updated, retained_joints=[], new_identification_complete=len(updated) == len(selected),
        fit_source='fresh_measurements_only', fitting_duration_s=time.monotonic()-fit_started, prior_comparison_sha256=content_hash(previous) if previous else None,
        validation=validation, dither=dict(collected=False, independent=True),
        training_spatial_checks_passed=bool(models) and all(m['training_review']['spatial_passed'] for m in models.values()),
        hardware_qualified=False, automatic_deployment_allowed=False,
        hardware_replacement_allowed=source == 'hardware' and bool(usable),
        gravity_or_current_conversion_changed=False)
