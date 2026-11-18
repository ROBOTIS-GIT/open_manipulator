"""Fresh reconstruction of the historical load/angle law for loaded joints.

One recipe and algorithm serve J2 and J3. First fit a directional speed/load
macro, refine observable load coordinates at each speed, then fit zero-mean
angular residuals. The existing positive periodic runtime evaluates the result.
No archived actuator coefficient, posture ID or dithering value enters fitting.
"""
from copy import deepcopy
import numpy as np
from scipy.optimize import nnls

from . import common_friction as common, periodic_friction as periodic
from .automatic_calibration import content_hash
from .kinetic_provenance import development_provenance
from .kinetic_review import review
from .kinetic_selection import selection_folds
from .kinetic_support import measured_support
from .single_joint_model import table_values


def _macro(cycles, joint, recipe):
    # Both alternatives use only this fit's data. This is an initializer for
    # unsupported load coordinates, never a previously identified actuator.
    candidates = []
    for family in ('smooth_load', 'signed_load'):
        try:
            m = common.fit(cycles, joint, family, recipe['macro_smoothing'],
                           recipe['load_width_nm'] if family == 'smooth_load' else None,
                           spatial=True)
            candidates.append((review(m, cycles)['score'], m))
        except ValueError:
            continue
    if not candidates:
        raise ValueError('No observable fresh load macro; additional load conditions are required')
    original = min(candidates, key=lambda item: item[0])[1]
    g = np.concatenate([c['signed_gravity_ros_nm'] for c in cycles])
    n = recipe['load_knots']
    if type(n) is not int or not 3 <= n <= 16 or np.ptp(g) < common.MIN_LOAD_SPAN_NM:
        raise ValueError('Invalid or unobservable load coordinates')
    knots = np.linspace(float(min(g)), float(max(g)), n)
    speeds = original['speed_knots_rad_s']
    tables = {d: np.column_stack([table_values(original, v, knots, d) for v in speeds])
              for d in ('positive', 'negative')}
    smoothing = recipe['load_smoothing']
    if not np.isfinite(smoothing) or smoothing < 0:
        raise ValueError('Invalid load smoothing')
    second = np.diff(np.eye(n), n=2, axis=0)
    rows = []
    for k, speed in enumerate(speeds):
        local = [c for c in cycles if c['speed'] == speed]
        x = np.vstack([common.weights(knots, c['signed_gravity_ros_nm']) / np.sqrt(len(c['q']))
                       for c in local])
        active = np.max(abs(x), axis=0) > 1e-10
        a = x[:, active]
        norms = np.linalg.norm(a, axis=0)
        normalized = a / norms
        rank = int(np.linalg.matrix_rank(normalized, tol=1e-7))
        condition = float(np.linalg.cond(normalized))
        observed = rank == a.shape[1] and condition < 100.
        row = dict(speed_rad_s=speed, rank=rank, active_coordinates=int(sum(active)),
                   condition_number=condition if np.isfinite(condition) else None,
                   refined=bool(observed), measured_load_range_nm=[float(min(v for c in local
                       for v in c['signed_gravity_ros_nm'])), float(max(v for c in local
                       for v in c['signed_gravity_ros_nm']))],
                   unobserved_coordinates_use='fresh_macro_only_not_measured_support')
        if observed:
            for direction in tables:
                y = np.concatenate([np.asarray(c[direction+'_resistance_proxy_nm']) /
                                    np.sqrt(len(c['q'])) for c in local])
                prior = tables[direction][:, k]
                # The observed coordinates are fitted, while unobserved ones
                # retain the fresh macro. Regularization never supplies rank.
                lhs = np.vstack([a, np.sqrt(smoothing)*second[:, active]])
                rhs = np.r_[y, -np.sqrt(smoothing)*second[:, ~active]@prior[~active]]
                value = nnls(lhs, rhs, maxiter=10000)[0]
                if k == 0 and np.any(value <= 0):
                    raise ValueError('Nonpositive first-speed load yield; do not silently floor fitted friction')
                tables[direction][active, k] = value
        rows.append(row)
    macro = deepcopy(original)
    macro.update(family='signed_load', gravity_knots_nm=knots.tolist(),
                 tables={d: v.tolist() for d, v in tables.items()}, soft_load_coefficients={},
                 load_width_nm=None, smoothing=recipe['macro_smoothing'])
    return macro, dict(initializer_family=original['family'], rows=rows,
        refined_speeds_rad_s=[r['speed_rad_s'] for r in rows if r['refined']],
        full_speed_load_rectangle_measured=False)


def _angular_cutoff(cycles, joint, frequency, orders, maximum):
    resolved = set()
    required_gap = 2*np.pi/(6*frequency*max(orders))
    for c in cycles:
        q = np.asarray(c['q'])[:, joint]
        raw = c.get('raw_angular_support', {})
        if (np.ptp(q) >= 2*np.pi/frequency and np.max(np.diff(q)) <= required_gap
                and all(d in raw and 0 < raw[d]['max_gap_rad'] <= required_gap
                        and np.ptp(raw[d]['angle_interval_rad']) >= 2*np.pi/frequency
                        for d in ('positive', 'negative'))):
            resolved.add(c['speed'])
    cutoff = None
    for speed in sorted({c['speed'] for c in cycles}):
        if speed > maximum or speed not in resolved:
            break
        cutoff = speed
    if cutoff is None:
        raise ValueError('Lowest speed needs a resolved full angular period for loaded-joint identification')
    return cutoff, sorted(resolved)


def fit(cycles, joint, recipe, *, frequency):
    common.check(cycles, joint)
    macro, refinement = _macro(cycles, joint, recipe)
    cutoff, resolved = _angular_cutoff(cycles, joint, frequency, recipe['orders'],
                                     recipe['angular_max_speed'])
    model = periodic.fit(cycles, joint, frequency=frequency, orders=recipe['orders'],
        ridge=recipe['ridge'], angular_max_speed=cutoff, load_width_nm=recipe['load_width_nm'],
        macro_model=macro, separable_speed_fit=True)
    model.update(fit_method='fresh_load_table_mean_preserving_periodic',
        identification_profile='load_table_periodic_v1', fit_source='fresh_measurements_only',
        load_refinement=refinement, rest_identification_included=False,
        measurement_support=measured_support(cycles, joint),
        angular_support=dict(requested_max_speed_rad_s=recipe['angular_max_speed'],
            fitted_max_speed_rad_s=cutoff, resolved_speeds_rad_s=resolved,
            kinetic_speed_knots_unchanged=True),
        missing=['fresh_validation', 'kinetic_release_coverage'])
    model['training_review'] = review(model, cycles)
    return model


def select(cycles, joint, recipe, fraction=.95, max_folds=3):
    groups, coverage = selection_folds(cycles, joint, max_folds)
    candidates, rejected = [], []
    for frequency in recipe['frequencies']:
        try:
            model = fit(cycles, joint, recipe, frequency=frequency)
        except (ValueError, RuntimeError) as exc:
            rejected.append(dict(frequency=frequency, reason=str(exc)))
            continue
        folds, unavailable = [], []
        for ids in groups:
            train = [c for c in cycles if c['cycle_id'] not in ids]
            held = [c for c in cycles if c['cycle_id'] in ids]
            try:
                local = fit(train, joint, recipe, frequency=frequency)
                folds.append(dict(training_cycle_ids=[c['cycle_id'] for c in train],
                    held_cycle_ids=ids, **review(local, held, fraction)))
            except (ValueError, RuntimeError) as exc:
                unavailable.append(dict(held_cycle_ids=ids, reason=str(exc)))
        if len(folds) < 2:
            rejected.append(dict(frequency=frequency, reason='Fewer than two trainable loaded configuration folds',
                                 unavailable=unavailable))
            continue
        model['posture_review'] = dict(means_passed=all(f['means_passed'] for f in folds),
            passed=not unavailable and all(f['spatial_passed'] for f in folds),
            checks=sum(len(f['rows']) for f in folds))
        model['selection'] = dict(method='loaded_joint_configuration_cv', folds=folds,
            score=float(np.sqrt(np.mean([f['score']**2 for f in folds]))), fresh_validation=False,
            unavailable_folds=unavailable)
        model['configuration_cv_coverage'] = dict(coverage,
            all_configurations_assessed=coverage['all_configurations_assessed'] and not unavailable)
        candidates.append(model)
    if not candidates:
        raise ValueError('No observable load/angle candidate: '+str(rejected))
    fitted = [m for m in candidates if m['training_review']['means_passed']]
    held = [m for m in fitted if m['posture_review']['means_passed']]
    chosen = min(held or fitted or candidates, key=lambda m:m['selection']['score'])
    chosen['fresh_selection'] = dict(method='loaded_joint_configuration_cv',
        recipe_sha256=content_hash(recipe), previous_coefficients_used=False,
        previous_support_used=False, rejected=rejected,
        candidates=[dict(frequency=m['frequency_rad_per_rad'], score=m['selection']['score'],
                        training_means_passed=m['training_review']['means_passed']) for m in candidates])
    if not chosen['posture_review']['means_passed']:
        chosen['missing'].append('unseen_configuration_mean_prediction')
    if not chosen['training_review']['spatial_passed'] or not chosen['posture_review']['passed']:
        chosen['missing'].append('spatial_prediction')
    if not chosen['configuration_cv_coverage']['all_configurations_assessed']:
        chosen['missing'].append('incomplete_configuration_cv')
    chosen.update(development_provenance(cycles))
    return chosen
