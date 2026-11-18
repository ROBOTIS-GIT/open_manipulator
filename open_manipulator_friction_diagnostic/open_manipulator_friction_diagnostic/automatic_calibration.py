"""Evidence-driven calibration: reuse, freeze, independent evaluation, missing work.

This is the identification side of the automatic pipeline. It has no motor or
controller interfaces. A current-derived model is never relabelled as measured
output torque, and a passed steady fit never qualifies a freedrive controller.
"""
import argparse
from copy import deepcopy
from contextlib import contextmanager
from contextvars import ContextVar
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import nnls


SCHEMA = 'automatic_friction_v1'
FAMILIES = ('velocity', 'signed_load', 'bearing_load')
SCALES = (.004, .012, .04)
POLICY = dict(absolute_error_nm=.08, relative_error=.20,
              minimum_spatial_pass_fraction=.95, matched_center_tolerance_rad=.01,
              matched_load_difference_nm=.25, minimum_groups=3,
              dither_peak_to_peak_counts=[2., 8.], dither_max_drift_counts=2.)

_cycle_hashes = ContextVar('immutable_calibration_cycle_hashes', default=None)
_PAYLOAD_FIELDS = ('joint', 'speed', 'q', 'features', 'target',
    'signed_gravity_ros_nm', 'positive_resistance_proxy_nm', 'negative_resistance_proxy_nm')


def _canonical_bytes(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()


@contextmanager
def immutable_cycle_hashes(cycles):
    """Cache exact provenance bytes for a private immutable fitting dataset.

    Callers own a deep copy and must not mutate it inside this scope. Arbitrary
    model dictionaries are never cached. Context-local storage isolates parallel
    fits and disappears before caller-visible inputs can change.
    """
    records = {id(c): (c, _canonical_bytes(c), hashlib.sha256(
        _canonical_bytes({k: c[k] for k in _PAYLOAD_FIELDS})).hexdigest()) for c in cycles}
    token = _cycle_hashes.set((records, {}))
    try:
        yield
    finally:
        _cycle_hashes.reset(token)


def content_hash(value):
    cache = _cycle_hashes.get()
    if cache is not None and isinstance(value, list):
        records, subsets = cache
        key = tuple(id(c) for c in value)
        if all(k in records and records[k][0] is c for k, c in zip(key, value)):
            if key not in subsets:
                digest = hashlib.sha256(b'[')
                for index, k in enumerate(key):
                    if index:
                        digest.update(b',')
                    digest.update(records[k][1])
                digest.update(b']')
                subsets[key] = digest.hexdigest()
            return subsets[key]
    return hashlib.sha256(_canonical_bytes(value)).hexdigest()


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_new(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def checked_cycles(cycles):
    """Validate complete paired cycles; bins are not independent repetitions."""
    if not isinstance(cycles, list) or not cycles:
        raise ValueError('No complete cycles')
    ids = set()
    for c in cycles:
        if c['cycle_id'] in ids:
            raise ValueError('Duplicate cycle ID')
        ids.add(c['cycle_id'])
        j, v = c['joint'], c['speed']
        if j not in range(6) or not np.isfinite(v) or v <= 0:
            raise ValueError('Invalid joint or speed')
        q = np.asarray(c['q'], dtype=float)
        n = len(q)
        if n < 8 or q.shape != (n, 6):
            raise ValueError('Missing spatial configurations')
        arrays = [q]
        for name in ('target', 'signed_gravity_ros_nm', 'positive_resistance_proxy_nm',
                     'negative_resistance_proxy_nm'):
            a = np.asarray(c[name], dtype=float)
            if a.shape != (n,):
                raise ValueError('Cycle field shape mismatch: '+name)
            arrays.append(a)
        features = np.asarray(c['features'], dtype=float)
        if features.shape != (n, 3) or np.any(features < 0):
            raise ValueError('Invalid nominal load features')
        arrays.append(features)
        if not all(np.isfinite(a).all() for a in arrays):
            raise ValueError('Nonfinite cycle evidence')
        if (min(c['target']) <= 0 or c.get('max_direction_configuration_difference_rad', 1.) > .003
                or not np.allclose(features[:, 0], abs(arrays[2]), atol=1e-7, rtol=1e-7)):
            raise ValueError('Unmatched directions or inconsistent load features')
        if not np.allclose(c['target'], (arrays[3]+arrays[4])/2, atol=1e-7, rtol=1e-7):
            raise ValueError('Directional residuals disagree with paired resistance')
        if c.get('human_contact') or c.get('failure'):
            raise ValueError('Contact/failed acquisition cannot identify friction')
    return cycles


def payload_hash(cycle):
    # Catch renamed/relabelled copies, not just matching cycle IDs.
    cache = _cycle_hashes.get()
    if cache is not None and id(cycle) in cache[0] and cache[0][id(cycle)][0] is cycle:
        return cache[0][id(cycle)][2]
    return content_hash({k: cycle[k] for k in _PAYLOAD_FIELDS})


def audit(cycles, joints=range(6)):
    checked_cycles(cycles)
    rows = []
    for j in joints:
        selected = [c for c in cycles if c['joint'] == j]
        matched = []
        for i, a in enumerate(selected):
            for b in selected[i+1:]:
                if a['group'] == b['group'] or not np.isclose(a['speed'], b['speed'], rtol=.01):
                    continue
                qa, qb = np.asarray(a['q'])[:, j], np.asarray(b['q'])[:, j]
                if (abs(qa.mean()-qb.mean()) > POLICY['matched_center_tolerance_rad'] or
                        min(max(qa), max(qb))-max(min(qa), min(qb)) <= 0):
                    continue
                load_delta = abs(np.mean(a['signed_gravity_ros_nm'])-np.mean(b['signed_gravity_ros_nm']))
                bearing_delta = abs(np.mean(np.asarray(a['features'])[:, 1])-
                                    np.mean(np.asarray(b['features'])[:, 1]))
                if max(load_delta, bearing_delta) >= POLICY['matched_load_difference_nm']:
                    matched.append([a['cycle_id'], b['cycle_id']])
        groups = sorted({c['group'] for c in selected})
        low = [c for c in selected if c['speed'] < .02]
        means = {g: float(np.mean([np.mean(c['target']) for c in low if c['group'] == g]))
                 for g in groups if any(c['group'] == g for c in low)}
        rows.append(dict(joint=j+1, cycles=len(selected), groups=groups,
            speed_range_rad_s=[min(c['speed'] for c in selected), max(c['speed'] for c in selected)] if selected else None,
            matched_angle_load_pairs=matched, low_speed_group_means_nm=means,
            low_speed_group_means_are_speed_and_history_confounded=True,
            missing=['fresh_validation_after_model_freeze', 'known_input_rest_and_reversal_response']+
                    ([] if matched else ['matched_angle_load_comparison'])))
    return dict(schema=SCHEMA, stage='REUSE_REVIEW', joints=rows,
        historical_data_role='development_only', measured_output_torque=False,
        automatic_recollection_started=False, hardware_deployable=False)


def matrix(cycle, family, scale):
    n = len(cycle['target']); v = cycle['speed']
    g = np.asarray(cycle['signed_gravity_ros_nm'])
    z = np.asarray(cycle['features'])
    cols = [np.ones(n), np.full(n, v), np.full(n, np.exp(-(v/scale)**2))]
    names = ['coulomb', 'viscous', 'low_speed_peak']
    if family != 'velocity':
        cols += [np.maximum(g, 0), np.maximum(-g, 0)]
        names += ['positive_nominal_gravity', 'negative_nominal_gravity']
    if family == 'bearing_load':
        cols += [z[:, 1]]
        names += ['nominal_bending_moment']
    return np.column_stack(cols), names


def fit_model(cycles, direction, family, scale):
    x = np.concatenate([matrix(c, family, scale)[0] for c in cycles])
    y = np.concatenate([c[direction+'_resistance_proxy_nm'] for c in cycles])
    if np.min(y) <= 0:
        raise ValueError('Directional residual not positive; separate gravity/conversion error first')
    # Each complete cycle has total weight one, independently of its bin count.
    w = np.concatenate([np.full(len(c['target']), 1/len(c['target'])) for c in cycles])
    norm = np.sqrt(np.average(x*x, weights=w, axis=0))
    active = norm > 1e-10
    xs = x[:, active]/norm[active]
    a = xs*np.sqrt(w[:, None]/sum(w)); b = y*np.sqrt(w/sum(w))
    # Rank is checked without the regularizer; a prior cannot create observations.
    rank = int(np.linalg.matrix_rank(a, tol=1e-7))
    coefficients, _ = nnls(np.vstack([a, 1e-4*np.eye(a.shape[1])]),
                           np.r_[b, np.zeros(a.shape[1])], maxiter=2000)
    result = np.zeros(x.shape[1]); result[active] = coefficients/norm[active]
    return dict(family=family, speed_scale_rad_s=scale, coefficients=result.tolist(),
        feature_names=matrix(cycles[0], family, scale)[1], direction=direction,
        rank=rank, active_columns=int(sum(active)), full_rank=bool(rank == sum(active)),
        zero_unobserved_columns=np.flatnonzero(~active).tolist(),
        coefficient_meaning='directional current residual, not independently measured friction torque')


def predict(model, cycle):
    return matrix(cycle, model['family'], model['speed_scale_rad_s'])[0]@model['coefficients']


def score(model, cycles):
    rows = []
    for c in cycles:
        y = np.asarray(c[model['direction']+'_resistance_proxy_nm'])
        e = predict(model, c)-y
        allowance = POLICY['absolute_error_nm']+POLICY['relative_error']*np.maximum(y, 0)
        fraction = float(np.mean(abs(e) <= allowance))
        rows.append(dict(cycle_id=c['cycle_id'], group=c['group'], direction=model['direction'],
            measured_proxy_mean_nm=float(y.mean()), error_mean_nm=float(e.mean()),
            rmse_nm=float(np.sqrt(np.mean(e*e))), normalized_mse=float(np.mean((e/allowance)**2)),
            max_overprediction_nm=float(max(0., max(e))), max_underprediction_nm=float(max(0., max(-e))),
            spatial_pass_fraction=fraction,
            passed=bool(abs(e.mean()) <= POLICY['absolute_error_nm']+POLICY['relative_error']*max(0., y.mean())
                        and fraction >= POLICY['minimum_spatial_pass_fraction'])))
    return dict(passed=bool(rows) and all(r['passed'] for r in rows), rows=rows,
        normalized_rmse=float(np.sqrt(np.mean([r['normalized_mse'] for r in rows]))) if rows else None)


def freeze(cycles, identity, joints=range(6)):
    checked_cycles(cycles)
    if set(identity) != {'robot_unit', 'robot_fingerprint', 'baseline_sha256'} or not all(identity.values()):
        raise ValueError('Robot, geometry and unchanged baseline identity required')
    models, selection, gaps = {}, {}, []
    for j in joints:
        rows = [c for c in cycles if c['joint'] == j]
        groups = sorted({c['group'] for c in rows})
        if len(groups) < POLICY['minimum_groups']:
            gaps.append(dict(joint=j+1, reason='Need three complete posture groups for grouped model selection'))
            continue
        for direction in ('positive', 'negative'):
            key = f'joint{j+1}/{direction}'
            candidates = []
            for family in FAMILIES:
                for scale in SCALES:
                    folds = []
                    for heldout in groups:
                        train = [c for c in rows if c['group'] != heldout]
                        test = [c for c in rows if c['group'] == heldout]
                        try:
                            model = fit_model(train, direction, family, scale)
                            folds.append(dict(group=heldout, full_rank=model['full_rank'], **score(model, test)))
                        except ValueError:
                            folds = []
                            break
                    if folds and all(f['full_rank'] for f in folds):
                        candidates.append(dict(family=family, scale=scale, folds=folds,
                            score=float(np.sqrt(np.mean([f['normalized_rmse']**2 for f in folds])))))
            if not candidates:
                gaps.append(dict(joint=j+1, direction=direction, reason='No observable positive directional model'))
                continue
            chosen = min(candidates, key=lambda a: (a['score'], FAMILIES.index(a['family']), a['scale']))
            models[key] = fit_model(rows, direction, chosen['family'], chosen['scale'])
            models[key]['speed_range_rad_s'] = [min(c['speed'] for c in rows), max(c['speed'] for c in rows)]
            models[key]['load_feature_range'] = [np.min(np.concatenate([np.column_stack(
                [c['signed_gravity_ros_nm'], np.asarray(c['features'])[:, 1]]) for c in rows]), axis=0).tolist(),
                np.max(np.concatenate([np.column_stack([c['signed_gravity_ros_nm'], np.asarray(c['features'])[:, 1]])
                                       for c in rows]), axis=0).tolist()]
            selection[key] = chosen
            if not all(f['passed'] for f in chosen['folds']):
                gaps.append(dict(joint=j+1, direction=direction,
                    reason='Whole-posture development validation failed; do not fix by scaling compensation'))
    return dict(schema=SCHEMA, stage='FROZEN', identity=identity, policy=deepcopy(POLICY),
        implementation_sha256=file_hash(__file__),
        frozen_utc=datetime.now(timezone.utc).isoformat(), models=models, selection=selection,
        joints=[j+1 for j in joints], missing=gaps,
        development_cycle_ids=[c['cycle_id'] for c in cycles],
        development_payload_hashes=[payload_hash(c) for c in cycles],
        cycles_sha256=content_hash(cycles), independent_final_validation_used=False,
        temperature_model=False, gravity_or_current_conversion_modified=False,
        physical_output_torque_identified=False, hardware_deployable=False)


def validate(frozen, cycles, reservation):
    """One frozen candidate, one separate reserved measurement set; never refit."""
    checked_cycles(cycles)
    if (frozen['schema'] != SCHEMA or frozen['policy'] != POLICY or
            frozen.get('implementation_sha256') != file_hash(__file__)):
        raise ValueError('Unknown frozen method/policy')
    if (reservation.get('candidate_sha256') != content_hash(frozen) or
            reservation.get('identity') != frozen['identity'] or
            reservation.get('purpose') != 'fresh_final_validation' or
            reservation.get('contact_free') is not True or
            reservation.get('completed') is not True):
        raise ValueError('Missing exact-candidate validation reservation/provenance')
    if (set(c['cycle_id'] for c in cycles) != set(reservation['cycle_ids']) or
            set(c['joint']+1 for c in cycles) != set(frozen['joints']) or
            set(c['cycle_id'] for c in cycles) & set(frozen['development_cycle_ids']) or
            set(payload_hash(c) for c in cycles) & set(frozen['development_payload_hashes'])):
        raise ValueError('Leaked, duplicate or missing final validation cycles')
    required = {(j, d) for j in frozen['joints'] for d in ('positive', 'negative')}
    checks = {}; gaps = deepcopy(frozen['missing'])
    for j, d in sorted(required):
        key = f'joint{j}/{d}'
        rows = [c for c in cycles if c['joint'] == j-1]
        if key not in frozen['models'] or len({c['group'] for c in rows}) < 2:
            gaps.append(dict(joint=j, reason='Missing candidate or two fresh validation load conditions'))
            continue
        m = frozen['models'][key]
        checks[key] = score(m, rows)
        # Report extrapolation as an evidence failure; no runtime clipping is installed.
        lo, hi = np.asarray(m['load_feature_range'])
        outside = []
        for c in rows:
            z = np.column_stack([c['signed_gravity_ros_nm'], np.asarray(c['features'])[:, 1]])
            if (c['speed'] < m['speed_range_rad_s'][0]-1e-9 or
                    c['speed'] > m['speed_range_rad_s'][1]+1e-9 or
                    np.any(z < lo-1e-7) or np.any(z > hi+1e-7)):
                outside.append(c['cycle_id'])
        checks[key]['outside_observed_marginal_bounds'] = outside
        if not checks[key]['passed'] or outside:
            gaps.append(dict(joint=j, direction=d, reason='Independent prediction or coverage failed'))
    return dict(schema=SCHEMA, stage='NEEDS_DATA' if gaps else 'STEADY_VALIDATED',
        steady_passed=not gaps, candidate_sha256=content_hash(frozen), checks=checks, missing=gaps,
        validation_payload_hashes=[payload_hash(c) for c in cycles],
        marginal_bounds_are_not_joint_support_certificate=True,
        next_required='known-input rest/start/reversal and closed-loop validation',
        hardware_deployable=False)


def harmonic(t, y, frequency):
    x = np.column_stack([np.sin(2*np.pi*frequency*t), np.cos(2*np.pi*frequency*t),
                         np.ones(len(t)), t-np.mean(t)])
    a, b, offset, drift = np.linalg.lstsq(x, y, rcond=None)[0]
    return dict(amplitude=float(np.hypot(a, b)), phase_rad=float(np.arctan2(b, a)),
                drift_per_s=float(drift), residual_rms=float(np.sqrt(np.mean((y-x@[a,b,offset,drift])**2))))


def assess_rest_trace(trace, encoder_step_rad, response_profile=None):
    """Known-input operational response; not physical stiffness or breakaway ID."""
    if response_profile not in (None, 'soft_start_v1'):
        raise ValueError('Unknown rest response profile')
    if (trace.get('contact_free') is not True or trace.get('input_kind') != 'known_sine'
            or trace.get('source') not in ('hardware', 'synthetic') or
            not np.isfinite(encoder_step_rad) or encoder_step_rad <= 0):
        raise ValueError('No-contact, known-input trace and encoder resolution required')
    t, q, cmd, current = [np.asarray(trace[k], float) for k in ('t', 'q', 'command_nm', 'current_proxy_nm')]
    if (t.ndim != 1 or len(t) < 100 or any(a.shape != t.shape for a in (q, cmd, current)) or
            not all(np.isfinite(a).all() for a in (t, q, cmd, current)) or
            min(np.diff(t)) <= 0 or max(np.diff(t)) > .02):
        raise ValueError('Invalid or stale rest trace')
    f, amplitude = trace['frequency_hz'], trace['amplitude_nm']
    if not np.isfinite([f, amplitude]).all() or f <= 0 or amplitude <= 0 or f*max(np.diff(t)) > .1:
        raise ValueError('Insufficient sampling for requested dither')
    # Ignore known ramp; a minimum of ten full cycles must remain.
    ramp = trace.get('ramp_s', .5)
    mask = (t >= t[0]+max(1., ramp)) & (t <= t[-1]-ramp)
    if sum(mask) < 100 or t[mask][-1]-t[mask][0] < 10/f:
        raise ValueError('Too short after dither ramp')
    t, q, cmd, current = [a[mask] for a in (t, q, cmd, current)]
    command, sensed, position = [harmonic(t-t[0], y, f) for y in (cmd, current, q)]
    if abs(command['amplitude']-amplitude) > max(.02, amplitude*.05):
        raise ValueError('Actual command differs from declared dither amplitude')
    pp = 2*position['amplitude']/encoder_step_rad
    drift = abs(position['drift_per_s'])*(t[-1]-t[0])/encoder_step_rad
    lo, hi = POLICY['dither_peak_to_peak_counts']
    residual_limit = 2.
    if response_profile == 'soft_start_v1':
        # Explicit comfort tradeoff: <=0.1 degree coherent peak-to-peak.
        # Non-sinusoidal friction can introduce small harmonics. Neither this
        # nor the legacy criterion identifies physical breakaway torque.
        hi = np.deg2rad(.1)/encoder_step_rad
        residual_limit = max(2., .1*pp)
    passed = (lo <= pp <= hi and drift <= POLICY['dither_max_drift_counts'] and
              position['residual_rms']/encoder_step_rad <= residual_limit)
    result = dict(passed=bool(passed), coherent_motion_peak_to_peak_counts=pp,
        drift_counts=float(drift), position_fit=position, command=command, current_proxy=sensed,
        current_to_command_amplitude_ratio=sensed['amplitude']/command['amplitude'],
        phase_is_combined_loop_and_sensor_response=True,
        encoder_response_is_not_output_breakaway_or_stiffness=True)
    if response_profile is not None:
        result.update(response_profile=response_profile, acceptance=dict(
            peak_to_peak_counts=[lo, float(hi)], max_drift_counts=POLICY['dither_max_drift_counts'],
            max_residual_rms_counts=float(residual_limit)))
    return result


def select_rest(traces, encoder_step_rad, required_conditions, response_profile=None,
                selection_policy=None):
    if selection_policy not in (None, 'fixed_compromise_v1', 'shorter_period_v1'):
        raise ValueError('Unknown rest selection policy')
    if selection_policy and response_profile != 'soft_start_v1':
        raise ValueError('Rest selection policy requires an explicit comfort response profile')
    if not traces:
        raise ValueError('No rest measurements')
    joint, identity, source = (traces[0].get(k) for k in ('joint', 'identity', 'source'))
    if (joint not in range(1, 7) or not identity or
            any((t.get('joint'), t.get('identity'), t.get('source')) != (joint, identity, source) for t in traces)):
        raise ValueError('Rest traces must share one joint, robot/baseline identity and source type')
    groups = {}; seen = set()
    for trace in traces:
        if trace['trace_id'] in seen:
            raise ValueError('Duplicate rest trace')
        seen.add(trace['trace_id'])
        k = (trace['frequency_hz'], trace['amplitude_nm'])
        groups.setdefault(k, []).append(dict(trace_id=trace['trace_id'], condition=trace['condition'],
            repeat=trace['repeat'], source=trace['source'],
            **assess_rest_trace(trace, encoder_step_rad, response_profile)))
    required = set(required_conditions)
    if len(required) < 2:
        raise ValueError('At least two specified load conditions required')
    candidates = []
    for (frequency, amplitude), rows in groups.items():
        coverage = all(len({r['repeat'] for r in rows if r['condition'] == condition}) >= 2
                       for condition in required)
        if len({(r['condition'], r['repeat']) for r in rows}) != len(rows):
            raise ValueError('Duplicate rest condition/repetition')
        passed = coverage and all(r['passed'] for r in rows)
        candidates.append(dict(frequency_hz=frequency, amplitude_nm=amplitude,
                               coverage=coverage, passed=passed, traces=rows))
        if selection_policy == 'fixed_compromise_v1':
            # Keep each failed lower-response result: only lack of encoder
            # motion may be traded off. Missing repeats, drift and excessive
            # vibration never become eligible through this selection mode.
            envelope_ok = all(
                r['coherent_motion_peak_to_peak_counts'] <= r['acceptance']['peak_to_peak_counts'][1]
                and r['drift_counts'] <= r['acceptance']['max_drift_counts']
                and r['position_fit']['residual_rms']/encoder_step_rad <= r['acceptance']['max_residual_rms_counts']
                for r in rows)
            responding = sorted(condition for condition in required
                if coverage and all(r['passed'] for r in rows if r['condition'] == condition))
            candidates[-1].update(responding_conditions=responding,
                unresponsive_conditions=sorted(required-set(responding)),
                fixed_compromise_eligible=bool(coverage and envelope_ok and responding))
    eligible = [c for c in candidates if c['passed']]
    # Smallest measured amplitude meeting the specified encoder criterion at
    # every load/repetition. This is controller selection, not a friction scale.
    selected = min(eligible, key=lambda c: (c['amplitude_nm'], c['frequency_hz'])) if eligible else None
    if selection_policy == 'shorter_period_v1':
        # An explicit alternative when the slower measured waveform causes
        # stick/slip in the combined release test. Never interpolate frequency
        # or waive any standalone response criterion. It still needs a new
        # complete combined-controller release sequence before handoff.
        selected = min(eligible, key=lambda c: (c['amplitude_nm'], -c['frequency_hz'])) if eligible else None
    if selection_policy == 'fixed_compromise_v1':
        eligible = [c for c in candidates if c['fixed_compromise_eligible']]
        selected = min(eligible, key=lambda c: (-len(c['responding_conditions']), c['amplitude_nm'],
            max(r['coherent_motion_peak_to_peak_counts'] for r in c['traces']),
            c['frequency_hz'])) if eligible else None
    result = dict(stage='REST_CANDIDATE' if selected else 'NEEDS_REST_DATA', selected=selected,
        joint=joint, identity=identity, source=source, trace_content_sha256=content_hash(traces),
        candidates=candidates, requirement_origin='Operational design criterion, not a universal paper constant',
        required_conditions=sorted(required), physical_breakaway_identified=False,
        mechanical_stiffness_identified=False, hardware_deployable=False,
        next_required='Fresh start/reversal/release validation of the exact combined controller')
    if response_profile is not None:
        result['response_profile'] = response_profile
    if selection_policy:
        result['selection_policy'] = selection_policy
        result['selection_objective'] = ('minimum fully passing measured amplitude, then shortest measured period'
            if selection_policy == 'shorter_period_v1' else
            'maximum repeatable condition coverage, then minimum amplitude, then minimum peak vibration')
        if selected and not selected['passed']:
            result['stage'] = 'REST_COMPROMISE_CANDIDATE'
    return result


def assemble_result(candidate, steady, rest):
    """Portable analysis result; deliberately not a runtime profile schema."""
    if steady.get('candidate_sha256') != content_hash(candidate):
        raise ValueError('Validation belongs to a different frozen model')
    settings = {}; missing = deepcopy(steady['missing'])
    for joint in candidate['joints']:
        key = f'joint{joint}'; r = rest.get(key)
        if r and (r.get('joint') != joint or r.get('identity') != candidate['identity']):
            raise ValueError('Rest response belongs to a different joint/robot/baseline')
        if not r or r.get('selected') is None:
            missing.append(dict(joint=joint, reason='Missing independently repeatable known-input rest response'))
            continue
        selection = r['selected']
        if not selection['passed']:
            missing.append(dict(joint=joint, reason='Fixed rest compromise leaves unresponsive measured conditions'))
        settings[key] = dict(dither_amplitude_nm=selection['amplitude_nm'],
            dither_frequency_hz=selection['frequency_hz'],
            origin='measured encoder response criterion; controller setting, not physical friction',
            response_evidence_sha256=content_hash(r),
            elastic_stiffness_nm_rad=None, decay_per_s=None,
            elastic_parameters_require_separate_design_and_combined_validation=True)
    return dict(schema='automatic_calibration_result_v1', identity=candidate['identity'],
        state='NEEDS_DATA' if missing else 'NEEDS_COMBINED_CONTROLLER_VALIDATION',
        model_parameters=candidate['models'], controller_candidates=settings,
        candidate_sha256=content_hash(candidate), steady_validation_sha256=content_hash(steady),
        missing=missing, model_coefficients_scaled=False,
        hardware_deployable=False, runtime_parameters=None,
        required_next=['known-input start/reversal/release tests of the combined controller',
                       'runtime adapter matching the validated model and exact parameters'],
        physical_output_torque_validated=False, static_breakaway_identified=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    for name in ('audit', 'freeze'):
        s = sub.add_parser(name); s.add_argument('--cycles', required=True)
        s.add_argument('--output', required=True); s.add_argument('--joints', default='1,2,3,4,5,6')
        if name == 'freeze': s.add_argument('--identity', required=True)
    s = sub.add_parser('validate')
    for key in ('candidate', 'cycles', 'reservation', 'output'): s.add_argument('--'+key, required=True)
    s = sub.add_parser('rest')
    for key in ('traces', 'output'): s.add_argument('--'+key, required=True)
    s.add_argument('--encoder-step-rad', type=float, required=True)
    s.add_argument('--conditions', required=True)
    s = sub.add_parser('result')
    for key in ('candidate', 'steady', 'rest', 'output'): s.add_argument('--'+key, required=True)
    args = p.parse_args()
    read = lambda name: json.loads(Path(getattr(args, name)).read_text())
    if args.command in ('audit', 'freeze'):
        joints = [int(j)-1 for j in args.joints.split(',')]
        if not joints or len(set(joints)) != len(joints) or any(j not in range(6) for j in joints):
            p.error('joints must be unique numbers in 1..6')
        result = audit(read('cycles'), joints) if args.command == 'audit' else freeze(read('cycles'), read('identity'), joints)
    elif args.command == 'validate':
        result = validate(read('candidate'), read('cycles'), read('reservation'))
    elif args.command == 'rest':
        result = select_rest(read('traces'), args.encoder_step_rad, args.conditions.split(','))
    else:
        result = assemble_result(read('candidate'), read('steady'), read('rest'))
    result['input_files_sha256'] = {k: file_hash(getattr(args, k)) for k in
        ('cycles', 'identity', 'candidate', 'reservation', 'traces', 'steady', 'rest') if hasattr(args, k)}
    write_new(args.output, result)
    print(json.dumps(dict(stage=result.get('stage', result.get('state')), output=args.output, hardware_deployable=False)))


if __name__ == '__main__':
    main()
