"""Fit only observable paired friction. Keep baseline gravity and motor constants fixed."""
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

from .protocol import JOINTS, sha

ESTIMATOR = 'paired_spatial_mean_v2'


def spatial_weights(position):
    """Trapezoid quadrature weights, not independent statistical sample weights."""
    dx = np.diff(position)
    return np.r_[dx[0], dx[:-1]+dx[1:], dx[-1]]/(2*np.sum(dx))


def weighted_quantile(values, weights, fraction):
    order = np.argsort(values)
    return float(np.interp(fraction, np.cumsum(weights[order]), np.asarray(values)[order]))


def magnitude(v, p, load=0.):
    fc, fs, vs, viscous = p[:4]
    load_gain = p[4] if len(p) > 4 else 0.
    return fc+(fs-fc)*np.exp(-(np.abs(v)/vs)**2)+viscous*np.abs(v)+load_gain*np.abs(load)


def validate_log(meta, data):
    t = np.asarray(data['t'])
    if len(t) < 100 or t.ndim != 1 or any(np.shape(data[k]) != (len(t), 6) for k in ('q', 'v', 'effort')):
        raise ValueError('Incomplete measured position/velocity/joint-equivalent effort arrays')
    if any(not np.isfinite(data[k]).all() for k in ('t', 'q', 'v', 'effort')):
        raise ValueError('Nonfinite measurements')
    if t[0] > .1 or t[-1] < meta['duration_s']-.1 or t[-1] > meta['duration_s']+.2:
        raise ValueError('Incomplete or mismatched collection')
    from .acquisition_timing import AcquisitionTiming
    AcquisitionTiming(meta).check(t)
    if np.any(np.max(abs(data['q']), axis=0) > np.asarray(meta.get('motion_limits_rad', meta['config']['joint_excursion_rad']))+.02):
        raise ValueError('Measured excursion outside protocol')


def pair_samples(meta, data, include_directional=False):
    t, q, v, effort = (data[k] for k in ('t', 'q', 'v', 'effort'))
    c = meta['config']; dt = c['sample_period_s']
    # The same joint position occurs at different sensor-history times in each direction.
    # Forward filtering q approximately aligns slowly varying gravity with filtered effort.
    # This is exact for affine gravity at constant speed, approximate for nonlinear gravity;
    # paired spread and independent repeats still have to pass. No inverse current filter.
    q_aligned = np.empty_like(q)
    sample_dt = np.median(np.diff(t))
    uniform = np.arange(t[0], t[-1]+sample_dt/2, sample_dt)
    for j in range(6):
        tau = c['current_filter_time_constant_s'][j]
        delay = c['current_measurement_delay_s'][j]
        if tau == 0:
            # Zero timing must be an identity. Resampling to a uniform clock and
            # back can split identical encoder counts into fictitious positions.
            q_aligned[:, j] = q[:, j] if delay == 0 else np.interp(t-delay, t, q[:, j])
        else:
            position = np.interp(uniform, t, q[:, j])
            decay = np.exp(-sample_dt/tau)
            position, _ = lfilter([1-decay], [1, -decay], position, zi=[decay*position[0]])
            q_aligned[:, j] = np.interp(t-delay, uniform, position)
    grouped = {}; starts = []
    for s in meta['segments']:
        j = s['joint']
        if j < 0:
            continue
        begin, end = s['start']*dt, s['stop']*dt
        if s['event'] == 'start':
            # A filtered sensor peak is an observation, not an identified static threshold.
            mask = (t >= begin-.1) & (t <= end+c['sensor_settling_s'][j])
            before = (t >= begin-.5) & (t < begin-.1)
            moving = np.flatnonzero(mask & (abs(v[:, j]) > c['velocity_deadband_rad_s']))
            starts.append(dict(joint=j, repeat=s['repeat'], direction=int(np.sign(s['speed_rad_s'])),
                               validation_pose=s.get('validation_pose', ''),
                               speed_rad_s=abs(s['speed_rad_s']),
                               onset_s=float(t[moving[0]]-begin) if len(moving) else None,
                               observed_effort_peak_nm=float(np.max(np.sign(s['speed_rad_s'])*effort[mask, j])),
                               resting_effort_nm=float(np.median(effort[before, j])) if np.any(before) else None,
                               static_threshold_qualified=False,
                               reason='Filtered motor effort includes gravity/inertia; no deconvolved breakaway claim'))
        if s['event'] != 'steady':
            continue
        mask = (t >= begin+c['sensor_settling_s'][j]) & (t < end-.05)
        others = [k for k in range(6) if k != j]
        if np.count_nonzero(mask) < 20:
            raise ValueError(f'joint{j+1}: inadequate settled measured samples')
        # Instantaneous velocity gating selects a phase of the position servo's
        # oscillation and therefore biases its correlated current. Check motion
        # over the complete settled window instead; retain every current sample.
        measured_q, measured_v = q[mask], v[mask]
        elapsed = t[mask]-t[mask][0]
        mean_speed = float(np.polyfit(elapsed, measured_q[:, j], 1)[0])
        direction = int(np.sign(s['speed_rad_s']))
        travel = direction*np.diff(measured_q[:, j])
        reverse_fraction = float(-np.sum(np.minimum(travel, 0))/max(np.sum(abs(travel)), 1e-12))
        speed_tolerance = max(.0001 if c.get('sweep_protocol') == 'matched_v2' else .001,
                              abs(s['speed_rad_s'])*.1)
        if (abs(mean_speed-s['speed_rad_s']) > speed_tolerance
                or reverse_fraction > .01
                or np.max(np.ptp(measured_q[:, others], axis=0)) > .003
                or np.max(abs(np.mean(measured_v[:, others], axis=0))) > .003):
            raise ValueError(f'joint{j+1}: inconsistent settled motion (mean speed {mean_speed:.6f}, reverse fraction {reverse_fraction:.3f})')
        # Matching actual position, not commanded time, cancels position-dependent gravity.
        x, inverse, counts = np.unique(q_aligned[mask, j], return_inverse=True, return_counts=True)
        if len(x) < 20:
            raise ValueError(f'joint{j+1}: inadequate distinct settled positions')
        # Quantized positions repeat. Average all their effort readings, not an
        # arbitrary first item from an unstable sort.
        current = np.bincount(inverse, weights=effort[mask, j])/counts
        raw_positions = np.unique(measured_q[:, j])
        diagnostic = dict(samples=int(np.sum(mask)), unique_positions=len(x),
                          raw_angular_support=dict(angle_interval_rad=[float(raw_positions[0]),float(raw_positions[-1])],
                              max_gap_rad=float(np.max(np.diff(raw_positions))),unique_positions=len(raw_positions)),
                          measured_mean_speed_rad_s=mean_speed, reverse_travel_fraction=reverse_fraction,
                          legacy_velocity_gate_retained_fraction=float(np.mean(
                              abs(measured_v[:, j]-s['speed_rad_s']) <= max(.001, abs(s['speed_rad_s'])*.1))))
        group = grouped.setdefault((j, s['repeat'], abs(s['speed_rad_s']), s.get('validation_pose', '')), {})
        if direction in group:
            raise ValueError('Duplicate direction segment')
        positions = None
        if include_directional:
            positions = np.column_stack([
                np.bincount(inverse, weights=q[mask, k])/counts for k in range(6)])
        group[direction] = (x, current, diagnostic, positions)
    pairs = []
    for (j, repeat, speed, pose), dirs in grouped.items():
        if set(dirs) != {-1, 1}:
            raise ValueError('Missing reverse-direction segment')
        xp, ep, dp, qp = dirs[1]; xn, en, dn, qn = dirs[-1]
        low, high = max(xp[0], xn[0]), min(xp[-1], xn[-1])
        if high-low < speed*.25:
            raise ValueError('Too little equal-position overlap after sensor settling')
        # Integrate over the union of both traces' knots. A fixed 80-point grid
        # can alias repeatable spatial ripple and miss a local minimum.
        grid = np.unique(np.r_[low, xp[(xp > low) & (xp < high)], xn[(xn > low) & (xn < high)], high])
        positive, negative = np.interp(grid, xp, ep), np.interp(grid, xn, en)
        raw_support = {}
        for name, diagnostic in (('positive', dp), ('negative', dn)):
            # Filtering can fill encoder gaps too. Retain the unfiltered
            # sweep's worst gap, with support limited to the paired overlap.
            source = diagnostic['raw_angular_support']
            raw_support[name] = dict(source, angle_interval_rad=[
                max(float(low),source['angle_interval_rad'][0]),
                min(float(high),source['angle_interval_rad'][1])])
        f = .5*(positive-negative)
        load = abs(.5*(positive+negative))
        weights = spatial_weights(grid)
        mean = float(weights@f)
        median = weighted_quantile(f, weights, .5)
        spread = weighted_quantile(abs(f-mean), weights, .95)
        if np.min(f) <= 0:
            raise ValueError(f'joint{j+1}: invalid friction sign or inconsistent pair (spread {spread:.3f} Nm)')
        pairs.append(dict(joint=j, repeat=repeat, speed=speed, validation_pose=pose,
                          friction_nm=mean, friction_median_nm=median, spread_nm=spread,
                          minimum_friction_nm=float(np.min(f)), maximum_friction_nm=float(np.max(f)),
                          mean_load_nm=float(weights@load), direction_diagnostics={'positive': dp, 'negative': dn},
                          position_rad=grid.tolist(), friction_samples_nm=f.tolist(), load_samples_nm=load.tolist(),
                          raw_angular_support=raw_support))
        if include_directional:
            # Retain measured directions and full configurations. Half-sum is
            # never used as a runtime load feature or a ground-truth load sensor.
            q_positive = np.column_stack([np.interp(grid, xp, qp[:, k]) for k in range(6)])
            q_negative = np.column_stack([np.interp(grid, xn, qn[:, k]) for k in range(6)])
            pairs[-1].update(positive_effort_samples_nm=positive.tolist(),
                             negative_effort_samples_nm=negative.tolist(),
                             positive_q_rad=q_positive.tolist(), negative_q_rad=q_negative.tolist())
    return pairs, starts


def identify(meta, data, select_velocity_model=False):
    validate_log(meta, data)
    pairs, starts = pair_samples(meta, data)
    c = meta['config']; models, checks, load_bounds = [], [], []
    use_load = c['friction_model'] == 'velocity_load'
    for j in range(6):
        train = [r for r in pairs if r['joint'] == j and r['repeat'] == 0 and not r['validation_pose']]
        full_train = train
        if c.get('sweep_protocol') == 'matched_v2':
            from .matched_analysis import training_window
            train = [training_window(r, c) for r in train]
        val = [r for r in pairs if r['joint'] == j and r['repeat'] > 0]
        for repeat in range(c['passes']):
            observed = [r['speed'] for r in pairs if r['joint'] == j and r['repeat'] == repeat]
            if sorted(observed) != sorted(c['speeds_rad_s']):
                raise ValueError(f'joint{j+1}: incomplete identification/validation speeds')
        for pose in meta['posture_validation_poses']:
            observed = [r['speed'] for r in val if r['validation_pose'] == pose['name']]
            if sorted(observed) != sorted(c['posture_validation_speeds_rad_s']):
                raise ValueError(f'joint{j+1}: incomplete held-out posture {pose["name"]}')
        def arrays(rows):
            speed = np.concatenate([np.full(len(r['friction_samples_nm']), r['speed']) for r in rows])
            torque = np.concatenate([r['friction_samples_nm'] for r in rows])
            load = np.concatenate([r['load_samples_nm'] for r in rows])
            return speed, torque, load
        all_speed, all_torque, load = arrays(train)
        # A velocity-only curve estimates a cycle mean. Thousands of correlated
        # servo samples are not thousands of independent friction observations.
        speed = all_speed if use_load else np.array([r['speed'] for r in train])
        torque = all_torque if use_load else np.array([r['friction_nm'] for r in train])
        fit_load = load if use_load else np.array([r['mean_load_nm'] for r in train])
        observable = use_load and np.ptp(load) >= .05
        def unpack(z):
            return np.array([z[0], z[0]+z[1], z[2], z[3], z[4] if observable else 0.])
        solutions = []
        for vs in (.008, .02, .05):
            initial = [min(torque)*.8, max(torque)-min(torque)+.01, vs, .1]
            lower = [0, 0, .001, 0]; upper = [max(torque)*3, max(torque)*5, .2, max(torque)/min(speed)]
            if observable:
                initial += [.02]; lower += [0]; upper += [1.]
            solutions.append(least_squares(lambda z: magnitude(speed, unpack(z), fit_load)-torque,
                initial, bounds=(lower, upper), loss='soft_l1',
                f_scale=c['validation_absolute_error_nm'], max_nfev=3000))
        best = min(solutions, key=lambda s: s.cost); p = unpack(best.x)
        selection = None
        if select_velocity_model:
            if use_load:
                raise ValueError('Nested velocity model selection cannot identify a load coefficient')
            from .curve_selection import select_curve
            p, selection = select_curve(speed, torque, c['validation_absolute_error_nm'],
                                        c['validation_relative_error'])
        test_speed, test_torque, test_load = arrays(val)
        errors = abs(magnitude(test_speed, p, test_load)-test_torque)
        allowed = c['validation_absolute_error_nm']+c['validation_relative_error']*test_torque
        # Margins cover sensor noise and small baseline gravity-model mismatch,
        # not arbitrary extrapolation to unmeasured payloads.
        margin = max(.10, .10*np.ptp(load))
        bounds = [max(0., float(min(load)-margin)), float(max(load)+margin)]
        coverage = bool(np.all((test_load >= bounds[0]) & (test_load <= bounds[1])))
        def mean_check(row):
            error = float(magnitude(row['speed'], p, row['mean_load_nm'])-row['friction_nm'])
            return dict(speed=row['speed'], repeat=row['repeat'], validation_pose=row['validation_pose'],
                        mean_error_nm=error, passed=bool(abs(error) <=
                            c['validation_absolute_error_nm']+c['validation_relative_error']*row['friction_nm']))
        train_means = [mean_check(row) for row in train]
        full_train_means = [mean_check(row) for row in full_train]
        val_means = [mean_check(row) for row in val]
        repeat_checks = []
        for row in val:
            if row['validation_pose']:
                continue
            reference = next(x for x in full_train if x['speed'] == row['speed'])
            difference = row['friction_nm']-reference['friction_nm']
            # Smaller observed mean sets the relative tolerance; no validation refit.
            repeat_checks.append(dict(speed=row['speed'], repeat=row['repeat'], mean_difference_nm=difference,
                passed=bool(abs(difference) <= c['validation_absolute_error_nm']+
                            c['validation_relative_error']*min(row['friction_nm'], reference['friction_nm']))))
        mean_passed = all(x['passed'] for x in train_means+full_train_means+val_means+repeat_checks)
        per_cycle = []
        for row in val:
            residual = abs(magnitude(row['speed'], p, np.array(row['load_samples_nm']))-row['friction_samples_nm'])
            per_cycle.append(dict(speed=row['speed'], repeat=row['repeat'],
                                  validation_pose=row['validation_pose'],
                                  residual_95_nm=weighted_quantile(residual, spatial_weights(row['position_rad']), .95)))
        posture_checks = []
        for pose in meta['posture_validation_poses']:
            rows = [r for r in val if r['validation_pose'] == pose['name']]
            pv, pf, pl = arrays(rows)
            residual = magnitude(pv, p, pl)-pf
            ok = (np.all(abs(residual) <= c['validation_absolute_error_nm']+c['validation_relative_error']*pf)
                  and max(x['residual_95_nm'] for x in per_cycle if x['validation_pose'] == pose['name']) <= c['max_pair_spread_nm'])
            posture_checks.append(dict(name=pose['name'], passed=bool(ok),
                max_error_nm=float(max(abs(residual))), rmse_nm=float(np.sqrt(np.mean(residual**2))),
                mean_residual_nm=float(np.mean(residual)), samples_used_for_fit=False))
        passed = bool(mean_passed and np.all(errors <= allowed) and (coverage or not use_load) and
                      max(r['residual_95_nm'] for r in per_cycle) <= c['max_pair_spread_nm'])
        checks.append(dict(joint=JOINTS[j], passed=passed, model_selection=selection,
                           fitting_angle_window_rad=train[0].get('prespecified_angle_window_rad'),
                           full_range_training_mean_checks=full_train_means,
                           mean_model_passed=bool(mean_passed), training_mean_checks=train_means,
                           validation_mean_checks=val_means, repeat_mean_checks=repeat_checks,
                           validation_mean_max_error_nm=max(abs(x['mean_error_nm']) for x in val_means),
                           validation_rmse_nm=float(np.sqrt(np.mean(errors**2))),
                           validation_max_error_nm=float(max(errors)),
                           validation_load_covered=coverage, load_bounds_nm=bounds,
                           load_coverage_required=use_load, posture_checks=posture_checks,
                           action_if_failed='Inspect sensor timing, payload and residuals; selectively expand excitation or qualify a load model. No automatic fallback.',
                           load_coefficient_identifiable=bool(observable), cycles=per_cycle,
                           independent_repeat_validation=True,
                           unseen_posture_validation=bool(posture_checks) and all(x['passed'] for x in posture_checks),
                           repeat_changed_posture=not np.allclose(
                               c['calibration_centers_rad'][j], c['validation_centers_rad'][j]),
                           fs_is_extrapolated=True,
                           direction_model='symmetric_pair_average'))
        models.append(p.tolist()); load_bounds.append(bounds)
    return dict(schema=3, estimator=ESTIMATOR, acceptance_policy='pointwise_residual_v1',
                mean_model_passed=all(x['mean_model_passed'] for x in checks),
                method='paired_load_stribeck' if use_load else 'paired_stribeck', parameters=models,
                load_compensation_enabled=use_load,
                training_scope=('prespecified central active-joint angle window, first upright/aligned pass only'
                    if c.get('sweep_protocol') == 'matched_v2' else 'first upright/aligned pass only'),
                posture_validation_used_for_fit=False,
                parameter_order=['fc_nm', 'fs_curve_nm', 'vs_rad_s', 'viscous_nm_s_rad', 'load_gain'],
                load_bounds_nm=load_bounds, pairs=pairs, start_events=starts, validation=checks,
                passed=all(x['passed'] for x in checks), temperature_model=False,
                static_breakaway_identified=False, rigid_parameters_modified=False,
                torque_conversion_modified=False, pose_whitelist=False,
                load_observation='absolute half-sum of equal-position opposite-direction effort',
                sensor_alignment='forward-filtered position; quasi-static gravity approximation')


def save_result(meta, data, folder, source, robot_fingerprint, baseline):
    folder = Path(folder)
    if meta.get('record_only') or (source == 'hardware' and
                                  meta['config'].get('sensor_timing_verified') is not True):
        raise ValueError('Record-only or unknown sensor timing: raw data cannot export a deployment profile')
    result = identify(meta, data)
    from .completion import check_completion_load
    result['completion_load_check'] = check_completion_load(
        meta['config'], folder/'robot.urdf', baseline, result['load_bounds_nm'])
    result['completion_load_check']['required_for_acceptance'] = result['load_compensation_enabled']
    result['passed'] = result['passed'] and (not result['load_compensation_enabled'] or result['completion_load_check']['passed'])
    (folder/'identification.json').write_text(json.dumps(result, indent=2))
    if not result['passed']:
        failed = [v['joint'] for v in result['validation'] if not v['passed']]
        raise ValueError(f'Held-out validation or required load coverage failed: {failed}; inspect identification.json; previous profile remains unchanged')
    c = meta['config']; models = np.array(result['parameters'])
    profile = dict(schema=3, source=source, robot_unit=c['robot_unit'], joints=JOINTS,
                   robot_fingerprint=robot_fingerprint, baseline_hash=sha(baseline),
                   plan_sha256=meta['sha256'], measurements_sha256=sha(folder/'measurements.npz'),
                   identification_sha256=sha(folder/'identification.json'),
                   effort_unit='joint_equivalent_Nm', method=result['method'],
                   hardware_freedrive_validated=False, static_breakaway_identified=False,
                   parameters=models.tolist(), config=c,
                   load_bounds_nm=result['load_bounds_nm'],
                   load_compensation_enabled=result['load_compensation_enabled'],
                   passed=True)
    # No profile is activated inside a running current controller. Separate launch loads it.
    temporary = folder/'profile.json.tmp'
    temporary.write_text(json.dumps(profile, indent=2)); temporary.replace(folder/'profile.json')
    return result
