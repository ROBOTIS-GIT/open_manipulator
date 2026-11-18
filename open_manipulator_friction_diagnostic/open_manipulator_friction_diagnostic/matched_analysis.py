"""Prespecified fitting window and angle/load-proxy matched diagnostics.

Half-sum effort is a load PROXY: directional friction asymmetry can enter it.
Matching this proxy does not establish equal physical load or full configuration.
Validation never selects the fitting window, parameters or acceptance thresholds.
"""
import numpy as np


def integral_mean(x, y):
    return float(np.sum(np.diff(x)*(y[:-1]+y[1:])/2)/(x[-1]-x[0]))


def crop(row, low, high):
    x = np.asarray(row['position_rad'])
    if high <= low or x[0] > low or x[-1] < high or np.sum((x >= low) & (x <= high)) < 20:
        raise ValueError('Prespecified fitting angle window is not covered by measured positions')
    grid = np.unique(np.r_[low, x[(x > low) & (x < high)], high])
    f = np.interp(grid, x, row['friction_samples_nm'])
    load = np.interp(grid, x, row['load_samples_nm'])
    return dict(row, position_rad=grid.tolist(), friction_samples_nm=f.tolist(),
                load_samples_nm=load.tolist(), friction_nm=integral_mean(grid, f),
                mean_load_nm=integral_mean(grid, load), minimum_friction_nm=float(min(f)),
                maximum_friction_nm=float(max(f)), prespecified_angle_window_rad=[low, high])


def training_window(row, c):
    center = c['calibration_centers_rad'][row['joint']][row['joint']]
    half = c['fit_angle_half_width_rad']
    return crop(row, center-half, center+half)


def compare_rows(reference, observed, c):
    x, y = np.asarray(reference['position_rad']), np.asarray(observed['position_rad'])
    low, high = max(x[0], y[0]), min(x[-1], y[-1])
    result = dict(joint=observed['joint']+1, repeat=observed['repeat'],
                  validation_pose=observed['validation_pose'], speed_rad_s=observed['speed'],
                  reference_speed_rad_s=reference['speed'],
                  whole_reference_mean_nm=reference['friction_nm'],
                  whole_observed_mean_nm=observed['friction_nm'],
                  angle_overlap_rad=None, load_proxy_matched_fraction=0.,
                  comparable=False, samples_used_for_fit=False)
    if high <= low or min(np.sum((x >= low) & (x <= high)), np.sum((y >= low) & (y <= high))) < 20:
        return dict(result, reason='No adequate common active-joint angle interval')
    grid = np.unique(np.r_[low, x[(x > low) & (x < high)], y[(y > low) & (y < high)], high])
    rf, of = [np.interp(grid, r['position_rad'], r['friction_samples_nm']) for r in (reference, observed)]
    rl, ol = [np.interp(grid, r['position_rad'], r['load_samples_nm']) for r in (reference, observed)]
    load_limit = c.get('matched_load_absolute_nm', .25)+c.get('matched_load_relative', .1)*np.minimum(rl, ol)
    matched = abs(rl-ol) <= load_limit
    # Count only intervals whose BOTH endpoints match. Do not integrate across
    # gaps or treat the surviving knots as one contiguous interval.
    weights = np.diff(grid)*(matched[:-1] & matched[1:])
    length = float(np.sum(weights)); fraction = float(np.clip(length/(high-low), 0., 1.))
    def mean(values):
        return float(weights@((values[:-1]+values[1:])/2)/length) if length > 0 else None
    return dict(result, angle_overlap_rad=[float(low), float(high)],
                angle_matched_reference_mean_nm=integral_mean(grid, rf),
                angle_matched_observed_mean_nm=integral_mean(grid, of),
                angle_matched_reference_load_proxy_nm=integral_mean(grid, rl),
                angle_matched_observed_load_proxy_nm=integral_mean(grid, ol),
                load_proxy_matched_fraction=float(fraction),
                angle_and_load_matched_reference_mean_nm=mean(rf),
                angle_and_load_matched_observed_mean_nm=mean(of), comparable=bool(fraction >= .8),
                reason='Active angle and load proxy only; other joint positions may differ',
                full_configuration_matched=False, physical_load_equality_proven=False)


def diagnostics(base_pairs, heldout_pairs, c):
    train = [r for r in base_pairs if r['repeat'] == 0 and not r['validation_pose']]
    tests = [r for r in base_pairs if r['repeat'] > 0]+heldout_pairs
    comparisons = [compare_rows(next(t for t in train if t['joint'] == r['joint'] and
                      t['speed'] == r['speed']), r, c) for r in tests]
    velocity_load = []
    if c.get('sweep_protocol') == 'matched_v2':
        for j in range(6):
            rows = sorted([training_window(r, c) for r in train if r['joint'] == j], key=lambda r:r['speed'])
            velocity_load.extend(compare_rows(rows[0], r, c) for r in rows[1:])
    return dict(method='same_active_angle_then_load_proxy_v1', comparisons=comparisons,
                training_speed_load_comparisons=velocity_load,
                acceptance_overridden=False, validation_used_for_refit=False,
                load_proxy='absolute half-sum of equal-position opposite-direction effort; may include friction asymmetry',
                interpretation='Differences without sufficient overlap cannot be attributed solely to posture or speed. '
                               'Matching is diagnostic; full-range model errors still count as failures.')
