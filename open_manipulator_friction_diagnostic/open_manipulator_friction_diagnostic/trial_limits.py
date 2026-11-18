"""Empirical limits for a supervised partial-assistance trial, not full cancellation.

Fixed engineering margins, never optimized against the held-out pass. The floor
is symmetric paired motor effort: it does not prove directional output friction,
passivity, unknown sensor dynamics, or behavior at untested poses/loads.
"""
import numpy as np

from .fit import ESTIMATOR

QUALIFICATION = 'experimental_zero_delay_bounded_mean_v1'
ACCEPTANCE = 'mean_repeat_and_observed_floor_v1'
GAIN_CEILING = .25
TRAINING_FLOOR_FRACTION = .25
HELD_OUT_FLOOR_FRACTION = .5


def limits_for_report(result, config):
    if (result.get('estimator') != ESTIMATOR or result.get('load_compensation_enabled')
            or config['friction_model'] != 'velocity_only'
            or any(r['validation_pose'] for r in result['pairs'])):
        raise ValueError('Bounded trial requires the spatial-mean velocity-only repeat estimator')
    training_floor, held_out_floor = [], []
    for joint in range(6):
        train = [r for r in result['pairs'] if r['joint'] == joint and r['repeat'] == 0]
        val = [r for r in result['pairs'] if r['joint'] == joint and r['repeat'] > 0]
        if not train or not val:
            raise ValueError('Missing independent paired trial evidence')
        for rows, floor in ((train, training_floor), (val, held_out_floor)):
            floor.append(float(min(np.min(r['friction_samples_nm']) for r in rows)))
    if not np.isfinite(training_floor+held_out_floor).all() or min(training_floor+held_out_floor) <= 0:
        raise ValueError('Trial needs strictly positive finite observed paired friction')
    # A global per-joint cap also bounds a previous, slewed command when speed
    # changes. A per-speed steady-state prediction would not bound that history.
    caps = np.minimum(config['max_compensation_nm'],
                      TRAINING_FLOOR_FRACTION*np.array(training_floor))
    reserve_passed = caps <= HELD_OUT_FLOOR_FRACTION*np.array(held_out_floor)
    return dict(gain=min(GAIN_CEILING, config['compensation_gain']),
                max_compensation_nm=caps.tolist(),
                training_minimum_paired_friction_nm=training_floor,
                held_out_minimum_paired_friction_nm=held_out_floor,
                training_floor_fraction=TRAINING_FLOOR_FRACTION,
                held_out_floor_fraction=HELD_OUT_FLOOR_FRACTION,
                observed_reserve_passed=reserve_passed.tolist(),
                limits_selected_from='first_pass_only',
                scope='observed_symmetric_pair_effort_only', passivity_proven=False)


def qualify_bounded_trial(result, config):
    """Keep strict accuracy visible; qualify only the separate bounded trial."""
    limits = limits_for_report(result, config)
    result['pointwise_model_passed'] = result['passed']
    result['acceptance_policy'] = ACCEPTANCE
    result['trial_runtime_limits'] = limits
    for j, check in enumerate(result['validation']):
        check['pointwise_model_passed'] = check['passed']
        check['observed_reserve_passed'] = limits['observed_reserve_passed'][j]
        check['passed'] = bool(check['mean_model_passed'] and check['observed_reserve_passed'])
    result['passed'] = all(x['passed'] for x in result['validation'])
    return result
