"""Explicit runtime experiments, separate from acquisition/model qualification."""
import numpy as np

from .trial_limits import limits_for_report

PRESETS = ('recorded', 'strong')


def strong_trial_settings(report, config):
    """More assistance for a supervised trial, without changing saved evidence.

    Limits come from the training pass; the held-out pass checks the reserve.
    This is not a validation of directional friction or of freedrive stability.
    """
    evidence = limits_for_report(report, config)
    caps = .6 * np.asarray(evidence['training_minimum_paired_friction_nm'])
    reserve = .75 * np.asarray(evidence['held_out_minimum_paired_friction_nm'])
    if not np.all(caps <= reserve):
        raise ValueError('Strong trial exceeds the held-out observed friction reserve')
    return dict(calibrated_gain=.6, calibrated_max_torque=caps.tolist(),
                calibrated_slew_rate=[10.] * 6)
