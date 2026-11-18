"""Derive a controller design from identified speed coverage, not a physical stiffness fit."""
import math


def lowest_speed_design(positive_yield, negative_yield, minimum_speed, decay):
    values = [positive_yield, negative_yield, minimum_speed, decay]
    if not all(math.isfinite(x) and x > 0 for x in values):
        raise ValueError('Positive finite friction levels, speed and decay are required')
    # Steady elastic output is min(K*|v|/alpha, F). At the lowest
    # measured knot it should reach the sourced yield, in both directions.
    stiffness = decay*max(positive_yield, negative_yield)/minimum_speed
    return dict(method='reach_yield_at_lowest_measured_speed_v1',
                stiffness_nm_rad=stiffness, decay_per_s=decay,
                minimum_measured_speed_rad_s=minimum_speed,
                positive_yield_nm=positive_yield, negative_yield_nm=negative_yield,
                physical_stiffness_identified=False,
                position_quantum_command_nm=stiffness*.000011984224905356572,
                stationary_95_percent_decay_s=-math.log(.05)/decay,
                hardware_qualified=False)
