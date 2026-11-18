"""Restrict legacy steady-state profiles to offline analysis and simulation.

These profiles do not qualify transient or closed-loop compensation behavior.
There is no hardware launch override for this legacy path.
"""


def require_simulation(mock):
    if not mock:
        raise ValueError(
            'Legacy steady-state profiles are restricted to offline analysis and '
            'simulation; transient and closed-loop compensation behavior is not '
            'qualified. No hardware launch override.')
