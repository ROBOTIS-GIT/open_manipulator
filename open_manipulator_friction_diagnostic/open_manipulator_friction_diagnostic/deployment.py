"""Hardware hold following the 2026-10-27 uncontrolled-motion incident.

There is deliberately no launch override. Offline analysis and isolated mocks
remain available. Removing this hold requires a reviewed deployment path with
transient identification and closed-loop evidence, not a smaller prediction RMSE.
"""


def require_simulation(mock):
    if not mock:
        raise ValueError(
            'Hardware friction freedrive is disabled after the 2026-10-27 '
            'J6/J2 uncontrolled-motion incident. Steady-fit profiles are analysis '
            'artifacts, not qualified compensation controllers. No launch override.')
