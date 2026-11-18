"""Resolve device identity at execution; never encode a robot serial in source."""
import os
from pathlib import Path
import re
import socket


def robot_unit(value='auto'):
    """Stable unit identity, including containers whose hostname is a random ID.

    This identifies the installed computer/robot pairing, not an actuator read.
    Set OMY_ROBOT_UNIT when the computer moves to another arm.
    """
    if value in (None, 'auto'):
        value = os.environ.get('OMY_ROBOT_UNIT')
        if not value or value == 'auto':
            host_file = Path('/run/omy/hostname')  # optional host identity mount in custom deployments
            value = host_file.read_text().strip() if host_file.is_file() else socket.gethostname()
            value = value.split('.')[0]
            if re.fullmatch(r'[0-9a-f]{12,64}', value) or value.lower() in ('localhost', 'open_manipulator'):
                raise ValueError('Cannot identify OMY from container hostname; set OMY_ROBOT_UNIT to the robot ID')
    if not isinstance(value, str):
        raise ValueError('OMY robot ID must be a string')
    value = value.strip()
    value = (value[4:] if value.lower().startswith('omy-') else value).upper()
    if not re.fullmatch(r'[A-Z0-9][A-Z0-9_-]{0,95}', value) or value == 'AUTO':
        raise ValueError('OMY robot ID must contain only letters, digits, underscores or hyphens')
    return value


def data_directory(unit=None):
    """Writable per-unit state, never inside the installed source package."""
    root = Path(os.environ.get('OMY_FRICTION_DATA_DIR', '~/.ros/omy_friction')).expanduser()
    return root.resolve() / 'units' / robot_unit(unit)


def require_unit(data, unit=None):
    expected = robot_unit(unit)
    if data.get('robot_unit') != expected:
        raise ValueError(f"Calibration/settings belong to {data.get('robot_unit')!r}, not {expected}. "
                         'Collect calibration on this OMY; do not reuse another unit\'s coefficients.')
    identity = data.get('identity')
    if identity and identity.get('robot_unit') != expected:
        raise ValueError('Calibration identity disagrees with robot_unit')
    for model in data.get('models', {}).values():
        if model.get('identity') and model['identity'].get('robot_unit') != expected:
            raise ValueError('Model identity disagrees with calibration robot_unit')
