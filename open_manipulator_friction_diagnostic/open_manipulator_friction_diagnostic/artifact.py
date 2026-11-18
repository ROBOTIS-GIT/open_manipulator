"""Read the portable runtime artifact. No fitting, replay or historical lookup."""
from copy import deepcopy
import math
import os
from pathlib import Path
import tempfile
import yaml

JOINTS = [f'joint{i}' for i in range(1, 7)]
SCHEMA = 'omy_friction_calibration_v1'


def resource_path(relative):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('open_manipulator_friction_diagnostic')) / relative


def calibration_path():
    """Resolve per-unit storage, importing matching legacy data once if present."""
    from .device_identity import data_directory, require_unit
    override = os.environ.get('OM_FRICTION_CALIBRATION_FILE')
    if override:
        return Path(override).expanduser().resolve()
    destination = data_directory() / 'current.yaml'
    if not destination.exists():
        legacy = resource_path('calibration/current.yaml').resolve()
        if legacy.is_file():
            data = load_calibration(legacy, allow_partial=True)
            # A copied source tree must never seed another OMY's calibration.
            try:
                require_unit(data)
            except ValueError:
                return destination
            import_calibration(legacy, destination)
            print(f'Migrated same-unit calibration to {destination}; legacy file retained', flush=True)
    return destination


def import_calibration(source, destination=None):
    """Copy matching-unit legacy data atomically without replacing existing data."""
    from .device_identity import data_directory, require_unit
    source = Path(source).expanduser().resolve()
    data = load_calibration(source, allow_partial=True)
    require_unit(data)
    destination = Path(destination).expanduser().resolve() if destination else data_directory() / 'current.yaml'
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.import-', dir=destination.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(source.read_bytes())
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, destination)  # atomic no-clobber publication
    finally:
        os.unlink(temporary)
    return destination


def load_runtime_calibration(path, *, hardware=True):
    """Every controller launch uses the same device-binding check before bringup."""
    from .device_identity import require_unit
    path = Path(path).expanduser()
    if not path.is_file():
        raise ValueError(f'No calibration for this OMY at {path}. Run calibrate.launch.py first.')
    data = load_calibration(path)
    if hardware:
        require_unit(data)
        if data['source'] != 'hardware':
            raise ValueError('A simulation calibration cannot be used on real hardware')
    return data


def _finite(value):
    if isinstance(value, dict):
        for child in value.values():
            _finite(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            _finite(child)
    elif isinstance(value, float) and not math.isfinite(value):
        raise ValueError('Calibration contains a non-finite number')


def validate_calibration(data, *, allow_partial=False):
    if not isinstance(data, dict) or data.get('schema') != SCHEMA:
        raise ValueError('Unsupported calibration schema')
    if data.get('joint_names') != JOINTS or not data.get('robot_unit'):
        raise ValueError('Calibration joint order or robot identity is missing')
    if data.get('source') not in ('hardware', 'gazebo'):
        raise ValueError('Unknown calibration source')
    params = data.get('controller_parameters', {})
    _finite(params)
    if params.get('joints') != JOINTS or not params.get('stateful_bank_enabled'):
        raise ValueError('A complete common controller bank is required')
    mask = params.get('stateful_joint_mask', [])
    if len(mask) != 6 or any(type(x) is not int or x not in (0, 1) for x in mask) or not any(mask):
        raise ValueError('Invalid calibration joint mask')
    if not allow_partial and mask != [1] * 6:
        missing = [name for name, enabled in zip(JOINTS, mask) if not enabled]
        raise ValueError('Incomplete calibration: all six joint models required; missing '+', '.join(missing))
    for joint, enabled in zip(JOINTS, mask):
        if not enabled:
            continue
        prefix = f'stateful_bank.{joint}.'
        speeds = params.get(prefix + 'speeds', [])
        if (len(speeds) < 2 or any(type(x) not in (int, float) or x <= 0 for x in speeds)
                or any(a >= b for a, b in zip(speeds, speeds[1:]))):
            raise ValueError(f'{joint}: invalid speed knots')
        for direction in ('positive', 'negative'):
            values = params.get(prefix + direction, [])
            if not values or any(type(x) not in (int, float) for x in values):
                raise ValueError(f'{joint}: invalid {direction} coefficients')
    return data


def load_calibration(path, *, allow_partial=False):
    return validate_calibration(yaml.safe_load(Path(path).read_text()), allow_partial=allow_partial)


def runtime_snapshot(data):
    """Keep coefficients/support; detailed fit/CV rows remain in identification.json.

    Model selection can contain tens of megabytes of candidate residuals. They
    have no role in the control law or in loading an already calibrated robot.
    """
    def compact(value):
        if isinstance(value, list):
            return [compact(item) for item in value]
        if not isinstance(value, dict):
            return value
        result = {}
        for key, item in value.items():
            if key in ('training_review', 'posture_review', 'selection') and isinstance(item, dict):
                result[key] = {k: deepcopy(item[k]) for k in (
                    'method', 'score', 'means_passed', 'spatial_passed', 'passed', 'fresh_validation')
                    if k in item}
            else:
                result[key] = compact(item)
        return result
    result = deepcopy(data)
    result['models'] = compact(data.get('models', {}))
    result['audit_details'] = 'Full fit and validation residuals are saved in collection/identification.json'
    return result


def save_calibration(path, data, *, check_cancelled=None, allow_partial=False):
    validate_calibration(data, allow_partial=allow_partial)
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.calibration-', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        if check_cancelled is not None:
            check_cancelled()
        os.replace(name, path)
        parent = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(parent)
        finally:
            os.close(parent)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def controller_parameters(data, ratios, dither_enabled, *, hardware=True):
    validate_calibration(data)
    if hardware and data['source'] != 'hardware':
        raise ValueError('A simulation calibration cannot be used on real hardware')
    if (len(ratios) != 6 or any(type(x) not in (float, int) or
            not math.isfinite(x) or not 0 <= x <= 1 for x in ratios)):
        raise ValueError('Specify six compensation ratios between zero and one')
    if len(dither_enabled) != 6 or any(type(x) is not bool for x in dither_enabled):
        raise ValueError('Specify six dithering switches')
    params = deepcopy(data['controller_parameters'])
    for i, joint in enumerate(JOINTS):
        prefix = f'stateful_bank.{joint}.'
        params[prefix + 'fraction'] = float(ratios[i])
        if not dither_enabled[i]:
            params[prefix + 'dither_amplitude'] = 0.
    return params
