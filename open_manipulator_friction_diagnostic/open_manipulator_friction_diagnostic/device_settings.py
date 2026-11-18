"""Optional per-OMY acquisition settings; no control law or trajectory changes."""
from copy import deepcopy
from pathlib import Path
import math
import yaml

from .device_identity import data_directory, require_unit

TIMING_FIELDS = ('current_filter_time_constant_s', 'current_measurement_delay_s', 'sensor_settling_s')


def apply_settings(collection, path=None):
    result = deepcopy(collection)
    explicit = path is not None
    path = Path(path).expanduser().resolve() if path else data_directory(result['robot_unit']) / 'device.yaml'
    if not path.is_file():
        if explicit or path.exists():
            raise ValueError(f'Not a device settings file: {path}')
        return result
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get('schema') != 'omy_device_settings_v1':
        raise ValueError('Invalid OMY device settings schema')
    require_unit(data, result['robot_unit'])
    if set(data) - {'schema', 'robot_unit', 'sensor_timing'}:
        raise ValueError('Unknown device setting; trajectories and gains belong in their own YAML files')
    timing = data.get('sensor_timing', {})
    if not isinstance(timing, dict) or set(timing) != {*TIMING_FIELDS, 'sensor_timing_verified'}:
        raise ValueError('Provide all sensor timing fields together')
    if type(timing['sensor_timing_verified']) is not bool:
        raise ValueError('sensor_timing_verified must be a boolean')
    for key in TIMING_FIELDS:
        values = timing[key]
        if (not isinstance(values, list) or len(values) != 6 or any(
                type(v) not in (float, int) or not math.isfinite(v) or v < 0 for v in values)):
            raise ValueError(f'{key}: six finite nonnegative values required')
    if any(5*t+d > s for t,d,s in zip(*(timing[k] for k in TIMING_FIELDS))):
        raise ValueError('sensor_settling_s must cover five filter time constants plus delay')
    result.update(deepcopy(timing))
    return result
