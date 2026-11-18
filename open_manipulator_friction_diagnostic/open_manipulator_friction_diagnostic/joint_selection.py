"""Explicit acquisition selection and atomic-artifact merge, independent of ROS."""
from copy import deepcopy

from .artifact import JOINTS, validate_calibration
from .kinetic_bank import ARRAYS


def parse_joints(value):
    if isinstance(value, str):
        if value.strip().lower() == 'all':
            return list(range(1, 7))
        try:
            value = [int(part.strip()) for part in value.split(',')]
        except ValueError as exc:
            raise ValueError('Specify joints as all or comma-separated numbers, e.g. 2,3') from exc
    if (not isinstance(value, (list, tuple)) or not value or
            any(type(j) is not int or j not in range(1, 7) for j in value) or
            len(set(value)) != len(value)):
        raise ValueError('Select distinct joints 1 through 6')
    return sorted(value)


def identity(data):
    if data.get('identity'):
        return data['identity']
    identities = [m.get('identity') for m in data.get('models', {}).values()]
    return identities[0] if identities and all(i == identities[0] for i in identities) else None


def check_partial_identity(previous, current_identity, joints):
    if previous and len(joints) < 6 and identity(previous) != current_identity:
        raise ValueError('Partial calibration identity differs: use joints:=all or a new calibration_file')


def merge_calibration(fresh, previous=None):
    """Replace requested axes only; never use old coefficients inside fitting.

    A first partial acquisition is saved in the same current.yaml. It remains
    unloadable by the controller until all six models have been accumulated.
    The caller publishes this value only after successful identification.
    """
    validate_calibration(fresh, allow_partial=True)
    if not fresh.get('usable'):
        raise ValueError('Cannot publish failed identification')
    p = fresh['controller_parameters']
    selected = [j for j, enabled in enumerate(p['stateful_joint_mask'], 1) if enabled]
    check_partial_identity(previous, identity(fresh), selected)
    same = bool(previous and identity(previous) == identity(fresh) and previous['source'] == fresh['source'])
    if previous and len(selected) < 6 and not same:
        raise ValueError('Partial calibration source or identity differs')
    result = deepcopy(fresh)
    old = previous if same else {}
    retained = []
    if same and len(selected) < 6:
        validate_calibration(previous, allow_partial=True)
        before = previous['controller_parameters']
        # These shared settings affect retained axes. Legacy single-axis law
        # fields are inactive in bank mode and are deliberately left untouched.
        shared = lambda params: {k:v for k,v in params.items() if not k.startswith('stateful_') or
                                 k in ('stateful_observer_window', 'stateful_friction_enabled', 'stateful_bank_enabled')}
        if shared(before) != shared(p):
            raise ValueError('Shared controller settings differ; recalibrate all joints together')
        merged = deepcopy(before)
        for j in selected:
            prefix = f'stateful_bank.joint{j}.'
            for key in list(merged):
                if key.startswith(prefix):
                    del merged[key]
            merged.update({k:deepcopy(v) for k,v in p.items() if k.startswith(prefix)})
            for field in ARRAYS:
                merged['stateful_'+field][j-1] = p['stateful_'+field][j-1]
            merged['stateful_joint_mask'][j-1] = 1
        retained = [j for j, enabled in enumerate(before['stateful_joint_mask'], 1) if enabled and j not in selected]
        result['controller_parameters'] = merged
        result['models'] = {**deepcopy(previous['models']), **deepcopy(fresh['models'])}
    # Dither calibration remains independent: inherit same-device settings,
    # optionally overlay only the explicitly newly measured waveforms.
    result['dither'] = deepcopy(old.get('dither', {}))
    before = old.get('controller_parameters', {})
    for j in selected:
        prefix = f'stateful_bank.joint{j}.'
        for suffix in ('amplitude', 'frequency', 'full_velocity', 'zero_velocity', 'ramp'):
            key = prefix+'dither_'+suffix
            if key in before:
                result['controller_parameters'][key] = before[key]
        if not same:
            result['controller_parameters'][prefix+'dither_amplitude'] = 0.
        values = fresh.get('dither', {}).get(f'joint{j}')
        if fresh.get('dither_collection', {}).get('usable') and values:
            result['dither'][f'joint{j}'] = deepcopy(values)
            for key, value in values.items():
                result['controller_parameters'][prefix+'dither_'+key] = value
    result['dither_reused_from_same_device'] = same
    if old.get('dither_collection') and not fresh.get('dither_collection'):
        result['dither_collection'] = deepcopy(old['dither_collection'])
    result.update(units='joint_equivalent_Nm', reidentified_joints=selected, retained_joints=retained,
                  calibration_complete=all(result['controller_parameters']['stateful_joint_mask']))
    result['missing_joints'] = [j for j, enabled in enumerate(result['controller_parameters']['stateful_joint_mask'], 1) if not enabled]
    validate_calibration(result, allow_partial=True)
    return result
