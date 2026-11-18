"""Narrow, source-bound compatibility for previously measured kinetic laws.

The reviewed change adds optional load-hat coordinates. With an empty knot
vector it preserves the old expressions, iteration order and all other paths.
It does not change any state observer, elastic state or dither waveform.
The second reviewed change adds an explicitly disabled mean-extension branch.
Unknown revisions still require their own review; measurements are never edited.
"""
import hashlib
from pathlib import Path

PREFIX = 'open_manipulator_friction_diagnostic/include/friction_compensation_controller/'
NAMES = ('stateful_friction.hpp', 'elastoplastic_replay.hpp', 'directional_load_law.hpp',
         'kinetic_curve.hpp', 'rest_dither.hpp', 'position_velocity_observer.hpp')
EMPTY_HAT_EXTENSION = (
    '0de7c2b247196934fcd22561f471a662c90746fcf52c29ab50d18f9f683c184c',
    '523ff2f31422a80ade7e9654daaa9919e739e81c5475aa699b01540f4930a12d',
)
DISABLED_MEAN_EXTENSION = (
    EMPTY_HAT_EXTENSION[1],
    'b48f2fbf0324265bc00e993a0cfc77f65e68053b06b80250334af0e25b51ecd5',
)


def verify(recorded, model, root):
    """Check all six runtime sources, returning explicit compatible differences."""
    compatible = []
    for name in NAMES:
        key = PREFIX+name
        current = hashlib.sha256((Path(root)/key).read_bytes()).hexdigest()
        previous = recorded.get(key)
        if previous == current:
            continue
        disabled_mean=(not model.get('load_hat_mean_extension',False)
                       and current==DISABLED_MEAN_EXTENSION[1])
        if (name=='directional_load_law.hpp' and previous==DISABLED_MEAN_EXTENSION[0] and disabled_mean):
            compatible.append(dict(source=key,recorded_sha256=previous,current_sha256=current,
                                   reason='disabled_mean_load_extension'))
        elif (name == 'directional_load_law.hpp'
                and previous==EMPTY_HAT_EXTENSION[0]
                and (current==EMPTY_HAT_EXTENSION[1] or disabled_mean)
                and model.get('schema') in ('common_periodic_friction_v1', 'common_friction_v1')
                and model.get('load_hat_knots_nm', []) == []):
            compatible.append(dict(source=key, recorded_sha256=previous, current_sha256=current,
                                   reason='empty_load_hat_extension'))
        else:
            raise ValueError('Physical compensation implementation changed: '+name)
    return compatible
