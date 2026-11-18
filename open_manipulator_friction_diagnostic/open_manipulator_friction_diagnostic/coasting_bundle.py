"""A separately evidenced first J6 coasting trial; no physical qualification.

The old D=16 physical trial is never reused as evidence for this different law.
All other joint/gravity/current parameters stay in the original configuration.
"""
import copy
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def runtime_hashes():
    root = Path(__file__).resolve().parents[2]
    names = ['open_manipulator_friction_diagnostic/tools/stress_j6_curve.cpp',
        *['open_manipulator_friction_diagnostic/include/friction_compensation_controller/'+n
          for n in ('stateful_friction.hpp', 'kinetic_curve.hpp', 'elastoplastic_replay.hpp',
                    'position_velocity_observer.hpp', 'calibrated_friction.hpp')]]
    return {n: sha(root/n) for n in names}


def load_coasting_bundle(folder, urdf, baseline_path):
    from .integration import fingerprint, original_gravity
    folder = Path(folder)
    meta = json.loads((folder/'candidate.json').read_text())
    expected = {'curve_review.json', 'stress.json', 'curves.txt', 'plant_provenance.json'}
    if (meta.get('schema') != 'supervised_j6_coasting_v1' or
            meta.get('hardware_qualified') is not False or meta.get('joint') != 'joint6' or
            meta.get('robot_fingerprint') != fingerprint(urdf) or
            meta.get('baseline_sha256') != sha(baseline_path) or
            set(meta.get('files', {})) != expected or
            meta.get('runtime_hashes') != runtime_hashes()):
        raise ValueError('J6 coasting identity/source/evidence mismatch')
    for name, digest in meta['files'].items():
        if sha(folder/name) != digest:
            raise ValueError('Changed coasting evidence: '+name)
    if meta.get('design') != dict(stiffness=1500., decay=5., damping=0., output_scale=.95,
                                 observer_window=.02):
        raise ValueError('Unreviewed coasting design')
    stress = json.loads((folder/'stress.json').read_text())
    selected = [r for r in stress['results'] if r['scale'] == .95]
    if (stress.get('synthetic_only') is not True or stress.get('extra_damping') != 0. or
            stress.get('runtime_timing_faults') != 0 or len(selected) != 1 or
            selected[0]['cases'] != 864 or any(selected[0][k] != 0 for k in
                ('stationary_self_start', 'restart_after_hand_stop',
                 'speed_growth_after_release', 'numerical_escape'))):
        raise ValueError('Missing matching no-damping software stress evidence')
    review = json.loads((folder/'curve_review.json').read_text())
    if review.get('schema') != 'j6_coasting_curve_review_v1':
        raise ValueError('Unexpected curve review')
    curve = review['curve']; positive = curve['positive']; negative = curve['negative']
    rows = [[float(x) for x in line.split()] for line in (folder/'curves.txt').read_text().splitlines()]
    if (rows[0] != [13.] or len(rows) != 6 or
            rows[1] != positive['speed_rad_s'] or rows[1] != negative['speed_rad_s'] or
            rows[2] != positive['magnitude_nm'] or rows[3] != negative['magnitude_nm']):
        raise ValueError('Runtime knots differ from the stress-tested knots')
    params = copy.deepcopy(original_gravity(baseline_path))
    if params['joints'] != [f'joint{i}' for i in range(1, 7)]:
        raise ValueError('Expected unchanged original six-axis gravity configuration')
    params.update(calibrated_friction_enabled=False, stateful_friction_enabled=True,
        stateful_joint_mask=[0, 0, 0, 0, 0, 1], stateful_observer_window=.02,
        stateful_positive_yield=[0.]*5+[positive['magnitude_nm'][0]],
        stateful_negative_yield=[0.]*5+[negative['magnitude_nm'][0]],
        stateful_positive_viscous=[0.]*6, stateful_negative_viscous=[0.]*6,
        stateful_stiffness=[0.]*5+[1500.], stateful_decay=[0.]*5+[5.],
        stateful_damping=[0.]*6, stateful_maximum_velocity=[0.]*6,
        stateful_curve_j6_enabled=True, stateful_curve_speed=positive['speed_rad_s'],
        stateful_curve_positive=positive['magnitude_nm'],
        stateful_curve_negative=negative['magnitude_nm'], stateful_curve_output_scale=.95,
        friction_diagnostics_rate=50.)
    return params
