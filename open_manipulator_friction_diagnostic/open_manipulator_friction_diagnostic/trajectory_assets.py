"""Precomputed zero-to-zero joint blocks, composed without changing sampling.

Each block contains the existing generator's complete paired sweeps and holds.
Only the redundant initial hold of subsequent blocks is removed, exactly as in
the original all-joint loop. Current-pose homing is deliberately not stored.
"""
from copy import deepcopy
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile

import numpy as np

from . import automatic_motion, completion, protocol
from .artifact import resource_path
from .device_identity import data_directory


def inputs(collection, conditions, role, robot_fingerprint):
    collection = dict(collection, robot_unit='auto')
    return dict(schema=1, config=collection, conditions=conditions, role=role,
        robot_fingerprint=robot_fingerprint, sources={p.name: protocol.sha(p) for p in (
            Path(__file__), Path(automatic_motion.__file__), Path(completion.__file__), Path(protocol.__file__))})


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def _save(folder, signature, meta, data):
    folder.mkdir(parents=True, exist_ok=True)
    key = _digest(signature)
    fd, temp = tempfile.mkstemp(prefix='.trajectory-', suffix='.npz', dir=folder)
    os.close(fd)
    try:
        np.savez_compressed(temp, **data)
        entry = dict(inputs=signature, metadata=dict(meta, config=signature['config']),
                     trajectory_sha256=protocol.sha(temp))
        entry['metadata_sha256'] = _digest(entry['metadata'])
        os.chmod(temp, 0o644)  # Distributable assets may be built inside a root-owned container.
        os.replace(temp, folder/(key+'.npz'))
        fd, name = tempfile.mkstemp(prefix='.trajectory-', suffix='.json', dir=folder)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(entry, stream, allow_nan=False)
            os.chmod(name, 0o644)
            os.replace(name, folder/(key+'.json'))
        finally:
            if Path(name).exists():
                Path(name).unlink()
    finally:
        if Path(temp).exists():
            Path(temp).unlink()


def block(collection, conditions, role, robot_fingerprint, *, seed_dir=None, cache_dir=None):
    signature = inputs(collection, conditions, role, robot_fingerprint)
    key = _digest(signature)
    seed_dir = Path(seed_dir) if seed_dir is not None else resource_path('calibration/trajectories')
    cache_dir = Path(cache_dir) if cache_dir is not None else data_directory().parents[1]/'trajectory_cache'
    for label, folder in (('packaged', seed_dir), ('local', cache_dir)):
        try:
            entry = json.loads((folder/(key+'.json')).read_text())
            path = folder/(key+'.npz')
            if (entry['inputs'] != signature or protocol.sha(path) != entry['trajectory_sha256']
                    or _digest(entry['metadata']) != entry['metadata_sha256']):
                continue
            with np.load(path, allow_pickle=False) as archive:
                data = {k: archive[k].copy() for k in ('t', 'q', 'v', 'a')}
            meta = entry['metadata']
            if (len(data['t']) < 2 or any(data[k].shape != (len(data['t']), 6) for k in ('q','v','a'))
                    or any(not np.isfinite(v).all() for v in data.values())):
                continue
            print(f"TRAJECTORY loaded ({label}): J{conditions[0]['joint']+1} {role}", flush=True)
            return meta, data
        except (OSError, ValueError, KeyError, TypeError):
            continue
    print(f"TRAJECTORY configuration changed or asset missing: computing J{conditions[0]['joint']+1} {role}", flush=True)
    meta, data = automatic_motion.generate(collection, conditions, role)
    _save(cache_dir, signature, meta, data)
    return meta, data


def generate(collection, conditions, role, robot_fingerprint, **paths):
    if any(collection.get(k) for k in (
            'automatic_hold_history_diagnostic', 'automatic_preconditioned_entry_diagnostic',
            'automatic_spatial_history_diagnostic', 'automatic_shifted_angle_diagnostic')):
        return automatic_motion.generate(collection, conditions, role)
    selected = [p for p in conditions if p['role'] == role]
    if not selected:
        raise ValueError('No conditions in requested role')
    parts = [block(collection, [p for p in selected if p['joint'] == j], role,
                   robot_fingerprint, **paths) for j in sorted({p['joint'] for p in selected})]
    meta = deepcopy(parts[0][0])
    meta.update(config=deepcopy(collection), automatic_conditions=deepcopy(conditions),
                frozen_posture_roles={p['name']: p['role'] for p in conditions},
                segments=[], automatic_cycles=[])
    arrays = {k: [] for k in ('q', 'v', 'a')}
    count = 0
    joins = []
    for index, (info, data) in enumerate(parts):
        info = deepcopy(info)
        first = info['segments'][0]
        if first['phase'] != 'hold' or first['start'] != 0:
            raise ValueError('Precomputed block must begin with a zero hold')
        if any(np.max(abs(data[k][[0,-1]])) > 1e-10 for k in ('q','v','a')):
            raise ValueError('Precomputed block must start/end at stationary zero')
        cut = first['stop'] if index else 0
        if index:
            joins.append(count)
        offset, cycle_offset = count-cut, len(meta['automatic_cycles'])
        for segment in info['segments'][1 if index else 0:]:
            for key in ('start', 'stop'):
                segment[key] += offset
            if segment['repeat'] >= 0:
                segment['repeat'] += cycle_offset
            if 'full_cycle' in segment:
                segment['full_cycle'] += cycle_offset
            meta['segments'].append(segment)
        for cycle in info['automatic_cycles']:
            cycle['cycle_id'] += cycle_offset
            for key in ('motion_start_index', 'motion_stop_index'):
                cycle[key] += offset
            meta['automatic_cycles'].append(cycle)
        for key in arrays:
            arrays[key].append(data[key][cut:-1])
        count += len(data['t'])-cut-1
    data = {k: np.vstack([*chunks, parts[-1][1][k][-1:]]) for k,chunks in arrays.items()}
    data['t'] = np.arange(count+1)*collection['sample_period_s']
    # Removing the redundant hold changes the central-difference stencil at a
    # join. Recompute only these few samples, not the motion or full gradients.
    neighbors = sorted({i for join in joins for i in range(max(1, join-2), min(count, join+3))})
    for source, derivative in (('q', 'v'), ('v', 'a')):
        for i in neighbors:
            data[derivative][i] = (data[source][i+1]-data[source][i-1])/(2.*collection['sample_period_s'])
    meta.update(duration_s=float(data['t'][-1]), expected_paired_cycles=len(meta['automatic_cycles']),
        motion_limits_rad=np.maximum(np.max(abs(data['q']), axis=0), collection['joint_excursion_rad']).tolist())
    return meta, data


def main():
    parser = argparse.ArgumentParser(description='Precompute distributable OMY motion blocks; never start motors')
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from .calibration_plan import load_schedule
    from .integration import description, fingerprint
    _, collection, conditions, _ = load_schedule(args.config, validate=True)
    robot = fingerprint(description('position'))
    for role in ('development', 'fresh_final_validation'):
        cfg = dict(collection, automatic_repeats=2) if role == 'fresh_final_validation' else collection
        for joint in range(6):
            local = [p for p in conditions if p['role'] == role and p['joint'] == joint]
            if local:
                meta, data = automatic_motion.generate(cfg, local, role)
                _save(args.output, inputs(cfg, local, role, robot), meta, data)
                print(f'J{joint+1} {role}: {meta["duration_s"]:.2f}s', flush=True)


if __name__ == '__main__':
    main()
