"""Reuse complete paired motions, restarting the identical plan at a rest boundary.

Trajectory time is a plan coordinate, not a continuous physical clock. Preserve
the physical epoch of each acquired block; never fill in unmeasured samples.
"""
import json
import os
from pathlib import Path
import shutil
import numpy as np
from .fit import validate_log
from .protocol import sha


def command_suffix(data, index):
    return {key: values[index:]-values[index] if key == 't' else values[index:]
            for key, values in data.items()}


def checkpoint_approach(meta, data, index):
    """Use an existing zero-to-posture path, never invent a direct connector."""
    target = data['q'][index]
    if np.max(abs(target)) < 1e-9:
        return None
    entries = [c['motion_start_index'] for c in meta['automatic_cycles']
               if c['motion_start_index'] <= index
               and np.max(abs(data['q'][c['motion_start_index']]-target)) < 1e-9]
    if not entries:
        raise ValueError('No reviewed entry path to checkpoint posture')
    last_entry = max(entries)
    rest = np.max(abs(data['v'][:last_entry+1]), axis=1) < 1e-4
    zeros = np.flatnonzero(rest & (np.max(abs(data['q'][:last_entry+1]), axis=1) < 1e-9))
    if not len(zeros):
        raise ValueError('No reviewed stationary zero waypoint before checkpoint')
    start = int(zeros[-1])
    end = min(i for i in entries if i >= start)
    approach = {k: v[start:end+1].copy() for k,v in data.items()}
    approach['t'] -= approach['t'][0]
    if len(approach['t']) < 2 or np.max(abs(approach['v'][[0,-1]])) > 1e-4:
        raise ValueError('Checkpoint entry must start and end at rest')
    return approach


def motion_checkpoint(meta, data, last_time):
    cycles = meta.get('automatic_cycles', [])
    if not cycles or not all('motion_start_index' in c and 'motion_stop_index' in c for c in cycles):
        return None  # Legacy recordings retain the original complete-joint policy.
    boundaries = {0}
    for c in cycles:
        begin, end = c['motion_start_index'], c['motion_stop_index']
        if not 0 <= begin < end < len(data['t']):
            raise ValueError('Invalid paired-motion checkpoint')
        segments = [s for s in meta['segments'] if s.get('full_cycle') == c['cycle_id']]
        if (not segments or any(s['start'] < begin or s['stop'] > end for s in segments)
                or {np.sign(s['speed_rad_s']) for s in segments if s['event'] == 'steady'} != {-1,1}):
            raise ValueError('Checkpoint must enclose both directions of a complete pair')
        boundaries.update((begin, end))
    # Keep the final pair intact if only the ending transfer/hold failed.
    if last_time >= data['t'][max(boundaries)]:
        last_time = data['t'][cycles[-1]['motion_start_index']]
    index = max(i for i in boundaries if data['t'][i] <= last_time)
    if np.max(abs(data['v'][index])) > 1e-4:
        raise ValueError('Checkpoint is not a stationary paired-motion boundary')
    checkpoint_approach(meta, data, index)  # Fail before any motor activation.
    return index


def prepare_retry(folder, meta, data, *, attempt, limit, collector_code, runtime_code):
    """Only recover a proven data-quality stop after both processes have exited."""
    if collector_code == 0 or runtime_code != 0 or attempt >= limit:
        return None
    info, _ = prepare_resume(folder, meta, data, allow_supervisor_timeout=False)
    folder = Path(folder)
    # Preserve every failed raw journal before fixed-name collection rotation.
    # Files are closed and immutable here; hard links avoid copying hundreds of MB.
    archive = folder.with_name(folder.name+'.recovery')/sha(folder/'collection.json')[:16]
    archive.parent.mkdir(parents=True, exist_ok=True)
    if not archive.exists():
        shutil.copytree(folder, archive, copy_function=os.link)
    return dict(info, attempt=attempt+1, archive=str(archive))


def prepare_resume(folder, meta, data, *, allow_supervisor_timeout=True):
    folder = Path(folder)
    read = lambda name: json.loads((folder/name).read_text())
    old, checkpoint, report = [read(name) for name in (
        'plan.json', 'recording_checkpoint.json', 'collection.json')]
    gap = report.get('state_failure') or {}
    gap_values = [gap.get(key, float('nan')) for key in ('gap_s','stamp_s','previous_stamp_s')]
    gap_evidence = (gap.get('kind') == 'stamp_gap' and np.isfinite(gap_values).all()
        and gap_values[0] > .05 and abs(gap_values[1]-gap_values[2]-gap_values[0]) < 1e-6)
    gap_stop = (str(report.get('failure')).startswith('RuntimeError: Acquisition state gap')
        or report.get('failure') == 'RuntimeError: Joint state reception stopped')
    age_evidence = (gap.get('kind') == 'sample_age'
        and np.isfinite([gap.get('stamp_s', float('nan')),gap.get('stamp_age_s', float('nan'))]).all()
        and gap['stamp_age_s'] > .1
        and str(report.get('failure')).startswith('RuntimeError: Acquisition sample older than 100 ms'))
    runtime = read('runtime/runtime.json')
    # An explicitly requested resume can reuse proven data after a supervisor
    # heartbeat timeout. It never authorizes an automatic retry of that fault.
    supervisor_only = (allow_supervisor_timeout
        and str(report.get('failure')).startswith('RuntimeError: Position runtime heartbeat stopped:')
        and runtime.get('failure') is None and runtime.get('mode') == 'position'
        and runtime.get('phase') == 'stopped'
        and 0 < report.get('max_state_gap_s', float('inf')) <= .05
        and all(runtime.get('axes', {}).get(str(j), {}).get('Error Code') == 0 for j in range(1,7)))
    if (not meta.get('portable_calibration') or report.get('source') != 'hardware'
            or report.get('completed') or not checkpoint.get('recording_closed')
            or not runtime.get('all_off_confirmed')
            or not ((gap_stop and gap_evidence) or age_evidence or supervisor_only)
            or (folder/'human_contact_review.json').exists()):
        raise ValueError('Resume requires a closed, contact-free acquisition-gap stop with confirmed motor OFF')
    if (any(old[key] != meta[key] for key in ('sha256','robot_fingerprint','baseline_sha256','config','segments'))
            or sha(folder/'trajectory.npz') != meta['sha256']
            or checkpoint['trajectory_sha256'] != meta['sha256']):
        raise ValueError('Resume requires the identical trajectory, robot and measurement configuration')
    if checkpoint['filename'] != 'measured_rows.npy':
        raise ValueError('Unexpected journal filename')
    rows = np.load(folder/'measured_rows.npy', mmap_mode='r', allow_pickle=False)[:checkpoint['rows']]
    if rows.ndim != 2 or rows.shape[1] != 19 or not len(rows):
        raise ValueError('Invalid measured journal')
    dt = meta['config']['sample_period_s']
    tagged = [s for s in meta['segments'] if s['joint'] >= 0]
    order = list(dict.fromkeys(s['joint'] for s in tagged))
    completed = []
    for joint in order:
        end = max(s['stop'] for s in tagged if s['joint'] == joint)*dt
        if end > rows[-1,0]: break
        completed.append(joint)
    checkpoint = motion_checkpoint(meta, data, rows[-1,0])
    if checkpoint is not None:
        boundaries = [checkpoint]
        completed = [j for j in order if all(
            c['motion_stop_index'] <= checkpoint for c in meta['automatic_cycles'] if c['joint'] == j)]
    else:
        boundaries = joint_checkpoint(meta, data, completed, order)
    if not boundaries:
        raise ValueError('No stationary checkpoint before interrupted measurement')
    index = boundaries[0]; offset = float(data['t'][index])
    count = int(np.searchsorted(rows[:,0], offset))
    prefix = rows[:count]
    if len(prefix):
        validate_log(dict(meta, duration_s=offset), dict(t=prefix[:,0],q=prefix[:,1:7],
                     v=prefix[:,7:13],effort=prefix[:,13:19]))
    blocks = report.get('recording_segments')
    if not blocks and count:
        stamp = report.get('state_failure',{}).get('previous_stamp_s')
        if stamp is None:
            raise ValueError('Missing physical acquisition time provenance')
        blocks = [dict(row_start=0,row_stop=len(rows),epoch_s=stamp-float(rows[-1,0]))]
    retained = [dict(block,row_stop=min(count,block['row_stop']))
                for block in (blocks or []) if block['row_start'] < count]
    if count and (not retained or retained[0]['row_start'] != 0 or retained[-1]['row_stop'] != count
            or any(a['row_stop'] != b['row_start'] for a,b in zip(retained,retained[1:]))
            or not all(np.isfinite(b['epoch_s']) for b in retained)):
        raise ValueError('Incomplete physical time provenance')
    info = dict(schema=2, source_folder=str(folder), source_collection_sha256=sha(folder/'collection.json'),
        source_journal_sha256=sha(folder/'measured_rows.npy'), trajectory_sha256=meta['sha256'],
        completed_joints=[j+1 for j in completed], start_index=index, start_time_s=offset,
        checkpoint_pose_rad=data['q'][index].tolist(),
        completed_cycles=[c['cycle_id'] for c in meta.get('automatic_cycles', [])
                          if c.get('motion_stop_index', len(data['t'])) <= index],
        checkpoint_scope='paired_motion' if checkpoint is not None else 'legacy_joint',
        retained_rows=count, recording_segments=retained,
        note='Only complete paired motions reused. Partial pair discarded. No sample interpolation.')
    return info, prefix


def joint_checkpoint(meta, data, completed, order):
    """Compatibility for earlier plans without paired-motion boundaries."""
    tagged = [s for s in meta['segments'] if s['joint'] >= 0]
    if len(completed) == len(order):
        # A final return/settling failure still invalidates that joint block.
        completed.pop()
    previous_end = max((s['stop'] for s in tagged if s['joint'] in completed), default=0)
    next_start = min(s['start'] for s in tagged if s['joint'] not in completed)
    boundaries = [s['start'] for s in meta['segments'] if s['phase'] == 'hold'
        and previous_end <= s['start'] < s['stop'] <= next_start
        and np.max(abs(data['q'][s['start']:s['stop']+1])) < 1e-9
        and np.max(abs(data['v'][s['start']:s['stop']+1])) < 1e-4]
    if not completed:
        boundaries = [0] if np.max(abs(data['q'][0])) < 1e-9 else []
    return boundaries
