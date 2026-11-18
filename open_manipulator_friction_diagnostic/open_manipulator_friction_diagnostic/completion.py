"""Position-mode transfer to the existing OMY follower ready pose after fitting."""
import json
from pathlib import Path
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .protocol import JOINTS, sha, smooth, follower_ready_file, validate_urdf_plan


def completion_pose(c, urdf, ready_file=None):
    mode = c.get('completion_pose', 'follower_ready')
    info = dict(mode=mode, source_file=None, source_sha256=None)
    if mode == 'zero':
        target = np.zeros(6)
    elif mode == 'follower_ready':
        if ready_file is None:
            ready_file = follower_ready_file()
        ready_file = Path(ready_file)
        p = yaml.safe_load(ready_file.read_text())['joint_trajectory_executor']['ros__parameters']
        names, values = p['joint_names'], p['ready']
        if len(names) != len(values) or len(names) != len(set(names)):
            raise ValueError('Invalid follower ready pose joint mapping')
        target = np.array([values[names.index(j)] for j in JOINTS], dtype=float)
        info.update(source_file=str(ready_file), source_sha256=sha(ready_file))
    else:
        raise ValueError('completion_pose must be follower_ready or zero')
    if target.shape != (6,) or not np.isfinite(target).all():
        raise ValueError('Invalid completion pose')
    root = ET.fromstring(urdf)
    for name, value in zip(JOINTS, target):
        joint = root.find(f"joint[@name='{name}']")
        limit = joint.find('limit') if joint is not None else None
        if limit is None or not float(limit.get('lower')) <= value <= float(limit.get('upper')):
            raise ValueError(f'Completion pose outside URDF limits: {name}')
    info.update(q_rad=target.tolist(), friction_domain=c['friction_model']+'; no pose whitelist')
    return info


def check_completion_load(c, urdf_path, baseline, bounds):
    """Check model input coverage, not measured freedrive performance at this pose."""
    from .poses import Robot
    from .integration import original_gravity
    pose = completion_pose(c, Path(urdf_path).read_text())
    gravity = Robot(urdf_path, JOINTS).analyze(pose['q_rad'])['gravity_nm']
    # Match the existing C++ parameter default when upstream YAML omits the scale.
    scales = original_gravity(baseline).get('torque_scaling_factors', [1.]*6)
    load = abs(np.asarray(gravity)*scales)
    limits = np.asarray(bounds)
    covered = (load >= limits[:, 0]) & (load <= limits[:, 1])
    return dict(mode=pose['mode'], q_rad=pose['q_rad'], nominal_load_nm=load.tolist(),
                covered_per_joint=covered.tolist(), passed=bool(covered.all()),
                scope='Nominal gravity-load coverage only; no measured freedrive validation')


def transfer_plan(start, target, c):
    start, target = np.asarray(start, dtype=float), np.asarray(target, dtype=float)
    if start.shape != (6,) or target.shape != (6,) or not np.isfinite([start, target]).all():
        raise ValueError('Invalid transfer endpoints')
    delta = target-start
    distance = max(abs(delta))
    dt = c['sample_period_s']
    duration = max(5., 1.9*distance/c.get('transfer_velocity_rad_s', c['max_velocity_rad_s']),
                   np.sqrt(6*distance/c['max_acceleration_rad_s2']))
    duration = np.ceil(duration/dt)*dt
    t = np.arange(round(duration/dt)+1)*dt
    x = t/duration
    q = start+smooth(x)[:, None]*delta
    v = (30*x*x*(1-x)**2/duration)[:, None]*delta
    a = (60*x*(1-x)*(1-2*x)/duration**2)[:, None]*delta
    return dict(t=t, q=q, v=v, a=a)


def homing_plan(q, v, c, urdf, ready_file=None):
    """Recognize the start independently of the requested end-of-run pose."""
    q, v = np.asarray(q), np.asarray(v)
    if q.shape != (6,) or v.shape != (6,) or not np.isfinite([q, v]).all():
        raise ValueError('Invalid measured homing state')
    ready = completion_pose(dict(c, completion_pose='follower_ready'), urdf, ready_file)
    near_zero = np.max(abs(q)) <= c['homing_max_displacement_rad']
    tolerance = c.get('homing_ready_tolerance_rad', .02)
    near_ready = np.max(abs(q-ready['q_rad'])) <= tolerance
    if not (near_zero or near_ready) or np.max(abs(v)) > .01:
        raise ValueError('Start outside zero/follower-ready homing neighbourhood or robot moving: '
                         f'q={q.round(5).tolist()}, v={v.round(5).tolist()}')
    data = transfer_plan(q, np.zeros(6), c)
    validate_urdf_plan({}, data, urdf)
    return data, dict(start_region='zero' if near_zero else 'follower_ready',
                      measured_start_rad=q.tolist(), target_rad=[0.]*6,
                      duration_s=float(data['t'][-1]), ready_tolerance_rad=tolerance)


def move_to_completion(node, c, folder, pose):
    """No collection or parameter fitting during this final position transfer."""
    folder = Path(folder)
    if node.failure or time.monotonic()-node.last_rx > .1:
        raise RuntimeError(node.failure or 'Stale state before completion move')
    q, v, _ = node.latest
    if max(abs(q)) > .02 or max(abs(v)) > .01:
        raise RuntimeError('Completion move requires stopped zero pose after collection')
    data = transfer_plan(q, pose['q_rad'], c)
    np.savez_compressed(folder/'completion_trajectory.npz', **data)
    result = dict(pose, duration_s=float(data['t'][-1]), completed=False,
                  trajectory_sha256=sha(folder/'completion_trajectory.npz'))
    path = folder/'completion.json'
    path.write_text(json.dumps(result, indent=2))
    node.status(f'MOVING_TO_READY: {pose["mode"]}, {data["t"][-1]:.1f} seconds; position mode')
    node.run_goal(data, label='MOVING_TO_READY')
    deadline = time.monotonic()+5
    while True:
        if node.failure or time.monotonic()-node.last_rx > .1:
            raise RuntimeError(node.failure or 'Stale state at completion pose')
        q, v, _ = node.latest
        if max(abs(q-pose['q_rad'])) <= .01 and max(abs(v)) <= .01:
            break
        if time.monotonic() > deadline:
            raise RuntimeError('Completion pose did not settle')
        time.sleep(.01)
    result.update(completed=True, measured_q_rad=q.tolist())
    path.write_text(json.dumps(result, indent=2))


def activate_latest(folder, latest):
    latest = Path(latest)
    temporary = latest.with_name('.latest-'+uuid.uuid4().hex)
    temporary.symlink_to(Path(folder).resolve(), target_is_directory=True)
    temporary.replace(latest)
