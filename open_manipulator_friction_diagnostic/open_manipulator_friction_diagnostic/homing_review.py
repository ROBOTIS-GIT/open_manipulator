"""Compute actual-start geometry outside the ROS state callback's Python GIL."""
import argparse
from collections import deque
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

from .protocol import JOINTS, sha, smooth


def review_in_worker(q, velocity, config, urdf_path, folder):
    folder = Path(folder)
    request = folder/'homing_review_request.json'
    result = folder/'homing_review_result.json'
    trajectory = folder/'homing_review_result.npz'
    urdf_path = Path(urdf_path).resolve()
    payload = dict(q=np.asarray(q).tolist(), velocity=np.asarray(velocity).tolist(),
                   config=config, urdf=str(urdf_path), urdf_sha256=sha(urdf_path))
    request.write_text(json.dumps(payload))
    with (folder/'homing_review_worker.log').open('w') as log:
        env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        subprocess.run(['nice', '-n', '10', sys.executable, '-m', __name__, '--request', str(request),
                        '--result', str(result), '--trajectory', str(trajectory)],
                       check=True, timeout=120, stdout=log, stderr=subprocess.STDOUT, env=env)
    info = json.loads(result.read_text())
    if info.pop('request_sha256') != sha(request) or sha(urdf_path) != payload['urdf_sha256']:
        raise ValueError('Actual-start geometry inputs changed during review')
    with np.load(trajectory, allow_pickle=False) as saved:
        data = {key: saved[key] for key in saved.files}
    return data, info


def require_unchanged_start(reviewed_q, latest_q, latest_velocity):
    values = np.asarray([reviewed_q, latest_q, latest_velocity], float)
    if values.shape != (3, 6) or not np.isfinite(values).all():
        raise ValueError('Invalid state after homing geometry review')
    if np.max(abs(values[1]-values[0])) > .0003 or np.max(abs(values[2])) > .01:
        raise ValueError('Robot moved during homing geometry review; no trajectory submitted')


class EnableSettling:
    """A fresh, continuously stationary window after the brake/servo transition."""
    def __init__(self):
        self.samples = deque()
        self.last_stamp = None

    def observe(self, stamp, q, velocity):
        q, velocity = np.asarray(q, float), np.asarray(velocity, float)
        if (q.shape != (6,) or velocity.shape != (6,) or
                not np.isfinite(np.r_[stamp, q, velocity]).all()):
            raise ValueError('Invalid state while settling after motor enable')
        if self.last_stamp is not None:
            gap = stamp-self.last_stamp
            if gap < 0:
                raise ValueError('Nonmonotonic state while settling after motor enable')
            if gap == 0:
                return False
            if gap > .05:
                self.samples.clear()
        self.last_stamp = stamp
        if np.max(abs(velocity)) > .01:
            self.samples.clear()
            return False
        self.samples.append((stamp, q.copy()))
        # Retain the sample immediately before the 250 ms window boundary.
        while len(self.samples) > 1 and self.samples[1][0] <= stamp-.25:
            self.samples.popleft()
        return (stamp-self.samples[0][0] >= .25 and
                np.max(np.ptp([p for _, p in self.samples], axis=0)) <= .0003)


def joint_motion_radii(urdf_path):
    """Upper bounds on collision-point distance from each arm joint axis.

    Triangle inequalities over the kinematic tree cover all configurations,
    including attached end-effector collision shapes. Calculated with motors OFF.
    """
    import pinocchio as pin
    import xml.etree.ElementTree as ET
    root = ET.parse(urdf_path).getroot()
    for name in JOINTS:
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.get('type') not in ('revolute', 'continuous'):
            raise ValueError('Homing rebase requires six rotational arm joints')
    model = pin.buildModelFromUrdf(str(urdf_path))
    geoms = pin.buildGeomFromUrdf(model, str(urdf_path), pin.GeometryType.COLLISION)
    radii = np.zeros(6)
    for obj in geoms.geometryObjects:
        obj.geometry.computeLocalAABB()
        box = obj.geometry.aabb_local
        reach = np.linalg.norm(obj.placement.translation)+np.linalg.norm(
            np.maximum(abs(box.min_), abs(box.max_)))
        joint = obj.parentJoint
        while joint:
            if model.names[joint] in JOINTS:
                index = JOINTS.index(model.names[joint])
                radii[index] = max(radii[index], reach)
            reach += np.linalg.norm(model.jointPlacements[joint].translation)
            joint = model.parents[joint]
    if not np.isfinite(radii).all() or np.any(radii <= 0):
        raise ValueError('Missing finite collision geometry motion bounds')
    return radii.tolist()


def rebase_homing(data, info, q, velocity, config):
    """Connect the settled measured start within the already-reviewed clearances.

    At equal quintic phase each joint shifts by at most its start delta. A
    collision point shifts by at most sum(radius * abs(delta)); pair distance
    can shrink by twice that. Keep the original sample phases and stretch time
    if needed. This preserves the sampled review, not a continuous-collision proof.
    """
    q, velocity = np.asarray(q, float), np.asarray(velocity, float)
    radii = np.asarray(info['joint_motion_radius_m'], float)
    if (q.shape != (6,) or velocity.shape != (6,) or radii.shape != (6,) or
            not np.isfinite([q, velocity, radii]).all() or np.any(radii <= 0)):
        raise ValueError('Invalid settled start or geometry motion bounds')
    if np.max(abs(velocity)) > .01:
        raise ValueError('Motor-enabled homing start must be stationary')
    delta = q-data['q'][0]
    displacement = float(np.dot(radii, abs(delta)))
    geometry = info['geometry']
    self_clearance = geometry['minimum_distance_m']-2*displacement
    floor_clearance = geometry['floor_minimum_m']-displacement
    if (geometry.get('passed') is not True or
            self_clearance < geometry['minimum_required_distance_m'] or
            floor_clearance < geometry['minimum_table_clearance_m']):
        raise ValueError('Motor-enable shift exceeds reviewed homing clearance; no trajectory submitted')
    target = data['q'][-1]
    phase = data['t']/data['t'][-1]
    if not np.allclose(data['q'], data['q'][0]+smooth(phase)[:, None]*(target-data['q'][0]),
                       rtol=0., atol=1e-12):
        raise ValueError('Homing rebase requires the reviewed synchronized quintic path')
    distance = float(np.max(abs(target-q)))
    dt = config['sample_period_s']
    duration = max(float(data['t'][-1]),
                   1.9*distance/config.get('transfer_velocity_rad_s', config['max_velocity_rad_s']),
                   np.sqrt(6*distance/config['max_acceleration_rad_s2']))
    duration = float(np.ceil(duration/dt)*dt)
    travel = target-q
    rebased = dict(t=phase*duration, q=q+smooth(phase)[:, None]*travel,
                  v=(30*phase**2*(1-phase)**2/duration)[:, None]*travel,
                  a=(60*phase*(1-phase)*(1-2*phase)/duration**2)[:, None]*travel)
    updated = deepcopy(info)
    updated.update(start_rad=q.tolist(), duration_s=duration,
                   maximum_velocity_rad_s=float(np.max(abs(rebased['v']))),
                   enable_settling=dict(reviewed_start_rad=data['q'][0].tolist(),
                       settled_start_rad=q.tolist(), start_shift_rad=delta.tolist(),
                       max_start_shift_rad=float(np.max(abs(delta))),
                       collision_point_displacement_bound_m=displacement,
                       self_clearance_lower_bound_m=self_clearance,
                       floor_clearance_lower_bound_m=floor_clearance))
    return rebased, updated


def main():
    from .automatic_motion import reviewed_homing
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('request', 'result', 'trajectory'):
        parser.add_argument('--'+name, required=True)
    args = parser.parse_args()
    payload = json.loads(Path(args.request).read_text())
    if sha(payload['urdf']) != payload['urdf_sha256']:
        raise ValueError('Changed URDF before homing review')
    data, info = reviewed_homing(payload['q'], payload['velocity'],
                                 payload['config'], payload['urdf'])
    info['joint_motion_radius_m'] = joint_motion_radii(payload['urdf'])
    np.savez_compressed(args.trajectory, **data)
    Path(args.result).write_text(json.dumps(dict(info, request_sha256=sha(args.request)), indent=2))


if __name__ == '__main__':
    main()
