"""Offline geometry review of the exact collection and completion trajectories.

This is a sampled self-collision check, not a safety certificate or a check of
the physical workspace, cables, actuator response, or compensation stability.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from .protocol import JOINTS, config, generate, sha, validate_urdf_plan
from .completion import completion_pose, transfer_plan
from .integration import fingerprint


def source_hashes():
    return {name: sha(Path(__file__).with_name(name)) for name in
            ('protocol.py', 'completion.py', 'motion_review.py', 'settling.py')}


def review(c, urdf_path, output):
    import pinocchio as pin
    urdf = Path(urdf_path).read_text()
    meta, traj = generate(c)
    ending = transfer_plan(np.zeros(6), completion_pose(c, urdf)['q_rad'], c)
    start = transfer_plan(completion_pose(c, urdf)['q_rad'], np.zeros(6), c)
    positions = np.vstack([start['q'], traj['q'], ending['q']])
    validate_urdf_plan({}, dict(q=positions), urdf)
    model = pin.buildModelFromUrdf(str(urdf_path))
    geoms = pin.buildGeomFromUrdf(model, str(urdf_path), pin.GeometryType.COLLISION)
    def group(g):
        j = g.parentJoint
        while j and model.names[j] not in JOINTS:
            j = model.parents[j]
        return JOINTS.index(model.names[j])+1 if j else 0
    for i, a in enumerate(geoms.geometryObjects):
        for k in range(i+1, len(geoms.geometryObjects)):
            if abs(group(a)-group(geoms.geometryObjects[k])) > 1:
                geoms.addCollisionPair(pin.CollisionPair(i, k))
    if not len(geoms.collisionPairs):
        raise ValueError('No nonadjacent collision geometry')
    data = model.createData(); gd = pin.GeometryData(geoms)
    q = pin.neutral(model); minimum = float('inf'); collision_samples = 0; closest = None
    # Every generated point, including transitions and both directions.
    for row, position in enumerate(positions):
        for j, name in enumerate(JOINTS):
            q[model.joints[model.getJointId(name)].idx_q] = position[j]
        pin.computeDistances(model, data, geoms, gd, q)
        values = [r.min_distance for r in gd.distanceResults]
        index = int(np.argmin(values))
        if values[index] < minimum:
            minimum = float(values[index]); pair = geoms.collisionPairs[index]
            closest = dict(sample=row, q_rad=position.tolist(), links=[
                geoms.geometryObjects[pair.first].name, geoms.geometryObjects[pair.second].name])
        collision_samples += min(values) <= 0
    result = dict(schema=1, config=c, robot_fingerprint=fingerprint(urdf),
        source_hashes=source_hashes(), trajectory_positions_sha256=hashlib.sha256(positions.tobytes()).hexdigest(),
        duration_s=meta['duration_s'], completion_duration_s=float(ending['t'][-1]),
        samples=len(positions), sample_period_s=c['sample_period_s'],
        collision_pairs=len(geoms.collisionPairs), minimum_distance_m=minimum,
        minimum_required_distance_m=.01, closest=closest, collision_samples=int(collision_samples),
        passed=minimum >= .01 and collision_samples == 0,
        geometry_scope='URDF nonadjacent collision shapes; all generated samples, ready homing and completion',
        exclusions='Adjacent/fixed assembly, cables, external workspace, continuous interpolation clearance',
        hardware_validated=False, compensation_qualified=False,
        schedule=meta['sweep_schedule'], posture_speed_scope=meta['posture_speed_scope'])
    Path(output).write_text(json.dumps(result, indent=2))
    return result


def require_review(path, c, urdf):
    if c.get('sweep_protocol') not in ('distance_v1', 'matched_v2'):
        return
    if not path:
        raise ValueError('Distance study requires a matching offline motion_review JSON before bringup')
    r = json.loads(Path(path).expanduser().read_text())
    if (r.get('schema') != 1 or r.get('passed') is not True or r.get('config') != c or
            r.get('source_hashes') != source_hashes() or
            r.get('robot_fingerprint') != fingerprint(urdf) or r.get('collision_samples') != 0 or
            r.get('minimum_distance_m', 0) < .01 or r.get('samples', 0) < 100):
        raise ValueError('Motion review missing, failed, stale, or for a different robot/configuration')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True); p.add_argument('--urdf', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args(); r = review(config(a.config), a.urdf, a.output)
    print(json.dumps({k:r[k] for k in ('passed','duration_s','samples','minimum_distance_m','closest')}, indent=2))
    if not r['passed']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
