"""Freeze and review a new load-coverage experiment before position bringup.

No friction targets select the train/test split. This module never enables a
motor or produces compensation parameters. Reviews cover sampled URDF geometry.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from .completion import completion_pose, transfer_plan
from .integration import fingerprint
from .protocol import JOINTS, config, generate, sha, validate_urdf_plan

TRAIN = (0, 1, 3, 5, 6)
HOLDOUT = (2, 4, 7)


def sources():
    return {name: sha(Path(__file__).with_name(name)) for name in
            ('generalization_motion.py', 'protocol.py', 'completion.py',
             'settling.py', 'trajectory_transport.py', 'history_motion.py', 'profile_telemetry.py')}


def generate_study(c, proposal):
    if len(proposal['poses']) != 8 or proposal.get('pose_selection_uses_target_torque') is not False:
        raise ValueError('Expected eight geometry-only pose proposals')
    speeds = proposal['suggested_speed_rad_s']
    if speeds != [.0017453292519943296, .004, .012, .04, .10]:
        raise ValueError('Unexpected new-study speed schedule')
    working = deepcopy(c)
    working['posture_validation'] = [dict(name=f'load_{i+1:02d}', pose=p['q_rad'],
                                         sweep_sides=p.get('sweep_sides', [0]*6))
                                     for i, p in enumerate(proposal['poses'])]
    # Explicitly reviewed new envelope; do not clip or reshape the proposed poses.
    # URDF limits and full sampled self-clearance are checked before execution.
    working['posture_validation_limits_rad'] = [0.8, 2.0, 3.0, 1.7, 1.9, 0.8]
    meta, data = generate(working, skip_base=True, posture_joints=[1, 2, 3],
                          posture_repeats=2, posture_speeds=speeds)
    roles = {f'load_{i+1:02d}': 'development' if i in TRAIN else 'external_validation'
             for i in range(8)}
    for segment in meta['segments']:
        segment['study_role'] = roles.get(segment['validation_pose'], 'transition')
    meta.update(config=deepcopy(c), generalization_study=True, record_only=True,
                posture_validation_only=False, method='load_coverage_collection_v1',
                frozen_posture_roles=roles, roles_selected_without_measured_friction=True,
                study_speeds_rad_s=speeds, study_joints=['joint2', 'joint3', 'joint4'],
                study_repeats=2, expected_paired_cycles=8*3*5*2,
                generation_config=working, independent_repeat_validation=True,
                hardware_validated=False, compensation_qualified=False,
                scope='New load-coverage study only; J2/J3/J4, five speeds, two repeats. '
                      'Full holdout postures excluded from fitting and model selection.')
    return meta, data


def floor_bounds(geoms, gd):
    """Conservative world-Z bound of moving collision shapes; table at Z=0."""
    minimum = float('inf'); name = None
    for i, obj in enumerate(geoms.geometryObjects):
        if obj.parentJoint == 0:
            continue
        box = obj.geometry.aabb_local; tf = gd.oMg[i]
        z = tf.translation[2]+np.minimum(tf.rotation[2]*box.min_, tf.rotation[2]*box.max_).sum()
        if z < minimum:
            minimum = float(z); name = obj.name
    return minimum, name


def check_geometry(positions, urdf_path):
    import pinocchio as pin
    model = pin.buildModelFromUrdf(str(urdf_path)); data = model.createData()
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
        raise ValueError('Missing collision geometry')
    for obj in geoms.geometryObjects:
        obj.geometry.computeLocalAABB()
    gd = pin.GeometryData(geoms); q = pin.neutral(model)
    minimum = float('inf'); collisions = 0; closest = None
    floor_minimum = float('inf'); floor_closest = None
    # Exact duplicate resting positions need not run the same geometric query twice.
    unique = np.unique(positions, axis=0)
    for row, position in enumerate(unique):
        for j, name in enumerate(JOINTS):
            q[model.joints[model.getJointId(name)].idx_q] = position[j]
        pin.computeDistances(model, data, geoms, gd, q)
        values = [r.min_distance for r in gd.distanceResults]
        k = int(np.argmin(values))
        if values[k] < minimum:
            minimum = float(values[k]); pair = geoms.collisionPairs[k]
            closest = dict(q_rad=position.tolist(), links=[geoms.geometryObjects[pair.first].name,
                                                           geoms.geometryObjects[pair.second].name])
        collisions += min(values) <= 0
        floor, name = floor_bounds(geoms, gd)
        if floor < floor_minimum:
            floor_minimum = floor
            floor_closest = dict(geometry=name, q_rad=position.tolist())
        if row and row % 25000 == 0:
            print(f'GEOMETRY {row}/{len(unique)} unique positions; min={minimum:.6f} m', flush=True)
    return dict(passed=minimum >= .01 and collisions == 0 and floor_minimum >= .02,
                minimum_distance_m=minimum, floor_minimum_m=floor_minimum,
                table_plane_world_z_m=0., minimum_table_clearance_m=.02, floor_closest=floor_closest,
                minimum_required_distance_m=.01, collision_samples=int(collisions),
                samples=len(positions), unique_positions=len(unique), closest=closest,
                collision_pairs=len(geoms.collisionPairs),
                exclusions='Adjacent/fixed assembly, cables, external objects above the table and continuous interpolation')


def prepare(c, proposal_path, urdf_path, folder):
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=False)
    proposal = json.loads(Path(proposal_path).read_text())
    meta, data = generate_study(c, proposal)
    urdf = Path(urdf_path).read_text()
    ready = completion_pose(c, urdf)['q_rad']
    homing = transfer_plan(ready, np.zeros(6), c)
    ending = transfer_plan(np.zeros(6), ready, c)
    positions = np.vstack([homing['q'], data['q'], ending['q']])
    validate_urdf_plan(meta, dict(q=positions), urdf)
    np.savez_compressed(folder/'trajectory.npz', **data)
    meta.update(sha256=sha(folder/'trajectory.npz'), proposal_sha256=sha(proposal_path),
                source_hashes=sources(), robot_fingerprint=fingerprint(urdf))
    (folder/'plan.json').write_text(json.dumps(meta, indent=2))
    old_low, old_high = np.asarray(proposal['existing_coordinate_envelope_rad'])
    review = dict(schema=1, plan_sha256=sha(folder/'plan.json'), trajectory_sha256=meta['sha256'],
                  robot_fingerprint=fingerprint(urdf), source_hashes=sources(),
                  geometry=check_geometry(positions, urdf_path),
                  positions_sha256=hashlib.sha256(positions.tobytes()).hexdigest(),
                  duration_s=meta['duration_s'], homing_duration_s=float(homing['t'][-1]),
                  completion_duration_s=float(ending['t'][-1]),
                  max_abs_velocity_rad_s=np.max(abs(data['v']), axis=0).tolist(),
                  max_abs_acceleration_rad_s2=np.max(abs(data['a']), axis=0).tolist(),
                  min_q_rad=data['q'].min(axis=0).tolist(), max_q_rad=data['q'].max(axis=0).tolist(),
                  previous_measured_coordinate_envelope_rad=proposal['existing_coordinate_envelope_rad'],
                  extension_lower_rad=np.maximum(0., old_low-data['q'].min(axis=0)).tolist(),
                  extension_upper_rad=np.maximum(0., data['q'].max(axis=0)-old_high).tolist(),
                  sample_period_s=c['sample_period_s'], includes_ready_homing_and_completion=True,
                  hardware_started=False, compensation_qualified=False)
    (folder/'motion_review.json').write_text(json.dumps(review, indent=2))
    if not review['geometry']['passed']:
        raise ValueError('New study fails sampled geometry review; no hardware execution')
    return review


def load_reviewed(folder, c, urdf):
    folder = Path(folder)
    meta = json.loads((folder/'plan.json').read_text())
    review = json.loads((folder/'motion_review.json').read_text())
    if meta.get('full_calibration'):
        generator = sha(Path(__file__).with_name('full_calibration_motion.py'))
        if meta.get('full_generator_sha256') != generator or review.get('full_generator_sha256') != generator:
            raise ValueError('Full-calibration generator changed after geometry review')
    if (meta.get('generalization_study') is not True or meta.get('record_only') is not True or
            meta['config'] != c or meta.get('source_hashes') != sources() or
            review.get('source_hashes') != sources() or
            review.get('plan_sha256') != sha(folder/'plan.json') or
            review.get('trajectory_sha256') != sha(folder/'trajectory.npz') or
            meta.get('sha256') != review['trajectory_sha256'] or
            review.get('robot_fingerprint') != fingerprint(urdf) or
            meta.get('robot_fingerprint') != fingerprint(urdf) or
            review.get('geometry', {}).get('passed') is not True or
            review['geometry']['collision_samples'] != 0 or
            review['geometry']['minimum_distance_m'] < .01 or
            review['geometry'].get('table_plane_world_z_m') != 0. or
            review['geometry'].get('floor_minimum_m', -1) < .02):
        raise ValueError('New-study plan or geometry evidence is missing, stale or mismatched')
    data = dict(np.load(folder/'trajectory.npz'))
    validate_urdf_plan(meta, data, urdf)
    return meta, data


def copy_reviewed(source, output, c, urdf):
    import shutil
    meta, data = load_reviewed(source, c, urdf)
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    for filename in ('plan.json', 'trajectory.npz', 'motion_review.json'):
        shutil.copyfile(Path(source)/filename, output/filename)
    return meta, data


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('config', 'proposal', 'urdf', 'output'):
        p.add_argument('--'+name, required=True)
    a = p.parse_args()
    result = prepare(config(a.config), a.proposal, a.urdf, a.output)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
