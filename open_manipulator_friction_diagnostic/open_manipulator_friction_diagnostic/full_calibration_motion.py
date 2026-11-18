"""Frozen six-axis collection with directional/order and whole-posture holdouts.

Only position commands are generated. Friction coefficients, current limits,
gravity parameters and compensation activation are not changed here.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from .completion import completion_pose, transfer_plan
from .generalization_motion import check_geometry, sources
from .integration import fingerprint
from .protocol import JOINTS, config, sha, validate_urdf_plan
from .trajectory_transport import compact_points

SPEEDS = [.0017453292519943296, .004, .012, .04, .10]


def frozen_design(c, proposal, ready):
    if len(proposal['poses']) != 8 or proposal.get('pose_selection_uses_target_torque') is not False:
        raise ValueError('Expected geometry-only eight-pose proposal')
    tasks = []
    def task(name, pose, sides, joints, role, base=False):
        tasks.append(dict(name=name, pose=pose, sides=sides, joints=joints,
                          role=role, base=base, speeds=SPEEDS))
    for j in range(6):
        task(f'base_j{j+1}', c['calibration_centers_rad'][j], [0]*6, [j], 'development', True)
    task('follower_ready', ready, [0, 0, -1, 0, 0, 0], list(range(6)), 'development')
    # Select using signed gravity/bearing-load geometry, never new effort values.
    for i in (0, 1, 3, 5):
        p = proposal['poses'][i]
        task(f'load_{i+1:02d}', p['q_rad'], p.get('sweep_sides', [0]*6), [1, 2, 3], 'development')
    task('working', [0., -1.05, 1.9, -.85, 1.5708, 0.], [0]*6,
         list(range(6)), 'external_validation')
    p = proposal['poses'][2]
    task('load_03', p['q_rad'], p.get('sweep_sides', [0]*6), [1, 2, 3], 'external_validation')
    return dict(method='full_directional_conditioned_v1', tasks=tasks,
                repeats=2, conditioning_pairs=2, common_rest_s=2.,
                base_low_speed_stroke_rad=float(np.deg2rad(6)),
                loaded_low_speed_stroke_rad=float(np.deg2rad(1.5)),
                target_torque_used_for_design=False, temperature_model=False,
                condition_order=['increasing_speed_positive_first', 'decreasing_speed_negative_first'])


def generate_full(c, design):
    if (design['method'] != 'full_directional_conditioned_v1' or design['repeats'] != 2
            or design['conditioning_pairs'] != 2 or design['common_rest_s'] != 2.
            or design['target_torque_used_for_design'] is not False):
        raise ValueError('Unexpected full-calibration design')
    dt = c['sample_period_s']; chunks = []; segments = []; cycles = []
    count = 0; current = np.zeros(6); roles = {}
    def add(q, phase, **tags):
        nonlocal count, current
        if len(q) < 2 or not np.allclose(q[0], current, atol=1e-10, rtol=0):
            raise ValueError('Discontinuous or empty motion')
        segment = dict(start=count, stop=count+len(q)-1, phase=phase, joint=-1,
                       speed_rad_s=0., repeat=-1, event='', validation_pose='', study_role='transition')
        segment.update(tags); segments.append(segment); chunks.append(q[:-1])
        count += len(q)-1; current = q[-1].copy()
    def rest():
        add(np.tile(current, (round(design['common_rest_s']/dt)+1, 1)), 'hold')
    def move(target):
        if np.max(abs(current-target)) > 1e-10:
            add(transfer_plan(current, target, c)['q'], 'transition')
    def pair(task, j, speed, stroke, repeat, conditioning=False):
        center = np.asarray(task['pose'], float); side = task['sides'][j]
        low = center.copy(); high = center.copy()
        low[j] += -stroke if side == -1 else 0. if side == 1 else -stroke/2
        high[j] = low[j]+stroke
        directions = (1, -1) if repeat == 0 else (-1, 1)
        move(low if repeat == 0 else high); rest()
        ramp = np.ceil(max(c['ramp_s'], np.pi*speed/(2*c['max_acceleration_rad_s2']))/dt)*dt
        plateau = np.ceil((stroke/speed-ramp-1e-10)/dt)*dt
        if plateau < 3.: raise ValueError('Insufficient steady interval')
        actual = stroke/(plateau+ramp)
        cycle_id = len(cycles)
        role = 'conditioning' if conditioning else task['role']
        cycles.append(dict(cycle_id=cycle_id, joint=j, group=task['name'],
            requested_speed_rad_s=speed, speed_rad_s=actual, repeat_order=repeat,
            first_direction=directions[0], role=role, stroke_rad=stroke,
            first_sample=count, common_rest_s=design['common_rest_s']))
        for direction in directions:
            for event, duration in [('start', ramp), ('steady', plateau), ('stop', ramp)]:
                t = np.arange(round(duration/dt)+1)*dt
                x = actual*t if event == 'steady' else actual/2*(
                    t+(-1 if event == 'start' else 1)*ramp/np.pi*np.sin(np.pi*t/ramp))
                q = np.tile(current, (len(t), 1)); q[:, j] += direction*x
                add(q, 'posture_validation_'+('steady' if event == 'steady' else 'transient'),
                    joint=j, speed_rad_s=direction*actual, repeat=cycle_id, event=event,
                    validation_pose=task['name'], study_role=role, full_cycle=cycle_id)
            rest()
        cycles[-1]['last_sample'] = count
        if not np.allclose(current, low if repeat == 0 else high, atol=1e-10, rtol=0):
            raise ValueError('Sweep did not return to its start')
    rest()
    for task in design['tasks']:
        if task['role'] not in ('development', 'external_validation') or task['speeds'] != SPEEDS:
            raise ValueError('Invalid task role/speeds')
        roles[task['name']] = task['role']
        for j in task['joints']:
            # Every joint block starts from the zero pose to reuse the reviewed
            # star-shaped transfers, rather than interpolating between loads.
            move(np.zeros(6)); move(np.asarray(task['pose'], float))
            for repeat in range(2):
                for _ in range(2): pair(task, j, .04, .18, repeat, True)
                for speed in SPEEDS if repeat == 0 else reversed(SPEEDS):
                    low = design['base_low_speed_stroke_rad'] if task['base'] else design['loaded_low_speed_stroke_rad']
                    ramp = np.ceil(max(c['ramp_s'], np.pi*speed/(2*c['max_acceleration_rad_s2']))/dt)*dt
                    stroke = max(low, speed*(ramp+3.))
                    pair(task, j, speed, stroke, repeat)
            move(np.asarray(task['pose'], float)); move(np.zeros(6)); rest()
    q = np.vstack(chunks+[current[None]]); t = np.arange(len(q))*dt
    v = np.gradient(q, dt, axis=0, edge_order=2); a = np.gradient(v, dt, axis=0, edge_order=2)
    if np.max(abs(v)) > c['max_velocity_rad_s']+1e-8 or np.max(abs(a)) > c['max_acceleration_rad_s2']+1e-8:
        raise ValueError('Velocity/acceleration limit exceeded')
    meta = dict(schema=1, config=deepcopy(c), joints=JOINTS, segments=segments,
        duration_s=float(t[-1]), generalization_study=True, full_calibration=True, record_only=True,
        method=design['method'], full_design=design, full_cycles=cycles,
        frozen_posture_roles=roles, expected_paired_cycles=len(cycles),
        expected_identification_cycles=sum(x['role'] != 'conditioning' for x in cycles),
        motion_limits_rad=[.8, 2., 3., 1.9, 1.9, .8], independent_repeat_validation=True,
        sensor_delay_identified=False, compensation_qualified=False, temperature_model=False,
        scope='Full six-axis position collection with frozen whole-posture holdouts. '
              'Conditioning retained but excluded from stationary coefficient fitting. '
              'Direction order/speed order and all transient/rest data retained. '
              'No independent output torque measurement or automatic freedrive qualification.')
    return meta, dict(t=t, q=q, v=v, a=a)


def prepare(c, proposal_path, urdf_path, folder):
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=False)
    urdf = Path(urdf_path).read_text(); ready = completion_pose(c, urdf)['q_rad']
    proposal = json.loads(Path(proposal_path).read_text())
    design = frozen_design(c, proposal, ready); meta, data = generate_full(c, design)
    homing = transfer_plan(ready, np.zeros(6), c); ending = transfer_plan(np.zeros(6), ready, c)
    positions = np.vstack([homing['q'], data['q'], ending['q']])
    validate_urdf_plan(meta, dict(q=positions), urdf)
    _, transport = compact_points(data, [s[k] for s in meta['segments'] for k in ('start', 'stop')],
                                  acceleration_limit=c['max_acceleration_rad_s2'])
    np.savez_compressed(folder/'trajectory.npz', **data)
    meta.update(sha256=sha(folder/'trajectory.npz'), source_hashes=sources(),
                full_generator_sha256=sha(__file__), proposal_sha256=sha(proposal_path),
                robot_fingerprint=fingerprint(urdf))
    (folder/'plan.json').write_text(json.dumps(meta, indent=2))
    review = dict(schema=1, plan_sha256=sha(folder/'plan.json'), trajectory_sha256=meta['sha256'],
        source_hashes=sources(), full_generator_sha256=sha(__file__), robot_fingerprint=fingerprint(urdf),
        geometry=check_geometry(positions, urdf_path),
        positions_sha256=hashlib.sha256(positions.tobytes()).hexdigest(), duration_s=meta['duration_s'],
        homing_duration_s=float(homing['t'][-1]), completion_duration_s=float(ending['t'][-1]),
        max_abs_velocity_rad_s=np.max(abs(data['v']), axis=0).tolist(),
        max_abs_acceleration_rad_s2=np.max(abs(data['a']), axis=0).tolist(),
        min_q_rad=data['q'].min(axis=0).tolist(), max_q_rad=data['q'].max(axis=0).tolist(),
        transport=transport, includes_ready_homing_and_completion=True, compensation_qualified=False)
    (folder/'motion_review.json').write_text(json.dumps(review, indent=2))
    if not review['geometry']['passed']: raise ValueError('Full plan fails geometry review')
    return review


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('config', 'proposal', 'urdf', 'output'): p.add_argument('--'+name, required=True)
    a = p.parse_args(); print(json.dumps(prepare(config(a.config), a.proposal, a.urdf, a.output), indent=2))


if __name__ == '__main__': main()
