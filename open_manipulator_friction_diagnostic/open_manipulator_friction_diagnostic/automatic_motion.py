"""Targeted matched-angle collection through the existing position-mode ROS node.

Geometry selects conditions, never measured friction. Development and final
validation are separate plans. No current-mode experiment or freedrive starts
from this module. All starts, reversals and dwells remain in the raw recording.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from .automatic_calibration import content_hash, file_hash, write_new
from .completion import completion_pose, transfer_plan
from .generalization_motion import check_geometry, sources
from .integration import fingerprint, original_gravity
from .poses import Robot
from .protocol import JOINTS, config, sha, validate_urdf_plan


def select_conditions(cycles, joint, c, urdf_path, scaling):
    """Hold the target angle fixed, vary the remaining observed configurations."""
    robot = Robot(urdf_path, JOINTS)
    existing = [x for x in cycles if x['joint'] == joint]
    if not existing:
        raise ValueError(f'No geometry seeds for joint{joint+1}')
    center = c['calibration_centers_rad'][joint][joint]
    seeds = [np.asarray(c['calibration_centers_rad'][joint], float)]
    for group in sorted({x['group'] for x in existing}):
        q = np.mean(np.concatenate([x['q'] for x in existing if x['group'] == group]), axis=0)
        q[joint] = center
        seeds.append(q)
    # Small variations of already measured configurations. They are proposals
    # until the entire transfer/sweep (including the table) has been checked.
    low, high = np.min(np.concatenate([x['q'] for x in cycles]), axis=0), np.max(np.concatenate([x['q'] for x in cycles]), axis=0)
    for q in list(seeds):
        for k in range(1, 5):
            if k == joint:
                continue
            for delta in (-.25, .25):
                p = q.copy(); p[k] = np.clip(p[k]+delta, low[k], high[k]); seeds.append(p)
    poses = []
    for q in np.unique(seeds, axis=0):
        # Screen the widest default sweep and its entry transfers too. The
        # selected full trajectory is checked at every sample below.
        speed = .10
        ramp = max(c['ramp_s'], np.pi*speed/(2*c['max_acceleration_rad_s2']))
        half = speed*(max(3., 2*c['sensor_settling_s'][joint]+.022/speed)+ramp)/2+.002
        lo = q.copy(); hi = q.copy(); lo[joint] -= half; hi[joint] += half
        path = np.vstack([np.linspace(np.zeros(6), q, 31), np.linspace(q, lo, 9),
                          np.linspace(lo, hi, 31), np.linspace(hi, q, 9)])
        try:
            validate_urdf_plan({}, dict(q=path), Path(urdf_path).read_text())
        except ValueError:
            continue
        geometry = check_geometry(path, urdf_path)
        if geometry['passed']:
            loads = robot.static_load_batch(q[None])
            z = [float(loads['gravity_nm'][0, joint]*scaling[joint]),
                 float(loads['bending_moment_nm'][0, joint])]
            poses.append(dict(q_rad=q.tolist(), load_features=z))
    return choose_conditions(poses, joint, common=c.get('common_friction_identification', False))


def choose_conditions(poses, joint, common=False):
    """Reserve whole configurations before observing their friction targets."""
    poses = [dict(p) for i,p in enumerate(poses) if not any(
        np.max(abs(np.delete(np.array(p['q_rad'])-q['q_rad'], joint))) < .01
        for q in poses[:i])]
    required = 5 if common else 4
    if len(poses) < required:
        raise ValueError(f'joint{joint+1}: need {required} distinct clear matched-angle conditions')
    features = np.array([x['load_features'] for x in poses])
    span = np.ptp(features, axis=0)
    if max(span) < .25 and not common:
        raise ValueError(f'joint{joint+1}: posture alone does not vary observable load; use known payload experiment')
    z = (features-features.min(axis=0))/np.maximum(span, .25)
    # A load-neutral joint still needs held-configuration validation. Geometry
    # diversity supplies conditions, not fictitious load excitation or labels.
    if max(span) < .25:
        q = np.delete(np.array([p['q_rad'] for p in poses]), joint, axis=1)
        z = (q-q.min(axis=0))/np.maximum(np.ptp(q,axis=0), .1)
    chosen = [int(np.argmin(z[:, np.argmax(np.ptp(z,axis=0))]))]
    while len(chosen) < required:
        distances = np.min(np.sum((z[:, None]-z[chosen])**2, axis=2), axis=1)
        distances[chosen] = -1
        chosen.append(int(np.argmax(distances)))
    # The common fitter needs >=3 whole-configuration folds. Two additional
    # configurations stay reserved for independent validation.
    result = [poses[i] for i in chosen]
    for i, p in enumerate(result):
        p.update(name=f'matched_j{joint+1}_load{i}', joint=joint,
                 role='development' if i < required-2 else 'fresh_final_validation')
    return result


def condition_speeds(c, condition, fallback):
    values = tuple(condition.get('speeds_rad_s', fallback))
    if (not values or any(not np.isfinite(v) or v <= 0 or v > c['max_velocity_rad_s'] for v in values)
            or any(a >= b for a, b in zip(values, values[1:]))):
        raise ValueError('Invalid per-condition measurement speed')
    return values


def plateau_duration(c, speed, joint, condition=None):
    dt = c['sample_period_s']
    travel = (condition or {}).get('settled_travel_rad', c.get('automatic_minimum_settled_travel_rad', .022))
    minimum = c.get('automatic_minimum_plateau_s', 3.)
    settling = c['sensor_settling_s'][joint]
    if (not np.isfinite([dt,travel,minimum,settling,speed]).all() or dt<=0 or speed<=0
            or travel<.022 or minimum<1.5 or settling<0):
        raise ValueError('Invalid settled travel or plateau timing')
    return float(np.ceil(max(minimum, 2*settling+travel/speed)/dt)*dt)


def steady_half_span(c, speed, joint, condition=None):
    return speed*plateau_duration(c, speed, joint, condition)/2


def check_sparse_holdouts(c, conditions, previous):
    """Whole held-out configurations must not intersect any development sweep."""
    intervals = []
    fallback = c.get('automatic_speeds_rad_s', (.004,.012,.04,.1))
    for p in conditions:
        j = p['joint']
        half = max(steady_half_span(c, v, j, p) for v in condition_speeds(c,p,fallback))
        intervals.append((p['role'],j,np.asarray(p['q_rad']),p['q_rad'][j]-half,p['q_rad'][j]+half))
    for cycle in previous:
        q = np.asarray(cycle['q']); j = cycle['joint']
        intervals.append(('development',j,q.mean(axis=0),q[:,j].min(),q[:,j].max()))
    for role,j,q,lo,hi in intervals:
        if role != 'fresh_final_validation':
            continue
        for other,k,p,a,b in intervals:
            if (other=='development' and k==j and np.max(abs(np.delete(q-p,j)))<.01
                    and max(lo,a) <= min(hi,b)+.01):
                raise ValueError('Reserved validation sweep overlaps development configurations')


def generate(c, conditions, role, speeds=None, timing_only=False):
    if role not in ('development', 'fresh_final_validation') or not conditions:
        raise ValueError('Invalid collection role')
    speeds = tuple(speeds if speeds is not None else c.get('automatic_speeds_rad_s', (.004, .012, .04, .10)))
    if (not speeds or any(not np.isfinite(v) or v <= 0 or v > c['max_velocity_rad_s'] for v in speeds)
            or any(a >= b for a,b in zip(speeds,speeds[1:]))):
        raise ValueError('Invalid requested measurement speed')
    hold_history = c.get('automatic_hold_history_diagnostic', False)
    if type(hold_history) is not bool:
        raise ValueError('Hold-history flag must be an explicit boolean')
    if hold_history:
        if (role != 'development' or c.get('automatic_prediction_diagnostic') is not True or
                c.get('common_friction_identification') or len(conditions) != 1 or
                conditions[0]['role'] != 'development' or conditions[0]['joint'] not in range(6) or
                any(c.get(k) for k in ('automatic_sparse_collection', 'automatic_shifted_angle_diagnostic',
                    'automatic_spatial_history_diagnostic', 'automatic_cross_angle_validation',
                    'automatic_zero_load_reference', 'automatic_repeat_diagnostic',
                    'automatic_preconditioned_entry_diagnostic'))):
            raise ValueError('Hold-history requires one development prediction diagnostic; no automatic fit')
        pose = np.asarray(conditions[0]['q_rad'])
        if (pose.shape != (6,) or not np.isfinite(pose).all() or
                len(condition_speeds(c, conditions[0], speeds)) != 1 or c['dwell_s'] != 3.):
            raise ValueError('Hold-history requires one finite pose/speed and the fixed 3-second H hold')
    preconditioned = c.get('automatic_preconditioned_entry_diagnostic', False)
    if preconditioned:
        if (role != 'development' or c.get('automatic_prediction_diagnostic') is not True or
                c.get('common_friction_identification') or len(conditions) != 1 or
                conditions[0]['role'] != 'development' or conditions[0]['joint'] not in range(6) or
                any(c.get(k) for k in ('automatic_sparse_collection', 'automatic_shifted_angle_diagnostic',
                    'automatic_spatial_history_diagnostic', 'automatic_cross_angle_validation',
                    'automatic_zero_load_reference'))):
            raise ValueError('Preconditioned entry requires one development prediction diagnostic; no automatic fit')
        pose = np.asarray(conditions[0]['q_rad'])
        if (pose.shape != (6,) or not np.isfinite(pose).all() or
                len(condition_speeds(c, conditions[0], speeds)) != 1 or
                not np.isfinite(c['dwell_s']) or c['dwell_s'] <= 0):
            raise ValueError('Preconditioned entry requires one finite pose, one speed and a positive fixed hold')
    for condition in conditions:
        for v in condition_speeds(c, condition, speeds):
            plateau_duration(c, v, condition['joint'], condition)
    plateau_min = c.get('automatic_minimum_plateau_s', 3.)
    settled_travel = c.get('automatic_minimum_settled_travel_rad', .022)
    if not np.isfinite(settled_travel) or settled_travel < .022:
        raise ValueError('Settled travel cannot be reduced below the original acquisition span')
    local_speed = c.get('automatic_reposition_velocity_rad_s', c.get('transfer_velocity_rad_s', c['max_velocity_rad_s']))
    repeats = c.get('automatic_repeats', 2)
    if type(repeats) is not int or repeats not in (1, 2) or repeats == 1 and not c.get('automatic_sparse_collection'):
        raise ValueError('One paired pass is only supported by explicit sparse collection')
    if not np.isfinite([plateau_min, local_speed]).all() or plateau_min < 1.5 or not 0 < local_speed <= c['max_velocity_rad_s']:
        raise ValueError('Insufficient plateau or invalid local reposition speed')
    if c.get('automatic_j6_high_speed') and any(p['joint'] != 5 for p in conditions):
        raise ValueError('High-speed configuration is specific to J6; other joints require their own design')
    shifted = c.get('automatic_shifted_angle_diagnostic', False)
    if shifted and (role != 'development' or any(p['joint'] != 5 for p in conditions)):
        raise ValueError('Shifted-angle diagnostic is J6 development, not matched-load final validation')
    spatial = c.get('automatic_spatial_history_diagnostic', False)
    if spatial:
        joints = {p['joint'] for p in conditions}
        if (role != 'development' or len(joints) != 1 or len(conditions)<2 or
                any(p['role']!='development' for p in conditions) or
                c.get('common_friction_identification') or shifted or c.get('automatic_repeat_diagnostic')):
            raise ValueError('Spatial-history diagnostic requires one joint, multiple centres and no automatic fit')
        j = next(iter(joints))
        poses = np.asarray([p['q_rad'] for p in conditions])
        if (j not in range(6) or poses.shape!=(len(conditions),6) or not np.isfinite(poses).all() or
                np.ptp(poses[:,j])<1e-6 or np.max(np.ptp(np.delete(poses,j,axis=1),axis=0))>1e-10):
            raise ValueError('Only the selected joint centre may change in a spatial-history diagnostic')
    dt = c['sample_period_s']; current = np.zeros(6); count = 0
    chunks, segments, cycles = [], [], []
    def add(q, phase, **tags):
        nonlocal current, count
        if len(q) < 2 or not np.allclose(q[0], current, atol=1e-10):
            raise ValueError('Discontinuous motion')
        s = dict(start=count, stop=count+len(q)-1, phase=phase, joint=-1,
                 speed_rad_s=0., repeat=-1, event='', validation_pose='', study_role='transition')
        s.update(tags); segments.append(s)
        if not timing_only:
            chunks.append(q[:-1])
        count += len(q)-1
        current = q[-1].copy()
    def move(q, *, local=False):
        if max(abs(current-q)) > 1e-9:
            reposition = c
            if ((local and c.get('automatic_sparse_collection')) or
                    c.get('automatic_j6_high_speed') and np.allclose(current[:5], q[:5], atol=1e-10)):
                reposition = dict(c, transfer_velocity_rad_s=local_speed)
            add(transfer_plan(current, q, reposition)['q'], 'transition')
    def hold(duration):
        add(np.tile(current, (int(np.ceil(duration/dt))+1, 1)), 'hold', dwell_s=duration)
    def condition_move(q):
        # Consecutive measurements at the same posture need no zero-and-back
        # excursion. Keep every pair/dwell; retain the zero waypoint whenever
        # the arm posture actually changes.
        if np.allclose(current, q, atol=1e-10, rtol=0.):
            return
        # Only the diagnostic's J6 centre changes directly. A changed arm
        # posture still follows the original zero-waypoint transfer route.
        if not (shifted and np.allclose(current[:5], q[:5], atol=1e-10)):
            move(np.zeros(6))
        move(q)
    def pair(condition, speed, repeat, dwell, reference=False, history=None):
        motion_start = count
        j = condition['joint']; q = np.asarray(condition['q_rad'])
        ramp = np.ceil(max(c['ramp_s'], np.pi*speed/(2*c['max_acceleration_rad_s2']))/dt)*dt
        plateau = plateau_duration(c, speed, j, condition)
        stroke = speed*(plateau+ramp)
        if stroke/2 > c.get('automatic_target_half_range_rad', float('inf')):
            raise ValueError('Requested speed has insufficient room for ramp and settled plateau')
        lo = q.copy(); hi = q.copy(); lo[j] -= stroke/2; hi[j] += stroke/2
        directions = (1, -1) if preconditioned or history is not None or repeat == 0 else (-1, 1)
        if history is not None:
            move(lo, local=True)
        elif not preconditioned:
            move(lo if repeat == 0 else hi, local=True); hold(dwell)
        cycle_id = len(cycles)
        cycles.append(dict(cycle_id=cycle_id, group=condition['name'], joint=j,
            role=role, reference=reference, speed_rad_s=speed, repeat_order=repeat,
            dwell_s=dwell, stroke_rad=stroke, plateau_s=plateau,
            settled_travel_rad=speed*(plateau-2*c['sensor_settling_s'][j])))
        if preconditioned or history is not None:
            cycles[-1]['direction_order'] = list(directions)
        if history is not None:
            block, label, index = history
            cycles[-1].update(history_block=block, history_condition=label,
                history_role='conditioning' if index == 0 else 'evaluation',
                history_pair_index=index, diagnostic_only=True)
        for direction in directions:
            if history is not None and history[1] == 'H':
                hold(dwell)
            if preconditioned:
                # Local centre, not the robot zero waypoint. Reversal and
                # transfer entry must share the same approach before a sweep.
                move(q, local=True); hold(dwell)
                move(lo if direction > 0 else hi, local=True); hold(dwell)
            for event, duration in [('start', ramp), ('steady', plateau), ('stop', ramp)]:
                t = np.arange(round(duration/dt)+1)*dt
                x = speed*t if event == 'steady' else speed/2*(t+(-1 if event == 'start' else 1)*ramp/np.pi*np.sin(np.pi*t/ramp))
                target = np.tile(current, (len(t), 1)); target[:, j] += direction*x
                add(target, 'posture_validation_'+('steady' if event == 'steady' else 'transient'),
                    joint=j, speed_rad_s=direction*speed, repeat=cycle_id, full_cycle=cycle_id,
                    event=event, validation_pose=condition['name'], study_role=role)
            if history is None:
                hold(dwell)
        if history is None:
            move(q, local=True)
        cycles[-1].update(motion_start_index=motion_start, motion_stop_index=count)
    hold(c['dwell_s'])
    selected = [p for p in conditions if p['role'] == role]
    if not selected:
        raise ValueError('No conditions in requested role')
    # Counterbalance condition order AND direction/speed order. Reference pairs
    # bracket each joint block so time/history drift remains visible.
    for j in sorted({p['joint'] for p in selected}):
        tasks = [p for p in selected if p['joint'] == j]
        condition_move(np.asarray(tasks[0]['q_rad']))
        if hold_history:
            hold(c['dwell_s'])
            speed = condition_speeds(c, tasks[0], speeds)[0]
            for block, label in enumerate(('H', 'N', 'N', 'H')):
                for index in range(3):
                    pair(tasks[0], speed, 0, 3. if label == 'H' else 0.,
                         history=(block, label, index))
            hold(c['dwell_s'])
            move(np.asarray(tasks[0]['q_rad']))
            move(np.zeros(6)); hold(c['dwell_s'])
            continue
        reference_speed = condition_speeds(c, tasks[0], speeds)[0] if preconditioned else .04
        pair(tasks[0], reference_speed, 0, c['dwell_s'], True)
        for repeat in range(repeats):
            for condition in tasks if repeat == 0 else reversed(tasks):
                condition_move(np.asarray(condition['q_rad']))
                local_speeds = condition_speeds(c, condition, speeds)
                for k, speed in enumerate(local_speeds if repeat == 0 else tuple(reversed(local_speeds))):
                    # Rest durations are explicit experimental factors. Never
                    # shorten below the existing rest/settling requirement.
                    dwell = c['dwell_s'] if preconditioned else (c['dwell_s'], 3., 5.)[(k+repeat) % 3]
                    pair(condition, speed, repeat, dwell)
        condition_move(np.asarray(tasks[0]['q_rad']))
        pair(tasks[0], reference_speed, 1, c['dwell_s'], True)
        move(np.zeros(6)); hold(c['dwell_s'])
    if timing_only:
        return dict(duration_s=float(count*dt), expected_paired_cycles=len(cycles)), None
    q = np.vstack(chunks+[current[None]]); t = np.arange(len(q))*dt
    v = np.gradient(q, dt, axis=0, edge_order=2); a = np.gradient(v, dt, axis=0, edge_order=2)
    if max(abs(v).flat) > c['max_velocity_rad_s']+1e-7 or max(abs(a).flat) > c['max_acceleration_rad_s2']+1e-7:
        raise ValueError('Generated kinematics exceed the existing acquisition settings')
    meta = dict(schema=1, config=deepcopy(c), joints=JOINTS, segments=segments,
        duration_s=float(t[-1]), generalization_study=True, record_only=True,
        automatic_targeted=True, automatic_role=role, automatic_conditions=conditions,
        automatic_condition_design=('sparse_speed_load' if c.get('automatic_sparse_collection') else
                                    'shifted_angle_diagnostic' if shifted else
                                    'spatial_history_diagnostic' if spatial else
                                    'repeat_diagnostic' if hold_history or preconditioned or c.get('automatic_repeat_diagnostic') else 'matched_angle_load'),
        automatic_cycles=cycles, frozen_posture_roles={p['name']: p['role'] for p in conditions},
        expected_paired_cycles=len(cycles), roles_selected_without_measured_friction=not (hold_history or preconditioned or spatial or c.get('automatic_repeat_diagnostic',False)),
        repeated_condition_speed_pairs=repeats == 2,
        bracketing_reference_pairs_per_joint=2,
        motion_limits_rad=np.maximum(abs(q).max(axis=0), np.asarray(c['joint_excursion_rad'])).tolist(),
        temperature_model=False, current_control_probe=False, compensation_qualified=False)
    if hold_history:
        meta['automatic_entry_protocol'] = 'hold_history_hnnh_v1'
        meta['diagnostic_only'] = True
    if preconditioned:
        meta['automatic_entry_protocol'] = 'centre_hold_approach_hold_v1'
    return meta, dict(t=t, q=q, v=v, a=a)


def check_validation_scope(candidate,joints,diagnostic):
    common=candidate.get('schema') in ('common_friction_v1','common_periodic_friction_v1')
    if diagnostic and candidate.get('schema') not in ('single_joint_signed_load_v1','single_joint_signed_load_v2',
            'single_joint_signed_load_v3','common_friction_v1','common_periodic_friction_v1'):
        raise ValueError('Prediction diagnostic requires the explicit single-joint experimental model')
    unresolved=set(candidate.get('missing',[]))
    if common:
        # These are downstream stages, not evidence that development passed.
        unresolved-={'fresh_validation','joint_specific_transition_validation','kinetic_release_coverage'}
    scope=candidate.get('joints',[candidate.get('joint')])
    if (unresolved and not diagnostic) or set(scope)!={j+1 for j in joints}:
        raise ValueError('Resolve development failures and match candidate joint scope before final collection')


def prepare(c, cycles, urdf_path, baseline, output, joints, role, candidate=None, conditions=None, max_minutes=90.):
    output = Path(output)
    if output.exists():
        raise ValueError('Choose a new preparation directory')
    scaling = original_gravity(baseline).get('torque_scaling_factors', [1.]*6)
    urdf = Path(urdf_path).read_text()
    if conditions is None:
        conditions = [p for j in joints for p in select_conditions(cycles, j, c, urdf_path, scaling)]
    if set(p['joint'] for p in conditions) != set(joints):
        raise ValueError('Condition joint scope differs from requested joints')
    reference_only = c.get('automatic_zero_load_reference', False)
    repeat_diagnostic = (c.get('automatic_repeat_diagnostic', False) or
                         c.get('automatic_preconditioned_entry_diagnostic', False) or
                         c.get('automatic_hold_history_diagnostic', False))
    spatial_diagnostic = c.get('automatic_spatial_history_diagnostic', False)
    if repeat_diagnostic and (role != 'development' or len(joints) != 1 or len(conditions) != 1
            or conditions[0]['role'] != 'development' or c.get('common_friction_identification')):
        raise ValueError('Repeat diagnostic requires one joint/condition and cannot fit or deploy automatically')
    if reference_only and (role != 'development' or len(joints) != 1 or len(conditions) != 1 or
        conditions[0]['role'] != 'development' or not np.allclose(conditions[0]['q_rad'], np.zeros(6), atol=1e-10)):
        raise ValueError('Zero reference requires one joint and one all-zero development condition')
    cross_angle = c.get('automatic_cross_angle_validation', False)
    sparse = c.get('automatic_sparse_collection', False)
    if sparse and (not c.get('common_friction_identification') or spatial_diagnostic or
                   repeat_diagnostic or reference_only or c.get('automatic_shifted_angle_diagnostic')):
        raise ValueError('Sparse schedules require common identification without diagnostic flags')
    if sparse:
        check_sparse_holdouts(c, conditions, cycles)
    if cross_angle and (role != 'fresh_final_validation' or not c.get('automatic_prediction_diagnostic')):
        raise ValueError('Cross-angle collection is an explicitly reserved prediction diagnostic')
    for j in joints:
        selected = [p for p in conditions if p['joint'] == j]
        if (len({p['name'] for p in selected}) != len(selected) or
                (not sparse and not reference_only and not repeat_diagnostic and not spatial_diagnostic and any(sum(p['role'] == r for p in selected) < 2 for r in ('development', 'fresh_final_validation'))) or
                (not sparse and not c.get('automatic_shifted_angle_diagnostic') and not spatial_diagnostic and
                 np.ptp([p['q_rad'][j] for p in selected if not cross_angle or p['role']=='development']) > 1e-10)):
            raise ValueError('Need two development and two reserved conditions at the same target angle')
    if role == 'fresh_final_validation':
        if not candidate or candidate['identity'] != dict(robot_unit=c['robot_unit'],
                robot_fingerprint=fingerprint(urdf), baseline_sha256=sha(baseline)):
            raise ValueError('Freeze this robot/baseline candidate before final validation')
        diagnostic = c.get('automatic_prediction_diagnostic', False)
        check_validation_scope(candidate,joints,diagnostic)
    meta, data = generate(c, conditions, role)
    meta['unqualified_prediction_diagnostic'] = bool(c.get('automatic_prediction_diagnostic', False))
    meta['cross_angle_prediction_diagnostic'] = bool(cross_angle)
    meta['zero_load_reference_only'] = bool(reference_only)
    ready = completion_pose(c, urdf)['q_rad']
    homing = transfer_plan(ready, np.zeros(6), c); ending = transfer_plan(np.zeros(6), ready, c)
    total = meta['duration_s']+homing['t'][-1]+ending['t'][-1]
    if not np.isfinite(max_minutes) or max_minutes <= 0 or total > max_minutes*60:
        raise ValueError(f'{total/60:.1f} minutes required; split by --joints. No measurements silently removed.')
    positions = np.vstack([homing['q'], data['q'], ending['q']])
    # Includes the actual pose limits in this new reviewed protocol; inherited
    # old narrow acquisition limits are not a substitute for geometry checks.
    meta['motion_limits_rad'] = np.maximum(abs(positions).max(axis=0), meta['motion_limits_rad']).tolist()
    validate_urdf_plan(meta, dict(q=positions), urdf)
    geometry = check_geometry(positions, urdf_path)
    if not geometry['passed']:
        raise ValueError('Matched-angle proposal fails full sampled self/table clearance; no executable plan written')
    output.mkdir(parents=True)
    np.savez_compressed(output/'trajectory.npz', **data)
    meta.update(sha256=sha(output/'trajectory.npz'), source_hashes=sources(),
        automatic_generator_sha256=sha(__file__), robot_fingerprint=fingerprint(urdf),
        reuse_cycles_sha256=content_hash(cycles),
        automatic_candidate_sha256=content_hash(candidate) if candidate else None,
        baseline_sha256=sha(baseline))
    write_new(output/'plan.json', meta)
    review = dict(plan_sha256=sha(output/'plan.json'), trajectory_sha256=meta['sha256'],
        source_hashes=sources(), automatic_generator_sha256=sha(__file__),
        robot_fingerprint=fingerprint(urdf), geometry=geometry, duration_s=meta['duration_s'],
        total_with_nominal_homing_completion_s=float(total), includes_ready_homing_and_completion=True,
        hardware_started=False, compensation_qualified=False)
    write_new(output/'motion_review.json', review)
    write_new(output/'conditions.json', conditions)
    return review


def load_reviewed(folder, c, urdf):
    from .generalization_motion import load_reviewed as load_base
    meta, data = load_base(folder, c, urdf)
    review = json.loads((Path(folder)/'motion_review.json').read_text())
    if (not meta.get('automatic_targeted') or meta.get('automatic_generator_sha256') != sha(__file__)
            or review.get('automatic_generator_sha256') != sha(__file__)):
        raise ValueError('Automatic measurement generator changed after geometry review')
    return meta, data


def copy_reviewed(folder, output, c, urdf):
    from .generalization_motion import copy_reviewed as copy_base
    load_reviewed(folder, c, urdf)
    return copy_base(folder, output, c, urdf)


def reviewed_homing(q, velocity, c, urdf_path):
    """Review the actual start, not an arbitrary widened ready neighbourhood."""
    q, velocity = np.asarray(q, float), np.asarray(velocity, float)
    if (q.shape != (6,) or velocity.shape != (6,) or
            not np.isfinite([q, velocity]).all() or max(abs(velocity)) > .01):
        raise ValueError('Measured start must be finite and stationary')
    data = transfer_plan(q, np.zeros(6), c)
    validate_urdf_plan({}, data, Path(urdf_path).read_text())
    geometry = check_geometry(data['q'], urdf_path)
    if not geometry['passed']:
        raise ValueError('Actual-start homing fails self/table clearance')
    return data, dict(start_region='sampled_geometry_reviewed_current_pose',
        start_rad=q.tolist(), target_rad=[0.]*6, duration_s=float(data['t'][-1]),
        geometry=geometry, maximum_velocity_rad_s=float(np.max(abs(data['v']))))


def reference_check(cycles):
    """Compare equal-condition bracketing references without fitting a drift correction."""
    checks = []
    for group in sorted({(x['joint'], x['group']) for x in cycles}):
        rows = [x for x in cycles if (x['joint'], x['group']) == group]
        if len(rows) != 2 or not np.isclose(rows[0]['speed'], rows[1]['speed']):
            raise ValueError('Missing bracketing reference pair')
        a, b = [float(np.mean(x['target'])) for x in rows]
        allowance = .08+.20*min(a, b)
        checks.append(dict(joint=group[0]+1, group=group[1], before_nm=a, after_nm=b,
                           difference_nm=b-a, passed=abs(b-a) <= allowance))
    return dict(passed=bool(checks) and all(x['passed'] for x in checks), checks=checks,
                drift_correction_fitted=False)


def ingest(folder, baseline, allow_synthetic=False, spatial_bins=24):
    """Convert a completed targeted ROS acquisition, preserving raw provenance."""
    from .fit import validate_log, pair_samples
    from .generalization import bin_cycles, reject_contact
    from .settling import replay
    folder = Path(folder)
    for parent in (folder, folder.parent, folder.parent.parent):
        reject_contact(parent)
    meta = json.loads((folder/'plan.json').read_text())
    collected = json.loads((folder/'collection.json').read_text())
    if (not meta.get('automatic_targeted') or not collected.get('completed') or collected.get('failure')
            or collected.get('source') not in (('hardware', 'synthetic') if allow_synthetic else ('hardware',))
            or sha(baseline) != meta['baseline_sha256']
            or sha(folder/'trajectory.npz') != meta['sha256']
            or collected.get('plan_sha256') != meta['sha256']):
        raise ValueError('Incomplete, synthetic or mismatched targeted acquisition')
    # Completed measurements remain analysable after a generator update. Verify
    # the archived plan/review bytes here; execution still requires current
    # source hashes through load_reviewed(). No motion is issued by ingest().
    review = json.loads((folder/'motion_review.json').read_text())
    urdf = (folder/'robot.urdf').read_text()
    if (review.get('plan_sha256') != sha(folder/'plan.json') or
            review.get('trajectory_sha256') != meta['sha256'] or
            review.get('automatic_generator_sha256') != meta.get('automatic_generator_sha256') or
            review.get('source_hashes') != meta.get('source_hashes') or
            review.get('robot_fingerprint') != fingerprint(urdf) or
            meta.get('robot_fingerprint') != fingerprint(urdf) or
            not review.get('geometry',{}).get('passed')):
        raise ValueError('Archived plan/review integrity mismatch')
    validate_urdf_plan(meta, dict(np.load(folder/'trajectory.npz')), urdf)
    data = dict(np.load(folder/'measurements.npz'))
    validate_log(meta, data)
    settled = replay(meta, data)
    if not settled['passed']:
        raise ValueError('Rest checks failed')
    rows, starts = pair_samples(meta, data, include_directional=True)
    if len(rows) != meta['expected_paired_cycles']:
        raise ValueError('Missing complete paired cycles')
    scaling = original_gravity(baseline).get('torque_scaling_factors', [1.]*6)
    cycles = bin_cycles(rows, Robot(folder/'robot.urdf', JOINTS), scaling,bins=spatial_bins)
    if meta['config'].get('automatic_hold_history_diagnostic'):
        # These are an intervention diagnostic, not bracketing references or
        # fitting/validation cycles. Preserve every conditioning pair as well.
        labels = meta['automatic_cycles']
        if (meta.get('automatic_entry_protocol') != 'hold_history_hnnh_v1' or
                meta.get('automatic_role') != 'development' or
                meta['config'].get('common_friction_identification') or
                len(labels) != 12 or len(cycles) != 12 or
                [x['cycle_id'] for x in labels] != list(range(12)) or
                [x.get('history_condition') for x in labels] != ['H']*3+['N']*6+['H']*3 or
                [x.get('history_role') for x in labels] != ['conditioning','evaluation','evaluation']*4 or
                [x.get('history_block') for x in labels] != [0]*3+[1]*3+[2]*3+[3]*3 or
                any(x['reference'] for x in labels)):
            raise ValueError('Incomplete or changed hold-history diagnostic schedule')
        token = sha(folder/'measurements.npz')[:16]
        for x in cycles:
            label = labels[x['repeat']]
            x['cycle_id'] = token+'/'+x['cycle_id']
            x.update({k: label[k] for k in ('history_block','history_condition','history_role','history_pair_index')})
            x['diagnostic_only'] = True
        for start in starts:
            held = labels[start['repeat']]['history_condition'] == 'H'
            start.update(start_kind='held_start' if held else 'reversal',
                         preceding_rest_observed=held, breakaway_use_permitted=False)
            if not held:
                # The preceding half-second is deceleration, never a rest or
                # a static breakaway observation. The unchanged raw remains.
                start.update(resting_effort_nm=None, onset_s=None,
                    resting_effort_applicability='not_applicable', onset_applicability='not_applicable')
        evidence = dict(source=collected['source'], role='diagnostic', completed=True,
            purpose='hold_history_diagnostic', candidate_sha256=meta['automatic_candidate_sha256'],
            contact_free=True, contact_free_means_no_reported_contact_not_independent_contact_detection=True,
            identity=dict(robot_unit=meta['config']['robot_unit'], robot_fingerprint=fingerprint(urdf),
                          baseline_sha256=sha(baseline)), cycle_ids=[x['cycle_id'] for x in cycles],
            source_files_sha256={name: file_hash(folder/name) for name in
                ('plan.json','trajectory.npz','measurements.npz','collection.json','robot.urdf')},
            automatic_fit=False, fresh_final_validation=False, compensation_qualified=False)
        from .spatial_review import compare_spatial_support
        return dict(cycles=[], diagnostic_cycles=cycles, reference_cycles=[], evidence=evidence,
            reference_check=dict(applicable=False, reason='H/N intervention; separate H/H analysis required'),
            directional_pairs=rows, spatial_support_review=compare_spatial_support(rows),
            starts=starts, static_breakaway_identified=False)
    refs = {x['cycle_id'] for x in meta['automatic_cycles'] if x['reference']}
    reference = [x for x in cycles if x['repeat'] in refs]
    stability = reference_check(reference)
    if not stability['passed']:
        raise ValueError('Bracketing reference changed; separate load effects from time/history before fitting')
    cycles = [x for x in cycles if x['repeat'] not in refs]
    token = sha(folder/'measurements.npz')[:16]
    for x in cycles:
        x['cycle_id'] = token+'/'+x['cycle_id']
    evidence = dict(source=collected['source'], role=meta['automatic_role'], completed=True,
        candidate_sha256=meta['automatic_candidate_sha256'], contact_free=True,
        contact_free_means_no_reported_contact_not_independent_contact_detection=True,
        identity=dict(robot_unit=meta['config']['robot_unit'], robot_fingerprint=meta['robot_fingerprint'],
                      baseline_sha256=sha(baseline)),
        purpose=meta['automatic_role'], cycle_ids=[x['cycle_id'] for x in cycles],
        source_files_sha256={name: file_hash(folder/name) for name in
            ('plan.json', 'trajectory.npz', 'measurements.npz', 'collection.json', 'robot.urdf')})
    from .spatial_review import compare_spatial_support
    result=dict(cycles=cycles, evidence=evidence, reference_cycles=reference, reference_check=stability,
                directional_pairs=rows, spatial_support_review=compare_spatial_support(rows),
                starts=starts, static_breakaway_identified=False)
    if meta['config'].get('combined_current_response'):
        from .current_response import validate_shared_timing
        response=validate_shared_timing(meta['config']['combined_current_response'],evidence['identity'])
        result['current_reporting_response']=dict(profile=response,used_for='reporting_response_sensitivity',
            friction_samples_deconvolved=False,torque_constants_changed=False)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    s = sub.add_parser('prepare')
    for key in ('config', 'cycles', 'urdf', 'baseline', 'output'): s.add_argument('--'+key, required=True)
    s.add_argument('--joints', default='1,2,3,4,5,6')
    s.add_argument('--role', choices=['development', 'fresh_final_validation'], default='development')
    s.add_argument('--candidate'); s.add_argument('--conditions'); s.add_argument('--max-minutes', type=float, default=90.)
    s = sub.add_parser('ingest')
    for key in ('collection', 'baseline', 'output'): s.add_argument('--'+key, required=True)
    args = p.parse_args()
    read = lambda path: json.loads(Path(path).read_text())
    if args.command == 'prepare':
        joints = [int(j)-1 for j in args.joints.split(',')]
        if not joints or len(set(joints)) != len(joints) or any(j not in range(6) for j in joints):
            p.error('Invalid joint list')
        result = prepare(config(args.config), read(args.cycles), args.urdf, args.baseline, args.output,
            joints, args.role, read(args.candidate) if args.candidate else None,
            read(args.conditions) if args.conditions else None, args.max_minutes)
        print(json.dumps(result, indent=2))
    else:
        result = ingest(args.collection, args.baseline)
        out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
        for name, value in result.items(): write_new(out/(name+'.json'), value)
        print(json.dumps(dict(cycles=len(result['cycles']), output=str(out), hardware_deployable=False)))


if __name__ == '__main__':
    main()
