"""Counterbalanced J2 history experiment; position collection, no compensation."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .completion import completion_pose, transfer_plan
from .generalization_motion import check_geometry, sources
from .integration import fingerprint
from .protocol import JOINTS, sha, validate_urdf_plan

SERVICE = '/friction_diagnostic/arm/get_dxl_data'


def diagnostic_description(urdf):
    """Separate the arm read service from the gripper, without register changes."""
    root = ET.fromstring(urdf); changed = 0
    for hardware in root.findall('ros2_control'):
        if [j.get('name') for j in hardware.findall('joint')] != JOINTS:
            continue
        p = hardware.find("hardware/param[@name='get_dynamixel_data_srv_name']")
        if p is None:
            raise ValueError('Expected real arm DHI read-service parameter')
        p.text = SERVICE; changed += 1
    if changed != 1:
        raise ValueError('Expected exactly one six-axis arm hardware component')
    return ET.tostring(root, encoding='unicode')


def generate_history(c, design):
    method = design.get('method', 'counterbalanced_history_v1')
    convergence = method == 'repeated_history_v1'
    matched = method == 'time_matched_history_v1'
    if method not in ('counterbalanced_history_v1', 'repeated_history_v1', 'time_matched_history_v1'):
        raise ValueError('Unknown history experiment method')
    if design.get('executable') is not False or len(design['blocks']) != (8 if matched else 4 if convergence else 16):
        raise ValueError('Unexpected frozen exploratory design')
    if convergence and ''.join(b['condition'] for b in design['blocks']) != 'ABBA':
        raise ValueError('Convergence study requires frozen ABBA ordering')
    if matched and [(b['condition'],b.get('extra_rest_s')) for b in design['blocks']] != [
            ('A',0.),('B',28.),('B',0.),('A',28.),('A',28.),('B',0.),('B',28.),('A',0.)]:
        raise ValueError('Time-matched study requires balanced frozen rest/conditioning order')
    anchor = np.asarray(design['source_pose']['q_rad'], dtype=float)
    if anchor.shape != (6,) or design['source_pose']['sweep_sides'][1] != 1:
        raise ValueError('Expected reviewed one-sided load_01 posture')
    dt = c['sample_period_s']; chunks = []; segments = []; count = 0
    zero = np.zeros(6); current = zero.copy(); cycles = []; cycle_id = 0
    def add(q, phase, **tags):
        nonlocal count, current
        if not np.allclose(q[0], current, atol=1e-10, rtol=0):
            raise ValueError('Discontinuous experiment position')
        chunks.append(q[:-1]); segments.append(dict(start=count, stop=count+len(q)-1,
            phase=phase, joint=-1, speed_rad_s=0., repeat=-1, event='',
            validation_pose='load_01', study_role='exploratory_history', **tags))
        count += len(q)-1; current = q[-1].copy()
    def rest(seconds, tags=None):
        add(np.tile(current, (round(seconds/dt)+1, 1)), 'hold', **(tags or {}))
    def move(target):
        if np.max(abs(current-target)) > 1e-12:
            add(transfer_plan(current, target, c)['q'], 'transition')
    def pair(requested, stroke, role, block, condition, target_speed, repeat):
        nonlocal cycle_id
        move(anchor)
        ramp = np.ceil(max(c['ramp_s'], np.pi*requested/(2*c['max_acceleration_rad_s2']))/dt)*dt
        plateau = np.ceil((stroke/requested-ramp-1e-10)/dt)*dt
        if plateau < .5:
            raise ValueError('Insufficient constant-speed interval')
        speed = stroke/(plateau+ramp)  # common exact endpoints; <= one tick timing quantization
        cycle_id += 1
        info = dict(cycle_id=cycle_id, block=block, condition=condition, history_role=role,
                    target_speed_rad_s=target_speed, pair_repeat=repeat,
                    requested_speed_rad_s=requested, actual_speed_rad_s=speed,
                    stroke_rad=stroke, lower_rad=float(anchor[1]), upper_rad=float(anchor[1]+stroke))
        if matched:
            info['extra_conditioning_rest_s'] = design['blocks'][block-1]['extra_rest_s']
        cycles.append(info)
        tags = dict(history_cycle=cycle_id, history_role=role, history_block=block,
                    history_condition=condition, history_target_speed=target_speed)
        rest(2., tags)
        for direction in (1, -1):
            for event, duration in [('start', ramp), ('steady', plateau), ('stop', ramp)]:
                t = np.arange(round(duration/dt)+1)*dt
                if event == 'steady':
                    x = speed*t
                else:
                    sign = -1 if event == 'start' else 1
                    x = speed/2*(t+sign*ramp/np.pi*np.sin(np.pi*t/ramp))
                q = np.tile(current, (len(t), 1)); q[:, 1] += direction*x
                add(q, 'posture_validation_'+('steady' if event == 'steady' else 'transient'), **tags)
                segments[-1].update(joint=1, speed_rad_s=direction*speed, repeat=cycle_id, event=event)
            rest(2., tags)
        if not np.allclose(current, anchor, atol=1e-10, rtol=0):
            raise ValueError('Pair did not return to identical lower endpoint')
    rest(3.); move(anchor)
    for b in design['blocks']:
        repeats = 4 if matched else 6 if convergence else 2
        if (b['target_joint'] != 'joint2' or b['target_repeats'] != repeats or b['common_rest_s'] != 2.
                or ((convergence or matched) and b['speed_rad_s'] != .004)):
            raise ValueError('Unexpected history design')
        speed = b['speed_rad_s']; cond = b['condition']; block = b['block']
        if cond not in ('A','B'):
            raise ValueError('Unknown preconditioning')
        pair(.04, .18, 'reference_before', block, cond, speed, 0)
        conditioning_begin=count*dt
        if matched and cond=='B':
            # Both conditioning strokes have the same ramp and dwell times.
            # Match reference->conditioning-end time by waiting BEFORE B,
            # preserving the deliberately separate post-conditioning delay.
            ramp=np.ceil(max(c['ramp_s'],np.pi*.1/(2*c['max_acceleration_rad_s2']))/dt)*dt
            slow=np.ceil((.45/.012-ramp-1e-10)/dt)*dt
            fast=np.ceil((.45/.1-ramp-1e-10)/dt)*dt
            rest(2*(slow-fast))
        pair(.012 if cond=='A' else .10, .45, 'conditioning', block, cond, speed, 0)
        if matched:
            cycles[-1]['matched_conditioning_duration_s']=count*dt-conditioning_begin
            if b['extra_rest_s']:
                rest(b['extra_rest_s'])
        for repeat in range(repeats):
            pair(speed, np.pi/120, 'target', block, cond, speed, repeat)
        pair(.04, .18, 'reference_after', block, cond, speed, 0)
    move(zero); rest(3.)
    q = np.vstack(chunks+[current[None]]); t = np.arange(len(q))*dt
    v = np.gradient(q,dt,axis=0,edge_order=2); a = np.gradient(v,dt,axis=0,edge_order=2)
    if np.max(abs(v)) > c['max_velocity_rad_s'] or np.max(abs(a)) > c['max_acceleration_rad_s2']:
        raise ValueError('Motion limit exceeded')
    meta = dict(schema=1, config=deepcopy(c), joints=JOINTS, segments=segments,
        duration_s=float(t[-1]), generalization_study=True, history_study=True,
        method=method, record_only=True, independent_repeat_validation=False,
        frozen_posture_roles={'load_01':'exploratory_history'}, expected_paired_cycles=len(cycles),
        history_design=design, history_cycles=cycles, source_pose=design['source_pose'],
        motion_limits_rad=[.8,2.,3.,1.7,1.9,.8], sensor_delay_identified=False,
        compensation_qualified=False, temperature_model=False,
        telemetry=dict(service=SERVICE, joint_id=2, request_rate_hz=5.,
            items=['Goal Position','Position Trajectory','Velocity Trajectory']+(['Present Velocity'] if convergence or matched else []),
            simultaneous_sampling=False, millisecond_delay_identification=False),
        scope='Exploratory J2 history diagnosis. No coefficient fitting or freedrive deployment.')
    return meta, dict(t=t,q=q,v=v,a=a)


def prepare(c, design_path, urdf_path, output):
    root = Path(output); root.mkdir(parents=True, exist_ok=False)
    design = json.loads(Path(design_path).read_text()); meta,data=generate_history(c,design)
    urdf=Path(urdf_path).read_text(); ready=completion_pose(c,urdf)['q_rad']
    homing=transfer_plan(ready,np.zeros(6),c); ending=transfer_plan(np.zeros(6),ready,c)
    positions=np.vstack([homing['q'],data['q'],ending['q']]); validate_urdf_plan(meta,dict(q=positions),urdf)
    from .trajectory_transport import compact_points
    _,transport=compact_points(data,[s[k] for s in meta['segments'] for k in ('start','stop')],
                               acceleration_limit=c['max_acceleration_rad_s2'])
    np.savez_compressed(root/'trajectory.npz',**data)
    meta.update(sha256=sha(root/'trajectory.npz'),source_hashes=sources(),
                robot_fingerprint=fingerprint(urdf),design_sha256=sha(design_path))
    (root/'plan.json').write_text(json.dumps(meta,indent=2))
    review=dict(schema=1,plan_sha256=sha(root/'plan.json'),trajectory_sha256=meta['sha256'],
        source_hashes=sources(),robot_fingerprint=fingerprint(urdf),geometry=check_geometry(positions,urdf_path),
        positions_sha256=hashlib.sha256(positions.tobytes()).hexdigest(),transport=transport,
        duration_s=meta['duration_s'],homing_duration_s=float(homing['t'][-1]),completion_duration_s=float(ending['t'][-1]),
        min_q_rad=data['q'].min(axis=0).tolist(),max_q_rad=data['q'].max(axis=0).tolist(),
        hardware_started=False,compensation_qualified=False)
    (root/'motion_review.json').write_text(json.dumps(review,indent=2))
    if not review['geometry']['passed']:raise ValueError('History motion fails geometry review')
    return review
