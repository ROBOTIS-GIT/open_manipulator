"""Neuromeka-inspired stop/sweep/reverse protocol, bounded for upright OMY.

No Indy joint poses or link parameters are imported. Equal-position direction
pairs identify mean friction; directional asymmetry is not separately observable.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

JOINTS = [f'joint{i}' for i in range(1, 7)]


def follower_ready_file():
    try:
        from ament_index_python.packages import get_package_share_directory
        base = Path(get_package_share_directory('open_manipulator_bringup'))
    except (ImportError, LookupError):
        base = Path(__file__).resolve().parents[2]/'open_manipulator_bringup'
    return base/'config/omy_f3m_follower_ai/initial_positions.yaml'


def follower_ready():
    path = follower_ready_file()
    p = yaml.safe_load(path.read_text())['joint_trajectory_executor']['ros__parameters']
    if len(p['joint_names']) != len(p['ready']) or len(set(p['joint_names'])) != len(p['joint_names']):
        raise ValueError('Invalid follower ready joint mapping')
    return [float(p['ready'][p['joint_names'].index(j)]) for j in JOINTS]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config(path, hardware=False, record_only=False, *, device_settings=False):
    c = yaml.safe_load(Path(path).read_text())
    from .device_identity import robot_unit
    c['robot_unit'] = robot_unit(c.get('robot_unit'))
    if device_settings is not False:
        from .device_settings import apply_settings
        c = apply_settings(c, None if device_settings is True else device_settings)
    if c.get('combined_current_response_profile'):
        from .current_response import validate_shared_timing
        if 'combined_current_response' not in c:
            profile=Path(path).resolve().parent/c['combined_current_response_profile']
            c['combined_current_response']=yaml.safe_load(profile.read_text())
        response=c['combined_current_response']
        validate_shared_timing(response,response['identity'])
        if response['identity']['robot_unit']!=c.get('robot_unit'):
            raise ValueError('Current reporting response belongs to another robot')
    if c.get('schema') != 3:
        raise ValueError('Use schema 3 configuration and recalibrate with held-out posture checks')
    if c.get('friction_model') not in ('velocity_only', 'velocity_load'):
        raise ValueError('friction_model must be velocity_only or velocity_load')
    if not isinstance(c.get('sensor_timing_verified'), bool):
        raise ValueError('sensor_timing_verified must be a YAML boolean')
    if 'sweep_range_scale' not in c:
        raise ValueError('Use the current range-only configuration template; retain device settings and recalibrate')
    if c['variant'] != 'omy_f3m' or c['initial_pose'] != [0]*6:
        raise ValueError('Only unprefixed OMY F3M with six-zero initial pose is supported')
    if c.get('completion_pose', 'follower_ready') not in ('follower_ready', 'zero'):
        raise ValueError('completion_pose must be follower_ready or zero')
    tolerance = c.get('homing_ready_tolerance_rad', .02)
    if not np.isfinite(tolerance) or not 0 < tolerance <= .1:
        raise ValueError('homing_ready_tolerance_rad must be in (0, 0.1]')
    transfer_speed = c.get('transfer_velocity_rad_s', c['max_velocity_rad_s'])
    if not np.isfinite(transfer_speed) or not 0 < transfer_speed <= c['max_velocity_rad_s']:
        raise ValueError('Transfer speed must be positive and no faster than the sweep bound')
    current_policy = c.get('current_abort_policy', 'software_threshold')
    if current_policy not in ('software_threshold', 'firmware_only'):
        raise ValueError('Unknown current_abort_policy')
    if current_policy == 'firmware_only':
        if c.get('measured_torque_abort_nm', 'missing') is not None:
            raise ValueError('firmware_only requires an explicit null measured_torque_abort_nm')
        if hardware and not record_only:
            raise ValueError('firmware_only is limited to position-mode record-only collection')
    else:
        a = np.asarray(c['measured_torque_abort_nm'], dtype=float)
        if a.shape != (6,) or not np.isfinite(a).all() or np.any(a < 0):
            raise ValueError('Invalid six-joint setting: measured_torque_abort_nm')
    for key in ('joint_excursion_rad', 'sensor_settling_s',
                'max_compensation_nm', 'slew_rate_nm_s', 'sweep_range_scale',
                'current_filter_time_constant_s', 'current_measurement_delay_s',
                'posture_validation_limits_rad'):
        a = np.asarray(c[key], dtype=float)
        if a.shape != (6,) or not np.isfinite(a).all() or np.any(a < 0):
            raise ValueError(f'Invalid six-joint setting: {key}')
    for key in ('sample_period_s', 'max_velocity_rad_s', 'max_acceleration_rad_s2',
                'ramp_s', 'plateau_s', 'dwell_s', 'velocity_deadband_rad_s',
                'validation_relative_error', 'validation_absolute_error_nm',
                'max_pair_spread_nm', 'homing_max_displacement_rad'):
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'Invalid positive setting: {key}')
    for key in ('calibration_centers_rad', 'validation_centers_rad'):
        centers = np.asarray(c[key], dtype=float)
        if centers.shape != (6, 6) or not np.isfinite(centers).all() or np.any(abs(centers) > c['joint_excursion_rad']):
            raise ValueError('Invalid calibration posture centers')
    protocol = c.get('sweep_protocol', 'fixed_time_v1')
    minimum_speed = c.get('minimum_sweep_speed_rad_s', c['velocity_deadband_rad_s'])
    s = np.asarray(c['speeds_rad_s'])
    if (s.ndim != 1 or len(s) < 8 or not np.isfinite(s).all() or
            np.any(np.diff(s) <= 0) or s[0] <= minimum_speed or
            s[-1] > c['max_velocity_rad_s']):
        raise ValueError('Need eight distinct increasing speeds above deadband and within velocity bound')
    if protocol not in ('fixed_time_v1', 'distance_v1', 'matched_v2'):
        raise ValueError('Unknown sweep protocol')
    if protocol in ('distance_v1', 'matched_v2'):
        strokes = np.asarray(c.get('sweep_strokes_rad', []), dtype=float)
        if (strokes.shape != s.shape or not np.isfinite(strokes).all() or
                np.any(strokes <= 0) or (protocol == 'distance_v1' and np.any(np.diff(strokes) < 0)) or
                c['sweep_range_scale'] != [1]*6):
            raise ValueError('Distance sweeps require increasing positive strokes, one per speed, and unit scales')
        if c.get('posture_speed_scope') != 'listed_speeds_only':
            raise ValueError('Distance study must explicitly declare its held-out posture speed scope')
    if protocol == 'matched_v2':
        for key in ('minimum_sweep_speed_rad_s', 'fit_angle_half_width_rad',
                    'matched_load_absolute_nm', 'matched_load_relative', 'startup_dwell_s'):
            if not np.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'Invalid matched protocol setting: {key}')
        if c['friction_model'] != 'velocity_only' or c['startup_dwell_s'] < c['dwell_s']:
            raise ValueError('Matched study requires velocity-only fitting and longer startup rests')
        from .settling import validate_settings
        validate_settings(c)
    checks = np.asarray(c['posture_validation_speeds_rad_s'], dtype=float)
    if (checks.ndim != 1 or len(checks) < 3 or not np.isfinite(checks).all() or
            np.any(np.diff(checks) <= 0) or not all(x in s for x in checks) or
            (protocol != 'matched_v2' and checks[0] > s[1]) or
            (protocol == 'fixed_time_v1' and checks[-1] != s[-1])):
        raise ValueError('Posture checks need low, middle and maximum identified speeds')
    poses = validation_poses(c)
    if len(poses) < 2 or len({p['name'] for p in poses}) != len(poses):
        raise ValueError('Need two distinct named held-out validation poses')
    if c.get('completion_pose', 'follower_ready') == 'follower_ready' and not any(
            np.allclose(p['q_rad'], follower_ready()) for p in poses):
        raise ValueError('Follower completion pose must be included in measured posture validation')
    if (not 0 < c['compensation_gain'] <= .7 or c['passes'] < 2 or
            int(c['passes']) != c['passes'] or c['sample_period_s'] > .02 or
            min(c['joint_excursion_rad']) <= 0 or min(c['sweep_range_scale']) < 1 or
            min(c['slew_rate_nm_s']) <= 0):
        raise ValueError('Invalid gain/pass count/sampling/limits')
    if max(c['sensor_settling_s']) >= min(c['dwell_s'], c['plateau_s']/3):
        raise ValueError('Dwell/plateau must be longer than the measured sensor settling bound')
    if np.any(5*np.asarray(c['current_filter_time_constant_s'])+np.asarray(c['current_measurement_delay_s']) > c['sensor_settling_s']):
        raise ValueError('Sensor settling bound must cover five time constants plus delay')
    if hardware and ((not c['sensor_timing_verified'] and not record_only) or not c['robot_unit'] or
                     c['robot_unit'].startswith('REPLACE') or
                     (current_policy == 'software_threshold' and min(c['measured_torque_abort_nm']) <= 0) or
                     min(c['max_compensation_nm']) <= 0):
        raise ValueError('Set robot_unit, verified sensor settling, torque abort and compensation limits in config')
    return c


def validation_poses(c):
    poses = []
    for p in c['posture_validation']:
        q = follower_ready() if p['pose'] == 'follower_ready' else p['pose']
        q = np.asarray(q, dtype=float)
        if (not isinstance(p['name'], str) or not p['name'] or q.shape != (6,) or
                not np.isfinite(q).all() or np.any(abs(q) > c['posture_validation_limits_rad'])):
            raise ValueError('Invalid held-out validation pose')
        sides = p.get('sweep_sides', [0]*6)
        if len(sides) != 6 or any(x not in (-1, 0, 1) for x in sides):
            raise ValueError('Invalid posture sweep sides')
        poses.append(dict(name=p['name'], q_rad=q.tolist(), sweep_sides=sides))
    return poses


def validate_urdf_plan(meta, data, urdf):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf)
    for j, name in enumerate(JOINTS):
        limit = root.find(f"joint[@name='{name}']/limit")
        if (limit is None or np.min(data['q'][:, j]) < float(limit.get('lower')) or
                np.max(data['q'][:, j]) > float(limit.get('upper'))):
            raise ValueError(f'Trajectory outside URDF limits: {name}')


def smooth(x):
    return x*x*x*(10+x*(-15+6*x))


def sweep_timing(c, speed, scale=1., posture=False):
    """Independent distance and speed; never shorten settling to meet a stroke.

    Raised-cosine acceleration/deceleration each travel v*ramp/2. Round the
    plateau UP to a sample. The resulting stroke (including ramps) is audited.
    The legacy held-out postures retain their previously reviewed short sweeps.
    """
    dt = c['sample_period_s']
    ramp = np.ceil(max(c['ramp_s'], np.pi*speed/(2*c['max_acceleration_rad_s2']))/dt)*dt
    matched = c.get('sweep_protocol') == 'matched_v2'
    if c.get('sweep_protocol') in ('distance_v1', 'matched_v2') and not posture:
        index = c['speeds_rad_s'].index(speed)
        requested = c['sweep_strokes_rad'][index]
        required = requested/speed-ramp
        if required < c['plateau_s']-1e-9 and not matched:
            raise ValueError('Requested stroke is too short for acceleration and settled measurement')
        plateau = np.ceil((max(required, c['plateau_s'])-1e-10)/dt)*dt
    else:
        plateau = np.ceil((scale*(c['plateau_s']+ramp)-ramp)/dt)*dt
    if matched:
        # Both directions retain the predeclared central window after discarding
        # the sensor-settling interval. No window is selected from validation data.
        needed = 2*c['fit_angle_half_width_rad']/speed+2*(max(c['sensor_settling_s'])+.05)
        plateau = max(plateau, np.ceil(needed/dt)*dt)
    return float(ramp), float(plateau), float(speed*(plateau+ramp))


def sweep_half_ranges(c):
    return [max(sweep_timing(c, v, scale)[2]/2 for v in c['speeds_rad_s'])
            for scale in c['sweep_range_scale']]


def generate(c, base_only=False, *, skip_base=False, posture_joints=None,
             posture_repeats=1, posture_speeds=None, posture_repeat_offset=0, posture_waypoint=None):
    dt = c['sample_period_s']
    connected = c.get('sweep_protocol') == 'matched_v2'
    blocks, segments, count = [], [], 0
    zero = np.zeros(6)
    def times(duration):
        return np.arange(round(duration/dt)+1)*dt
    def add(q, phase, joint=-1, speed=0., repeat=-1, event='', validation_pose=''):
        nonlocal count
        blocks.append(q[:-1])
        segments.append(dict(start=count, stop=count+len(q)-1, phase=phase,
                             joint=joint, speed_rad_s=speed, repeat=repeat, event=event,
                             validation_pose=validation_pose))
        count += len(q)-1
    def hold(q, seconds, **kw):
        add(np.tile(q, (len(times(seconds)), 1)), 'hold', **kw)
    def transfer(a, b, validation_pose=''):
        distance = max(abs(b-a))
        if connected and distance < 1e-12:
            return
        duration = max(2., 1.9*distance/c.get('transfer_velocity_rad_s', c['max_velocity_rad_s']),
                       np.sqrt(6*distance/c['max_acceleration_rad_s2']))
        duration = np.ceil(duration/dt)*dt
        add(a+smooth(times(duration)/duration)[:, None]*(b-a), 'transition', validation_pose=validation_pose)
    def sweep(anchor, j, speed, repeat, scale=1., validation_pose='', side=0, previous=None):
        ramp, plateau, stroke = sweep_timing(c, speed, scale, bool(validation_pose))
        half = stroke/2
        center = anchor.copy(); center[j] += side*half
        limits = c['posture_validation_limits_rad'] if validation_pose else c['joint_excursion_rad']
        if abs(center[j])+half > limits[j]+1e-8:
            raise ValueError(f'joint{j+1}: sweep exceeds absolute motion bounds')
        start = center.copy(); start[j] -= half
        transfer(anchor if previous is None else previous, start, validation_pose)
        kw = dict(joint=j, repeat=repeat, validation_pose=validation_pose)
        dwell = (c['startup_dwell_s'] if connected and speed == min(c['speeds_rad_s']) else c['dwell_s'])
        hold(start, dwell, **kw)
        for direction in (1, -1):
            for event, duration in [('start', ramp), ('steady', plateau), ('stop', ramp)]:
                t = times(duration)
                if event == 'steady':
                    x = speed*t
                else:
                    sign = -1 if event == 'start' else 1
                    x = speed/2*(t+sign*ramp/np.pi*np.sin(np.pi*t/ramp))
                q = np.tile(start, (len(t), 1)); q[:, j] += direction*x
                phase = 'posture_validation' if validation_pose else 'friction'
                add(q, phase+('_steady' if event == 'steady' else '_transient'),
                    speed=direction*speed, event=event, **kw)
                start = q[-1]
            if not connected or direction == 1:
                hold(start, dwell, **kw)
        if not connected:
            transfer(start, anchor, validation_pose)
        return start if connected else anchor
    hold(zero, 3.)
    for repeat in range(0 if skip_base else c['passes']):
        # Validation uses reversed speed order to expose drift rather than sharing samples.
        speeds = c['speeds_rad_s'][::1 if repeat == 0 else -1]
        for j in range(6):
            anchor = np.array(c['calibration_centers_rad' if repeat == 0 else 'validation_centers_rad'][j], dtype=float)
            transfer(zero, anchor)
            previous = anchor
            for speed in speeds:
                previous = sweep(anchor, j, speed, repeat, c['sweep_range_scale'][j], previous=previous)
            transfer(previous, zero)
    hold(zero, 3.)
    base_duration = count*dt
    poses = [] if base_only else validation_poses(c)
    for pass_index in range(posture_repeat_offset, posture_repeat_offset+posture_repeats):
        for index, pose in enumerate(poses):
            anchor = np.asarray(pose['q_rad'])
            if posture_waypoint is not None:
                transfer(zero,np.asarray(posture_waypoint),pose['name'])
                transfer(np.asarray(posture_waypoint),anchor,pose['name'])
            else:
                transfer(zero, anchor, pose['name'])
            for j in (range(6) if posture_joints is None else posture_joints):
                previous = anchor
                speeds = c['posture_validation_speeds_rad_s'] if posture_speeds is None else posture_speeds
                for speed in speeds[::1 if pass_index == 0 else -1]:
                    previous = sweep(anchor, j, speed, c['passes']+index+pass_index*len(poses),
                          validation_pose=pose['name'], side=pose['sweep_sides'][j], previous=previous)
                if connected:
                    transfer(previous, anchor, pose['name'])
            if posture_waypoint is not None:
                transfer(anchor,np.asarray(posture_waypoint),pose['name'])
                transfer(np.asarray(posture_waypoint),zero,pose['name'])
            else:
                transfer(anchor, zero, pose['name'])
    if not base_only:
        hold(zero, 3.)
    q = np.vstack(blocks+[zero[None]])
    t = np.arange(len(q))*dt
    v = np.gradient(q, dt, axis=0, edge_order=2)
    a = np.gradient(v, dt, axis=0, edge_order=2)
    if np.max(abs(v)) > 1.01*c['max_velocity_rad_s'] or np.max(abs(a)) > 1.01*c['max_acceleration_rad_s2']:
        raise ValueError('Generated velocity/acceleration exceeds configured limit')
    motion_limits = (np.asarray(c['joint_excursion_rad']) if base_only else
                     np.maximum(c['joint_excursion_rad'], c['posture_validation_limits_rad']))
    if np.any(np.max(abs(q), axis=0) > motion_limits+1e-8):
        raise ValueError('Generated trajectory exceeds posture-specific motion limits')
    meta = dict(schema=1, config=c, joints=JOINTS, segments=segments, duration_s=float(t[-1]),
                max_abs_q_rad=np.max(abs(q), axis=0).tolist(),
                max_abs_v_rad_s=np.max(abs(v), axis=0).tolist(),
                max_abs_a_rad_s2=np.max(abs(a), axis=0).tolist(),
                motion_limits_rad=motion_limits.tolist(), base_duration_s=base_duration,
                posture_validation_poses=poses,
                record_only=base_only,
                min_q_rad=np.min(q, axis=0).tolist(), independent_repeat_validation=True,
                changed_posture_validation_joints=[JOINTS[j] for j in range(6) if not np.allclose(
                    c['calibration_centers_rad'][j], c['validation_centers_rad'][j])],
                sweep_half_ranges_rad=sweep_half_ranges(c),
                sweep_protocol=c.get('sweep_protocol', 'fixed_time_v1'),
                sweep_schedule=[dict(speed_rad_s=v, ramp_s=sweep_timing(c,v)[0],
                    steady_s=sweep_timing(c,v)[1], stroke_rad=sweep_timing(c,v)[2])
                    for v in c['speeds_rad_s']],
                posture_speed_scope=dict(speeds_rad_s=c['posture_validation_speeds_rad_s'],
                    covers_all_identification_speeds=max(c['posture_validation_speeds_rad_s']) == max(c['speeds_rad_s']),
                    untested_speeds_are_qualified=False),
                temperature_model=False, hardware_validated=False,
                method='paired_stribeck' if c['friction_model'] == 'velocity_only' else 'paired_load_stribeck',
                axis_parallel_at_zero=[True, False, False, False, True, False])
    return meta, dict(t=t, q=q, v=v, a=a)


def generate_posture_validation(c):
    """The unchanged held-out tail, collected separately after the fit run."""
    meta, data = generate(c)
    begin = round(meta['base_duration_s']/c['sample_period_s'])
    segments = [dict(s, start=s['start']-begin, stop=s['stop']-begin)
                for s in meta['segments'] if s['start'] >= begin]
    data = {k: v[begin:].copy() for k, v in data.items()}
    data['t'] -= data['t'][0]
    meta.update(segments=segments, duration_s=float(data['t'][-1]), base_duration_s=0.,
                record_only=True, posture_validation_only=True,
                max_abs_q_rad=np.max(abs(data['q']), axis=0).tolist(),
                max_abs_v_rad_s=np.max(abs(data['v']), axis=0).tolist(),
                max_abs_a_rad_s2=np.max(abs(data['a']), axis=0).tolist(),
                min_q_rad=np.min(data['q'], axis=0).tolist())
    return meta, data


def generate_loaded_study(c, validation=False, joints=None):
    """Resolve load-sensitive J2/J3/J4 at the same two poses and sweep envelopes."""
    joints=[1,2,3] if joints is None else joints
    if not joints or any(j not in (1,2,3) for j in joints):raise ValueError('Loaded study supports J2/J3/J4')
    meta, data = generate(c, skip_base=True, posture_joints=joints,
                          posture_repeats=1, posture_speeds=c['speeds_rad_s'],
                          posture_repeat_offset=int(validation))
    meta.update(record_only=True, loaded_study=True, posture_validation_only=False,
                loaded_training_repeats=[2, 3], loaded_held_out_repeats=[4, 5],
                loaded_study_joints=joints,
                loaded_study_role='validation' if validation else 'training',
                changed_posture_validation_joints=[])
    return meta, data


def generate_position_probe(c, configuration=False):
    from copy import deepcopy
    working=next(p['q_rad'] for p in validation_poses(c) if p['name']=='working')
    probe=deepcopy(c);probe['posture_validation']=[]
    axis=2 if configuration else 1
    for suffix,offset in (('lower',-.19),('upper',.19)):
        name=f'working_q{axis+1}_{suffix}'
        q=list(working);q[axis]+=offset
        probe['posture_validation'].append(dict(name=name,pose=q))
    meta,data=generate(probe,skip_base=True,posture_joints=[1],
        posture_speeds=c['speeds_rad_s'][:3],posture_waypoint=working)
    meta.update(config=c,record_only=True,position_probe=True,posture_validation_only=False,
                configuration_probe=configuration,
                changed_posture_validation_joints=[],method='independent_position_probe',
                position_probe_speeds=c['speeds_rad_s'][:3],route_via_working=working)
    return meta,data


def save_plan(c, folder, base_only=False, posture_only=False, loaded_study=False,
              loaded_validation=False, position_probe=False, validation_joint=None,configuration_probe=False):
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=False)
    if sum((base_only, posture_only, loaded_study, position_probe,configuration_probe)) > 1:
        raise ValueError('Choose original sweeps or held-out posture collection')
    if position_probe or configuration_probe:
        meta,data=generate_position_probe(c,configuration=configuration_probe)
    elif loaded_study:
        meta, data = generate_loaded_study(c, validation=loaded_validation,
            joints=[validation_joint] if validation_joint is not None else None)
    else:
        meta, data = generate_posture_validation(c) if posture_only else generate(c, base_only=base_only)
    np.savez_compressed(folder/'trajectory.npz', **data)
    meta['sha256'] = sha(folder/'trajectory.npz')
    (folder/'plan.json').write_text(json.dumps(meta, indent=2))
    return meta, data
