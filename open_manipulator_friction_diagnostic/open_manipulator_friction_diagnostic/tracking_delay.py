"""Position-loop lag review. This is neither current-sensor delay nor friction fitting."""
import json
from collections import deque
from pathlib import Path
import numpy as np
from .protocol import sha


def review(folder, joint=6):
    if joint not in range(1,7):raise ValueError('Invalid tracking joint')
    axis=joint-1
    folder = Path(folder)
    for parent in (folder, folder.parent, folder.parent.parent):
        if (parent/'human_contact_review.json').exists():
            raise ValueError('Contacted data cannot establish a tracking reference')
    report = json.loads((folder/'collection.json').read_text())
    if report.get('source') != 'hardware' or not report.get('generalization_study'):
        raise ValueError('Need recorded hardware position trajectories')
    with np.load(folder/'measurements.npz', allow_pickle=False) as raw:
        t, q = raw['t'], raw['q'][:, axis]
    with np.load(folder/'trajectory.npz', allow_pickle=False) as raw:
        rt, rq = raw['t'], raw['q'][:, axis]
    meta = json.loads((folder/'plan.json').read_text())
    legacy=joint==6 and meta['config'].get('automatic_j6_high_speed')
    measured_joints={s['joint'] for s in meta['segments'] if s['phase']=='posture_validation_steady'}
    if not legacy and (not meta['config'].get('common_friction_identification') or measured_joints!={axis}):
        raise ValueError('Lag review requires this joint\'s own common-model position acquisition')
    windows = []
    for s in meta['segments']:
        begin = rt[s['start']]+.5; end = rt[s['stop']]-.05
        speed = s['speed_rad_s']
        if (s['phase'] != 'posture_validation_steady' or s['joint'] != axis or
                abs(speed) < .1 or end > t[-1] or end-begin < .5):
            continue
        mask = (t >= begin) & (t <= end)
        if mask.sum() < 100:
            raise ValueError('Missing steady tracking samples')
        slope = np.polyfit(t[mask]-t[mask][0], q[mask], 1)[0]
        if abs(slope/speed-1) > .01:
            raise ValueError('Lag cannot explain incorrect steady speed')
        windows.append(dict(begin_s=float(begin), end_s=float(end), speed_rad_s=speed,
            actual_speed_rad_s=float(slope),
            apparent_delay_s=float(np.median((np.interp(t[mask], rt, rq)-q[mask])/speed)),
            role='fit' if abs(speed) <= .3 else 'held_out'))
    fit = [w for w in windows if w['role'] == 'fit']
    held = [w for w in windows if w['role'] == 'held_out']
    if len(fit) < 4 or len(held) < 4 or any(
            not {np.sign(w['speed_rad_s']) for w in rows} == {-1, 1} for rows in (fit, held)):
        raise ValueError('Need separate slower/faster bidirectional lag evidence')
    delay = float(np.median([w['apparent_delay_s'] for w in fit]))
    if not 0 < delay <= .1:
        raise ValueError('Unexpected position-loop delay')
    residual = q-np.interp(t-delay, rt, rq)
    for w in windows:
        mask = (t >= w['begin_s']) & (t <= w['end_s'])
        w['delayed_reference_max_error_rad'] = float(np.max(abs(residual[mask])))
    passed = (all(w['delayed_reference_max_error_rad'] < .01 for w in held) and
              np.max(abs(residual)) < .08)
    return dict(schema='j6_position_tracking_lag_v1' if legacy else 'joint_position_tracking_lag_v1', joint=joint, delay_s=delay,
        passed=bool(passed), windows=windows,
        maximum_observed_speed_rad_s=max(abs(w['speed_rad_s']) for w in windows),
        maximum_all_samples_delayed_error_rad=float(np.max(abs(residual))),
        residual_tracking_tolerance_rad=.08,
        sensor_timing_verified=False, firmware_settings_changed=False,
        source_sha256={n: sha(folder/n) for n in ('collection.json', 'measurements.npz', 'trajectory.npz', 'plan.json')},
        scope='Empirical host-position-to-encoder lag. Higher speeds still require new measurement; '
              'does not distinguish internal profiling, servo dynamics and transport.')


def review_pair(fit_folder, validation_folder, joint):
    """Fit slow-run lag and check a separate faster run, for any single joint.

    A tracking-aborted run can validate its completed timing windows. It remains
    ineligible for friction identification; no missing samples are synthesized.
    """
    if joint not in range(1,7):raise ValueError('Invalid tracking joint')
    axis=joint-1; windows=[]; runs=[]; identity=None
    paths=[Path(fit_folder).resolve(),Path(validation_folder).resolve()]
    if paths[0]==paths[1]:raise ValueError('Separate fit and validation collections required')
    for role,folder in zip(('fit','held_out'),paths):
        for parent in (folder,folder.parent,folder.parent.parent):
            if (parent/'human_contact_review.json').exists():
                raise ValueError('Contacted data cannot establish a tracking reference')
        report=json.loads((folder/'collection.json').read_text())
        meta=json.loads((folder/'plan.json').read_text());c=meta['config']
        measured={s['joint'] for s in meta['segments'] if s['phase']=='posture_validation_steady'}
        diagnostic=(c.get('automatic_repeat_diagnostic') is True and
                    meta.get('automatic_condition_design')=='repeat_diagnostic')
        if (report.get('source')!='hardware' or not report.get('generalization_study') or
                not meta.get('automatic_targeted') or meta.get('automatic_role')!='development' or
                measured!={axis} or not (c.get('common_friction_identification') or diagnostic)):
            raise ValueError('Need this joint\'s own common position acquisition')
        completed=report.get('completed') is True and not report.get('failure')
        if not completed and not (role=='held_out' and report.get('completed') is False and
                                  report.get('failure')=='Tracking error > 0.08 rad'):
            raise ValueError('Only a tracking-aborted validation may supply complete timing windows')
        current=(c.get('robot_unit'),meta.get('robot_fingerprint'),meta.get('baseline_sha256'))
        if (not all(current) or report.get('robot_fingerprint')!=current[1] or
                (identity is not None and current!=identity)):
            raise ValueError('Position timing robot/baseline identity differs')
        identity=current
        with np.load(folder/'measurements.npz',allow_pickle=False) as raw:t,q=raw['t'],raw['q'][:,axis]
        with np.load(folder/'trajectory.npz',allow_pickle=False) as raw:rt,rq=raw['t'],raw['q'][:,axis]
        if (len(t)<2 or len(rt)<2 or not all(np.isfinite(v).all() for v in (t,q,rt,rq)) or
                np.any(np.diff(t)<=0) or np.any(np.diff(rt)<=0)):
            raise ValueError('Invalid recorded timing samples')
        for s in meta['segments']:
            if s['phase']!='posture_validation_steady' or s['joint']!=axis:continue
            begin=rt[s['start']]+.5;end=rt[s['stop']]-.05;v=s['speed_rad_s']
            if (end>t[-1] or end-begin<.5 or abs(v)<.1 or
                    (role=='fit' and abs(v)>.3) or (role=='held_out' and abs(v)<=.3)):continue
            mask=(t>=begin)&(t<=end)
            if mask.sum()<100:raise ValueError('Missing steady tracking samples')
            slope=np.polyfit(t[mask]-t[mask][0],q[mask],1)[0]
            if abs(slope/v-1)>.01:raise ValueError('Lag cannot explain incorrect steady speed')
            windows.append(dict(role=role,begin_s=float(begin),end_s=float(end),speed_rad_s=v,
                actual_speed_rad_s=float(slope),apparent_delay_s=float(np.median((np.interp(t[mask],rt,rq)-q[mask])/v))))
        runs.append(dict(role=role,t=t,q=q,rt=rt,rq=rq,completed=completed,
            source_sha256={n:sha(folder/n) for n in ('collection.json','measurements.npz','trajectory.npz','plan.json')}))
    if any(len([w for w in windows if w['role']==r])<4 or
           {np.sign(w['speed_rad_s']) for w in windows if w['role']==r}!={-1,1}
           for r in ('fit','held_out')):
        raise ValueError('Need separate slower/faster bidirectional lag evidence')
    if runs[0]['source_sha256']['measurements.npz']==runs[1]['source_sha256']['measurements.npz']:
        raise ValueError('Duplicated measurement payload is not independent timing evidence')
    delay=float(np.median([w['apparent_delay_s'] for w in windows if w['role']=='fit']))
    if not 0<delay<=.1:raise ValueError('Unexpected position-loop delay')
    maximum=0.
    for r in runs:
        residual=r['q']-np.interp(r['t']-delay,r['rt'],r['rq'])
        maximum=max(maximum,float(np.max(abs(residual))))
        for w in windows:
            if w['role']==r['role']:
                mask=(r['t']>=w['begin_s'])&(r['t']<=w['end_s'])
                w['delayed_reference_max_error_rad']=float(np.max(abs(residual[mask])))
    passed=maximum<.08 and all(w['delayed_reference_max_error_rad']<.01 for w in windows if w['role']=='held_out')
    return dict(schema='joint_position_tracking_lag_v1',joint=joint,delay_s=delay,passed=bool(passed),
        windows=windows,maximum_observed_speed_rad_s=max(abs(w['speed_rad_s']) for w in windows),
        maximum_all_samples_delayed_error_rad=maximum,residual_tracking_tolerance_rad=.08,
        sensor_timing_verified=False,firmware_settings_changed=False,friction_identification_eligible=False,
        fit_collection_completed=runs[0]['completed'],validation_collection_completed=runs[1]['completed'],
        identity=dict(robot_unit=identity[0],robot_fingerprint=identity[1],baseline_sha256=identity[2]),
        source_collections=[dict(role=r['role'],source_sha256=r['source_sha256'],completed=r['completed']) for r in runs],
        scope='Own-joint host-position-to-encoder lag; completed steady windows only. '
              'A partial collection stays ineligible as friction data. This does not isolate servo, profile, '
              'transport or current-sensor delays; higher speeds require new measurement.')


def load(config):
    path = config.get('position_tracking_delay_review')
    if path is None:
        return np.zeros(6)
    joint=config.get('position_tracking_delay_joint',6)
    legacy='position_tracking_delay_joint' not in config
    if legacy and not config.get('automatic_j6_high_speed'):
        raise ValueError('Reviewed lag is only enabled for J6 high-speed position acquisition')
    if not legacy and (joint not in range(1,7) or not (config.get('common_friction_identification') or
                      config.get('automatic_repeat_diagnostic') is True)):
        raise ValueError('Joint-specific timing requires common position acquisition')
    if sha(path) != config.get('position_tracking_delay_review_sha256'):
        raise ValueError('Changed position tracking lag review')
    r = json.loads(Path(path).read_text())
    if (r.get('schema') != ('j6_position_tracking_lag_v1' if legacy else 'joint_position_tracking_lag_v1') or r.get('joint') != joint or
            r.get('passed') is not True or not 0 < r['delay_s'] <= .1 or
            r.get('residual_tracking_tolerance_rad') != .08):
        raise ValueError('Invalid position tracking lag review')
    result=np.zeros(6);result[joint-1]=r['delay_s'];return result


def at(reference, t, delays):
    # Search all six times together. This does not copy the whole trajectory at
    # 400 Hz, unlike repeatedly interpolating strided columns with np.interp.
    times = t-delays
    indices = np.clip(np.searchsorted(reference['t'], times), 1, len(reference['t'])-1)
    fraction = np.clip((times-reference['t'][indices-1]) /
                       (reference['t'][indices]-reference['t'][indices-1]), 0, 1)
    joints = np.arange(6)
    return reference['q'][indices-1, joints]+fraction*(
        reference['q'][indices, joints]-reference['q'][indices-1, joints])


class CollectionTrackingDelay:
    """Freeze each axis's host-position lag from this run's slow bidirectional moves.

    No extra motion, firmware changes or current-sensor correction. Once frozen,
    high-speed errors cannot change the delay and still face the 0.08 rad check.
    """
    def __init__(self, metadata, reference):
        self.reference = reference
        self.delays = np.zeros(6)
        self.results = {}
        self.windows = [[] for _ in range(6)]
        self.samples = []
        self.pending = deque(dict(joint=s['joint'], speed=s['speed_rad_s'],
            begin=float(reference['t'][s['start']])+.5,
            end=float(reference['t'][s['stop']])-.05)
            for s in metadata['segments'] if s['phase'] == 'posture_validation_steady'
            and .1-1e-9 <= abs(s['speed_rad_s']) <= .3+1e-9)

    def observe(self, t, position):
        while self.pending:
            window = self.pending[0]
            j = window['joint']
            if j in self.results:
                self.pending.popleft()
                continue
            if t < window['begin']:
                return
            if t <= window['end']:
                self.samples.append((t, position[j]))
                return
            self.pending.popleft()
            samples = np.asarray(self.samples)
            self.samples = []
            if (len(samples) < 100 or samples[-1, 0]-samples[0, 0] < .5
                    or samples[0, 0] > window['begin']+.05
                    or samples[-1, 0] < window['end']-.05):
                raise ValueError(f'joint{j+1}: missing position-lag measurement window')
            times, q = samples.T
            speed = window['speed']
            slope = float(np.polyfit(times-times[0], q, 1)[0])
            if abs(slope/speed-1) > .01:
                raise ValueError(f'joint{j+1}: measured speed {slope:.5f} does not match {speed:.5f}; lag cannot explain it')
            rt, rq = self.reference['t'], self.reference['q'][:, j]
            delay = float(np.median((np.interp(times, rt, rq)-q)/speed))
            residual = float(np.max(abs(q-np.interp(times-delay, rt, rq))))
            if not 0 <= delay <= .1 or residual > .01:
                raise ValueError(f'joint{j+1}: position lag outside 0–100 ms or inconsistent motion')
            self.windows[j].append(dict(window, measured_speed_rad_s=slope,
                delay_s=delay, residual_rad=residual))
            speeds = {round(w['speed'], 6) for w in self.windows[j]}
            if len({v for v in speeds if v > 0 and -v in speeds}) < 2:
                continue
            delays = [w['delay_s'] for w in self.windows[j]]
            if np.ptp(delays) > .01:
                raise ValueError(f'joint{j+1}: direction/speed-dependent lag is inconsistent')
            self.delays[j] = float(np.median(delays))
            self.results[j] = dict(delay_s=self.delays[j], frozen_at_s=t,
                                   windows=self.windows[j])

    def report(self):
        return dict(schema='collection_position_tracking_delay_v1',
            joints={str(j+1): r for j, r in self.results.items()},
            delays_s=self.delays.tolist(), current_sensor_timing_identified=False,
            firmware_settings_changed=False, residual_tracking_tolerance_rad=.08,
            scope='This acquisition only; low-speed fit frozen before high-speed checks. Raw measurements are unchanged.')
