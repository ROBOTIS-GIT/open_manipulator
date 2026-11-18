"""ROS action client: bounded homing, collection, held-out fit, atomic profile export."""
import argparse
import json
import uuid
from pathlib import Path
import time
import threading

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from unique_identifier_msgs.msg import UUID
from action_msgs.srv import CancelGoal

from .protocol import JOINTS, config, save_plan, sweep_half_ranges, validate_urdf_plan
from .trajectory_transport import compact_points
from .fit import save_result
from .integration import description, fingerprint, baseline_path
from .completion import completion_pose, homing_plan, move_to_completion, activate_latest


def prepare_goal(data, acceleration_limit, anchors=()):
    """Build large ROS messages before starting the 400 Hz state subscription.

    The header is stamped only at submission. The dense measurement reference,
    accepted interpolation tolerances and experiment timing remain unchanged.
    """
    sent, transport = compact_points(data, anchors, acceleration_limit=acceleration_limit)
    goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
    for t, q, v in zip(sent['t'], sent['q'], sent['v']):
        point = JointTrajectoryPoint(positions=q.tolist(), velocities=v.tolist())
        ns = round(t*1e9); point.time_from_start = Duration(sec=ns//10**9, nanosec=ns % 10**9)
        goal.trajectory.points.append(point)
    return goal, transport


class Calibration(Node):
    def __init__(self, c, folder, mock):
        super().__init__('friction_calibration')
        self.c, self.folder, self.mock = c, folder, mock
        from .tracking_delay import load
        self.tracking_delays = load(c)
        self.latest = None; self.failure = None; self.epoch = None
        self.abort_sample = None
        self.state_timing = dict(samples=0, maximum_stamp_age_s=0.,
                                 maximum_stamp_gap_s=0., stale_sample=None)
        self.last_rx = time.monotonic(); self.last_stamp = None
        self.record = False; self.count = 0; self.buffer = None; self.reference = None
        self.journal = None
        self.state_lock = threading.Lock()
        self.settling_monitor = None
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.cancel_client = self.create_client(CancelGoal, '/arm_controller/follow_joint_trajectory/_action/cancel_goal')
        self.create_subscription(JointState, '/joint_states', self.state, QoSProfile(
            # 1000 samples can queue 2.5 seconds of obsolete state at 400 Hz.
            # Keep a short history; the unchanged 50 ms gap and 100 ms age
            # checks still reject missing or stale acquisition data.
            depth=16, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.publisher = self.create_publisher(String, '~/status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '~/cancel', self.cancel)
        self.mock_filter = np.zeros(6)

    def status(self, text):
        self.publisher.publish(String(data=text)); self.get_logger().info(text)

    def cancel(self, request, response):
        self.failure = 'Cancelled via /friction_calibration/cancel'
        response.success = True; response.message = 'Cancelling position action; position controller holds.'
        return response

    def state(self, msg):
        # run_goal runs outside the ROS executor. Closing a recording must wait
        # for an already-entered callback, including NumPy operations that
        # release the GIL, before its count and array are snapshotted.
        with self.state_lock:
            Calibration._state_unlocked(self, msg)

    def stop_recording(self):
        with self.state_lock:
            self.record = False; self.reference = None; self.epoch = None

    def recorded_measurements(self):
        with self.state_lock:
            if self.record:
                raise RuntimeError('Cannot snapshot an active recording')
            a = self.buffer[:self.count]
            return dict(t=a[:, 0], q=a[:, 1:7], v=a[:, 7:13], effort=a[:, 13:19])

    def checkpoint_recording(self, closed=False):
        if self.journal is not None:
            with self.state_lock:
                count = self.count
            self.journal.commit(count, closed=closed)

    def _state_unlocked(self, msg):
        # Keep the first stop reason; callbacks during teardown must not replace it.
        if self.failure:
            return
        if not all(j in msg.name for j in JOINTS):
            return
        ids = [msg.name.index(j) for j in JOINTS]
        if any(len(x) != len(msg.name) for x in (msg.position, msg.velocity, msg.effort)):
            self.failure = 'Position/velocity/effort state interfaces are required'; return
        q, v, e = [np.asarray(x)[ids] for x in (msg.position, msg.velocity, msg.effort)]
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not np.isfinite(np.r_[stamp, q, v, e]).all():
            self.failure = 'Nonfinite joint state'; return
        if hasattr(self, 'state_timing') and self.last_stamp is not None:
            self.state_timing['maximum_stamp_gap_s'] = max(
                self.state_timing['maximum_stamp_gap_s'], stamp-self.last_stamp)
        if self.last_stamp is not None and (stamp <= self.last_stamp or stamp-self.last_stamp > .05):
            self.failure = f'Nonmonotonic timestamp or state gap > 50 ms: delta={stamp-self.last_stamp:.6f} s'; return
        self.last_stamp = stamp; self.last_rx = time.monotonic(); self.latest = (q, v, e)
        if (not self.mock and self.c.get('current_abort_policy', 'software_threshold') == 'software_threshold'
                and np.any(abs(e) > self.c['measured_torque_abort_nm'])):
            bad = np.flatnonzero(abs(e) > self.c['measured_torque_abort_nm'])
            # This sample precedes the normal recording branch. Preserve it
            # separately so an abort does not discard its triggering evidence.
            self.abort_sample = dict(stamp_s=stamp,
                trajectory_time_s=None if self.epoch is None else stamp-self.epoch,
                q_rad=q.tolist(), v_rad_s=v.tolist(), effort_ros_nm=e.tolist(),
                limits_ros_nm=list(self.c['measured_torque_abort_nm']))
            self.failure = 'Measured torque abort limit exceeded: '+', '.join(
                f'{JOINTS[j]}={e[j]:.3f}, limit={self.c["measured_torque_abort_nm"][j]:.3f} ROS Nm'
                for j in bad)
            return
        if self.epoch is None:
            return
        t = stamp-self.epoch
        if t < 0:
            return
        reference = self.reference
        if reference is not None:
            # Search once. Six np.interp calls on strided columns copied the entire
            # 35-minute reference at every 400 Hz state callback on the Pi.
            from .tracking_delay import at
            ref = at(reference, t, self.tracking_delays)
            if np.max(abs(q-ref)) > .08:
                self.failure = 'Tracking error > 0.08 rad'; return
        if self.record and reference is not None and t <= reference['t'][-1]+.05:
            if not self.mock:
                age = self.get_clock().now().nanoseconds*1e-9-stamp
                if hasattr(self, 'state_timing'):
                    self.state_timing['samples'] += 1
                    self.state_timing['maximum_stamp_age_s'] = max(
                        self.state_timing['maximum_stamp_age_s'], age)
                if age > .1:
                    if hasattr(self, 'state_timing'):
                        self.state_timing['stale_sample'] = dict(stamp_s=stamp,
                            stamp_age_s=age, trajectory_time_s=t, q_rad=q.tolist())
                    self.failure = 'Measured state is more than 100 ms old'; return
            if self.count >= len(self.buffer):
                self.failure = 'Recorded sample buffer overflow'; return
            self.buffer[self.count] = np.r_[t, q, v, e]; self.count += 1
            if self.settling_monitor is not None:
                try:
                    self.settling_monitor.observe(t, q)
                except ValueError as exc:
                    self.failure = str(exc)

    def run_goal(self, data, record=False, label='COLLECTING', anchors=(), prepared=None):
        goal, transport = prepared if prepared is not None else prepare_goal(
            data, self.c['max_acceleration_rad_s2'], anchors)
        (self.folder/(label.lower()+'_transport.json')).write_text(json.dumps(transport, indent=2))
        # A broad calibration sends many points; allow DDS serialization before its start.
        lead = 5. if record else .5
        with self.state_lock:
            self.reference = data; self.record = record
            self.epoch = self.get_clock().now().nanoseconds*1e-9+lead
        ns = round(self.epoch*1e9)
        goal.trajectory.header.stamp.sec = ns//10**9
        goal.trajectory.header.stamp.nanosec = ns % 10**9
        goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        start = time.monotonic(); handle = None
        try:
            if self.failure or time.monotonic()-self.last_rx > .1:
                raise RuntimeError(self.failure or 'Stale state before trajectory submission')
            future = self.client.send_goal_async(goal, goal_uuid=goal_id)
            while not future.done():
                time.sleep(.005)
                if self.failure or time.monotonic()-start > 30:
                    raise RuntimeError(self.failure or 'Goal acceptance timeout')
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('Position controller rejected trajectory')
            result = handle.get_result_async(); next_report = start+30
            while not result.done():
                time.sleep(.005)
                now = time.monotonic()
                if self.failure or now-self.last_rx > .1 or now-start > data['t'][-1]+30:
                    (self.folder/'runtime_abort.json').write_text(json.dumps(dict(
                        reason=self.failure or 'State/trajectory timeout',
                        state_receipt_age_s=now-self.last_rx,
                        state_stamp=self.last_stamp,
                        ros_now_s=self.get_clock().now().nanoseconds*1e-9,
                        elapsed_s=now-start, planned_duration_s=float(data['t'][-1])),indent=2))
                    raise RuntimeError(self.failure or 'State/trajectory timeout')
                if now >= next_report:
                    if record:
                        self.checkpoint_recording()
                    self.status(f'{label}: {max(0, now-start-lead):.0f}/{data["t"][-1]:.0f} seconds')
                    next_report = now+30
            if self.failure or result.result().result.error_code != 0:
                raise RuntimeError(self.failure or str(result.result().result))
        except BaseException:
            if rclpy.ok():
                if handle is not None:
                    cancel = handle.cancel_goal_async()
                else:
                    request = CancelGoal.Request(); request.goal_info.goal_id = goal_id
                    cancel = self.cancel_client.call_async(request)
                deadline = time.monotonic()+2
                while not cancel.done() and time.monotonic() < deadline:
                    time.sleep(.005)
            raise
        finally:
            self.stop_recording()
            if record:
                self.checkpoint_recording(closed=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True); parser.add_argument('--output', required=True)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--motion-review', default='')
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument('--record-only', action='store_true')
    scope.add_argument('--posture-only', action='store_true',
                       help='Collect the existing independent posture checks; no fitting or activation')
    scope.add_argument('--loaded-study', action='store_true',
                       help='J2/J3/J4, both existing postures, all speeds, training pass')
    scope.add_argument('--loaded-validation', action='store_true',
                       help='Independent reversed-speed repeat of the loaded study')
    scope.add_argument('--position-probe', action='store_true',help='Held-out J2 centers inside the reviewed working sweep')
    scope.add_argument('--configuration-probe',action='store_true',help='Held-out J2 low speeds at nearby J3 configurations')
    scope.add_argument('--prepared-study', help='Exact reviewed load-coverage plan directory; position collection only')
    parser.add_argument('--latest')
    parser.add_argument('--validation-joint',type=int,choices=[2,3,4],help='Fresh loaded validation of one repaired joint')
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if args.validation_joint is not None and not args.loaded_validation:
        parser.error('--validation-joint requires --loaded-validation')
    # A completed collection may be followed by another process in a campaign.
    # This file prevents that next process from moving the robot; it does not
    # interrupt an in-progress trajectory or discard its recorded samples.
    folder = Path(args.output).expanduser().resolve()
    if (folder.parent/'PAUSE_NEW_COLLECTIONS').exists():
        raise SystemExit('PAUSED_BY_USER: next collection was not started')
    loaded = args.loaded_study or args.loaded_validation
    collection_only = bool(args.record_only or args.posture_only or loaded or args.position_probe or args.configuration_probe or args.prepared_study)
    c = config(args.config, hardware=not args.mock, record_only=collection_only)
    urdf = description('position', args.mock)
    if args.prepared_study:
        from .generalization_motion import copy_reviewed
        if json.loads((Path(args.prepared_study)/'plan.json').read_text()).get('automatic_targeted'):
            from .automatic_motion import copy_reviewed
        meta, data = copy_reviewed(args.prepared_study, folder, c, urdf)
    else:
        meta, data = save_plan(c, folder, base_only=args.record_only, posture_only=args.posture_only,
                          loaded_study=loaded, loaded_validation=args.loaded_validation,position_probe=args.position_probe,
                          validation_joint=args.validation_joint-1 if args.validation_joint else None,configuration_probe=args.configuration_probe)
    from .profile_telemetry import collection_options
    telemetry_options=collection_options(c,meta)
    if telemetry_options is not None and not collection_only:
        raise ValueError('Profile telemetry is restricted to position collection')
    (folder/'config.yaml').write_text(yaml.safe_dump(c))
    validate_urdf_plan(meta, data, urdf)
    from .motion_review import require_review
    if not args.mock and not args.prepared_study:
        require_review(args.motion_review, c, urdf)
    finish_pose = completion_pose(dict(c, completion_pose='zero') if collection_only and not args.prepared_study else c, urdf)
    (folder/'robot.urdf').write_text(urdf)
    from .poses import report
    (folder/'poses.json').write_text(json.dumps(report(folder/'robot.urdf', c['calibration_centers_rad'], half_ranges=sweep_half_ranges(c)), indent=2))
    # Reduction and thousands of Python message allocations previously ran
    # after homing, contending with state callbacks before the first sample.
    # Prepare once before rclpy.init; do not relax age/gap checks or discard data.
    collection_goal = prepare_goal(data, c['max_acceleration_rad_s2'],
                                  [s[k] for s in meta['segments'] for k in ('start', 'stop')])
    rclpy.init(); node = Calibration(c, folder, args.mock)
    executor = SingleThreadedExecutor(); executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True); spin_thread.start()
    capacity = int((meta['duration_s']+1)*600)
    if meta.get('full_calibration'):
        from .recording_journal import RecordingJournal
        node.journal = RecordingJournal(folder, capacity, meta['sha256'])
        node.buffer = node.journal.buffer
    else:
        node.buffer = np.empty((capacity, 19))
    completed = False
    telemetry = None
    try:
        if telemetry_options is not None and not args.mock:
            from .profile_telemetry import ProfileTelemetry
            telemetry = ProfileTelemetry(node, **telemetry_options)
        node.status('WAITING: position controller and measured joint state')
        if c.get('current_abort_policy') == 'firmware_only':
            node.status('CURRENT_MONITOR: record measured effort unchanged; no additional software current threshold; existing actuator protection unchanged')
        start = time.monotonic()
        while (node.latest is None or not node.client.server_is_ready() or
               (telemetry is not None and not telemetry.ready())):
            time.sleep(.01)
            if node.failure or time.monotonic()-start > 90:
                raise RuntimeError(node.failure or 'Bringup readiness timeout')
        if telemetry is not None:
            # Profile reads diagnose the measured sweeps, not homing. Drain the
            # last startup request before submitting any position trajectory.
            telemetry.pause()
            deadline=time.monotonic()+2.
            while telemetry.pending is not None:
                if node.failure or time.monotonic()>deadline:
                    raise RuntimeError(node.failure or 'Telemetry drain timeout before homing')
                time.sleep(.005)
        q, v, _ = node.latest
        if node.failure:
            raise RuntimeError(node.failure)
        if meta.get('automatic_targeted'):
            from .homing_review import review_in_worker, require_unchanged_start
            homing, homing_info = review_in_worker(q, v, c, folder/'robot.urdf', folder)
            if node.failure or time.monotonic()-node.last_rx > .1:
                raise RuntimeError(node.failure or 'Stale state after homing geometry review')
            require_unchanged_start(q, node.latest[0], node.latest[1])
            (folder/'homing_geometry.json').write_text(json.dumps(homing_info['geometry'], indent=2))
        else:
            homing, homing_info = homing_plan(q, v, c, urdf)
        if args.prepared_study and not args.mock and not meta.get('automatic_targeted'):
            # Actual start can differ from nominal ready. Check that exact slow
            # homing path before sending any trajectory command.
            from .generalization_motion import check_geometry
            check = check_geometry(homing['q'], folder/'robot.urdf')
            (folder/'homing_geometry.json').write_text(json.dumps(check, indent=2))
            if not check['passed']:
                raise RuntimeError('Measured-start homing failed geometry review')
        np.savez_compressed(folder/'homing_trajectory.npz', **homing)
        homing_info['completed'] = False
        (folder/'homing.json').write_text(json.dumps(homing_info, indent=2))
        node.status(f'HOMING: {homing_info["start_region"]} -> [0,0,0,0,0,0], '
                    f'{homing_info["duration_s"]:.1f} seconds')
        node.run_goal(homing, label='HOMING')
        homing_info.update(completed=True, measured_end_rad=node.latest[0].tolist())
        (folder/'homing.json').write_text(json.dumps(homing_info, indent=2))
        if telemetry is not None:
            telemetry.resume()
        scope = ('frozen load-coverage study; no fit or profile' if args.prepared_study else
                 'held-out J2 configuration probe; no fit or profile' if args.configuration_probe else
                 'held-out J2 position probe; no fit or profile' if args.position_probe else
                 'loaded study '+('validation' if args.loaded_validation else 'training')+'; no fit or profile' if loaded else
                 'held-out posture collection; no fit or profile' if args.posture_only else
                 f'record-only {meta.get("sweep_protocol", "fixed_time_v1")} sweeps; no fit or profile' if args.record_only else
                 'narrow identification + independent posture validation')
        node.status(f'COLLECTING: {scope}, '
                    f'{meta["duration_s"]/60:.1f} minutes; do not touch robot')
        if c.get('sweep_protocol') == 'matched_v2':
            from .settling import SettlingMonitor
            node.settling_monitor = SettlingMonitor(meta)
        node.run_goal(data, record=True, prepared=collection_goal)
        if node.settling_monitor is not None:
            node.settling_monitor.finish()
        completed = True
        measured = node.recorded_measurements()
        np.savez_compressed(folder/'measurements.npz', **measured)
        if collection_only:
            if args.prepared_study:
                move_to_completion(node, c, folder, finish_pose)
            node.status(f'COLLECTION_COMPLETE: {"ready" if args.prepared_study else "zero"} pose held; {folder}/measurements.npz. '
                        f'Sensor timing verified={c["sensor_timing_verified"]}; '
                        'no fitting, profile or latest update')
        elif args.mock:
            # GenericSystem has no motor friction/current model. Never fit its zero effort.
            move_to_completion(node, c, folder, finish_pose)
            node.status('MOCK MOTION COMPLETE: ready pose reached; GenericSystem effort is not identification data; no profile exported')
        else:
            node.status('FITTING: robot holds zero in position mode')
            report = save_result(meta, measured, folder, 'hardware', fingerprint(urdf), baseline_path())
            if node.failure:
                (folder/'profile.json').unlink(missing_ok=True)
                raise RuntimeError(node.failure)
            move_to_completion(node, c, folder, finish_pose)
            if args.latest:
                activate_latest(folder, args.latest)
            node.status(f'COMPLETE: ready pose held in position mode; {folder}/profile.json. '
                        'Stop this launch, then start freedrive.launch.py (no homing)')
    except BaseException as exc:
        node.failure = node.failure or str(exc)
        (folder/'profile.json').unlink(missing_ok=True)
        if rclpy.ok():
            node.status('FAILED: '+str(exc)+'; no new profile activated')
        raise
    finally:
        if telemetry is not None:
            telemetry.close()
        if node.abort_sample is not None:
            (folder/'abort_sample.json').write_text(json.dumps(node.abort_sample, indent=2))
        (folder/'state_timing.json').write_text(json.dumps(node.state_timing, indent=2))
        if node.settling_monitor is not None:
            (folder/'settling.json').write_text(json.dumps(dict(
                checks=node.settling_monitor.results,
                pending=len(node.settling_monitor.pending),
                passed=completed and not node.failure and not node.settling_monitor.pending,
                current_timing_qualified=False), indent=2))
        node.stop_recording()
        measured = node.recorded_measurements()
        if len(measured['t']) and not (folder/'measurements.npz').exists():
            np.savez_compressed(folder/'measurements.npz', **measured)
        (folder/'collection.json').write_text(json.dumps(dict(completed=completed, failure=node.failure,
            source='generic_mock' if args.mock else 'hardware', rows=node.count,
            current_abort_policy=c.get('current_abort_policy', 'software_threshold'),
            measured_torque_abort_nm=c['measured_torque_abort_nm'],
            record_only=collection_only, posture_validation_only=args.posture_only,
            loaded_study=loaded,
            position_probe=args.position_probe or args.configuration_probe,
            configuration_probe=args.configuration_probe,
            generalization_study=bool(args.prepared_study),
            validation_joint=args.validation_joint,
            loaded_study_role=('validation' if args.loaded_validation else 'training') if loaded else None,
            sensor_timing_verified=c['sensor_timing_verified'],
            identification_eligible=completed and not node.failure and not collection_only
                and not args.mock and c['sensor_timing_verified'],
            plan_sha256=meta['sha256'], robot_fingerprint=fingerprint(urdf)), indent=2))
        executor.shutdown(timeout_sec=2); spin_thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
