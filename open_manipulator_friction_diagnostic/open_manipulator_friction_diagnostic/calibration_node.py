"""Shared Gazebo/hardware position collector and fixed-name calibration export."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import signal
import threading
import time

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as RosDuration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory

from .artifact import JOINTS, load_calibration, save_calibration, runtime_snapshot
from .node import prepare_goal
from .protocol import sha, validate_urdf_plan
from .tracking_delay import CollectionTrackingDelay, at, load
from .position_runtime import RuntimeHeartbeat


def export_identification(result, previous, target, *, check_cancelled=None):
    """Keep dither independent, and never replace a hardware model with simulation."""
    if result['source'] != 'hardware' or not result.get('usable'):
        return False
    from .joint_selection import merge_calibration
    merged = merge_calibration(result, previous)
    save_calibration(target, runtime_snapshot(merged), check_cancelled=check_cancelled, allow_partial=True)
    result.update(retained_joints=merged['retained_joints'], calibration_complete=merged['calibration_complete'],
                  missing_joints=merged['missing_joints'])
    return True


class Collector(Node):
    def __init__(self, folder, config, simulation):
        super().__init__('friction_calibration', parameter_overrides=[
            Parameter('use_sim_time', value=simulation)])
        self.folder, self.config, self.simulation = folder, config, simulation
        self.latest = None
        self.last_stamp = None
        self.last_rx = time.monotonic()
        self.failure = None
        self.state_failure = None
        self.tolerated_gaps = []
        self.timing = None
        self.reference_start = 0.
        self.recording_segments = []
        self.reference = None
        self.epoch = None
        self.record = False
        self.count = 0
        self.lock = threading.Lock()
        self.max_gap = 0.
        self.max_tracking = 0.
        self.max_raw_tracking = 0.
        self.tracking = None
        self.buffer = None
        self.runtime_monitor = False
        self.runtime_heartbeat = RuntimeHeartbeat()
        if not simulation:
            self.create_subscription(String, '/friction_runtime/heartbeat', self.receive_runtime,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
            self.create_timer(.05, self.poll_runtime)
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.create_subscription(JointState, '/joint_states', self.state, QoSProfile(
            depth=32, reliability=ReliabilityPolicy.BEST_EFFORT))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.recording = self.create_publisher(Bool, '~/recording', latched)
        self.publisher = self.create_publisher(String, '~/status', latched)

    def receive_runtime(self, message):
        try:
            self.runtime_heartbeat.observe(json.loads(message.data), time.monotonic())
            self.poll_runtime()
        except Exception as exc:
            self.failure = str(exc)

    def poll_runtime(self):
        if not self.runtime_monitor or self.failure:
            return
        try:
            self.runtime_heartbeat.check(time.monotonic(), preparing=self.reference is None)
        except Exception as exc:
            self.failure = str(exc)

    def check_runtime(self, *, preparing=False):
        if self.failure:
            raise RuntimeError(self.failure)
        from .position_runtime import require_position_runtime
        return require_position_runtime(json.loads(self.call('/friction_runtime/status', timeout=2.)),
                                        preparing=preparing)

    def wait_enable_settled(self):
        from .homing_review import EnableSettling
        monitor = EnableSettling()
        started = time.monotonic()
        self.status('SETTLING_AFTER_ENABLE')
        while True:
            with self.lock:
                q, v, _ = self.latest
                stamp, received = self.last_stamp, self.last_rx
                failure = self.failure
            if failure or time.monotonic()-received > .1:
                raise RuntimeError(failure or 'Stale position state after motor enable')
            if monitor.observe(stamp, q, v):
                self.check_runtime()
                return q.copy(), v.copy(), time.monotonic()-started
            if time.monotonic()-started > 5.:
                raise RuntimeError('Position did not settle within 5 seconds after motor enable; '
                                   f'last velocity={v.tolist()}')
            time.sleep(.005)

    def status(self, phase, **values):
        text = json.dumps(dict(phase=phase, **values))
        self.publisher.publish(String(data=text))
        self.get_logger().info(text)

    def state(self, msg):
        with self.lock:
            if self.failure or not all(j in msg.name for j in JOINTS):
                return
            if any(len(x) != len(msg.name) for x in (msg.position, msg.velocity, msg.effort)):
                self.failure = 'Position, velocity and effort states are required'
                return
            ids = [msg.name.index(j) for j in JOINTS]
            q, v, e = [np.asarray(x)[ids] for x in (msg.position, msg.velocity, msg.effort)]
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if not np.isfinite(np.r_[stamp, q, v, e]).all():
                self.failure = 'Nonfinite measured state'
                return
            if self.last_stamp is not None:
                gap = stamp-self.last_stamp
                # Gazebo can publish the same cached controller stamp more than once.
                if self.simulation and gap == 0:
                    return
                if gap <= 0:
                    self.failure = 'Nonmonotonic measured state'
                    return
                if self.record:
                    self.max_gap = max(self.max_gap, gap)
                    if not self.simulation and gap > .05:
                        detail = dict(kind='stamp_gap', stamp_s=stamp,
                            previous_stamp_s=self.last_stamp, gap_s=gap,
                            receive_gap_s=time.monotonic()-self.last_rx,
                            stamp_age_s=self.get_clock().now().nanoseconds*1e-9-stamp)
                        if self.timing is None or not self.timing.allowed(self.last_stamp-self.epoch, stamp-self.epoch):
                            self.state_failure = detail
                            self.failure = f'Acquisition state gap exceeds window limit: {gap:.6f} s'
                            return
                        self.tolerated_gaps.append(dict(detail, trajectory_time_s=stamp-self.epoch))
            self.latest = (q, v, e)
            self.last_stamp = stamp
            self.last_rx = time.monotonic()
            if self.reference is None or self.epoch is None:
                return
            t = stamp-self.epoch
            if t < self.reference_start:
                return
            if not hasattr(self, 'delays'):
                self.delays = np.zeros(6) if self.simulation else load(self.config)
            if self.tracking is not None:
                previous = len(self.tracking.results)
                try:
                    self.tracking.observe(t, q)
                except ValueError as exc:
                    self.failure = str(exc)
                    return
                self.delays = self.tracking.delays
                if len(self.tracking.results) != previous:
                    report = self.tracking.report()
                    self.status('TRACKING_DELAY_IDENTIFIED', delays_s=report['delays_s'])
            errors = abs(q-at(self.reference, t, self.delays))
            error = float(np.max(errors))
            raw = abs(q-at(self.reference, t, np.zeros(6)))
            self.max_raw_tracking = max(self.max_raw_tracking, float(np.max(raw)))
            self.max_tracking = max(self.max_tracking, error)
            if error > (.2 if self.simulation else .08):
                j = int(np.argmax(errors))
                self.failure = (f'Position tracking error on joint{j+1}: {error:.4f} rad '
                    f'(raw {raw[j]:.4f}, position lag {self.delays[j]:.4f} s)')
                return
            if self.record and t <= self.reference['t'][-1] + .05:
                if self.count >= len(self.buffer):
                    self.failure = 'Acquisition buffer full'
                    return
                if not self.simulation:
                    age = self.get_clock().now().nanoseconds*1e-9-stamp
                    if age > .1:
                        self.state_failure = dict(kind='sample_age', stamp_s=stamp, stamp_age_s=age)
                        self.failure = f'Acquisition sample older than 100 ms: {age:.6f} s'
                        return
                self.buffer[self.count] = np.r_[t, q, v, e]
                self.count += 1

    def call(self, name, timeout=30.):
        client = self.create_client(Trigger, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError('Service unavailable: '+name)
            future = client.call_async(Trigger.Request())
            end = time.monotonic()+timeout
            while not future.done():
                if time.monotonic() > end:
                    raise RuntimeError('Service timeout: '+name)
                time.sleep(.01)
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            return response.message
        finally:
            self.destroy_client(client)

    def _wait_ready(self, service_type, name, ready, timeout):
        """A live service can legitimately report that startup is still pending."""
        client = self.create_client(service_type, name)
        deadline = time.monotonic()+timeout
        try:
            while True:
                if self.failure or time.monotonic() >= deadline:
                    raise RuntimeError(self.failure or 'Readiness timed out: '+name)
                if not client.wait_for_service(timeout_sec=min(.2, deadline-time.monotonic())):
                    continue
                future = client.call_async(service_type.Request())
                while not future.done():
                    if self.failure or time.monotonic() >= deadline:
                        raise RuntimeError(self.failure or 'Readiness timed out: '+name)
                    time.sleep(.01)
                if ready(future.result()):
                    return
                time.sleep(.05)
        finally:
            self.destroy_client(client)

    def wait_controller_active(self, timeout=120.):
        # The action server is advertised during configure, before activation.
        from controller_manager_msgs.srv import ListControllers
        self._wait_ready(ListControllers, '/controller_manager/list_controllers',
                         lambda response: any(controller.name == 'arm_controller' and
                             controller.state == 'active' for controller in response.controller), timeout)

    def wait_video_ready(self, *, recording=False, timeout=120.):
        self._wait_ready(Trigger, '/friction_calibration/video/ready',
                         lambda response: response.success and
                             (not recording or response.message == 'recording'), timeout)

    def run_goal(self, data, prepared=None, *, record=False, label='COLLECTING', start_time=0.):
        goal, transport = prepared or prepare_goal(data, self.config['max_acceleration_rad_s2'])
        (self.folder/(label.lower()+'_transport.json')).write_text(json.dumps(transport, indent=2))
        lead = 30. if self.simulation and record else (5. if record else 1.)
        tracking, timing = None, None
        if record and not self.simulation:
            metadata = json.loads((self.folder/'plan.json').read_text())
            from .acquisition_timing import AcquisitionTiming
            timing = AcquisitionTiming(metadata)
            tracking = CollectionTrackingDelay(metadata, data)
            # No callback lock or active trajectory while replaying the prefix.
            for row in self.buffer[:self.count]:
                tracking.observe(row[0], row[1:7])
        with self.lock:
            self.reference, self.record = data, record
            self.reference_start = start_time
            if record and not self.simulation:
                self.timing, self.tracking = timing, tracking
                self.delays = tracking.delays
            self.epoch = self.get_clock().now().nanoseconds*1e-9 + lead-start_time
            if record:
                self.recording_segments.append(dict(row_start=self.count,epoch_s=self.epoch))
        ns = round((self.epoch+start_time)*1e9)
        goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = divmod(ns, 10**9)
        handle = None
        started = time.monotonic()
        try:
            if not self.simulation:
                self.check_runtime()
            request = self.client.send_goal_async(goal)
            while not request.done():
                if self.failure or time.monotonic()-started > 30:
                    raise RuntimeError(self.failure or 'Trajectory acceptance timed out')
                time.sleep(.005)
            handle = request.result()
            if not handle.accepted:
                raise RuntimeError('Trajectory rejected')
            result = handle.get_result_async()
            report_at = 0.
            while not result.done():
                now = time.monotonic()
                elapsed = self.get_clock().now().nanoseconds*1e-9-self.epoch
                if self.failure or now-self.last_rx > (10. if self.simulation else .1):
                    raise RuntimeError(self.failure or 'Joint state reception stopped')
                if elapsed > data['t'][-1]+15:
                    raise RuntimeError('Trajectory did not finish on time')
                if now >= report_at:
                    self.status(label, elapsed_s=max(0., elapsed), duration_s=float(data['t'][-1]))
                    if record:
                        self.journal.commit(self.count)
                        (self.folder/'runtime_monitor.json').write_text(
                            json.dumps(self.runtime_heartbeat.report(), indent=2))
                        with self.lock:
                            tracking_report = self.tracking.report() if self.tracking else None
                        if tracking_report is not None:
                            (self.folder/'position_tracking.json').write_text(json.dumps(tracking_report, indent=2))
                    report_at = now+30
                time.sleep(.005)
            if self.failure or result.result().result.error_code != 0:
                raise RuntimeError(self.failure or str(result.result().result))
        except BaseException:
            if handle is not None:
                cancelled = handle.cancel_goal_async()
                end = time.monotonic()+2
                while not cancelled.done() and time.monotonic() < end:
                    time.sleep(.005)
            raise
        finally:
            with self.lock:
                self.record = False
                self.reference = None
                self.epoch = None
            if record:
                self.recording_segments[-1]['row_stop'] = self.count
                self.journal.commit(self.count, closed=True)
                if self.tracking is not None:
                    (self.folder/'position_tracking.json').write_text(json.dumps(self.tracking.report(), indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--calibration-file', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--simulation', action='store_true')
    parser.add_argument('--validate', action='store_true')
    parser.add_argument('--collect-dither', action='store_true')
    parser.add_argument('--resume-from', type=Path)
    args = parser.parse_args(argv)
    folder = args.output
    meta = json.loads((folder/'plan.json').read_text())
    c = yaml.safe_load((folder/'config.yaml').read_text())
    data = dict(np.load(folder/'trajectory.npz'))
    resume, prefix, approach, approach_prepared = None, None, None, None
    if args.resume_from:
        from .collection_resume import prepare_resume, command_suffix, checkpoint_approach
        resume, prefix = prepare_resume(args.resume_from, meta, data)
        (folder/'resume.json').write_text(json.dumps(resume, indent=2))
        if resume['checkpoint_scope'] == 'paired_motion':
            approach = checkpoint_approach(meta, data, resume['start_index'])
            if approach is not None:
                np.savez_compressed(folder/'checkpoint_approach.npz', **approach)
                approach_prepared = prepare_goal(approach, c['max_acceleration_rad_s2'])
    start_index = resume['start_index'] if resume else 0
    commands = command_suffix(data, start_index) if resume else data
    prepared = prepare_goal(commands, c['max_acceleration_rad_s2'],
                            [s[k]-start_index for s in meta['segments'] for k in ('start', 'stop') if s[k]>=start_index])
    previous = load_calibration(args.calibration_file, allow_partial=True) if args.calibration_file.is_file() else None
    from .recording_journal import RecordingJournal
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Collector(folder, c, args.simulation)
    node.journal = RecordingJournal(folder, int((meta['duration_s']+1)*600), meta['sha256'])
    node.buffer = node.journal.buffer
    if resume:
        node.buffer[:len(prefix)] = prefix
        node.count = len(prefix)
        node.recording_segments = deepcopy(resume['recording_segments'])
        node.journal.commit(node.count)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    interrupted = [False]
    processing = [False]
    def stop(_signum, _frame):
        interrupted[0] = True
        node.failure = 'Collection interrupted'
        if processing[0]:
            raise KeyboardInterrupt('Calibration processing cancelled; previous artifact retained')
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    started = time.monotonic()
    report = dict(source='gazebo' if args.simulation else 'hardware', completed=False,
                  failure=None, plan_sha256=meta['sha256'], hardware_started=not args.simulation)
    try:
        node.status('WAITING_FOR_POSITION_CONTROLLER')
        deadline = time.monotonic()+120
        while node.latest is None or not node.client.server_is_ready():
            if node.failure or time.monotonic() > deadline:
                raise RuntimeError(node.failure or 'Position bringup timed out')
            time.sleep(.02)
        if not args.simulation:
            node.runtime_monitor = True
            while node.check_runtime(preparing=True)['phase'] != 'awaiting_position_start':
                if node.failure or time.monotonic() > deadline:
                    raise RuntimeError(node.failure or 'Drives did not become ready for torque-OFF preparation')
                time.sleep(.05)
        else:
            node.wait_controller_active()
            node.wait_video_ready()
            node.recording.publish(Bool(data=True))
            node.wait_video_ready(recording=True)
        from .homing_review import review_in_worker, require_unchanged_start, rebase_homing
        q, v, _ = node.latest
        node.status('PREPARING_HOMING', motors_enabled=False if not args.simulation else None)
        homing, homing_info = review_in_worker(q, v, c, folder/'robot.urdf', folder)
        require_unchanged_start(q, node.latest[0], node.latest[1])
        np.savez_compressed(folder/'homing_trajectory.npz', **homing)
        homing_prepared = prepare_goal(homing, c['max_acceleration_rad_s2'])
        if not args.simulation:
            node.check_runtime(preparing=True)
            require_unchanged_start(q, node.latest[0], node.latest[1])
            node.call('/friction_runtime/start_position')
            deadline = time.monotonic()+30.
            while not node.check_runtime(preparing=True)['ready']:
                if time.monotonic() > deadline:
                    raise RuntimeError('Motors did not become ready after preparation')
                time.sleep(.05)
            settled_q, settled_v, settling_s = node.wait_enable_settled()
            homing, homing_info = rebase_homing(homing, homing_info, settled_q, settled_v, c)
            homing_info['enable_settling']['wait_s'] = settling_s
            validate_urdf_plan({}, homing, (folder/'robot.urdf').read_text())
            homing_prepared = prepare_goal(homing, c['max_acceleration_rad_s2'])
            np.savez_compressed(folder/'homing_trajectory.npz', **homing)
            (folder/'homing.json').write_text(json.dumps(homing_info, indent=2))
            require_unchanged_start(settled_q, node.latest[0], node.latest[1])
        node.run_goal(homing, homing_prepared, label='HOMING')
        homing_info.update(completed=True, measured_end_rad=node.latest[0].tolist())
        (folder/'homing.json').write_text(json.dumps(homing_info, indent=2))
        if approach is not None:
            node.run_goal(approach, approach_prepared, label='RETURNING_TO_CHECKPOINT')
        node.run_goal(data, prepared, record=True, start_time=float(data['t'][start_index]))
        a = node.buffer[:node.count]
        stamps = np.empty(node.count)
        for block in node.recording_segments:
            section = slice(block['row_start'], block['row_stop'])
            stamps[section] = a[section,0]+block['epoch_s']
        np.savez_compressed(folder/'measurements.npz', t=a[:,0], q=a[:,1:7], v=a[:,7:13], effort=a[:,13:19], stamp_s=stamps)
        report.update(completed=True, samples=node.count, collection_duration_s=float(data['t'][-1]),
            homing_duration_s=float(homing['t'][-1]), max_state_gap_s=node.max_gap,
            max_tracking_error_rad=node.max_tracking, max_raw_tracking_error_rad=node.max_raw_tracking,
            position_tracking=node.tracking.report() if node.tracking else None,
            final_q_rad=node.latest[0].tolist())
    except BaseException as exc:
        report['failure'] = f'{type(exc).__name__}: {exc}'
        node.get_logger().error(report['failure'])
    finally:
        node.runtime_monitor = False
        report['wall_motion_s'] = time.monotonic()-started
        report.update(max_state_gap_s=node.max_gap, state_failure=node.state_failure,
                      runtime_monitor=node.runtime_heartbeat.report(),
                      tolerated_transition_gaps=node.tolerated_gaps, resume=resume,
                      recording_segments=node.recording_segments)
        (folder/'collection.json').write_text(json.dumps(report, indent=2))
        if args.simulation:
            node.status('COLLECTION_COMPLETE' if report['completed'] else 'FAILED', **report)
            node.recording.publish(Bool(data=False))
            if not node.recording.wait_for_all_acked(RosDuration(seconds=2)):
                node.get_logger().error('Video stop signal was not acknowledged before collector exit')
        else:
            node.status('STOPPING_MOTORS')
            try:
                from .runtime import wait_for_motor_shutdown
                state = json.loads((folder/'runtime/runtime.json').read_text())
                if not (state.get('software_stopped') or state.get('all_off_confirmed')):
                    node.call('/friction_runtime/stop', timeout=2.)
                wait_for_motor_shutdown(folder/'runtime/runtime.json')
            except Exception as exc:
                report.update(completed=False, failure=f'Motor shutdown unconfirmed: {exc}')
                (folder/'collection.json').write_text(json.dumps(report, indent=2))
        executor.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
    if not report['completed'] or interrupted[0]:
        raise SystemExit(1)
    processing[0] = True
    def check_cancelled():
        if interrupted[0]:
            raise InterruptedError('Calibration cancelled before publication')
    if args.simulation:
        # GazeboSimSystem position mode is a kinematic servo, not a Y-series current sensor.
        # Preserve real coefficients; report motion/collection proof without inventing torque data.
        result = dict(source='gazebo', usable=False, calibration_replaced=False,
            reason='Gazebo position servo does not identify physical actuator friction',
            measured_samples=node.count, motion_completed=True, validation_requested=args.validate)
    else:
        from .automatic_motion import ingest
        from .identification import identify
        check_cancelled()
        from .processing_progress import ProcessingProgress
        selected = meta.get('selected_joints', sorted({c['joint']+1 for c in meta['automatic_cycles']}))
        with ProcessingProgress(selected) as progress:
            progress.update('EXTRACTING_MEASUREMENTS')
            measured = ingest(folder, args.baseline, spatial_bins=int(c['kinetic_spatial_bins']))
            check_cancelled()
            (folder/'cycles.json').write_text(json.dumps(measured['cycles'], indent=2))
            identity = dict(robot_unit=c['robot_unit'], robot_fingerprint=meta['robot_fingerprint'], baseline_sha256=meta['baseline_sha256'])
            result = identify(measured['cycles'], identity, args.baseline, previous=previous,
                              fitting=folder/'fitting.yaml', validate=args.validate, joints=selected,
                              progress=progress.update)
            check_cancelled()
        if args.collect_dither and result['usable']:
            from .dither_calibration import collect
            # A partial dither result must not replace the previous full bank.
            result = collect(folder/'dither', result, previous)
            check_cancelled()
        if result['usable']:
            print('SAVING calibration: Writing results. Do not stop this process.', flush=True)
        result['calibration_replaced'] = export_identification(result, previous, args.calibration_file,
                                                               check_cancelled=check_cancelled)
    (folder/'identification.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    failed = not args.simulation and not result['calibration_replaced']
    print(json.dumps(dict(phase='FAILED' if failed else 'COMPLETE', calibration_replaced=result['calibration_replaced'],
                         selected_joints=result.get('selected_joints', meta.get('selected_joints')),
                         calibration_complete=result.get('calibration_complete'), missing_joints=result.get('missing_joints', []),
                         result=str(folder/'identification.json')), ensure_ascii=False), flush=True)
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
