"""Own ROS children through ordered motor shutdown and explicit process exit.

DYNAMIXEL hardware interfaces own both UARTs. This process uses direct startup
configuration reads, queued Torque Enable writes and periodic GPIO readback.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

from .drive_lifecycle import CONFIGURATION_ITEMS, DriveLifecycle, prepare_hardware_urdf


class ChildProcesses:
    """Separate process groups keep terminal/launch signals away during braking."""

    def __init__(self, output):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.children = []
        self.log_threads = []
        self.log_errors = {}

    def start(self, label, command):
        log = (self.output / (label + '.log')).open('wb')
        try:
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        except BaseException:
            log.close()
            raise
        self.children.append((label, process, log))
        def drain():
            with process.stdout:
                while chunk := process.stdout.read1(65536):
                    try:
                        log.write(chunk)
                        log.flush()
                    except OSError as exc:
                        self.log_errors[label] = str(exc)
                        # Keep draining: a full diagnostic pipe must not stop RT I/O.
        worker = threading.Thread(target=drain, name=label+'_log', daemon=True)
        worker.start()
        self.log_threads.append(worker)
        return process

    @staticmethod
    def _group_alive(group):
        # ros2 run can exit before the executable it spawned. Check the owned
        # process group, excluding zombies which cannot hold a UART or log pipe.
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) == group and fields[0] != 'Z':
                    return True
            except (OSError, ValueError, IndexError):
                continue
        return False

    def stop_all(self, *, confirmed_off, never_enabled=False, bus_owner_exited=False,
                 shutdown_requested=False):
        if not (confirmed_off or never_enabled or bus_owner_exited or shutdown_requested):
            raise RuntimeError('Torque OFF is unconfirmed; keep controller and bus owner alive')
        # Give the driver its normal SIGINT cleanup first. Each deadline is
        # shared by all children, rather than multiplied by the node count.
        groups = {process.pid for _, process, _ in self.children}
        for signum, grace in ((signal.SIGINT, 5.), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
            groups = {group for group in groups if self._group_alive(group)}
            if not groups:
                break
            for group in groups:
                try:
                    os.killpg(group, signum)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + grace
            while groups and time.monotonic() < deadline:
                for _, process, _ in self.children:
                    process.poll()
                groups = {group for group in groups if self._group_alive(group)}
                if groups:
                    time.sleep(.05)
        if groups:
            raise RuntimeError(f'ROS process groups did not exit: {sorted(groups)}')
        for (_, process, log), worker in zip(self.children, self.log_threads):
            try:
                process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                raise RuntimeError(f'Child {process.pid} was not reaped')
            worker.join(timeout=.5)
            if not worker.is_alive():
                log.close()


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    return value


def stop_output_acknowledged(mode, gate_may_be_enabled, probe_quiet):
    """A queued service reply alone does not prove the RT output is quiet."""
    if mode == 'current':
        return not gate_may_be_enabled
    if mode == 'probe':
        return probe_quiet
    return True


def wait_for_motor_shutdown(status_path, *, process=None, timeout=30.):
    """Bound collector/dither waits; software exit is not physical OFF proof."""
    deadline = time.monotonic() + timeout
    while True:
        try:
            state = json.loads(Path(status_path).read_text())
        except (OSError, ValueError):
            state = {}
        exited = process is not None and process.poll() is not None
        if state.get('all_off_confirmed'):
            if process is None or exited:
                return state
        elif (exited or state.get('software_stopped') or
              state.get('controller_manager_exit_code') is not None):
            raise RuntimeError('Runtime exited without confirmed motor OFF')
        if time.monotonic() >= deadline:
            raise TimeoutError('Motor shutdown confirmation timed out; no new motion will start')
        time.sleep(.05)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('current', 'position', 'probe'), required=True)
    parser.add_argument('--probe-joint', type=int, choices=range(1, 7))
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--controllers', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--defer-position-enable', action='store_true',
                        help='Keep torque OFF until the collector prepares its actual-start trajectory')
    parser.add_argument('--mock', action='store_true',
                        help='Developer only: require GenericSystem and never send drive writes')
    application_args = list(sys.argv[1:] if argv is None else argv)
    if '--ros-args' in application_args:
        from rclpy.utilities import remove_ros_args
        application_args = remove_ros_args(args=[sys.argv[0], *application_args])[1:]
    args = parser.parse_args(application_args)
    if (args.mode == 'probe') != (args.probe_joint is not None):
        parser.error('--probe-joint is required only for --mode probe')
    if not args.mock:
        # Another RMW/domain can still own the same physical OMY UARTs.
        # Never start a second writer beside an already running bringup.
        from .runtime_exclusion import require_no_controller
        require_no_controller()
    # Imports stay after parsing so --help is usable outside a ROS installation.
    import yaml
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from control_msgs.msg import DynamicJointState
    from dynamixel_interfaces.srv import GetDataFromDxl, SetDataToDxl
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger
    from .position_runtime import heartbeat_message

    args.output.mkdir(parents=True, exist_ok=True)
    urdf = prepare_hardware_urdf(args.urdf.read_text(), args.mode, probe_joint=args.probe_joint)
    root = ET.fromstring(urdf)
    plugins = [node.text for node in root.findall('ros2_control/hardware/plugin')]
    expected = ({'mock_components/GenericSystem'} if args.mock else
                {'dynamixel_hardware_interface/DynamixelHardware',
                 'open_manipulator_friction_diagnostic/GroupedDynamixelHardware'})
    if not plugins or any(plugin not in expected for plugin in plugins):
        raise ValueError('--mock must match every hardware plugin; physical mode requires DHI')
    arm = next(system for system in root.findall('ros2_control')
               if system.find("joint[@name='joint6']") is not None)
    torque_service = '/' + arm.findtext("hardware/param[@name='set_dynamixel_data_srv_name']").lstrip('/')
    read_service = '/' + arm.findtext("hardware/param[@name='get_dynamixel_data_srv_name']").lstrip('/')
    prepared = args.output / 'hardware.urdf'
    prepared.write_text(urdf)
    # The broadcaster must expose GPIOs, including the drive's own freshness tick.
    # Torque Enable=0 makes normal immediate hardware activation safe for priming.
    parameters = {
        'controller_manager': {'ros__parameters': {
            'robot_description': urdf,
        }},
        'robot_state_publisher': {'ros__parameters': {'robot_description': urdf}},
        'joint_state_broadcaster': {'ros__parameters': {
            'use_local_topics': False,
            'use_urdf_to_filter': False,
        }},
    }
    if args.mode == 'probe':
        parameters['current_probe'] = {'ros__parameters': {
            'target_joint': args.probe_joint, 'defer_probe_start': True,
            'monitor_drive_errors': True,
        }}
    parameter_file = args.output / 'runtime_parameters.yaml'
    parameter_file.write_text(yaml.safe_dump(parameters))
    children = ChildProcesses(args.output)
    machine = DriveLifecycle(args.mode, probe_joint=args.probe_joint, started_at=time.monotonic(),
                             defer_enable=args.defer_position_enable)
    requested_stop = [None]
    previous_handlers = {}
    control = None
    ever_enabled = False
    software_stopped = False
    gate_enabled = False
    gate_may_be_enabled = False
    probe_started = False
    probe_stop_sent = False
    probe_quiet = False
    probe_result = None
    probe_poll_at = 0.
    failure = None
    last_phase = None
    saved_at = 0.
    heartbeat_at = 0.
    heartbeat_sequence = 0
    last_spin = None
    diagnostics = dict(status_service_requests=0, max_running_loop_gap_s=0.)
    def elapsed_stage(name, started):
        elapsed = time.monotonic()-started
        key = 'max_'+name+'_s'
        diagnostics[key] = max(diagnostics.get(key, 0.), elapsed)
    from .runtime_journal import RuntimeJournal
    journal = RuntimeJournal(args.output)

    def record(event):
        journal.event(_json_safe({'time_monotonic': time.monotonic(), **event}))

    def on_signal(signum, _frame):
        if requested_stop[0] is None:
            requested_stop[0] = f'Signal {signal.Signals(signum).name}'

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, on_signal)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('friction_runtime')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    heartbeat_pub = node.create_publisher(String, '/friction_runtime/heartbeat',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def readback(message):
        rows = {name: dict(zip(item.interface_names, item.values))
                for name, item in zip(message.joint_names, message.interface_values)}
        machine.observe(rows, time.monotonic())

    def commands(message):
        machine.command(dict(zip(message.name, message.effort)), time.monotonic())

    node.create_subscription(DynamicJointState, '/dynamic_joint_states', readback, qos_profile_sensor_data)
    controller = 'current_probe' if args.mode == 'probe' else 'friction_compensation_controller'
    node.create_subscription(JointState, '/' + controller + '/command_state',
                             commands, qos_profile_sensor_data)
    writer = node.create_client(SetDataToDxl, torque_service)
    reader = node.create_client(GetDataFromDxl, read_service)
    gate = node.create_client(SetBool, '/friction_compensation_controller/accept_external_effort')
    gate_status = node.create_client(Trigger, '/friction_compensation_controller/get_friction_status')
    probe_start = node.create_client(Trigger, '/current_probe/start_probe') if args.mode == 'probe' else None
    probe_stop = node.create_client(Trigger, '/current_probe/stop_probe') if args.mode == 'probe' else None
    probe_status = node.create_client(Trigger, '/current_probe/save_status') if args.mode == 'probe' else None

    def brief_status():
        return dict(phase=machine.phase, reason=machine.reason,
                    ready=requested_stop[0] is None and machine.accept_external and
                           (probe_started if args.mode == 'probe' else args.mode == 'position' or gate_enabled),
                    failure=failure, controller_manager_exit_code=control.poll() if control is not None else None)

    def publish_heartbeat(force=False):
        nonlocal heartbeat_at, heartbeat_sequence
        now = time.monotonic()
        if force or now-heartbeat_at >= .05:
            heartbeat_sequence += 1
            heartbeat_pub.publish(String(data=json.dumps(
                heartbeat_message(brief_status(), heartbeat_sequence, now), allow_nan=False)))
            heartbeat_at = now
            elapsed_stage('heartbeat_publish', now)

    def status():
        result = {**machine.status(), **brief_status(), 'mode': args.mode, 'mock': args.mock,
                  'software_stopped': software_stopped,
                  'motor_enable_requested': ever_enabled, 'failure': failure,
                  'probe': probe_result,
                  'final_arm_modes_restored': False,
                  'mode_restore_policy': 'Next standard bringup configures its requested modes with torque OFF.',
                  'diagnostics': dict(diagnostics, heartbeat_count=heartbeat_sequence,
                                      process_cpu_s=time.process_time(),
                                      max_background_journal_write_s=journal.max_write_s,
                                      child_log_errors=dict(children.log_errors))}
        if args.mock:
            result['all_off_confirmed'] = False
            result['physical_state_confirmed'] = False
        return _json_safe(result)

    def stop_service(_request, response):
        requested_stop[0] = requested_stop[0] or 'Stop service requested'
        response.success = True
        response.message = 'Ordered stop requested; poll /friction_runtime/status for all_off_confirmed'
        return response

    def start_position_service(_request, response):
        try:
            if requested_stop[0] is not None:
                raise ValueError(requested_stop[0])
            machine.request_position_start(time.monotonic())
            response.success = True
            response.message = 'Position enable requested after actual-start preparation'
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def status_service(_request, response):
        diagnostics['status_service_requests'] += 1
        response.success = True
        response.message = json.dumps(status(), allow_nan=False)
        return response

    node.create_service(Trigger, '/friction_runtime/stop', stop_service)
    node.create_service(Trigger, '/friction_runtime/status', status_service)
    if args.defer_position_enable:
        node.create_service(Trigger, '/friction_runtime/start_position', start_position_service)

    def save(force=False):
        nonlocal last_phase, saved_at
        now = time.monotonic()
        if last_phase != machine.phase:
            record({'transition': status()})
            print(f'Friction runtime: {machine.phase} {machine.reason}', flush=True)
            if machine.phase == 'shutdown_unconfirmed':
                if not ever_enabled:
                    print('Startup failed before any motor ON request. Physical OFF readback is '
                          'unavailable; closing startup processes.', flush=True)
                elif control is None or control.poll() is not None:
                    print('Torque OFF is UNCONFIRMED. Controller/bus process is no longer alive.', flush=True)
                else:
                    print('Torque OFF is UNCONFIRMED. Motor power may be cut or communication lost. '
                          'Ctrl+C will close this runtime and its ROS children after the stop deadline; '
                          'software exit does not confirm motor OFF.', flush=True)
            if machine.phase == 'enabling' and machine.current_joints:
                print('Gravity/current goal primed. Firmware brake/goal delays are unchanged; '
                      'preloaded current is not proof of support during that transition.', flush=True)
            last_phase = machine.phase
            force = True
        publish_heartbeat(force)
        if force or now - saved_at >= .5:
            write_started = time.monotonic()
            journal.status(status())
            saved_at = now
            elapsed_stage('status_write', write_started)

    def spin(seconds=.02):
        nonlocal last_spin
        now = time.monotonic()
        if last_spin is not None and machine.phase == 'running':
            diagnostics['max_running_loop_gap_s'] = max(
                diagnostics['max_running_loop_gap_s'], now-last_spin)
        last_spin = now
        # Keep the executor registration and its ready-callback iterator alive.
        # rclpy.spin_once(node) adds/removes the node on every 400 Hz message.
        spin_started = time.monotonic()
        executor.spin_once(timeout_sec=seconds)
        elapsed_stage('executor_spin', spin_started)
        publish_heartbeat()

    def call(client, request, timeout=2.):
        deadline = time.monotonic() + timeout
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Service unavailable: {client.srv_name}')
            spin()
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                client.remove_pending_request(future)
                raise TimeoutError(f'Service response timed out: {client.srv_name}')
            spin()
        result = future.result()
        if result is None:
            raise RuntimeError(f'Empty service response: {client.srv_name}')
        return result

    def set_gate(enabled):
        nonlocal gate_enabled, gate_may_be_enabled
        if args.mode != 'current':
            return
        request = SetBool.Request()
        request.data = enabled
        if enabled:
            # A lost response does not prove the controller rejected ON.
            gate_may_be_enabled = True
        response = call(gate, request)
        if not response.success:
            raise RuntimeError('External effort gate rejected request: ' + response.message)
        deadline = time.monotonic()+3.
        while True:
            applied = call(gate_status, Trigger.Request())
            if applied.success and json.loads(applied.message).get('motor_ready_applied') is enabled:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Controller did not apply motor-ready gate in an RT update')
            spin()
        gate_enabled = enabled
        gate_may_be_enabled = enabled
        record({'accept_external_effort': enabled})

    def poll_probe():
        nonlocal probe_result, probe_poll_at, probe_quiet
        response = call(probe_status, Trigger.Request())
        probe_result = json.loads(response.message)
        probe_poll_at = time.monotonic()
        if not probe_result.get('journal_ok', False):
            raise RuntimeError('Probe CSV journal failed')
        probe_quiet = probe_result.get('state') in (2, 3, 5)
        return probe_result

    def apply_requests(requests):
        nonlocal ever_enabled, failure
        for joint, enabled in requests:
            if enabled and requested_stop[0] is not None:
                break
            if enabled and machine.phase != 'enabling':
                break
            machine.requested(joint, enabled, time.monotonic())
            ever_enabled = ever_enabled or enabled
            request = SetDataToDxl.Request()
            request.id = joint
            request.item_name = 'Torque Enable'
            request.item_data = int(enabled)
            try:
                response = call(writer, request)
                if not response.result:
                    raise RuntimeError(f'Drive {joint} rejected torque request')
                record({'torque_request': {'id': joint, 'enabled': enabled, 'queued': True}})
                if enabled:
                    # Service waits spin the executor and may expose a drive
                    # fault. Re-evaluate before sending ON to another axis.
                    machine.step(time.monotonic())
            except Exception as exc:
                failure = str(exc)
                record({'torque_request_failed': {'id': joint, 'enabled': enabled, 'error': str(exc)}})
                machine.request_stop(time.monotonic(), failure)
                if enabled:
                    break

    def wait_spawner(process):
        deadline = time.monotonic() + 50.
        while process.poll() is None:
            spin()
            if requested_stop[0] is not None:
                raise InterruptedError(requested_stop[0])
            if control.poll() is not None:
                raise RuntimeError('Controller manager exited during startup; see controller_manager.log')
            if time.monotonic() >= deadline:
                raise TimeoutError('Controller spawner timed out')
        if process.returncode:
            raise RuntimeError('Controller spawner failed; see spawner.log')

    try:
        try:
            children.start('robot_state_publisher', ['ros2', 'run', 'robot_state_publisher',
                'robot_state_publisher', '--ros-args', '--params-file', str(parameter_file)])
            control = children.start('controller_manager', ['ros2', 'run', 'controller_manager',
                'ros2_control_node', '--ros-args', '--params-file', str(args.controllers),
                '--params-file', str(parameter_file), '--disable-external-lib-logs'])
            controller = {'current': 'friction_compensation_controller',
                          'position': 'arm_controller', 'probe': 'current_probe'}[args.mode]
            spawner = children.start('spawner', ['ros2', 'run', 'controller_manager', 'spawner',
                'joint_state_broadcaster', controller, '--controller-manager-timeout', '40'])
            wait_spawner(spawner)
            if not args.mock:
                settings = {}
                for joint in range(1, 7):
                    settings[joint] = {}
                    for item in CONFIGURATION_ITEMS:
                        if requested_stop[0] is not None:
                            raise InterruptedError(requested_stop[0])
                        request = GetDataFromDxl.Request()
                        request.id, request.item_name, request.timeout_sec = joint, item, 1.
                        response = call(reader, request)
                        if not response.result:
                            raise RuntimeError(f'Drive {joint}: startup read failed for {item}')
                        settings[joint][item] = response.item_data
                machine.configure_readback(settings)
                record({'configuration_readback': settings,
                        'source': 'DHI direct item service before motor ON'})
            machine.started_at = time.monotonic()
            if args.mode == 'current':
                set_gate(False)
            if args.mode == 'probe':
                deadline = time.monotonic() + 3.
                while not probe_stop.service_is_ready():
                    if time.monotonic() >= deadline:
                        raise RuntimeError('CurrentProbe stop service is required before motor activation')
                    spin()
                result = poll_probe()
                if result.get('state') != 4 or result.get('fault') != 0:
                    raise RuntimeError('CurrentProbe must await gravity-only deferred start')
            if args.mock:
                print('Developer GenericSystem runtime: no physical torque requests or hardware confirmation.', flush=True)
                machine.phase = 'running'
            save(True)
        except Exception as exc:
            failure = str(exc)
            machine.request_stop(time.monotonic(), failure)
            record({'startup_failure': failure})

        while True:
            spin()
            now = time.monotonic()
            if requested_stop[0] is not None:
                machine.request_stop(now, requested_stop[0])
            if control is None or control.poll() is not None:
                if not machine.safe_to_exit:
                    failure = failure or 'Controller manager exited before physical OFF confirmation'
                    machine.phase = 'shutdown_unconfirmed'
                    machine.reason = failure
                save(True)
                break
            if args.mock:
                requests = ()
                if machine.stop_at is not None:
                    machine.phase = 'stopped'
            else:
                requests = machine.step(now)
            if machine.stop_at is not None and requested_stop[0] is None and failure is None:
                failure = machine.reason
            if machine.stop_at is not None and gate_may_be_enabled:
                try:
                    # Controller gate OFF also clears external input and friction;
                    # original gravity continues throughout the brake transition.
                    set_gate(False)
                except Exception as exc:
                    failure = str(exc)
                    record({'gate_disable_failed': failure})
            if args.mode == 'probe':
                try:
                    if machine.stop_at is not None and not probe_stop_sent:
                        response = call(probe_stop, Trigger.Request())
                        if not response.success:
                            raise RuntimeError('Probe stimulus stop rejected: ' + response.message)
                        probe_stop_sent = True
                        poll_probe()
                    elif now - probe_poll_at >= .25:
                        result = poll_probe()
                        if result.get('state') == 3:
                            raise RuntimeError('CurrentProbe fault: ' + str(result.get('fault')))
                        if result.get('state') == 2 and result.get('saved') and result.get('fault') == 0:
                            requested_stop[0] = requested_stop[0] or 'Probe completed'
                    if (machine.accept_external and requested_stop[0] is None and
                            not probe_started and machine.stop_at is None):
                        response = call(probe_start, Trigger.Request())
                        if not response.success:
                            raise RuntimeError('Probe start rejected: ' + response.message)
                        probe_started = True
                        record({'probe_started_after_all_drives_running': True})
                except Exception as exc:
                    failure = str(exc)
                    machine.request_stop(time.monotonic(), failure)
                    record({'probe_service_failure': failure})
            # A stop acknowledgement queues work; wait for RT to report its
            # gravity-only stopping/completion state before torque-OFF writes.
            if machine.stop_at is None or stop_output_acknowledged(
                    args.mode, gate_may_be_enabled, probe_quiet):
                apply_requests(requests)
            if (machine.accept_external and requested_stop[0] is None and
                    args.mode == 'current' and not gate_enabled):
                try:
                    set_gate(True)
                except Exception as exc:
                    failure = str(exc)
                    machine.request_stop(time.monotonic(), failure)
            save()
            if machine.safe_to_exit:
                save(True)
                break
            if machine.phase == 'shutdown_unconfirmed' and (not ever_enabled or requested_stop[0] is not None):
                # The generated URDF inhibited automatic ON, and this process
                # sent no ON request. An explicit stop also closes processes
                # after the bounded OFF attempt (e.g. power cut by E-stop).
                # Neither case turns unavailable readback into confirmed OFF.
                save(True)
                break
    finally:
        if args.mode == 'probe' and control is not None and control.poll() is None:
            try:
                poll_probe()
            except Exception as exc:
                failure = failure or str(exc)
        explicit_exit = requested_stop[0] is not None and machine.phase == 'shutdown_unconfirmed'
        can_stop = machine.safe_to_exit or not ever_enabled or control is None or control.poll() is not None or explicit_exit
        if can_stop:
            children.stop_all(confirmed_off=machine.safe_to_exit, never_enabled=not ever_enabled,
                              bus_owner_exited=control is None or control.poll() is not None,
                              shutdown_requested=explicit_exit)
            software_stopped = True
            if not machine.safe_to_exit and ever_enabled:
                print('ROS child processes stopped. Motor OFF remains UNCONFIRMED; '
                      'check emergency-stop/motor power before another bringup.', flush=True)
        else:
            # Unexpected exceptions must not silently kill the only gravity/bus
            # owner. Normal signal/service shutdown is handled by the loop above.
            machine.phase = 'shutdown_unconfirmed'
            machine.reason = 'Supervisor failed before OFF; ROS children remain alive'
            print(machine.reason, flush=True)
        save(True)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        journal.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return 0 if machine.safe_to_exit and failure is None else 1


if __name__ == '__main__':
    raise SystemExit(main())
