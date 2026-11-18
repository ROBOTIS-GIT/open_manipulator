"""Run reviewed finite dither jobs through the ordinary ROS motor supervisor."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import yaml

from .dither_calibration import prepare_dither, record_run, finish_dither, apply_result


def _read(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def _start_runtime(folder, urdf, controllers, mode, joint=None):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'input.urdf').write_text(urdf)
    (folder / 'controllers.yaml').write_text(yaml.safe_dump(controllers))
    command = [sys.executable, '-m', 'open_manipulator_friction_diagnostic.runtime',
        '--mode', mode, '--urdf', str(folder / 'input.urdf'),
        '--controllers', str(folder / 'controllers.yaml'), '--output', str(folder)]
    if joint is not None:
        command += ['--probe-joint', str(joint)]
    log = (folder / 'supervisor.log').open('w')
    try:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    except BaseException:
        log.close()
        raise
    return process, log


def _stop_and_wait(process, folder):
    """The shared supervisor owns bounded cleanup of its bus-owning children."""
    from .runtime import wait_for_motor_shutdown
    if process.poll() is None:
        try:
            os.kill(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    return wait_for_motor_shutdown(folder/'runtime.json', process=process)


def _position_transfer(job, urdf, config, interruption):
    # ROS imports stay in the hardware entry point; the planning/fitting modules
    # can be used without ROS installed.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from .calibration_node import Collector
    from .completion import transfer_plan
    from .generalization_motion import check_geometry
    from .launch_support import controller_document
    from .protocol import validate_urdf_plan

    directory = Path(job['directory']) / 'position'
    process, log = _start_runtime(directory, urdf, controller_document('position'), 'position')
    node = executor = worker = None
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Collector(directory, config, False)
        interruption['node'] = node
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()
        deadline = time.monotonic()+120
        while (node.latest is None or not node.client.server_is_ready() or
               not _read(directory / 'runtime.json').get('ready')):
            if interruption['requested'] or node.failure or process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError(node.failure or 'Dither position transfer runtime did not become ready')
            time.sleep(.02)
        q, v, _ = node.latest
        if np.max(abs(v)) > .01:
            raise RuntimeError('Dither transfer start is moving')
        zero = np.zeros(6)
        # Small neutral probes return through zero, using the same slow position
        # path, exact original robot geometry, and ordinary drive mode startup.
        first = transfer_plan(q, zero, config)
        second = transfer_plan(zero, job['center_rad'], config)
        data = {k: np.concatenate([first[k][:-1], second[k]]) for k in ('q', 'v', 'a')}
        data['t'] = np.arange(len(data['q']))*config['sample_period_s']
        validate_urdf_plan({}, data, urdf)
        geometry = check_geometry(data['q'], directory / 'input.urdf')
        (directory / 'geometry.json').write_text(json.dumps(geometry, indent=2))
        if not geometry['passed'] or np.max(abs(node.latest[0]-q)) > .001:
            raise RuntimeError('Dither transfer geometry or unchanged starting pose check failed')
        np.savez_compressed(directory / 'trajectory.npz', **data)
        node.run_goal(data, label='DITHER_TRANSFER')
        if np.max(abs(node.latest[0]-job['center_rad'])) > .001:
            raise RuntimeError('Dither position transfer did not reach the reviewed pose')
    finally:
        interruption['node'] = None
        try:
            _stop_and_wait(process, directory)
        finally:
            log.close()
            if executor is not None:
                executor.shutdown()
            if worker is not None:
                worker.join(timeout=2.)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def _run_probe(job, urdf, interruption):
    directory = Path(job['directory']) / 'runtime'
    controllers = {'controller_manager': {'ros__parameters': {
        'update_rate': 400,
        'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        'current_probe': {'type': 'friction_compensation_controller/CurrentProbeController'}}},
        'current_probe': {'ros__parameters': job['parameters']}}
    process, log = _start_runtime(directory, urdf, controllers, 'probe', job['target_joint'])
    try:
        deadline = time.monotonic()+job['duration_s']+120
        while process.poll() is None:
            if interruption['requested'] or time.monotonic() > deadline:
                raise RuntimeError('Dither probe interrupted or exceeded its finite schedule')
            time.sleep(.1)
    finally:
        try:
            final = _stop_and_wait(process, directory)
        finally:
            log.close()
    if process.returncode != 0 or final.get('failure'):
        raise RuntimeError('Dither probe failed: '+str(final.get('failure')))
    events = [json.loads(line) for line in (directory / 'runtime_events.jsonl').read_text().splitlines()]
    proof = dict(source='hardware', all_motors_off=final['all_off_confirmed'],
        torque_enable=[final['axes'][str(j)]['Torque Enable'] for j in range(1, 7)],
        started_after_all_running=any(e.get('probe_started_after_all_drives_running') is True for e in events),
        runtime_sha256=__import__('hashlib').sha256((directory / 'runtime.json').read_bytes()).hexdigest(),
        register_modes_restored=False,
        mode_restore_policy='Each next standard runtime configures its mode with torque OFF')
    return record_run(job['directory'], final['probe'], proof)


def collect(output_dir, identification, previous=None):
    """Acquire every selected joint now; return an independently updated artifact.

    This entry point is called after the position collection runtime has stopped.
    Any missing, faulted, motionless, or unstable response fails the optional
    stage and leaves the on-disk current calibration unchanged.
    """
    from ament_index_python.packages import get_package_share_directory
    from .integration import description
    if identification.get('source') != 'hardware':
        raise ValueError('Current-probe dither selection requires physical motor measurements')
    folder = Path(output_dir).resolve()
    share = Path(get_package_share_directory('open_manipulator_friction_diagnostic'))
    identity = identification['identity']
    urdf = description('position')
    # The previous runtime records OFF before it finishes terminating its ROS
    # children. Do not let a second controller manager race for the UART.
    prior_runtime = folder.parent / 'runtime/runtime.json'
    deadline = time.monotonic()+120
    while prior_runtime.exists():
        state = _read(prior_runtime)
        if state.get('all_off_confirmed') and state.get('controller_manager_exit_code') is not None:
            break
        if time.monotonic() > deadline:
            raise RuntimeError('Previous position bus owner has not completed all-six OFF shutdown')
        time.sleep(.1)
    manifest = prepare_dither(share / 'config/dither.yaml', folder, urdf, identity, previous=previous,
                             joints=identification['reidentified_joints'])
    config_path = folder.parent / 'config.yaml'
    config = yaml.safe_load((config_path if config_path.exists() else share / 'config/collection.yaml').read_text())
    config['transfer_velocity_rad_s'] = min(config['transfer_velocity_rad_s'], .12)
    interruption = dict(requested=False, node=None)
    def stop(_signum, _frame):
        interruption['requested'] = True
        if interruption['node'] is not None:
            interruption['node'].failure = 'Dither collection interrupted'
    handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for index, job in enumerate(manifest['jobs']):
            if interruption['requested']:
                raise RuntimeError('Dither collection interrupted')
            print(f'Dither {index+1}/{len(manifest["jobs"])}: joint{job["target_joint"]} {job["plan"]["condition"]}', flush=True)
            _position_transfer(job, urdf, config, interruption)
            _run_probe(job, urdf, interruption)
        # Return to zero in position mode and confirm all motors OFF once more.
        final = dict(manifest['jobs'][-1], directory=str(folder / 'return_zero'), center_rad=[0.]*6)
        _position_transfer(final, urdf, config, interruption)
        if interruption['requested']:
            raise InterruptedError('Dither collection interrupted')
        result = finish_dither(manifest)
        if interruption['requested']:
            raise InterruptedError('Dither collection interrupted')
        (folder / 'result.json').write_text(json.dumps(result, indent=2))
        if not result['usable']:
            raise RuntimeError('Dither measurements produced no usable full-axis fixed-wave result: '+str(result['failures']))
        # Newly measured dither belongs to this calibration's device identity.
        return apply_result(identification, result)
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        # A late signal during selection/export must reach the parent collector,
        # whose handler was temporarily replaced while these runtimes stopped.
        if interruption['requested']:
            raise InterruptedError('Dither collection interrupted')
