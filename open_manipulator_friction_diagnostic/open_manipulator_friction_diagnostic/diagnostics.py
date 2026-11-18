"""Read-only controller/effort recording. Never starts, stops or enables a robot."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid

from .protocol import JOINTS, config, sha

FIELDS = ('position', 'velocity', 'model_magnitude_nm', 'requested_nm', 'target_nm',
          'assistance_nm', 'baseline_nm', 'total_command_nm', 'flags', 'period_s',
          'window_cycles', 'window_assist_cycles', 'window_peak_nm',
          'window_mean_abs_nm', 'window_flags')
FLAGS = {1: 'disabled', 2: 'fault', 4: 'load_outside_range', 8: 'rest',
         16: 'overspeed', 32: 'low_speed_fade', 64: 'high_speed_fade',
         128: 'torque_cap', 256: 'slew', 512: 'reversal',
         1024: 'outside_identification_range', 2048: 'acceleration_trip',
         4096: 'lugre_damping_bound'}


def expected_current_counts(total, baseline, unit):
    """Installed DHI conversion estimate, not a Goal Current register readback."""
    if not all(math.isfinite(x) for x in (total, baseline, unit)) or unit <= 0:
        raise ValueError('Invalid current conversion inputs')
    total_count = math.trunc(total/unit)
    return total_count, total_count-math.trunc(baseline/unit)


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from control_msgs.msg import DynamicJointState
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import GetParameters

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--output', default='~/.ros/omy_calibration/diagnostics')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be positive and finite')
    c = config(args.config, record_only=True)
    units = c.get('expected_current_units_nm_per_count', {})
    scales = [units.get('ym080_230_a099')]*2 + [units.get('ym070_210_a099')]*4
    if c['variant'] != 'omy_f3m' or any(x is None or not math.isfinite(x) or x <= 0 for x in scales):
        parser.error('Config must specify OMY F3M installed current conversion units')
    root = Path(args.output).expanduser()/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
    root.mkdir(parents=True, exist_ok=False)
    report = dict(schema=1, robot_unit=c['robot_unit'], config_sha256=sha(args.config),
                  controls_robot=False, goal_current_register_readback=False,
                  expected_nm_per_count=scales, flags=FLAGS, diagnostic_messages=0,
                  joint_state_messages=0, joint_state_max_rate_hz=100,
                  hardware_performance_validated=False)
    rclpy.init()
    node = rclpy.create_node('friction_diagnostic_recorder')
    csv_file = (root/'commands.csv').open('w', newline='')
    states_file = (root/'joint_states.csv').open('w', newline='')
    command_writer, state_writer = csv.writer(csv_file), csv.writer(states_file)
    command_writer.writerow(['stamp_s', 'joint', *FIELDS,
                             'expected_goal_current_count', 'expected_extra_current_count'])
    state_writer.writerow(['stamp_s', 'joint', 'position', 'velocity', 'measured_effort_nm'])
    last_state_stamp = None
    last_diagnostic_stamp = None
    max_gap = 0.
    summary = {j: dict(cycles=0, assist_cycles=0, sum_abs=0., peak_abs=0., window_flags=0) for j in JOINTS}

    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def diagnostic(msg):
        nonlocal last_diagnostic_stamp, max_gap
        now = stamp(msg)
        if last_diagnostic_stamp is not None:
            max_gap = max(max_gap, now-last_diagnostic_stamp)
        last_diagnostic_stamp = now
        if len(msg.joint_names) != len(msg.interface_values):
            raise ValueError('Malformed diagnostic message')
        for joint, entry in zip(msg.joint_names, msg.interface_values):
            if joint not in JOINTS:
                continue
            values = dict(zip(entry.interface_names, entry.values))
            raw, delta = expected_current_counts(values['total_command_nm'], values['baseline_nm'],
                                                 scales[JOINTS.index(joint)])
            command_writer.writerow([now, joint, *[values[k] for k in FIELDS], raw, delta])
            s = summary[joint]
            cycles = int(values['window_cycles'])
            s['cycles'] += cycles
            s['assist_cycles'] += int(values['window_assist_cycles'])
            s['sum_abs'] += values['window_mean_abs_nm']*cycles
            s['peak_abs'] = max(s['peak_abs'], values['window_peak_nm'])
            s['window_flags'] |= int(values['window_flags'])
        report['diagnostic_messages'] += 1

    def state(msg):
        nonlocal last_state_stamp
        now = stamp(msg)
        if last_state_stamp is not None and now-last_state_stamp < .01:
            return
        if any(len(x) != len(msg.name) for x in (msg.position, msg.velocity, msg.effort)):
            return
        last_state_stamp = now
        for i, joint in enumerate(msg.name):
            if joint in JOINTS:
                state_writer.writerow([now, joint, msg.position[i], msg.velocity[i], msg.effort[i]])
        report['joint_state_messages'] += 1

    def request(kind, name, message):
        client = node.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=2.):
            return None
        future = client.call_async(message)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
        return future.result() if future.done() else None

    def read_status():
        result = request(Trigger, '/gravity_compensation_controller/get_friction_status', Trigger.Request())
        return json.loads(result.message) if result and result.success else None

    node.create_subscription(DynamicJointState, '/gravity_compensation_controller/friction_diagnostics',
                             diagnostic, qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', state, qos_profile_sensor_data)
    interrupted = False
    try:
        report['status_start'] = read_status()
        names = ['calibrated_gain', 'calibrated_max_torque', 'calibrated_slew_rate',
                 'calibrated_deadband', 'calibrated_min_velocity', 'calibrated_max_velocity',
                 'friction_diagnostics_rate']
        result = request(GetParameters, '/gravity_compensation_controller/get_parameters',
                         GetParameters.Request(names=names))
        report['parameters'] = {name: (list(v.double_array_value) if v.type == 8 else v.double_value)
                                for name, v in zip(names, result.values)} if result else None
        (root/'metadata.json').write_text(json.dumps(report, indent=2))
        print(f'Recording only: {root}', flush=True)
        end = time.monotonic()+args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        report['status_end'] = read_status()
    except KeyboardInterrupt:
        interrupted = True
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        report['interrupted'] = interrupted
        report['recording_completed'] = not interrupted and 'error' not in report and report['diagnostic_messages'] > 0
        report['max_diagnostic_stamp_gap_s'] = max_gap
        report['joints'] = summary
        (root/'summary.json').write_text(json.dumps(report, indent=2))
        csv_file.close(); states_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(json.dumps(dict(output=str(root), recording_completed=report['recording_completed'],
                              controls_robot=False)), flush=True)
    if not report['recording_completed'] and not interrupted:
        raise SystemExit(1)
