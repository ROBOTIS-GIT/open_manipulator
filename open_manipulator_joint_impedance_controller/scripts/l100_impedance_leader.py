#!/usr/bin/env python3
"""L100 position reference bridge; never starts or switches follower controllers."""
import math
import time


def ordered_positions(names, values, required):
    if len(names) != len(values) or len(set(names)) != len(names):
        return None
    mapping = dict(zip(names, values))
    if any(name not in mapping for name in required):
        return None
    result = [mapping[name] for name in required]
    return result if all(math.isfinite(x) for x in result) else None


def waiting_reason(enabled, leader_age, follower_age, subscribers, timeout, states):
    """Explain loss of readiness without changing the reference/enable policy."""
    if follower_age >= timeout:
        return 'FOLLOWER_STALE: no fresh follower telemetry'
    faults = [s for s in states if s.get('mode') == 2.]
    if faults:
        s = faults[0]
        return ('QP_FALLBACK: follower holds its fault pose; '
                f"failure_reason={s.get('failure_reason')} qp_status={s.get('qp_status')} "
                f"qp_iterations={s.get('qp_iterations')}; explicit reset required")
    if not enabled:
        return 'FOLLOWER_NOT_READY: follower reports enabled=0'
    if leader_age >= timeout:
        return 'LEADER_STALE: no fresh L100 trajectory'
    if subscribers == 0:
        return 'NO_REFERENCE_SUBSCRIBER: follower reference endpoint unavailable'
    return ''


class Synchronizer:
    """Quintic startup reference, then live leader targets; timing uses monotonic time."""
    def __init__(self, duration, speed, timeout):
        if any(not math.isfinite(x) or x <= 0 for x in (duration, speed, timeout)):
            raise ValueError('Sync duration, reference speed and timeout must be positive')
        self.minimum_duration, self.speed = duration, speed
        self.timeout = timeout
        self.reset()

    def reset(self):
        self.phase = 'WAITING'
        self.last = None

    def _start(self, now, leader, reference):
        self.start = list(reference)
        # Peak derivative of quintic smoothstep is 1.875 for a stationary leader.
        delta = max(abs(a-b) for a, b in zip(leader, self.start))
        self.duration = max(self.minimum_duration, 1.875 * delta / self.speed)
        self.started = now
        self.phase = 'SYNCING'

    def step(self, now, leader, reference):
        if self.last is not None and now-self.last > self.timeout:
            self.reset()
        if self.phase == 'WAITING':
            self._start(now, leader, reference)
        self.last = now
        if self.phase == 'FOLLOWING':
            return list(leader)
        t = min(1., max(0., (now-self.started)/self.duration))
        blend = t*t*t*(10.+t*(-15.+6.*t))
        # Blend toward the latest leader pose. At t=1 this is exactly the live
        # target, so transitioning to tracking introduces no target discontinuity.
        result = [a+blend*(b-a) for a, b in zip(self.start, leader)]
        if t == 1.:
            self.phase = 'FOLLOWING'
        return result


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile
    from control_msgs.msg import DynamicJointState
    from std_msgs.msg import Float64MultiArray, String
    from trajectory_msgs.msg import JointTrajectory

    class Bridge(Node):
        def __init__(self):
            super().__init__('l100_impedance_leader')
            defaults = {
                'leader_topic': '/leader/joint_trajectory',
                'follower_state_topic': '/friction_compensation_controller/impedance_state',
                'reference_topic': '/friction_compensation_controller/reference',
                'joint_names': [f'joint{i}' for i in range(1,7)],
                'publish_rate_hz': 100., 'sync_duration_s': 5.,
                'sync_reference_speed_rad_s': .2,
                'state_timeout_s': .3,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            param = lambda name: self.get_parameter(name).value
            self.names = param('joint_names')
            if len(self.names) != 6 or len(set(self.names)) != 6:
                raise ValueError('joint_names must contain six unique leader arm joints in follower order')
            rate = param('publish_rate_hz')
            if not math.isfinite(rate) or rate <= 0 or rate > 400:
                raise ValueError('publish_rate_hz must be in (0, 400]')
            self.timeout = param('state_timeout_s')
            self.sync = Synchronizer(param('sync_duration_s'), param('sync_reference_speed_rad_s'),
                                     self.timeout)
            self.leader = self.reference = None
            self.leader_time = self.follower_time = float('-inf')
            self.sequence = None
            self.enabled = False
            self.follower_states = []
            self.previous_phase = None
            self.previous_wait_reason = None
            self.publisher = self.create_publisher(Float64MultiArray, param('reference_topic'), QoSProfile(depth=1))
            self.status = self.create_publisher(String, '~/status', 1)
            self.create_subscription(JointTrajectory, param('leader_topic'), self.on_leader, qos_profile_sensor_data)
            self.create_subscription(DynamicJointState, param('follower_state_topic'), self.on_follower, qos_profile_sensor_data)
            self.create_timer(1./rate, self.tick)
            self.get_logger().info('Waiting for fresh leader commands and enabled follower impedance telemetry. '
                                   'Keep the leader still during SYNCING; gripper uses the separate gripper bridge.')

        def on_leader(self, msg):
            values = (ordered_positions(msg.joint_names, msg.points[0].positions, self.names)
                      if len(msg.points) == 1 else None)
            if values is None:
                self.leader_time = float('-inf')
                return
            self.leader, self.leader_time = values, time.monotonic()

        def on_follower(self, msg):
            # Synchronization starts from the current target, not the tracking error.
            try:
                if len(msg.joint_names) != len(msg.interface_values) or len(set(msg.joint_names)) != len(msg.joint_names):
                    raise ValueError('malformed names')
                joints = {}
                for name, state in zip(msg.joint_names, msg.interface_values):
                    if len(state.interface_names) != len(state.values):
                        raise ValueError('malformed interfaces')
                    joints[name] = dict(zip(state.interface_names, state.values))
                states = [joints[f'joint{i}'] for i in range(1,7)]
                self.follower_states = states
                reference = [s['reference_position'] for s in states]
                sequence = states[0]['update_sequence']
                if not all(math.isfinite(x) for x in reference+[sequence]):
                    raise ValueError('nonfinite state')
                if self.sequence is not None and sequence <= self.sequence:
                    self.sync.reset()
                    if sequence == self.sequence:
                        return
                self.sequence = sequence
                self.enabled = all(s['enabled'] == 1. for s in states)
                if not self.enabled:
                    self.sync.reset()
                self.reference = reference
                self.follower_time = time.monotonic()
            except (KeyError, ValueError):
                self.enabled = False
                self.sync.reset()

        def tick(self):
            now = time.monotonic()
            ready = (self.enabled and now-self.leader_time < self.timeout
                     and now-self.follower_time < self.timeout
                     and self.publisher.get_subscription_count() > 0)
            if not ready:
                self.sync.reset()
            else:
                target = self.sync.step(now, self.leader, self.reference)
                self.publisher.publish(Float64MultiArray(data=target))
            if self.sync.phase != self.previous_phase:
                detail = {'WAITING': 'No new targets; follower holds its last reference.',
                          'SYNCING': 'Slow-start reference interpolation active; completion is time-based.',
                          'FOLLOWING': 'Synchronization complete; live leader tracking active.'}[self.sync.phase]
                self.get_logger().info(f'{self.sync.phase}: {detail}')
                self.previous_phase = self.sync.phase
            reason = (waiting_reason(self.enabled, now-self.leader_time,
                      now-self.follower_time, self.publisher.get_subscription_count(),
                      self.timeout, self.follower_states)
                      if self.sync.phase == 'WAITING' else '')
            if reason and reason != self.previous_wait_reason:
                self.get_logger().warning(reason)
            self.previous_wait_reason = reason
            self.status.publish(String(data=self.sync.phase))

    rclpy.init(args=args)
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
