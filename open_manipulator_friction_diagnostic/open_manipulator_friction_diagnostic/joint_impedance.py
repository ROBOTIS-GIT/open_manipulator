"""Temporary, separate joint-space spring/damper input for the compensated arm."""
import json
import math
import time

JOINTS = [f'joint{j}' for j in range(1, 7)]


def impedance_effort(reference, position, velocity, stiffness, damping):
    vectors = (reference, position, velocity, stiffness, damping)
    if any(len(values) != 6 or not all(math.isfinite(x) for x in values) for values in vectors):
        raise ValueError('Impedance requires six finite values per vector')
    if any(x < 0 for values in (stiffness, damping) for x in values):
        raise ValueError('Stiffness and damping must be nonnegative')
    return [kp*(target-q)-kd*v for target, q, v, kp, kd in zip(*vectors)]


def make_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String
    from .position_runtime import RuntimeHeartbeat

    class JointImpedanceNode(Node):
        def __init__(self):
            super().__init__('joint_impedance')
            self.stiffness = self.declare_parameter('stiffness', [5., 5., 4., 2., 1., .8]).value
            self.damping = self.declare_parameter('damping', [.6, .8, .6, .25, .15, .12]).value
            rate = self.declare_parameter('publish_rate_hz', 400.).value
            self.state_timeout = self.declare_parameter('state_timeout_s', .1).value
            impedance_effort([0.]*6, [0.]*6, [0.]*6, self.stiffness, self.damping)
            if any(not math.isfinite(x) or x <= 0 for x in (rate, self.state_timeout)):
                raise ValueError('publish_rate_hz and state_timeout_s must be positive and finite')
            self.reference = None
            self.sample = None
            self.heartbeat = RuntimeHeartbeat()
            self.ready_since = None
            self.halted = False
            latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.publisher = self.create_publisher(Float64MultiArray,
                '/friction_compensation_controller/commands', 1)
            self.create_subscription(JointState, '/joint_states', self.receive_state, latest)
            self.create_subscription(String, '/friction_runtime/heartbeat', self.receive_runtime, 1)
            self.timer = self.create_timer(1./rate, self.update)
            self.get_logger().info('Waiting for ready compensation runtime and fresh joint states; '
                                   'the first ready pose becomes the fixed reference.')

        def receive_state(self, message):
            try:
                if len(message.position) != len(message.name) or len(message.velocity) != len(message.name):
                    raise ValueError('Joint position and velocity are required')
                indices = [message.name.index(name) for name in JOINTS]
                q = [message.position[i] for i in indices]
                v = [message.velocity[i] for i in indices]
                if not all(math.isfinite(x) for x in q+v):
                    raise ValueError('Nonfinite joint state')
                stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
                age = self.get_clock().now().nanoseconds*1e-9-stamp
                if not math.isfinite(age) or not -.02 <= age <= self.state_timeout:
                    raise ValueError('Joint state timestamp is stale or in the future')
                self.sample = (q, v, time.monotonic())
            except (ValueError, IndexError) as exc:
                self.sample = None
                if self.reference is not None:
                    self.halt(str(exc))

        def receive_runtime(self, message):
            try:
                now = time.monotonic()
                self.heartbeat.observe(json.loads(message.data), now)
                if self.ready_since is None and self.runtime_ready(now):
                    self.ready_since = now
            except (ValueError, KeyError, TypeError) as exc:
                if self.reference is not None:
                    self.halt(f'Invalid runtime heartbeat: {exc}')

        def runtime_ready(self, now):
            state = self.heartbeat.state
            return (state is not None and 0 <= now-state['generated_at'] <= .5 and
                    state.get('phase') == 'running' and state.get('ready') and
                    not state.get('failure') and state.get('controller_manager_exit_code') is None)

        def halt(self, reason):
            if not self.halted:
                self.halted = True
                self.publisher.publish(Float64MultiArray(data=[0.]*6))
                self.get_logger().error(f'Impedance input stopped: {reason}. Restart this node to capture a new pose.')

        def update(self):
            if self.halted:
                return
            now = time.monotonic()
            if not self.runtime_ready(now):
                if self.reference is not None:
                    self.halt('Compensation runtime is not ready')
                else:
                    self.ready_since = None
                return
            if self.sample is None or now-self.sample[2] > self.state_timeout:
                if self.reference is not None:
                    self.halt('Joint state reception stopped')
                return
            q, v, received = self.sample
            if self.reference is None:
                if self.ready_since is None or received < self.ready_since:
                    return
                self.reference = list(q)
                self.get_logger().info(f'Impedance ON: reference_rad={self.reference}, '
                                       f'stiffness_Nm_per_rad={self.stiffness}, damping_Nms_per_rad={self.damping}')
            effort = impedance_effort(self.reference, q, v, self.stiffness, self.damping)
            self.publisher.publish(Float64MultiArray(data=effort))

        def stop(self):
            self.timer.cancel()
            if rclpy.ok():
                self.publisher.publish(Float64MultiArray(data=[0.]*6))

    return JointImpedanceNode()


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = None
    try:
        node = make_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
