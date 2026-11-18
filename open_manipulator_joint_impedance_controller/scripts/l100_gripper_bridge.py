#!/usr/bin/env python3
"""L100's already-mapped trigger -> independent follower gripper position controller."""
import math
import time
from l100_impedance_leader import ordered_positions, Synchronizer


def gripper_position(names, values):
    value = ordered_positions(names, values, ['rh_r1_joint'])
    # This topic already reverses the leader trigger and adds +0.2. Do not repeat it.
    return None if value is None else min(1.13514578304, max(0., value[0]))


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from control_msgs.msg import DynamicJointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from std_srvs.srv import SetBool
    from dynamixel_interfaces.srv import GetDataFromDxl

    class Bridge(Node):
        def __init__(self):
            super().__init__('l100_gripper_bridge')
            defaults = dict(leader_topic='/leader/joint_trajectory',
                follower_state_topic='/friction_compensation_controller/impedance_state',
                joint_states_topic='/joint_states',
                gripper_topic='/leader_gripper_controller/joint_trajectory',
                torque_service='/omyf3mendunitsystem/set_dxl_torque',
                read_service='/omyf3mendunitsystem/get_dxl_data',
                use_mock_hardware=False, state_timeout_s=.3, sync_duration_s=1.)
            for name,value in defaults.items(): self.declare_parameter(name,value)
            p=lambda key:self.get_parameter(key).value
            self.mock=p('use_mock_hardware');self.timeout=p('state_timeout_s')
            self.sync=Synchronizer(p('sync_duration_s'),.5,self.timeout)
            self.leader=self.actual=None;self.enabled=False;self.armed=False
            self.lt=self.at=self.ft=float('-inf');self.stage='waiting';self.future=None
            self.retry_at=0.;self.primed_at=None
            self.pub=self.create_publisher(JointTrajectory,p('gripper_topic'),1)
            self.enable=self.create_client(SetBool,p('torque_service'))
            self.read=self.create_client(GetDataFromDxl,p('read_service'))
            self.create_subscription(JointTrajectory,p('leader_topic'),self.on_leader,qos_profile_sensor_data)
            self.create_subscription(JointState,p('joint_states_topic'),self.on_actual,qos_profile_sensor_data)
            self.create_subscription(DynamicJointState,p('follower_state_topic'),self.on_follower,qos_profile_sensor_data)
            self.create_timer(.02,self.tick)
            self.get_logger().info('Waiting for follower readiness, gripper controller and fresh L100 trigger.')

        def on_leader(self,msg):
            self.leader=gripper_position(msg.joint_names,msg.points[0].positions) if len(msg.points)==1 else None
            self.lt=time.monotonic() if self.leader is not None else float('-inf')

        def on_actual(self,msg):
            value=ordered_positions(msg.name,msg.position,['rh_r1_joint'])
            if value is not None:self.actual=value[0];self.at=time.monotonic()

        def on_follower(self,msg):
            self.enabled=False
            if len(msg.joint_names)!=len(msg.interface_values):return
            states=dict(zip(msg.joint_names,msg.interface_values))
            try:
                self.enabled=all(dict(zip(states[f'joint{i}'].interface_names,states[f'joint{i}'].values))['enabled']==1.
                                 for i in range(1,7))
                self.ft=time.monotonic()
            except KeyError:pass

        def publish(self,value):
            msg=JointTrajectory();msg.joint_names=['rh_r1_joint']
            # Match the stock L100 broadcaster: apply each streamed position now.
            # A 100 ms horizon replaced every 20 ms creates an unintended lag.
            # Startup/reconnect smoothing is already handled by Synchronizer.
            point=JointTrajectoryPoint();point.positions=[value]
            msg.points=[point];self.pub.publish(msg)

        def fail(self,message):
            self.get_logger().error(message)
            self.armed=False;self.stage='waiting';self.future=None
            self.retry_at=time.monotonic()+2.;self.primed_at=None;self.sync.reset()

        def tick(self):
            now=time.monotonic()
            ready=(self.enabled and now-self.ft<self.timeout and now-self.lt<self.timeout
                   and now-self.at<self.timeout and self.pub.get_subscription_count()>0)
            if not ready:
                if self.sync.phase!='WAITING' and self.actual is not None and now-self.at<self.timeout:
                    self.publish(self.actual)
                self.sync.reset();self.primed_at=None
                return
            if not self.armed:
                self.publish(self.actual) # Activate the position controller at measured position first.
                if now<self.retry_at:return
                if self.primed_at is None:self.primed_at=now;return
                if now-self.primed_at<.15:return
                if self.mock:self.armed=True
                elif self.stage=='waiting':
                    if not self.enable.service_is_ready() or not self.read.service_is_ready():return
                    request=SetBool.Request();request.data=True
                    self.future=self.enable.call_async(request);self.stage='enabling';self.request_at=now
                    return
                elif not self.future.done():
                    if now-self.request_at>2.:self.fail('Gripper torque/readback service timed out')
                    return
                elif self.stage=='enabling':
                    response=self.future.result()
                    if not response or not response.success:self.fail('Gripper torque enable failed');return
                    request=GetDataFromDxl.Request();request.id=7;request.item_name='Torque Enable';request.timeout_sec=1.
                    self.future=self.read.call_async(request);self.stage='verifying';self.request_at=now
                    return
                else:
                    response=self.future.result()
                    if not response or not response.result or response.item_data!=1:
                        self.fail('Gripper torque ON readback not confirmed');return
                    self.armed=True
                self.get_logger().info('Gripper ready; trigger synchronization starts.')
            self.publish(self.sync.step(now,[self.leader],[self.actual])[0])

    rclpy.init();node=Bridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        # Follower position controller retains its last goal. Full follower shutdown disables hardware.
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
