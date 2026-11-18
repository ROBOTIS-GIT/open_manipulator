#!/usr/bin/env python3
"""Mapped L100 joint angles -> follower FK -> Cartesian reference; no follower motor calls."""
import math
import time
from xml.etree.ElementTree import ParseError
import numpy as np
from cartesian_leader_math import FollowerFK, PoseSynchronizer, unit_quaternion


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from trajectory_msgs.msg import JointTrajectory
    from control_msgs.msg import DynamicJointState
    from std_msgs.msg import String

    class Bridge(Node):
        def __init__(self):
            super().__init__('l100_cartesian_leader')
            defaults=dict(leader_topic='/leader/joint_trajectory',
                follower_state_topic='/friction_compensation_controller/task_state',
                follower_reference_topic='/friction_compensation_controller/target_pose_state',
                reference_topic='/friction_compensation_controller/leader_target_pose',
                robot_description_topic='/robot_description',root_link='link0',tip_link='end_effector_link',
                joint_names=[f'joint{i}' for i in range(1,7)],publish_rate_hz=100.,
                sync_duration_s=5.,sync_linear_speed_m_s=.04,sync_angular_speed_rad_s=.2,state_timeout_s=.3)
            for k,v in defaults.items():self.declare_parameter(k,v)
            p=lambda k:self.get_parameter(k).value
            self.names=p('joint_names');self.root=p('root_link');self.tip=p('tip_link')
            if len(self.names)!=6 or len(set(self.names))!=6:raise ValueError('Require six unique arm joints')
            rate=p('publish_rate_hz')
            if not math.isfinite(rate) or not 0<rate<=400:raise ValueError('Invalid publication rate')
            self.timeout=p('state_timeout_s')
            self.sync=PoseSynchronizer(p('sync_duration_s'),p('sync_linear_speed_m_s'),p('sync_angular_speed_rad_s'),self.timeout)
            self.fk=None;self.xml=None;self.leader=None;self.reference=None;self.sequence=None;self.epoch=None;self.enabled=False
            self.lt=self.ft=self.pt=float('-inf');self.previous=None
            self.reference_stamp=None
            self.pub=self.create_publisher(PoseStamped,p('reference_topic'),1)
            self.status=self.create_publisher(String,'~/status',1)
            self.create_subscription(String,p('robot_description_topic'),self.on_model,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(JointTrajectory,p('leader_topic'),self.on_leader,qos_profile_sensor_data)
            self.create_subscription(DynamicJointState,p('follower_state_topic'),self.on_state,qos_profile_sensor_data)
            self.create_subscription(PoseStamped,p('follower_reference_topic'),self.on_pose,qos_profile_sensor_data)
            self.create_timer(1./rate,self.tick)
            self.get_logger().info('Waiting for follower URDF/readiness and L100. Keep leader still during time-based slow start.')

        def on_model(self,msg):
            if msg.data==self.xml:return
            self.sync.reset();self.fk=None
            try:self.fk=FollowerFK(msg.data,self.root,self.tip,self.names);self.xml=msg.data
            except (ValueError,KeyError,AttributeError,ParseError) as e:self.get_logger().error(f'Follower FK rejected: {e}')

        def on_leader(self,msg):
            self.lt=float('-inf')
            if len(msg.points)!=1 or len(msg.joint_names)!=len(msg.points[0].positions) or len(set(msg.joint_names))!=len(msg.joint_names):return
            values=dict(zip(msg.joint_names,msg.points[0].positions))
            if any(n not in values for n in self.names):return
            q=[values[n] for n in self.names]
            if not np.isfinite(q).all():return
            self.leader=q;self.lt=time.monotonic()

        def on_pose(self,msg):
            self.pt=float('-inf')
            if msg.header.frame_id!=self.root:return
            p=msg.pose.position;q=msg.pose.orientation
            try:ref=(np.array([p.x,p.y,p.z]),unit_quaternion([q.x,q.y,q.z,q.w]))
            except ValueError:return
            if not np.isfinite(ref[0]).all():return
            stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
            if stamp!=self.reference_stamp:self.sync.reset()
            self.reference_stamp=stamp
            self.reference=ref;self.pt=time.monotonic()

        def on_state(self,msg):
            self.enabled=False
            try:
                if len(msg.joint_names)!=len(msg.interface_values) or len(set(msg.joint_names))!=len(msg.joint_names):raise ValueError()
                states={n:dict(zip(s.interface_names,s.values)) for n,s in zip(msg.joint_names,msg.interface_values) if len(s.interface_names)==len(s.values)}
                states=[states[f'joint{i}'] for i in range(1,7)]
                sequence=states[0]['update_sequence']
                epoch=states[0]['reference_epoch']
                if not math.isfinite(sequence) or not math.isfinite(epoch):raise ValueError()
                if self.epoch is not None and epoch!=self.epoch:
                    self.sync.reset();self.pt=float('-inf')
                self.epoch=epoch
                if self.sequence is not None and sequence<=self.sequence:
                    self.sync.reset()
                    if sequence==self.sequence:return
                    self.pt=float('-inf') # Wait for this controller incarnation's reference.
                self.sequence=sequence
                self.enabled=all(s['enabled']==1. for s in states)
                if not self.enabled:self.sync.reset()
                self.ft=time.monotonic()
            except (ValueError,KeyError):self.sync.reset()

        def tick(self):
            now=time.monotonic()
            ready=(self.fk is not None and self.enabled and now-self.lt<self.timeout and
                now-self.ft<self.timeout and now-self.pt<self.timeout and self.pub.get_subscription_count()>0)
            if not ready:self.sync.reset()
            else:
                leader=self.fk.evaluate(self.leader)
                pos,q=self.sync.step(now,leader,self.reference)
                msg=PoseStamped();msg.header.stamp.sec,msg.header.stamp.nanosec=self.reference_stamp
                msg.header.frame_id=self.root
                msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,pos)
                msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=map(float,q)
                self.pub.publish(msg)
            if self.previous!=self.sync.phase:
                self.get_logger().info(self.sync.phase+': '+{'WAITING':'holding last target; waiting for fresh enabled follower/leader/model',
                    'SYNCING':'Cartesian slow start (time based; no arrival test)',
                    'FOLLOWING':'live follower-geometry FK targets'}[self.sync.phase]);self.previous=self.sync.phase
            self.status.publish(String(data=self.sync.phase))

    rclpy.init(args=args);node=None
    try:node=Bridge();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
