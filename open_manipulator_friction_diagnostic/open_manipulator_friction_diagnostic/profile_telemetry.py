"""Low-rate read-only profile telemetry through the existing arm DHI service.

Five single-item requests/s, at most one outstanding. No serial port or writes.
Request/receipt intervals are retained; these are not simultaneous samples or
millisecond sensor-delay measurements.
"""
import json
import time
import threading

from .history_motion import SERVICE

DIAGNOSTIC_ITEMS=('Goal Position','Position Trajectory','Velocity Trajectory','Moving Status')
# Same named registers/widths in the preserved YM070-A099 and YM080-A099 tables.
# Keep the register value; these units are not a new sensor/output calibration.
RAW_ITEMS={
    'Goal Position':(532,4,'position_count'),
    'Position Trajectory':(560,4,'position_count'),
    'Velocity Trajectory':(564,4,'velocity_count_0p01_rpm'),
    'Present Velocity':(548,4,'velocity_count_0p01_rpm'),
    'Moving Status':(541,1,'bitmask'),
}


def collection_options(config,meta):
    """Opt-in observations for reviewed position collection; legacy history unchanged."""
    if 'position_profile_telemetry_joint' in config:
        joint=config['position_profile_telemetry_joint']
        if type(joint) is not int or joint not in range(1,7) or meta.get('history_study') or not meta.get('generalization_study'):
            raise ValueError('Profile telemetry requires one joint and a reviewed ordinary position study')
        return dict(joint_id=joint,items=DIAGNOSTIC_ITEMS,purpose='position_servo_state_diagnostic')
    if meta.get('history_study'):
        return dict(items=meta['telemetry']['items'])
    return None


def signed32(value):
    if not 0 <= value < 2**32:
        raise ValueError('Expected uint32 register response')
    return value-2**32 if value >= 2**31 else value


class ProfileTelemetry:
    def __init__(self, node, io_node=None, items=None, joint_id=2, purpose=None):
        if type(joint_id) is not int or joint_id not in range(1,7) or purpose not in (None,'position_servo_state_diagnostic'):
            raise ValueError('Invalid profile telemetry target/purpose')
        from dynamixel_interfaces.srv import GetDataFromDxl
        self.executor=None;self.thread=None
        if io_node is None:
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            io_node=Node('friction_profile_telemetry')
            self.executor=SingleThreadedExecutor();self.executor.add_node(io_node)
        self.io_node=io_node
        self.node=node; self.service_type=GetDataFromDxl
        self.client=io_node.create_client(GetDataFromDxl,SERVICE)
        self.items=tuple(items or ('Goal Position','Position Trajectory','Velocity Trajectory'))
        if (len(set(self.items))!=len(self.items) or
                not set(self.items)<=set(RAW_ITEMS) or
                ('Moving Status' in self.items and purpose is None)):
            raise ValueError('Unsupported read-only telemetry item')
        self.joint_id=joint_id;self.purpose=purpose
        self.pending=None; self.index=0; self.seen=set(); self.failures=0
        self.file=(node.folder/'profile_telemetry.jsonl').open('x')
        self.timer=io_node.create_timer(.2,self.tick)
        self.closed=False;self.paused=False
        if self.executor is not None:
            self.thread=threading.Thread(target=self.executor.spin,daemon=True)
            self.thread.start()

    def tick(self):
        if (self.closed or self.paused or self.node.failure or self.node.latest is None or
                not self.node.client.server_is_ready() or not self.client.service_is_ready()):return
        if self.pending is not None:
            if time.monotonic()-self.sent > 1.:
                self.node.failure='Profile telemetry response timeout'
            return
        item=self.items[self.index%len(self.items)];self.index+=1
        req=self.service_type.Request();req.id=self.joint_id;req.item_name=item;req.timeout_sec=.1
        stamp=self.node.get_clock().now().nanoseconds*1e-9
        epoch=self.node.epoch;record=self.node.record
        state_stamp=getattr(self.node,'last_stamp',None)
        reported_velocity=float(self.node.latest[1][self.joint_id-1]) if len(self.node.latest[1])==6 else None
        req.header.stamp=self.node.get_clock().now().to_msg()
        self.sent=time.monotonic()
        future=self.client.call_async(req);self.pending=future
        def done(f):
            receipt=self.node.get_clock().now().nanoseconds*1e-9
            try:
                response=f.result();ok=bool(response.result);raw=int(response.item_data)
                if item=='Moving Status' and not 0<=raw<=255:raise ValueError('Moving Status is an unsigned byte')
                row=dict(item=item,id=self.joint_id,request_ros_s=stamp,receipt_ros_s=receipt,
                    latency_s=time.monotonic()-self.sent,result=ok,raw_uint32=raw,
                    signed_value=signed32(raw),collection_active=record,
                    joint_state_stamp_ros_s_at_request=state_stamp,
                    joint_state_velocity_rad_s_at_request=reported_velocity,
                    trajectory_time_request_s=None if epoch is None else stamp-epoch,
                    trajectory_time_receipt_s=None if epoch is None else receipt-epoch)
                if self.purpose is not None:
                    address,width,unit=RAW_ITEMS[item]
                    row.update(purpose=self.purpose,requested_target_joint=self.joint_id,
                        control_table_address=address,width_bytes=width,raw_unit=unit,
                        value_encoding='uint8_bitfield' if item=='Moving Status' else 'int32',
                        simultaneous_sampling=False,request_rate_hz=5.)
                if ok:self.seen.add(item);self.failures=0
                else:self.failures+=1
                if not self.closed:
                    self.file.write(json.dumps(row)+'\n');self.file.flush()
                if self.failures>=3:self.node.failure='Three failed profile telemetry reads'
            except Exception as exc:
                self.node.failure='Profile telemetry failure: '+str(exc)
            finally:self.pending=None
        future.add_done_callback(done)

    def ready(self):
        return self.seen==set(self.items)

    def pause(self):
        """Stop new requests; the caller must also drain the outstanding one."""
        self.paused=True

    def resume(self):
        self.paused=False

    def close(self):
        self.closed=True;self.timer.cancel()
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.)
            self.thread.join(timeout=2.)
            self.io_node.destroy_node()
        self.file.close()
