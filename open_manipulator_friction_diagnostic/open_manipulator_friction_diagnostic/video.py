"""Record actual Gazebo camera frames on simulation time, directly to H.264."""
import argparse
import json
import math
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time


class FrameTimeline:
    """Map camera timestamps to video slots without deleting unobserved time.

    Missing slots hold the last observed image; they are counted and disclosed.
    No interpolated camera view or commanded robot pose is synthesized.
    """
    def __init__(self, fps, start_s):
        if not math.isfinite(fps) or not 1 <= fps <= 30 or not math.isfinite(start_s):
            raise ValueError('Invalid camera frame rate or start timestamp')
        self.fps, self.start_s = float(fps), float(start_s)
        self.source_frames = self.encoded_frames = self.held_frames = 0
        self.duplicate_stamps = 0
        self.first_stamp = self.last_stamp = None
        self.maximum_gap = 0.
        self.last_index = -1

    def seed(self, stamp_s):
        """Start from the latest real image while retaining its capture time."""
        if self.source_frames or not math.isfinite(stamp_s) or stamp_s > self.start_s:
            raise ValueError('Invalid initial camera frame')
        self.first_stamp = self.last_stamp = stamp_s
        self.source_frames = self.encoded_frames = 1
        self.last_index = 0
        return dict(index=0, elapsed_s=0., hold_frames=0)

    def observe(self, stamp_s):
        if not math.isfinite(stamp_s):
            raise ValueError('Nonfinite camera timestamp')
        if self.last_stamp is not None:
            if stamp_s < self.last_stamp:
                raise ValueError('Camera simulation clock moved backward')
            if stamp_s == self.last_stamp:
                self.duplicate_stamps += 1
                return None
        elapsed = stamp_s-self.start_s
        if elapsed < -1e-8:
            return None
        if self.last_stamp is not None:
            self.maximum_gap = max(self.maximum_gap, stamp_s-self.last_stamp)
        if self.first_stamp is None:
            self.first_stamp = stamp_s
        self.last_stamp = stamp_s
        self.source_frames += 1
        index = int(math.floor(max(0., elapsed)*self.fps + 1e-7))
        if index <= self.last_index:
            return None
        held = max(0, index-self.encoded_frames)
        self.held_frames += held
        self.encoded_frames = index+1
        self.last_index = index
        return dict(index=index, elapsed_s=max(0., elapsed), hold_frames=held)

    def finish(self, end_s):
        if not math.isfinite(end_s) or end_s < self.start_s:
            raise ValueError('Invalid recording end timestamp')
        count = int(math.ceil((end_s-self.start_s)*self.fps-1e-7))
        held = max(0, count-self.encoded_frames)
        self.held_frames += held
        self.encoded_frames += held
        return held

    def report(self, end_s):
        leading = None if self.first_stamp is None else max(0., self.first_stamp-self.start_s)
        initial_age = None if self.first_stamp is None else max(0., self.start_s-self.first_stamp)
        trailing = None if self.last_stamp is None else max(0., end_s-self.last_stamp)
        gap_limit = max(.5, 2./self.fps)
        coverage = (self.source_frames > 0 and max(leading, initial_age, trailing, self.maximum_gap) <= gap_limit+1e-8)
        return dict(start_sim_s=self.start_s, end_sim_s=end_s,
                    elapsed_sim_s=end_s-self.start_s, source_frames=self.source_frames,
                    encoded_frames=self.encoded_frames, held_frames=self.held_frames,
                    duplicate_stamps=self.duplicate_stamps, video_fps=self.fps,
                    maximum_source_gap_s=self.maximum_gap, first_frame_delay_s=leading,
                    initial_frame_age_s=initial_age,
                    last_frame_age_s=trailing, coverage_gap_limit_s=gap_limit,
                    coverage_complete=coverage,
                    gap_policy='Hold last observed image; retain source timestamps and report gaps')


def overview_timing(duration_s):
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('Invalid recorded duration')
    speed = max(1, math.ceil(duration_s/57.))
    return dict(speed_multiplier=speed, duration_s=duration_s/speed)


def _probe(path):
    return json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-show_entries',
        'format=duration,size:stream=width,height,codec_name,nb_frames,r_frame_rate',
        '-of', 'json', str(path)]))


def export_overview(folder, duration_s):
    folder = Path(folder)
    timing = overview_timing(duration_s)
    source, output = folder/'gazebo_full_1x.mp4', folder/'gazebo_overview.mp4'
    speed = timing['speed_multiplier']
    filters = (f'settb=AVTB,setpts=(PTS-STARTPTS)/{speed},fps=30,'
               'drawbox=x=0:y=0:w=iw:h=34:color=0x2d231e:t=fill,'
               f"drawtext=text='OMY | Gazebo | {speed}x | Entire collection':"
               'x=18:y=8:fontsize=20:fontcolor=white')
    subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-n', '-i', str(source),
                    '-vf', filters, '-an', '-c:v', 'libx264', '-threads', '2',
                    '-preset', 'veryfast', '-crf', '23', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', '-t', str(timing['duration_s']), str(output)],
                   check=True, start_new_session=True)
    full_probe, fast_probe = _probe(source), _probe(output)
    full_duration = float(full_probe['format']['duration'])
    actual = float(fast_probe['format']['duration'])
    if actual > 60.1 or abs(actual-timing['duration_s']) > .25:
        raise RuntimeError('Overview duration differs from the full recording')
    return dict(full_video=source.name, overview_video=output.name, **timing,
                full_video_duration_s=full_duration,
                full_video_time_quantization_s=full_duration-duration_s,
                actual_overview_duration_s=actual, full_probe=full_probe, overview_probe=fast_probe)


def make_recorder(output, fps=5., sim_duration=None):
    """Import ROS only for execution so timing helpers remain usable offline."""
    import cv2
    import numpy as np
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    class Recorder(Node):
        def __init__(self):
            super().__init__('friction_calibration_video', parameter_overrides=[
                Parameter('use_sim_time', value=True)])
            self.output = Path(output).resolve()
            self.output.mkdir(parents=True, exist_ok=True)
            if any((self.output/name).exists() for name in
                   ['gazebo_full_1x.mp4', 'video_timestamps.jsonl', 'video_summary.json']):
                raise ValueError('Refusing to overwrite an existing video recording')
            self.timeline = self.encoder = self.previous_frame = None
            self.encoder_log = None
            self.encoder_failure = None
            self.frame_queue = queue.Queue(maxsize=64)
            self.encoder_thread = threading.Thread(target=self.encode_frames, daemon=True)
            self.encoder_thread.start()
            self.latest_frame = None
            self.latest_stamp = None
            self.started_wall = self.end_s = None
            self.stop_requested_wall = None
            self.stop_reason = self.failure = None
            self.done = False
            self.status_text = 'Waiting for collection'
            self.index = (self.output/'video_timestamps.jsonl').open('w', buffering=1)
            self.status_log = (self.output/'video_status.jsonl').open('w', buffering=1)
            reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Image, '/review/image', self.image,
                                     QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE))
            self.create_subscription(Bool, '/friction_calibration/recording', self.recording, reliable)
            self.create_subscription(String, '/friction_calibration/status', self.status, reliable)
            self.create_service(Trigger, '/friction_calibration/video/stop', self.stop_service)
            self.create_service(Trigger, '/friction_calibration/video/ready', self.ready_service)
            self.create_timer(.1, self.check_duration)

        def now_s(self):
            return self.get_clock().now().nanoseconds*1e-9

        def ready_service(self, request, response):
            response.success = (self.latest_frame is not None and self.now_s() > 0 and not self.failure
                                and self.now_s()-self.latest_stamp <= max(.5, 2./fps))
            response.message = ('recording' if self.timeline is not None else 'Gazebo camera ready') if response.success else 'Waiting for Gazebo camera/clock'
            return response

        def stop_service(self, request, response):
            self.stop('stop_service')
            response.success = True
            response.message = 'Finalizing recorded Gazebo frames'
            return response

        def stop(self, reason):
            if self.timeline is not None and self.end_s is None:
                self.end_s = max(self.now_s(), self.timeline.last_stamp or self.timeline.start_s)
                self.stop_reason = reason
                self.stop_requested_wall = time.monotonic()
                # Clock and image travel through independent queues. Drain the
                # camera through this timestamp before stopping the simulator.
                self.done = reason in ('error', 'signal')

        def check_duration(self):
            if self.stop_requested_wall is not None and time.monotonic()-self.stop_requested_wall > 3.:
                self.done = True
            if self.timeline is not None and sim_duration is not None:
                if self.now_s()-self.timeline.start_s >= sim_duration:
                    self.stop('sim_duration')

        def recording(self, msg):
            if msg.data and self.timeline is None:
                self.timeline = FrameTimeline(fps, self.now_s())
                self.started_wall = time.monotonic()
                if self.latest_frame is not None and self.latest_stamp <= self.timeline.start_s:
                    slot = self.timeline.seed(self.latest_stamp)
                    self.index.write(json.dumps(dict(source_sim_stamp_s=self.latest_stamp,
                        elapsed_sim_s=self.latest_stamp-self.timeline.start_s, slot=slot, initial_cached_frame=True))+'\n')
                    self.previous_frame = self.overlay(self.latest_frame.copy(), 0.)
                    self.write_frame(self.previous_frame)
                self.get_logger().info(f'Recording actual Gazebo camera at {fps:g} fps')
            elif not msg.data and self.timeline is not None:
                self.stop('recording_false')
            elif not msg.data:
                self.stop_reason = 'recording_false_before_start'
                self.done = True

        def status(self, msg):
            try:
                value = json.loads(msg.data)
            except (ValueError, TypeError):
                value = msg.data
            if isinstance(value, dict):
                self.status_text = str(value.get('phase', value.get('state', value.get('stage', value))))
            else:
                self.status_text = str(value)
            self.status_log.write(json.dumps(dict(sim_stamp_s=self.now_s(), status=value))+'\n')

        def write_frame(self, frame):
            if self.encoder_failure is not None:
                raise RuntimeError(self.encoder_failure)
            self.frame_queue.put(frame, timeout=10.)

        def encode_frames(self):
            try:
                while True:
                    frame = self.frame_queue.get()
                    if frame is None:
                        break
                    self.encode_frame(frame)
                if self.encoder is not None:
                    self.encoder.stdin.close()
                    result = self.encoder.wait(timeout=120)
                    if result != 0:
                        raise RuntimeError(f'Video encoder failed with exit status {result}')
            except Exception as exc:
                self.encoder_failure = str(exc)

        def encode_frame(self, frame):
            if self.encoder is None:
                height, width = frame.shape[:2]
                self.encoder_log = (self.output/'video_encoder.log').open('w')
                self.encoder = subprocess.Popen([
                    'ffmpeg', '-v', 'error', '-nostdin', '-n', '-f', 'rawvideo',
                    '-pix_fmt', 'bgr24', '-video_size', f'{width}x{height}',
                    '-framerate', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                    '-threads', '2', '-preset', 'veryfast', '-crf', '23',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                    str(self.output/'gazebo_full_1x.mp4')], stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL, stderr=self.encoder_log, start_new_session=True)
                cv2.imwrite(str(self.output/'gazebo_initial.png'), frame)
            self.encoder.stdin.write(frame.tobytes())

        def overlay(self, frame, elapsed_s):
            scale = frame.shape[1]/960.
            cv2.rectangle(frame, (0, 0), (frame.shape[1], round(68*scale)), (30, 35, 45), -1)
            for row, text in enumerate([
                f'OMY | Actual Gazebo camera | t={elapsed_s:.1f}s | 1x simulation time',
                self.status_text[:110]]):
                cv2.putText(frame, text, (round(18*scale), round((26+row*28)*scale)),
                            cv2.FONT_HERSHEY_SIMPLEX, .56*scale, (240, 240, 240), 1, cv2.LINE_AA)
            return frame

        def image(self, msg):
            try:
                if msg.encoding not in ('rgb8', 'bgr8'):
                    raise ValueError(f'Unsupported image encoding {msg.encoding}')
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
                frame = raw[:, :msg.width*3].reshape(msg.height, msg.width, 3).copy()
                if msg.encoding == 'rgb8':
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.latest_frame, self.latest_stamp = frame, stamp
                if self.timeline is None or self.done:
                    return
                if self.end_s is not None and stamp >= self.end_s:
                    self.done = True
                    return
                slot = self.timeline.observe(stamp)
                self.index.write(json.dumps(dict(source_sim_stamp_s=stamp,
                    elapsed_sim_s=stamp-self.timeline.start_s, slot=slot))+'\n')
                if slot is None:
                    return
                frame = self.overlay(frame, slot['elapsed_s'])
                for _ in range(slot['hold_frames']):
                    self.write_frame(self.previous_frame if self.previous_frame is not None else frame)
                self.write_frame(frame)
                self.previous_frame = frame
            except Exception as exc:
                self.failure = str(exc)
                self.get_logger().error(self.failure)
                self.stop('error')

        def finish(self):
            if self.timeline is None:
                self.frame_queue.put(None)
                self.encoder_thread.join(timeout=1)
                self.index.close()
                self.status_log.close()
                self.failure = 'Collection stopped before camera recording began'
                report = dict(source='gazebo', hardware_access=False, completed=False,
                              stop_reason=self.stop_reason or 'signal_before_start',
                              failure=self.failure, source_frames=0, encoded_frames=0,
                              elapsed_sim_s=0., elapsed_wall_s=0.)
                (self.output/'video_summary.json').write_text(json.dumps(report, indent=2))
                return
            if self.end_s is None:
                self.stop('signal')
            wall_s = time.monotonic()-self.started_wall
            try:
                for _ in range(self.timeline.finish(self.end_s)):
                    if self.previous_frame is not None:
                        self.write_frame(self.previous_frame)
                self.frame_queue.put(None, timeout=10.)
                self.encoder_thread.join(timeout=120)
                if self.encoder_thread.is_alive():
                    raise RuntimeError('Video encoder finalization timeout')
                if self.encoder_failure is not None:
                    raise RuntimeError(self.encoder_failure)
                if self.encoder is not None:
                    cv2.imwrite(str(self.output/'gazebo_final.png'), self.previous_frame)
                else:
                    raise RuntimeError('No Gazebo camera frames were recorded')
            except Exception as exc:
                self.failure = self.failure or str(exc)
            finally:
                self.index.close()
                self.status_log.close()
                if self.encoder_log is not None:
                    self.encoder_log.close()
            report = dict(source='gazebo', hardware_access=False,
                          **self.timeline.report(self.end_s), elapsed_wall_s=wall_s,
                          stop_reason=self.stop_reason, failure=self.failure,
                          timestamp_map='video_timestamps.jsonl', status_events='video_status.jsonl')
            if self.failure is None:
                try:
                    report['videos'] = export_overview(self.output, report['elapsed_sim_s'])
                except Exception as exc:
                    self.failure = report['failure'] = str(exc)
            report['completed'] = (self.failure is None and report['coverage_complete'] and
                                   self.stop_reason == 'recording_false')
            (self.output/'video_summary.json').write_text(json.dumps(report, indent=2))
            print(json.dumps(report), flush=True)

    return Recorder()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--fps', type=float, default=5.)
    parser.add_argument('--sim-duration', type=float)
    args = parser.parse_args()
    FrameTimeline(args.fps, 0.)
    if args.sim_duration is not None and (not math.isfinite(args.sim_duration) or args.sim_duration <= 0):
        parser.error('--sim-duration must be positive')
    import rclpy
    rclpy.init()
    node = make_recorder(args.output, args.fps, args.sim_duration)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Launch may interrupt the collector and recorder together. Finish the
        # encoder and timestamp manifest before honoring further shutdown signals.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.finish()
        failure = node.failure
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failure:
        raise RuntimeError(failure)


if __name__ == '__main__':
    main()
