"""Visible progress after ROS collection stops, including long numerical fits."""
import json
import threading
import time


class ProcessingProgress:
    def __init__(self, joints, interval=30.):
        self.joints = list(joints)
        self.interval = interval
        self.started = time.monotonic()
        self.state = {'phase': 'CALCULATING'}
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.worker = threading.Thread(target=self._heartbeat, daemon=True)

    def _print(self):
        print(json.dumps(dict(self.state, selected_joints=self.joints,
            elapsed_s=round(time.monotonic()-self.started, 1),
            message='Calculation in progress. Do not stop this process until the results have been saved.'),
            ensure_ascii=False), flush=True)

    def update(self, phase, **values):
        with self.lock:
            self.state = dict(phase=phase, **values)
            self._print()

    def _heartbeat(self):
        while not self.stopped.wait(self.interval):
            with self.lock:
                self._print()

    def __enter__(self):
        self.update('CALCULATING')
        self.worker.start()
        return self

    def __exit__(self, *exc):
        self.stopped.set()
        self.worker.join()
