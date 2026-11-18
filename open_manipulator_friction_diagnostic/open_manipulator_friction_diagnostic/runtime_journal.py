"""Persist supervisor diagnostics without SD-card writes on its control loop."""
import json
from pathlib import Path
import threading
import time


class RuntimeJournal:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.condition = threading.Condition()
        self.pending = None
        self.events = []
        self.closing = False
        self.error = None
        self.max_write_s = 0.
        self.worker = threading.Thread(target=self._run, name='runtime_journal', daemon=True)
        self.worker.start()

    def _check(self):
        if self.error is not None:
            raise OSError(f'Runtime journal failed: {self.error}') from self.error

    def status(self, state):
        with self.condition:
            self._check()
            self.pending = state  # Superseded periodic snapshots need not reach disk.
            self.condition.notify()

    def event(self, event):
        with self.condition:
            self._check()
            self.events.append(event)  # Transitions are ordered and never coalesced.
            self.condition.notify()

    def _write_status(self, state):
        temporary = self.folder/'runtime.json.tmp'
        temporary.write_text(json.dumps(state, indent=2, allow_nan=False))
        temporary.replace(self.folder/'runtime.json')

    def _run(self):
        try:
            with (self.folder/'runtime_events.jsonl').open('a') as stream:
                while True:
                    with self.condition:
                        self.condition.wait_for(lambda: self.pending is not None or self.events or self.closing)
                        state, events = self.pending, self.events
                        self.pending, self.events = None, []
                        if state is None and not events and self.closing:
                            return
                    started = time.monotonic()
                    for event in events:
                        stream.write(json.dumps(event, allow_nan=False)+'\n')
                    if events:
                        stream.flush()
                    if state is not None:
                        self._write_status(state)
                    self.max_write_s = max(self.max_write_s, time.monotonic()-started)
        except Exception as exc:
            with self.condition:
                self.error = exc

    def close(self):
        with self.condition:
            self.closing = True
            self.condition.notify()
        self.worker.join(timeout=10.)
        if self.worker.is_alive():
            raise TimeoutError('Runtime journal did not finish after motor shutdown')
        self._check()
