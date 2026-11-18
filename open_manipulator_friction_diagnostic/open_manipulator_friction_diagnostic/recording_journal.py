"""Crash-recoverable measured-row prefix for long acquisitions.

Only the non-ROS recording thread commits; the state callback only appends a
row to the array. A recovered prefix is diagnostic evidence, never a completed
collection or deployment approval.
"""
import json
import os
from pathlib import Path

import numpy as np


class RecordingJournal:
    def __init__(self, folder, capacity, trajectory_sha256):
        self.folder = Path(folder)
        self.path = self.folder/'measured_rows.npy'
        if self.path.exists(): raise ValueError('Never replace an acquisition journal')
        self._disk = np.lib.format.open_memmap(self.path, mode='w+', dtype=np.float64, shape=(capacity, 19))
        # ROS writes anonymous memory only. A file-backed page can wait for
        # writeback on first write even when fsync itself releases Python's GIL.
        self.buffer = np.empty((capacity, 19), dtype=np.float64)
        self.capacity = capacity; self.count = 0; self.digest = trajectory_sha256
        self.commit(0)

    def commit(self, complete_rows, closed=False):
        if not isinstance(complete_rows, int) or not self.count <= complete_rows <= self.capacity:
            raise ValueError('Invalid completed journal row count')
        # Earlier rows are immutable. New rows remain in RAM until the next
        # checkpoint; only this non-ROS thread touches the mapped file.
        self._disk[self.count:complete_rows] = self.buffer[self.count:complete_rows]
        # mmap.flush() holds CPython's GIL during msync: on the robot it paused
        # the 400 Hz callback for 60–100+ ms. Linux fsync drains this file's
        # shared dirty mappings too, while Python releases the GIL for the I/O.
        with self.path.open('rb') as stream:
            os.fsync(stream.fileno())
        report = dict(schema=1, rows=complete_rows, capacity=self.capacity,
            filename=self.path.name, trajectory_sha256=self.digest, recording_closed=bool(closed),
            columns=['time_s']+[f'{j}_{field}' for field in ('position', 'velocity', 'effort')
                               for j in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')],
            last_time_s=float(self.buffer[complete_rows-1, 0]) if complete_rows else None,
            acquisition_completed=False, compensation_qualified=False)
        temp = self.folder/'recording_checkpoint.json.tmp'
        temp.write_text(json.dumps(report, indent=2)); temp.replace(self.folder/'recording_checkpoint.json')
        self.count = complete_rows
