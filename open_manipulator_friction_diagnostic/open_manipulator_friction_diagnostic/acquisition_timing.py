"""Separate measured-window quality from recovered transport gaps in transitions."""
import numpy as np
from .settling import windows


class AcquisitionTiming:
    def __init__(self, meta):
        self.portable = meta.get('portable_calibration') is True
        self.required = []
        if self.portable:
            dt = meta['config']['sample_period_s']
            # Protect complete tagged measurements, including starts and reversals.
            self.required = [(s['start']*dt, s['stop']*dt)
                             for s in meta['segments'] if s['joint'] >= 0]
            self.required += [(w['begin'], w['end']) for w in windows(meta)]

    def allowed(self, before, after):
        gap = after-before
        if gap <= 0 or gap > .1:
            return False
        if gap <= .05:
            return True
        return self.portable and not any(before <= end and after >= begin
                                         for begin, end in self.required)

    def check(self, times):
        differences = np.diff(times)
        if np.any(differences <= 0):
            raise ValueError('Nonmonotonic or stale measurements')
        for i in np.flatnonzero(differences > .05):
            if not self.allowed(times[i], times[i+1]):
                raise ValueError('Nonmonotonic or stale measurements in required acquisition window')
