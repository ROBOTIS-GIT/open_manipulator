"""Mechanical rest qualification; this never measures current-sensor latency.

The configured dwell is fixed, not adaptively extended inside a queued action.
A failed/missing rest check cancels collection before the next commanded start.
"""
from collections import deque

import numpy as np


def validate_settings(c):
    g = c.get('settling_guard', {})
    for key in ('window_s', 'cancel_margin_s', 'position_span_rad', 'slope_rad_s'):
        if not np.isfinite(g.get(key, float('nan'))) or g[key] <= 0:
            raise ValueError(f'Invalid settling_guard.{key}')
    if (g['window_s'] < .2 or g['cancel_margin_s'] < .15 or
            g['window_s']+g['cancel_margin_s']+max(c['sensor_settling_s']) > c['dwell_s']):
        raise ValueError('Dwell must cover assumed sensor settling, rest window and cancellation margin')


def windows(meta):
    c = meta['config']; dt = c['sample_period_s']
    g = c['settling_guard']
    return [dict(segment=i, begin=s['stop']*dt-g['cancel_margin_s']-g['window_s'],
                 end=s['stop']*dt-g['cancel_margin_s'], joint=s['joint'],
                 repeat=s['repeat'], validation_pose=s.get('validation_pose', ''))
            for i, s in enumerate(meta['segments']) if s['phase'] == 'hold']


def assess_window(t, q, window, settings):
    t, q = np.asarray(t), np.asarray(q)
    mask = (t >= window['begin']-1e-7) & (t <= window['end']+1e-7)
    t, q = t[mask], q[mask]
    valid = (len(t) >= 6 and t[0] <= window['begin']+.05 and
             t[-1] >= window['end']-.05 and np.all(np.diff(t) > 0) and
             np.max(np.diff(t)) <= .05 and np.isfinite(q).all())
    result = dict(window, passed=False, samples=len(t), current_timing_qualified=False)
    if not valid:
        return dict(result, reason='Missing, stale or nonfinite rest samples')
    x = t-np.mean(t)
    slope = (x[:, None]*(q-np.mean(q, axis=0))).sum(axis=0)/(x@x)
    span = np.ptp(q, axis=0)
    passed = bool(np.all(span <= settings['position_span_rad']) and
                  np.all(abs(slope) <= settings['slope_rad_s']))
    return dict(result, passed=passed, span_rad=span.tolist(), slope_rad_s=slope.tolist(),
                reason='Mechanical rest passed' if passed else 'Robot has not mechanically settled')


def replay(meta, data):
    results = []
    t = data['t']
    for w in windows(meta):
        low, high = np.searchsorted(t, [w['begin']-.001, w['end']+.001])
        results.append(assess_window(t[low:high], data['q'][low:high], w, meta['config']['settling_guard']))
    return dict(passed=all(r['passed'] for r in results), checks=results,
                current_timing_qualified=False)


class SettlingMonitor:
    def __init__(self, meta):
        self.pending = deque(windows(meta)); self.history = deque(); self.results = []
        self.settings = meta['config']['settling_guard']

    def observe(self, t, q):
        self.history.append((t, np.array(q, copy=True)))
        while self.history and self.history[0][0] < t-self.settings['window_s']-.15:
            self.history.popleft()
        if not self.pending or t < self.pending[0]['end']:
            return
        w = self.pending.popleft()
        if t-w['end'] > .05:
            r = dict(w, passed=False, reason='Rest check deadline missed')
        else:
            times, positions = zip(*self.history)
            r = assess_window(times, positions, w, self.settings)
        self.results.append(r)
        if not r['passed']:
            raise ValueError(f'Settling check failed at segment {w["segment"]}: {r["reason"]}')

    def finish(self):
        if self.pending:
            raise ValueError('Collection ended before all settling checks')
