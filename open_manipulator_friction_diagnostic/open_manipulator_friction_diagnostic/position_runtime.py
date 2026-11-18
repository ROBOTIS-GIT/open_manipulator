"""Collector interlock: joint states alone do not prove that drives are running."""
import math


def heartbeat_message(state, sequence, generated_at):
    """Same-host monotonic time prevents a queued message renewing liveness."""
    return dict(sequence=sequence, generated_at=generated_at,
                **{k: state[k] for k in ('phase', 'ready', 'failure', 'reason',
                                       'controller_manager_exit_code')})


class RuntimeHeartbeat:
    """Latest supervisor state, independent of request/reply service scheduling.

    The 0.5 s supervisor deadline is unchanged. The supervisor separately checks
    advancing drive ticks at 0.25 s; the collector checks acquisition at 0.1 s.
    """
    def __init__(self):
        self.state = None
        self.wait_started = None
        self.received = 0
        self.duplicates = 0
        self.max_age = 0.
        self.max_gap = 0.

    def observe(self, state, now):
        stamp, sequence = state['generated_at'], state['sequence']
        if (not isinstance(sequence, int) or sequence < 1 or
                not math.isfinite(stamp) or not math.isfinite(now) or stamp > now+.02):
            raise ValueError('Invalid runtime heartbeat timestamp or sequence')
        if self.state is not None:
            if sequence <= self.state['sequence'] or stamp <= self.state['generated_at']:
                self.duplicates += 1
                return
            self.max_gap = max(self.max_gap, stamp-self.state['generated_at'])
        self.state = state
        self.received += 1
        self.max_age = max(self.max_age, now-stamp)

    def check(self, now, *, preparing=False):
        if self.wait_started is None:
            self.wait_started = now
        generated = self.state['generated_at'] if self.state is not None else self.wait_started
        age = now-generated
        if age > .5:
            raise RuntimeError(f'Position runtime heartbeat stopped: age={age:.3f} s')
        if self.state is None:
            return None
        return require_position_runtime(self.state, preparing=preparing)

    def report(self):
        return dict(received=self.received, duplicates=self.duplicates,
                    max_transport_age_s=self.max_age, max_generation_gap_s=self.max_gap,
                    last_sequence=self.state['sequence'] if self.state else None,
                    supervisor_timeout_s=.5)


def require_position_runtime(state, *, preparing=False):
    permitted = ('priming', 'awaiting_position_start', 'enabling', 'running') if preparing else ('running',)
    if (state.get('phase') not in permitted or state.get('failure') or
            state.get('controller_manager_exit_code') is not None or
            (not preparing and not state.get('ready'))):
        detail = state.get('failure') or state.get('reason') or state.get('phase', 'missing status')
        raise RuntimeError('Position runtime unavailable: ' + str(detail))
    return state
