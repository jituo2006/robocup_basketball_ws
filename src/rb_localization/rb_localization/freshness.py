"""Track actual position-source timestamps instead of publication timer ticks."""
import math


class SourceFreshness:
    def __init__(self):
        self.stamp = None
        self.by_source = {}
        self.last_clock = None

    def _check_clock(self, now):
        if self.last_clock is not None and now < self.last_clock:
            self.stamp = None
            self.by_source.clear()
        self.last_clock = now

    def accept(self, source, stamp, now, timeout):
        self._check_clock(now)
        if not math.isfinite(stamp) or stamp <= 0 or not 0 <= now - stamp <= timeout:
            return False
        if stamp <= self.by_source.get(source, -math.inf):
            return False
        # Do not update the filter backwards when two position sources interleave.
        if self.stamp is not None and stamp < self.stamp:
            return False
        self.by_source[source] = stamp
        self.stamp = stamp
        return True

    def valid(self, now, timeout):
        self._check_clock(now)
        return self.stamp is not None and 0 <= now - self.stamp <= timeout
