"""Navigation feedback and arrival confirmation, independent of ROS."""
from collections import deque
from dataclasses import dataclass
import math
from .geometry import wrap_pi


def fresh(now, stamp, timeout):
    return stamp is not None and 0.0 <= now - stamp <= timeout


class PoseFeedback:
    """Estimate measured speed from distinct source-stamped poses, not commands."""
    def __init__(self):
        self.stamp = None
        self.last_clock = None
        self.samples = deque()
        self.linear_speed = math.inf
        self.angular_speed = math.inf

    def update(self, pose, stamp, now, timeout, window):
        if not all(math.isfinite(v) for v in (*pose, stamp, now)) or stamp <= 0:
            return False
        if self.last_clock is not None and now < self.last_clock:
            self.stamp = None
            self.samples.clear()
            self.linear_speed = self.angular_speed = math.inf
        self.last_clock = now
        if not fresh(now, stamp, timeout):
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False  # timer republication is not a new stationary measurement
        if self.stamp is None or stamp - self.stamp > timeout:
            self.samples.clear()
            self.linear_speed = self.angular_speed = math.inf
        self.stamp = stamp
        self.samples.append((stamp, pose))
        while len(self.samples) > 1 and stamp - self.samples[1][0] >= window:
            self.samples.popleft()
        t0, p0 = self.samples[0]
        dt = stamp - t0
        if dt >= window:
            self.linear_speed = math.hypot(pose[0] - p0[0], pose[1] - p0[1]) / dt
            self.angular_speed = abs(wrap_pi(pose[2] - p0[2])) / dt
        return True


@dataclass(frozen=True)
class ArrivalConfig:
    position_tolerance: float = 0.10
    yaw_tolerance: float = 0.08
    exit_margin_m: float = 0.02
    linear_speed_tolerance: float = 0.03
    angular_speed_tolerance: float = 0.05
    settle_time_s: float = 0.5
    settle_timeout_s: float = 5.0


class ArrivalGate:
    def __init__(self):
        self.reset()

    def reset(self):
        self.confirming = False
        self.started = None
        self.stable_since = None

    def update(self, now, distance, yaw_error, linear_speed, angular_speed, cfg, sample_time=None):
        """Return approach/correct/settling/complete/timeout.

        The confirmation deadline survives excursions until the whole goal resets.
        Completion always uses the strict inner tolerance, even in the hysteresis band.
        """
        if not all(math.isfinite(v) for v in (now, distance, yaw_error)):
            self.reset()
            return 'settling'
        if self.started is not None and now < self.started:
            self.reset()
        if not self.confirming and distance <= cfg.position_tolerance:
            self.confirming = True
            if self.started is None:
                self.started = now
        if self.confirming and distance > cfg.position_tolerance + cfg.exit_margin_m:
            self.confirming = False
            self.stable_since = None
        if self.started is not None and now - self.started >= cfg.settle_timeout_s:
            return 'timeout'
        if not self.confirming:
            return 'approach'
        if sample_time is None:
            sample_time = now
        stable = (distance <= cfg.position_tolerance and abs(yaw_error) <= cfg.yaw_tolerance
                  and math.isfinite(linear_speed) and math.isfinite(angular_speed)
                  and linear_speed <= cfg.linear_speed_tolerance
                  and angular_speed <= cfg.angular_speed_tolerance)
        if stable:
            if self.stable_since is None:
                self.stable_since = max(now, sample_time)
            if sample_time - self.stable_since >= cfg.settle_time_s:
                return 'complete'
        else:
            self.stable_since = None
        # Correct position in the outer band so the robot cannot remain stranded there.
        return 'correct' if distance > cfg.position_tolerance else 'settling'
