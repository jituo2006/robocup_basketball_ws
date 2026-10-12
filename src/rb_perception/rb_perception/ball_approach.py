"""Body-frame control: rear rail-tip midpoint to BALL CENTRE is 0.60 m."""
import math
import numpy as np
from .precision_ball import fresh


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def geometry(cfg):
    if not cfg.get("intake_confirmed", False):
        raise ValueError("measure intake position and outward axis first")
    point = np.asarray(cfg["intake_position_base_m"], dtype=float)
    axis = np.asarray(cfg["intake_outward_xy"], dtype=float)
    if (point.shape != (3,) or axis.shape != (2,) or not np.isfinite(point).all()
            or not np.isfinite(axis).all() or abs(np.linalg.norm(axis) - 1) > 1e-5
            or axis[0] > -.9):
        raise ValueError("invalid rear intake geometry")
    if "target_gap_m" in cfg or cfg.get("target_center_distance_m") != .60:
        raise ValueError("this procedure requires the agreed 0.60 m rail-tip-to-ball-centre distance")
    for key, default, lower, upper in (
            ("distance_tolerance_m", .02, .005, .03),
            ("lateral_tolerance_m", .02, .005, .03),
            ("heading_tolerance_rad", .035, .01, .05),
            ("settle_s", .5, .5, 2.), ("speed_limit_mps", .08, .01, .08),
            ("max_age_s", .3, .05, .3), ("stage_timeout_s", 45., 5., 120.)):
        v = float(cfg.get(key, default))
        if not math.isfinite(v) or not lower <= v <= upper:
            raise ValueError(f"invalid {key}: expected {lower}..{upper}")
    return point, axis


def errors(target, cfg, rear=True):
    point, axis = geometry(cfg)
    radius = float(target["radius_m"])
    centre = np.asarray(target["centre_base_m"], dtype=float)
    if centre.shape != (3,) or not np.isfinite(centre).all() or not .05 <= radius <= .2:
        raise ValueError("invalid ball geometry")
    # Before the 180-degree turn the required base vector is the negation.
    if not rear:
        point, axis = point * [-1., -1., 1.], -axis
    relative = centre - point
    centre_distance = float(np.linalg.norm(relative))
    lateral = float(relative[:2] @ np.array([-axis[1], axis[0]]))
    heading = wrap(math.atan2(relative[1], relative[0]) - math.atan2(axis[1], axis[0]))
    return centre_distance - cfg["target_center_distance_m"], lateral, heading, axis


class Arrival:
    def __init__(self, cfg):
        geometry(cfg)
        self.cfg = cfg
        self.since = None
        self.last_stamp = 0.

    def reset(self):
        self.since = None

    def update(self, target, stamp, now, speed, wz, rear=True):
        if not fresh(stamp, now, self.cfg.get("max_age_s", .3)) or stamp <= self.last_stamp:
            self.reset()
            return False
        self.last_stamp = stamp
        distance, lateral, heading, _ = errors(target, self.cfg, rear)
        values = [speed, wz, distance, lateral, heading]
        inside = (all(math.isfinite(v) for v in values)
                  and abs(distance) <= self.cfg.get("distance_tolerance_m", .02)
                  and abs(lateral) <= self.cfg.get("lateral_tolerance_m", .02)
                  and abs(heading) <= self.cfg.get("heading_tolerance_rad", .035)
                  and abs(speed) <= .03 and abs(wz) <= .05)
        if not inside:
            self.reset()
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.cfg.get("settle_s", .5)


def command(target, cfg, rear=True):
    distance, lateral, heading, axis = errors(target, cfg, rear)
    # Near the target stop requesting movement, then require measured standstill.
    if (abs(distance) <= cfg.get("distance_tolerance_m", .02)
            and abs(lateral) <= cfg.get("lateral_tolerance_m", .02)
            and abs(heading) <= cfg.get("heading_tolerance_rad", .035)):
        return 0., 0., 0.
    wz = float(np.clip(1.2 * heading, -.18, .18))
    if abs(heading) > .15:
        return 0., 0., wz
    velocity = axis * distance * .6 + np.array([-axis[1], axis[0]]) * lateral * .6
    norm = np.linalg.norm(velocity)
    limit = min(.08, float(cfg.get("speed_limit_mps", .08)))
    if norm > limit:
        velocity *= limit / norm
    return float(velocity[0]), float(velocity[1]), wz
