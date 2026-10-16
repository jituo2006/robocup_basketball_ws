"""Cooperative round return request; never publishes motion or mechanism commands."""
from contextlib import contextmanager
import math
import time


class ReturnRequested(Exception):
    """Unwind task motion to the single owner of the round workflow."""


def validate_return_budget(cfg):
    options = cfg.get('timed_return', {})
    if not isinstance(options, dict):
        raise ValueError('timed_return 须为字典')
    if not isinstance(options.get('enabled', False), bool):
        raise ValueError('timed_return.enabled 须为布尔值')
    tasks = options.get('tasks', ['shoot'])
    if not isinstance(tasks, list) or any(t not in ('pass', 'shoot') for t in tasks):
        raise ValueError('timed_return.tasks 须为 pass/shoot 列表')
    defaults = {'round_duration_s': 120., 'return_reserve_s': 35.,
                'turn_reserve_s': 5., 'margin_s': 5.}
    for key, default in defaults.items():
        value = options.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'timed_return.{key} 须为有限正数')
    if options.get('return_reserve_s', 35.) >= options.get('round_duration_s', 120.):
        raise ValueError('回位预留须小于回合总时限')
    return options


def translation_seconds(distance, speed, kp=1.5, approach_radius=.25,
                        approach_speed=.12, tolerance=.02):
    """Integrate dr/dt=-min(kp*r, speed cap), including terminal slow-down."""
    if distance <= tolerance:
        return 0.
    cuts = sorted({tolerance, distance, *[v for v in
        (approach_radius, speed/kp, approach_speed/kp)
        if tolerance < v < distance]})
    total = 0.
    for lo, hi in zip(cuts, cuts[1:]):
        middle = (lo+hi)/2
        cap = min(speed, approach_speed) if approach_radius > 0 and approach_speed > 0 and middle < approach_radius else speed
        total += math.log(hi/lo)/kp if kp*middle < cap else (hi-lo)/cap
    return total


class RoundReturnBudget:
    def __init__(self, runner, cfg, number, task, emit=print, clock=None):
        from match_round_route import StartFrame
        self.runner, self.cfg, self.task = runner, cfg, task
        self.options = validate_return_budget(cfg)
        self.enabled = self.options.get('enabled', False) and task in self.options.get('tasks', ['shoot'])
        self.clock = clock or time.monotonic
        self.started = self.clock()
        self.frame = StartFrame(runner.pose, cfg['start'])
        self.profile = cfg.get('speed_profiles', {}).get(str(number), {})
        self.emit = emit
        self.returning = False
        self.deferred = 0
        self.requested = False

    def return_seconds(self):
        r, cfg, p = self.runner, self.cfg, self.profile
        if r.pose is None:
            return self.options.get('return_reserve_s', 35.)
        targets = ([cfg['corridor']] if self.task == 'shoot' else []) + [cfg['entry'], cfg['start']]
        points = [r.pose[:2]] + [self.frame.point(t) for t in targets]
        # A held ball can remain aboard on early return; use the slower profile.
        speed = min(p.get('travel_mps', cfg['speed_mps']), p.get('carry_mps', cfg['speed_mps']))
        seconds = sum(translation_seconds(math.dist(a, b), speed,
            kp=getattr(r.args, 'kp', 1.5),
            approach_radius=getattr(r.args, 'approach_radius', p.get('approach_radius_m', .25)),
            approach_speed=getattr(r.args, 'approach_speed', p.get('approach_mps', .12)),
            tolerance=getattr(r.args, 'step_tol', .02)) for a, b in zip(points, points[1:]))
        # Includes E -> turn to the original heading -> S, not a direct shortcut.
        max_ang = getattr(r.args, 'max_ang', 1.)
        turn = max(self.options.get('turn_reserve_s', 5.), math.pi/max_ang + 2.)
        return seconds + turn + self.options.get('margin_s', 5.)

    def check(self, extra_s=0.):
        if not self.enabled or self.returning:
            return
        elapsed = self.clock()-self.started
        reserve = max(self.options.get('return_reserve_s', 35.), self.return_seconds())
        if elapsed + reserve + extra_s >= self.options.get('round_duration_s', 120.):
            self.requested = True
        if self.requested and not self.deferred:
            raise ReturnRequested(f'已运行{elapsed:.1f}秒，回位预留{reserve:.1f}秒')

    def check_service_wait(self):
        """An unknown in-flight mechanism result must stop, not drive home."""
        if self.deferred:
            self.check()
            if self.requested:
                raise RuntimeError('计时回位已请求，但机构服务结果未确认；停车，不自动回位')
        else:
            self.check()

    @contextmanager
    def mechanism(self, seconds):
        # Do not begin a fixed sequence if its completion consumes the reserve.
        self.check(extra_s=seconds)
        self.deferred += 1
        try:
            yield
        finally:
            self.deferred -= 1
        # Caller checks only after a True result; a False result must stay a fault.


def check_runner(runner, service_wait=False):
    budget = getattr(runner, 'round_return_budget', None)
    if budget is not None:
        if service_wait:
            budget.check_service_wait()
        else:
            budget.check()
