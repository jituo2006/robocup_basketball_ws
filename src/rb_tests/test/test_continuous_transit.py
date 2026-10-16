"""无ROS惯性/延迟反馈仿真，导入生产转场控制。"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import transit_motion as motion


class Plant:
    def __init__(self, delay=0.1, speed=0.85, yaw=0.0, accel=1.5):
        self.clock = 0.0
        self.delay = delay
        self.accel = accel
        self.actual = [0.0, 0.0]
        self.velocity = [0.0, 0.0]
        self.pose = (0.0, 0.0, yaw)
        self.pose_stamp = 100.0 - delay
        self.yaw_rate = 0.0
        self.command = (0.0, 0.0)
        self.args = NS(
            speed=speed, max_ang=0.7, kp_yaw=2.0, kd_yaw=0.25, route_heading_tol=0.1
        )
        self.node = NS(
            get_clock=lambda: NS(
                now=lambda: NS(nanoseconds=int((100.0 + self.clock) * 1e9))
            )
        )
        self.history = [(-1.0, (0.0, 0.0))]
        self.commands = []
        self.stops = []
        self.events = []
        self._Twist = lambda: NS(linear=NS(x=0.0, y=0.0), angular=NS(z=0.0))

    def require_fresh_pose(self):
        pass

    def stop(self):
        self.stops.append(self.clock)
        self.command = (0.0, 0.0)

    def publish_command(self, t):
        a = self.pose[2]
        self.command = (
            t.linear.x * math.cos(a) - t.linear.y * math.sin(a),
            t.linear.x * math.sin(a) + t.linear.y * math.cos(a),
        )
        self.commands.append((self.clock, *self.command))

    def spin(self, dt):
        self.clock += dt
        if self.clock > 60:
            raise RuntimeError(
                "simulation did not converge "
                + repr((self.actual, self.velocity, self.pose, self.command))
            )
        desired = self.command if math.hypot(*self.command) > 0.035 else (0.0, 0.0)
        ax, ay = desired[0] - self.velocity[0], desired[1] - self.velocity[1]
        ax, ay = motion.limit(ax, ay, self.accel * dt)
        self.velocity[0] += ax
        self.velocity[1] += ay
        self.actual[0] += self.velocity[0] * dt
        self.actual[1] += self.velocity[1] * dt
        self.history.append((self.clock, tuple(self.actual)))
        source = math.floor((self.clock - self.delay) * 10 + 1e-8) / 10
        if source + 100 > self.pose_stamp + 1e-8:
            sample = next(
                (xy for when, xy in reversed(self.history) if when <= source + 1e-8),
                (0.0, 0.0),
            )
            self.pose = (*sample, self.pose[2])
            self.pose_stamp = 100 + source

    def emit(self, event, **data):
        self.events.append((self.clock, event, data))


def run(points, delay=0.1, speed=0.85, yaw=0.0, accel=1.5):
    r = Plant(delay, speed, yaw, accel)
    old = motion.time.monotonic
    motion.time.monotonic = lambda: r.clock
    try:
        assert motion.follow_path(r, points, 0.02, emit=r.emit)
        done = r.clock
        for _ in range(20):
            r.spin(0.02)
        return r, done
    finally:
        motion.time.monotonic = old


import pytest
from match_round_route import RoundRoute, load_config


@pytest.mark.parametrize("delay", [0.0, 0.1, 0.2])
@pytest.mark.parametrize("accel", [0.6, 1.5])
@pytest.mark.parametrize("yaw", [0.0, math.pi / 2])
@pytest.mark.parametrize("points", [[(1.2, 0.0)], [(1.2, 0.0), (1.2, 1.2), (3.0, 1.2)]])
def test_delayed_inertial_plant_converges_without_intermediate_stop(
    delay, accel, yaw, points
):
    r, t = run(points, delay=delay, accel=accel, yaw=yaw)
    assert math.dist(r.actual, points[-1]) <= 0.03
    assert math.hypot(*r.velocity) < 0.05
    assert len(r.stops) == 1
    assert max(math.hypot(x, y) for _, x, y in r.commands) > 0.6
    arrival = [data for _, event, data in r.events if event == "transit_arrival"]
    assert len(arrival) == 1
    assert arrival[0]["error_m"] <= 0.02 and arrival[0]["measured_mps"] <= 0.05
    assert len([e for e in r.events if e[1] == "transit_progress"]) <= t * 5 + 2
    for (ta, xa, ya), (tb, xb, yb) in zip(r.commands, r.commands[1:]):
        assert math.hypot(xb - xa, yb - ya) <= 1.21 * (tb - ta) + 1e-6


def test_corner_passes_slowly_without_zero_command_and_terminal_brakes():
    r, t = run([(2.0, 0.0), (2.0, 2.0)])
    passing = [when for when, event, data in r.events if event == "transit_waypoint"]
    assert len(passing) == 1
    corner = [
        math.hypot(x, y) for when, x, y in r.commands if abs(when - passing[0]) < 0.12
    ]
    assert corner and min(corner) > 0.05 and max(corner) < 0.5
    # 进最终点前已经减速；绝不满速冲入2cm圈后才硬停。
    near = [
        d
        for _, event, d in r.events
        if event == "transit_progress" and d["final_error_m"] < 0.15
    ]
    assert near and max(d["command_mps"] for d in near) < 0.2


def test_final_goal_cannot_be_accepted_by_merely_crossing_it(monkeypatch):
    r = Plant()
    r.pose = (0.0, 0.0, 0.0)
    steps = [0]

    def spin(dt):
        r.clock += dt
        steps[0] += 1
        r.pose = (1.05, 0.0, 0.0)
        r.pose_stamp = 100 + r.clock
        if steps[0] > 8:
            raise RuntimeError("feedback never inside goal")

    r.spin = spin
    monkeypatch.setattr(motion.time, "monotonic", lambda: r.clock)
    with pytest.raises(RuntimeError, match="never inside"):
        motion.follow_path(r, [(1.0, 0.0)], 0.02)
    assert r.stops


def test_duplicate_pose_stamp_does_not_fake_stop_confirmation(monkeypatch):
    r = Plant()
    r.pose = (1.0, 0.0, 0.0)

    def spin(dt):
        r.clock += dt
        if r.clock > 0.3:
            raise RuntimeError("no new source frames")

    r.spin = spin
    monkeypatch.setattr(motion.time, "monotonic", lambda: r.clock)
    with pytest.raises(RuntimeError, match="no new source"):
        motion.follow_path(r, [(1.0, 0.0)], 0.02)
    assert r.stops


@pytest.mark.parametrize("failure", [RuntimeError("board reset"), KeyboardInterrupt()])
def test_faults_and_cancellation_stop_and_propagate(monkeypatch, failure):
    r = Plant()
    r.publish_command = lambda tw: (_ for _ in ()).throw(failure)
    monkeypatch.setattr(motion.time, "monotonic", lambda: r.clock)
    with pytest.raises(type(failure)):
        motion.follow_path(r, [(1.0, 0.0), (1.0, 1.0)], 0.02)
    assert r.stops


def test_reversal_waypoint_requires_stop():
    r, t = run([(1.0, 0.0), (0.0, 0.0)])
    assert len(r.stops) >= 2 and math.dist(r.actual, (0.0, 0.0)) < 0.03


@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_speed_stops_without_driving(value):
    r = Plant()
    r.args.speed = value
    with pytest.raises(ValueError):
        motion.follow_path(r, [(1.0, 0.0)], 0.02)
    assert r.stops and not r.commands


def test_actual_runner_dispatches_whole_path_and_records_passed_points(monkeypatch):
    from test_perimeter_ball import Runner

    r = Plant()
    r.args.tol = 0.02
    passed = []
    monkeypatch.setattr(motion.time, "monotonic", lambda: r.clock)
    assert Runner.goto(
        r,
        1.0,
        1.0,
        timeout=0.0,
        transit=True,
        path_points=[(1.0, 0.0), (1.0, 1.0)],
        path_progress=passed.append,
    )
    assert passed == [(1.0, 0.0), (1.0, 1.0)]
    assert len(r.stops) == 1


@pytest.mark.parametrize("task", ["pass", "shoot"])
@pytest.mark.parametrize("side", ["A", "B"])
def test_round_path_sends_all_coordinates_in_one_controller_call(task, side):
    cfg = load_config(start_point=side)
    calls = []
    r = Plant()
    r.pose = (*cfg["start"], 0.0)
    r.args.step_tol = 0.02

    def goto(x, y, **kw):
        calls.append((x, y, r.args.speed, kw))
        for p in kw.get("path_points", [(x, y)]):
            if kw.get("path_progress"):
                kw["path_progress"](p)
        r.pose = (x, y, 0.0)
        return True

    r.goto = goto
    r.rotate_to = lambda *a, **kw: True
    route = RoundRoute(r, cfg, 2, task, "collect", emit=lambda m: None)
    points = [cfg["corridor"], cfg["release"][task]["point"]]
    assert route.move_path(points, remember=True)
    assert len(calls) == 1
    for actual, expected in zip(calls[0][3]["path_points"], points):
        assert actual == pytest.approx(expected)
    assert len(route.trail) == 2
    assert calls[0][2] == cfg["speed_profiles"]["2"]["travel_mps"]
    assert r.args.speed == 0.85


def test_search_still_uses_unchanged_scan_speed():
    cfg = load_config()
    r = Plant()
    r.args.step_tol = 0.02
    calls = []
    r.scan_to = lambda *a, **kw: (calls.append(kw) or ("empty", None))
    route = RoundRoute(r, cfg, 1, "shoot", "collect", emit=lambda m: None)
    assert route.scan(cfg["entry"], 0.0, centered=True) == "empty"
    assert calls[0]["speed"] == 0.25 and calls[0]["detect"] and calls[0]["centered"]


@pytest.mark.parametrize(
    "value",
    [
        {"brake_mps2": 0.0},
        {"accel_mps2": float("nan")},
        {"arrival_frames": 1.5},
        {"prediction_max_s": 1.0},
        {"unknown": 1.0},
    ],
)
def test_invalid_control_options_rejected(value):
    with pytest.raises(ValueError):
        motion.parameters(value)


def test_weak_acceleration_and_delayed_feedback_do_not_cut_far_across_corner():
    r, t = run([(2.0, 0.0), (2.0, 2.0)], delay=0.2, accel=0.6)

    def distance(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        q = max(
            0.0,
            min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)),
        )
        return math.dist(p, (a[0] + q * dx, a[1] + q * dy))

    deviation = max(
        min(distance(p, (0.0, 0.0), (2.0, 0.0)), distance(p, (2.0, 0.0), (2.0, 2.0)))
        for _, p in r.history
    )
    assert deviation < 0.12


@pytest.mark.parametrize("task", ["pass", "shoot"])
@pytest.mark.parametrize("side", ["A", "B"])
@pytest.mark.parametrize("number", [1, 2, 3])
def test_all_rounds_keep_ball_class_and_complete_return(
    task, side, number, tmp_path, monkeypatch
):
    import match_round_steps as stages
    from test_match_round_route import fake_runner

    monkeypatch.setattr(stages, "STATE_DIR", tmp_path)
    r, events = fake_runner()
    r.args.route_intake = False
    r.scan_to = lambda *a, **kw: ("found", NS())
    cfg = load_config(start_point=side)
    origin = r.pose
    assert RoundRoute(r, cfg, number, task, "cycle", emit=lambda m: None).execute()
    assert r.pose == pytest.approx(origin)
    assert r.args.ball_label == (
        "ball_volleyball" if task == "shoot" else "ball_basketball"
    )
    assert stages.workflow_state(number, task)["finished"]
