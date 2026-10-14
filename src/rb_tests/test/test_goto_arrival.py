"""Exercise the route's production goto with source-stamped simulated feedback."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from rb_mission.geometry import goto_command


@pytest.fixture
def motion(monkeypatch):
    path = Path(__file__).parents[3] / 'tools/test_perimeter_ball.py'
    spec = importlib.util.spec_from_file_location('goto_arrival_runner', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    r = mod.Runner.__new__(mod.Runner)
    r.args = NS(tol=.05, kp=1.5, kp_yaw=2., speed=.25, max_ang=.8,
                approach_radius=.4, approach_speed=.12, pose_timeout=.5,
                motion_stall_seconds=.2)
    r.pose, r.pose_stamp, r.odom_stamp = (0., 0., 0.), 100., 100.
    r.pose_history = []
    r.motion_fault = None
    clock = [0.]
    monkeypatch.setattr(mod.time, 'monotonic', lambda: clock[0])
    # A wall-clock jump must not change action timeout.
    monkeypatch.setattr(mod.time, 'time', lambda: 1e12-clock[0]*1e6)
    r.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=int((100.+clock[0])*1e9))))
    r._goto_command = goto_command
    r._Twist = lambda: NS(linear=NS(x=0., y=0.), angular=NS(z=0.))
    commands, stops = [], []
    r.publish_command = lambda t: commands.append((t.linear.x, t.linear.y, t.angular.z))
    r.stop = lambda: stops.append(clock[0])
    r.spin = lambda seconds: clock.__setitem__(0, clock[0]+seconds)
    return r, clock, commands, stops


def sample(r, clock, pose):
    r.pose = pose
    r.pose_stamp = r.odom_stamp = 100.+clock[0]


def test_pass_through_needs_fresh_stopped_samples_and_corrects_overshoot(motion):
    r, clock, commands, stops = motion
    positions = iter([0., .3, .7, .97, 1.12, 1.08] + [1.]*20)
    def spin(seconds):
        clock[0] += seconds
        sample(r, clock, (next(positions, 1.), 0., 0.))
    r.spin = spin
    assert r.goto(1., 0., timeout=.01)
    assert clock[0] > .8
    assert any(vx < 0 for vx, vy, wz in commands)
    assert all(abs(vx) <= .08 for vx, vy, wz in commands if vx < 0)
    assert stops[-1] == clock[0]


def test_repeated_pose_cannot_confirm_arrival(motion):
    r, clock, commands, stops = motion
    r.pose = (1., 0., 0.)
    with pytest.raises(RuntimeError, match='时间戳过期'):
        r.goto(1., 0., timeout=2.)
    assert stops and all(c == (0., 0., 0.) for c in commands)


def test_stale_odometry_at_goal_cannot_report_success(motion):
    r, clock, commands, stops = motion
    r.pose = (1., 0., 0.)
    r.odom_stamp = 99.
    with pytest.raises(RuntimeError, match='雷达里程计'):
        r.goto(1., 0., timeout=2.)
    assert stops and not commands


def test_world_velocity_is_rotated_once_and_approach_is_limited(motion):
    r, clock, commands, stops = motion
    def spin(seconds):
        clock[0] += seconds
        sample(r, clock, (0., 0., math.pi/2))
    r.spin = spin
    assert not r.goto(.2, 0., timeout=.2)
    assert commands
    for vx, vy, wz in commands:
        assert abs(vx) < 1e-9 and vy == pytest.approx(-.12) and wz == 0.
    assert stops


def test_rotation_at_goal_prevents_stable_arrival(motion):
    r, clock, commands, stops = motion
    def spin(seconds):
        clock[0] += seconds
        if clock[0] > 1.5:
            raise KeyboardInterrupt()
        sample(r, clock, (1., 0., clock[0]))
    r.spin = spin
    with pytest.raises(KeyboardInterrupt):
        r.goto(1., 0., timeout=.01)
    assert stops and all(c == (0., 0., 0.) for c in commands)


@pytest.mark.parametrize('target,timeout,tol', [(math.nan, 1., .05), (1., 1., -1.)])
def test_invalid_goal_stops_without_motion(motion, target, timeout, tol):
    r, clock, commands, stops = motion
    with pytest.raises(ValueError):
        r.goto(target, 0., timeout=timeout, tolerance=tol)
    assert stops and not commands


def test_cancellation_stops_and_propagates(motion):
    r, clock, commands, stops = motion
    r.spin = lambda seconds: (_ for _ in ()).throw(KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        r.goto(1., 0., timeout=2.)
    assert stops


def test_scaled_quaternion_yaw_is_normalized(motion):
    r, clock, commands, stops = motion
    stamp = NS(sec=100, nanosec=100000000)
    q = NS(x=0., y=0., z=2.*math.sin(math.pi/4), w=2.*math.cos(math.pi/4))
    r._on_pose(NS(header=NS(stamp=stamp),
                  pose=NS(pose=NS(position=NS(x=0., y=0., z=123.), orientation=q))))
    assert r.pose[2] == pytest.approx(math.pi/2)
