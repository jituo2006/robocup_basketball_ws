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


def test_goal_tolerance_stops_without_velocity_confirmation_or_dwell(motion):
    r, clock, commands, stops = motion
    positions = iter([0., .3, .7, .97])
    def spin(seconds):
        clock[0] += seconds
        sample(r, clock, (next(positions, .97), 0., 0.))
    r.spin = spin
    assert r.goto(1., 0., timeout=.01)
    assert clock[0] == pytest.approx(.2)
    assert all(vx >= 0 for vx, vy, wz in commands)
    assert stops[-1] == clock[0]


def test_stale_pose_at_goal_still_stops_instead_of_reporting_arrival(motion):
    r, clock, commands, stops = motion
    r.pose, r.pose_stamp = (1., 0., 0.), 99.
    with pytest.raises(RuntimeError, match='时间戳过期'):
        r.goto(1., 0., timeout=2.)
    assert stops and not commands


def test_independent_raw_odometry_does_not_block_valid_navigation_pose(motion):
    r, clock, commands, stops = motion
    r.pose = (1., 0., 0.); r.odom_stamp = 99.
    assert r.goto(1., 0., timeout=2.)
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


def test_goal_does_not_wait_for_measured_angular_speed(motion):
    r, clock, commands, stops = motion
    def spin(seconds):
        clock[0] += seconds
        sample(r, clock, (1., 0., clock[0]))
    r.spin = spin
    assert r.goto(1., 0., timeout=.01)
    assert clock[0] == pytest.approx(.05)
    assert stops and not commands


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
