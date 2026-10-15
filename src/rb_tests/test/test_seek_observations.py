"""离线控制回归：源时间戳、目标确认、观测失效停车，不启动 ROS 或设备。"""
import importlib.util
import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

WS = Path(__file__).resolve().parents[3]


def load():
    spec = importlib.util.spec_from_file_location('seek_test', WS / 'tools/test_perimeter_ball.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def detection(t, bearing=0.02, distance=1.0, confidence=0.8):
    return NS(stamp=NS(sec=int(t), nanosec=round((t-int(t))*1e9)),
              label='ball_basketball', source='camera', confidence=confidence,
              bearing_rad=bearing, distance_m=distance)


def runner(mod, now=100.0):
    run = mod.Runner.__new__(mod.Runner)
    clock = NS(t=now)
    run.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=round(clock.t*1e9))))
    run.args = NS(ball_label='ball_basketball', ball_confidence=0.5, detection_timeout=0.5,
                  pose_match_gap=0.15, settle_seconds=0.3, settle_yaw_rate=0.08,
                  stable_frames=3, track_bearing_deg=6.0, track_distance=0.25,
                  cam_yaw_offset=0.0, step_tol=0.02)
    run.pose = (0.0, 0.0, 0.0)
    run.pose_stamp = 100.0
    run.pose_history = deque([(99.7, (0.0, 0.0, 0.0)), (99.8, (0.0, 0.0, 0.0))])
    run.yaw_rate = 0.0
    run.stop = lambda: None
    run.spin = lambda sec: setattr(clock, 't', clock.t+sec)
    return run, clock


def test_pose_interpolation_wraps_yaw_and_refuses_extrapolation():
    mod = load()
    history = [(1.0, (0, 0, math.radians(179))), (1.1, (1, 2, math.radians(-179)))]
    x, y, yaw = mod.pose_at(history, 1.05)
    assert (x, y) == pytest.approx((0.5, 1.0))
    assert abs(abs(yaw)-math.pi) < 1e-8
    assert mod.pose_at(history, 1.2) is None
    assert mod.pose_at(history, 1.05, max_gap=0.05) is None


@pytest.mark.parametrize('stamp,confidence,source', [(98, .9, 'camera'), (99.8, .2, 'camera'), (99.8, .9, 'lidar')])
def test_array_receipt_cannot_make_invalid_detection_usable(stamp, confidence, source):
    mod = load()
    run, _ = runner(mod)
    run.det_stamp = run.last_det_time = 100.0
    det = detection(stamp, confidence=confidence)
    det.source = source
    run.dets = [det]
    assert run.best_ball() is None


def test_world_ball_uses_exposure_pose_instead_of_latest_pose():
    mod = load()
    run, _ = runner(mod)
    run.pose = (10, 10, math.pi/2)
    run.dets = [detection(99.8, bearing=0.0, distance=2.0)]
    assert run.ball_field_xy(run.best_ball()) == pytest.approx((2.0, 0.0))


def test_missing_pose_bracket_rejects_detection():
    mod = load()
    run, _ = runner(mod)
    run.dets = [detection(99.9)]
    assert run.best_ball() is None


def measurement(sequence):
    mod = load()
    run, clock = runner(mod)
    queue = iter(sequence)
    run.observation_pose = lambda det: (0.0, 0.0, 0.0)
    def spin(sec):
        clock.t += sec
        run.current = next(queue, run.current) if sec == 0.05 else run.current
    run.current = None
    run.spin = spin
    run.best_ball = lambda: run.current
    result = run.measure_ball_stable(timeout=1.0)
    return result


def test_duplicate_cached_frames_cannot_confirm_target():
    det = detection(100.4)
    assert measurement([det]*20) is None


def test_confirmation_requires_three_independent_consistent_frames():
    result = measurement([detection(100.4), detection(100.5), detection(100.6)])
    assert result == pytest.approx((0.02, 1.0, 3))


def test_target_jump_discards_previous_confirmation():
    assert measurement([detection(100.4), detection(100.5),
                        detection(100.6, bearing=0.5)]) is None


def test_lost_visual_observation_stops_before_nonzero_command():
    mod = load()
    run, _ = runner(mod)
    run.visual_guard, run.motion_guard = True, False
    run.best_ball = lambda: None
    stops, sent = [], []
    run.stop = lambda: stops.append(True)
    run.cmd_pub = NS(publish=lambda msg: sent.append(msg))
    with pytest.raises(RuntimeError, match='观测失效'):
        run.publish_command(NS())
    assert stops == [True] and sent == []


def test_small_servo_step_uses_tight_tolerance_and_visual_guard():
    mod = load()
    run, _ = runner(mod)
    run.measure_ball_stable = lambda: (0.0, 1.0, 3)
    moves = []
    run.goto = lambda x, y, **kw: moves.append((x, y, kw, run.visual_guard)) or True
    run.servo_to_ball(0.7, 1.5, 0.05, 1, 0.12, 12)
    assert moves[0][:2] == pytest.approx((0.12, 0.0))
    assert moves[0][2]['tolerance'] == .02 and moves[0][3]
    assert not run.visual_guard


def test_invalid_cli_parameters_fail_before_process_or_device_actions():
    mod = load()
    for args in (['--route', 'legacy', '--servo-move', '-1'],
                 ['--route', 'legacy', '--stable-frames', '0'], ['--kd-yaw', 'nan']):
        with pytest.raises(SystemExit) as exc:
            mod.main(args)
        assert exc.value.code == 2


def test_rotation_finishes_at_angle_without_low_rate_or_dwell_gate(monkeypatch):
    mod = load()
    run, clock = runner(mod)
    run.args.yaw_tol, run.args.max_ang = .015, .3
    run.args.kp_yaw, run.args.kd_yaw = 2.0, .25
    ticks, speeds = [], []
    def spin(sec):
        clock.t += sec
        ticks.append(clock.t)
        if len(ticks) == 1:
            run.pose, run.yaw_rate = (0, 0, .2), 0
        else:
            run.pose, run.yaw_rate = (0, 0, .01), .4 if len(ticks) < 5 else 0
    monkeypatch.setattr(mod, 'time', NS(time=lambda: clock.t, monotonic=lambda: clock.t,
                                        strftime=lambda *a: 'test'))
    run.spin = spin
    run._Twist = lambda: NS(angular=NS(z=0))
    run.publish_command = lambda msg: speeds.append(msg.angular.z)
    assert run.rotate_to(0, timeout=2)
    assert clock.t == pytest.approx(100.1)  # 角度到位即停，不等待低转速和额外稳定时间。
    assert speeds and max(abs(v) for v in speeds) <= .3
