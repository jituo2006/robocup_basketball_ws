"""第一回合分段提速，保留识球/取球速度与用户覆盖。无ROS或硬件。"""
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from test_match_round_route import fake_runner, console
from test_seek_observations import load as load_cli
from match_round_route import RoundRoute, load_config
import match_round_steps as stages


def instrument():
    r, events = fake_runner(); r.pose = (1.5, 8., 0.)
    moves, scans, scoops = [], [], []
    goto, scoop = r.goto, r.aligned_intake_approach
    def move(x, y, **kw):
        moves.append((x, y, r.args.speed))
        return goto(x, y, **kw)
    def scan(x, y, heading, **kw):
        scans.append(kw['speed'])
        return 'found', NS()
    def approach(*args, **kw):
        scoops.append(r.args.route_scoop_speed)
        return scoop(*args, **kw)
    r.goto, r.scan_to, r.aligned_intake_approach = move, scan, approach
    return r, events, moves, scans, scoops


def test_preloaded_round_uses_loaded_transit_fast_empty_transit_and_slow_scan_and_pickup(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); r, events, moves, scans, scoops = instrument()
    origin = r.pose
    assert RoundRoute(r, cfg, 1, 'pass', 'cycle', emit=lambda msg: None).execute()
    assert [p[2] for p in moves[:3]] == [.35]*3  # 预装球到传球点。
    assert [p[2] for p in moves[3:5]] == [.5]*2  # 首次传球后空车去中线。
    assert scans == [.25]
    assert scoops == [.08]
    assert moves[5][2] == .12  # 收球后先低速退出球位至检测停车点。
    assert [p[2] for p in moves[6:-2]] == [.35]*len(moves[6:-2])
    assert [p[2] for p in moves[-2:]] == [.5]*2
    assert r.args.speed == .25  # 临时选速没有污染全局/扫描速度。
    assert r.pose == pytest.approx(origin)
    assert stages.workflow_state(1, 'pass')['finished']


def test_loaded_speed_persists_when_each_stage_uses_new_runner(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); origin = (1.5,8.,0.); pose = origin
    for index in (1, 2, 3):
        r, events, moves, scans, scoops = instrument(); r.pose = pose
        assert stages.execute_step(r, cfg, 1, 'pass', index, release_kind='match', emit=lambda msg: None)
        if index == 1:
            assert {m[2] for m in moves} == {.35}
        if index == 2:
            assert moves[0][2] == .5 and moves[-1][2] == .35
        if index == 3:
            assert {m[2] for m in moves} == {.5}
        assert stages.load_session(1,'pass')['carrying'] is False
        pose = r.pose
    assert pose == pytest.approx(origin)


@pytest.mark.parametrize('number', [2, 3])
def test_other_rounds_preserve_original_speed(number, tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    r, events, moves, scans, scoops = instrument()
    assert RoundRoute(r, load_config(), number, 'pass', 'cycle', emit=lambda msg: None).execute()
    assert {m[2] for m in moves} == {.25} and scans == [.25, .25]
    assert scoops == [.08, .08]


def test_movement_failure_restores_previous_speed():
    r, events = fake_runner(); r.args.speed = .19
    def fail(*a, **kw):
        assert r.args.speed == .35
        raise RuntimeError('mock localization failure')
    r.goto = fail
    plan = RoundRoute(r, load_config(), 1, 'pass', 'collect')
    with pytest.raises(RuntimeError): plan.move((1.5,6.8))
    assert r.args.speed == .19


@pytest.mark.parametrize('number,extra,angular,radius', [
    (1, [], .4, .6), (2, [], .3, .45), (3, [], .3, .45),
    (1, ['--max-ang','.2','--approach-radius','.8','--approach-speed','.1'], .2, .8)])
def test_cli_profile_defaults_and_explicit_overrides_reach_runner(number, extra, angular, radius, tmp_path, monkeypatch):
    mod = load_cli(); captured = []
    monkeypatch.setattr(mod.subprocess, 'run', lambda *a, **kw: NS(returncode=0, stdout=''))
    class NoHardwareRunner:
        def __init__(self, args): captured.append(args)
        def wait_pose(self): return False
        def close(self): pass
    monkeypatch.setattr(mod, 'Runner', NoHardwareRunner)
    assert mod.main(['--route', f'round{number}', '--round-mode', 'cycle', '--no-mechanism',
                     '--dry', '--yes', '--log-file',str(tmp_path/'run.jsonl')] + extra) == 1
    args = captured[0]
    assert args.max_ang == angular and args.approach_radius == radius
    assert args.approach_speed == (.1 if extra else .12)
    assert args.route_scoop_speed == .08 and args.route_rear_align_speed == .04


@pytest.mark.parametrize('value', [0, -1, True, float('nan'), float('inf')])
def test_invalid_profile_values_are_rejected(tmp_path, value):
    cfg = load_config(); cfg['speed_profiles']['1']['travel_mps'] = value
    path = tmp_path/'config.json'; path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError): load_config(path)


def test_missing_speed_profile_keeps_legacy_config_usable(tmp_path):
    cfg = load_config(); cfg.pop('speed_profiles')
    path = tmp_path/'legacy.json'; path.write_text(json.dumps(cfg))
    cfg = load_config(path); r, events, moves, scans, scoops = instrument()
    plan = RoundRoute(r,cfg,1,'pass','collect')
    assert plan.move((1.5,6.8))
    assert moves[0][2] == .25


def test_console_reports_actual_segment_speeds_without_motion():
    mod=console(); unused=NS()
    client=mod.make_app(unused,unused,unused,unused,unused,unused,{}).test_client()
    cfg=client.get('/api/round-config').get_json()
    assert cfg['speed_profiles']['1']['travel_mps'] == .5
    assert cfg['speed_profiles']['1']['scan_mps'] == .25


# 使用生产goto与源时间戳反馈，检验快速段和终点减速均真正影响速度输出。
from test_goto_arrival import motion


def test_fast_goto_preserves_near_goal_deceleration_and_final_stop(motion):
    r, clock, commands, stops = motion
    r.args.speed = .5; r.args.approach_radius = .6; r.args.approach_speed = .12
    positions = iter([0., .2, .5, .8, .99])
    def spin(seconds):
        clock[0] += seconds
        r.pose = (next(positions, .99), 0., 0.)
        r.pose_stamp = 100.+clock[0]
    r.spin = spin
    assert r.goto(1., 0., timeout=.01)
    assert [vx for vx,vy,wz in commands] == pytest.approx([.5, .5, .12, .12])
    assert all(vy == 0 and wz == 0 for vx,vy,wz in commands)
    assert stops[-1] == clock[0]
