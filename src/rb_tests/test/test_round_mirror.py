"""A/B 基准镜像及 B 专用现场校准；无 ROS、无硬件。"""
import math
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from match_round_route import RoundRoute, load_config, mirror_to_b, search_plan, wrap
import match_round_steps as stages


def baseline_config(side):
    a = load_config(start_point='A')
    return mirror_to_b(a) if side == 'B' else a


def test_explicit_a_overrides_saved_b_selection(tmp_path):
    config = Path(__file__).resolve().parents[3] / 'tools' / 'match_rounds.json'
    raw = json.loads(config.read_text())
    raw['start_point'] = 'B'
    path = tmp_path / 'rounds.json'
    path.write_text(json.dumps(raw))
    selected = load_config(path, start_point='A')
    assert selected['start_point'] == 'A'
    assert selected['start'] == raw['start']
    assert search_plan(selected, 'middle', round_number=1)[0][1] == pytest.approx(-math.pi/2)


@pytest.mark.parametrize('task', ['pass', 'shoot'])
@pytest.mark.parametrize('number', [1, 2, 3])
@pytest.mark.parametrize('cycle', [0, 1])
@pytest.mark.parametrize('area', ['middle', 'arc', 'inner'])
def test_search_points_and_headings_are_exact_reflections(task, number, cycle, area):
    a = search_plan(baseline_config('A'), area, cycle, number, task)
    b = search_plan(baseline_config('B'), area, cycle, number, task)
    assert len(a) == len(b)
    for ((ax, ay), ah), ((bx, by), bh) in zip(a, b):
        assert (bx, by) == pytest.approx((ax, -ay))
        assert wrap(ah + bh) == pytest.approx(0.)


def runner(origin):
    r = NS(pose=origin, args=NS(route_intake=False, speed=.5, step_tol=.02, max_ang=1.))
    r.events = []
    def goto(x, y, **kw):
        r.events.append(('move', x, y))
        r.pose = (x, y, r.pose[2])
        return True
    def turn(yaw, **kw):
        r.events.append(('turn', yaw))
        r.pose = (*r.pose[:2], yaw)
        return True
    r.goto, r.rotate_to = goto, turn
    r.stop = lambda: None
    r.spin = lambda seconds: r.events.append(('wait', seconds))
    return r


@pytest.mark.parametrize('side,sign', [('A', -1), ('B', 1)])
@pytest.mark.parametrize('number', [1, 2, 3])
@pytest.mark.parametrize('mode', ['cycle', 'drive', 'collect'])
def test_all_entry_modes_turn_correctly_after_ten_seconds(tmp_path, monkeypatch, side, sign, number, mode):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(start_point=side)
    r = runner((12., -7., .37))
    # 入场后结束测试，不发送真实运动或机构命令。
    monkeypatch.setattr(RoundRoute, 'move', lambda *a, **k: False)
    assert not RoundRoute(r, cfg, number, 'pass', mode, emit=lambda m: None).execute()
    assert sum(e[1] for e in r.events if e[0] == 'wait') == pytest.approx(10.)
    assert next(e[1] for e in r.events if e[0] == 'turn') == pytest.approx(wrap(.37 + sign*math.pi/2))
    assert all(e[0] == 'wait' for e in r.events[:10])


@pytest.mark.parametrize('side,sign', [('A', 1), ('B', -1)])
@pytest.mark.parametrize('origin', [(0., 0., 0.), (12., -7., .63)])
@pytest.mark.parametrize('position,target_x,expected', [
    ((3.55, 1.2), 11., [4.8, 6.4]),
    ((1.5, 'corridor'), 11., [6.4]),
    ((1.5, 'outside'), 11., [6.4]),
    ((8.3, 1.2), 3.55, [6.4, 4.8]),
    ((3.55, 1.2), 2., []),
])
def test_crossing_uses_nominal_coordinates_and_correct_side(side, sign, origin, position, target_x, expected):
    cfg = load_config(start_point=side)
    r = runner(origin)
    plan = RoundRoute(r, cfg, 3, 'pass', 'collect', emit=lambda m: None)
    corridor_y = cfg['corridor'][1]
    y = (corridor_y if position[1] == 'corridor' else
         corridor_y + sign*.15 if position[1] == 'outside' else sign*position[1])
    r.pose = (*plan.frame.point((position[0], y)), origin[2])
    actual = plan.crossing_via(target_x)
    assert len(actual) == len(expected)
    for p, x in zip(actual, expected):
        assert p == pytest.approx((x, corridor_y))


@pytest.mark.parametrize('task', ['pass', 'shoot'])
def test_release_and_home_remain_reflections(task):
    paths = []
    for side in ('A', 'B'):
        cfg = baseline_config(side)
        r = runner((*cfg['start'], 0.))
        plan = RoundRoute(r, cfg, 1, task, 'collect', emit=lambda m: None)
        assert plan.action_point()
        assert plan.home()
        paths.append(r.events)
    assert len(paths[0]) == len(paths[1])
    for a, b in zip(*paths):
        assert a[0] == b[0]
        if a[0] == 'move':
            assert b[1:] == pytest.approx((a[1], -a[2]))
        elif a[0] == 'turn':
            assert wrap(a[1]+b[1]) == pytest.approx(0.)


@pytest.mark.parametrize('number', [1, 2, 3])
@pytest.mark.parametrize('task', ['pass', 'shoot'])
def test_complete_cycles_mirror_motion_and_keep_serial_commands(tmp_path, monkeypatch, number, task):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    paths = []
    for side in ('A', 'B'):
        cfg = baseline_config(side)
        origin = (12., -7., .37)
        r = runner(origin)
        r.args.route_intake = True
        r.args.route_heading_tol = .1
        r.args.route_scoop_speed = .08
        r.args.route_scoop_distance = .59
        r.args.route_scoop_timeout = 15.
        def scan(x, y, heading, **kw):
            r.events.append(('scan', x, y, heading))
            r.pose = (x, y, heading)
            return 'found', NS(bearing_rad=0.)
        def scoop(approach=True):
            r.events.append(('scoop', approach))
            if approach:
                x, y, yaw = r.pose
                d = r.args.route_scoop_distance
                r.pose = (x-d*math.cos(yaw), y-d*math.sin(yaw), yaw)
            return True
        def send(command):
            r.events.append(('text', command))
            return True, 'simulated'
        def mechanism_release(commands, waits, default_wait):
            # 兼容新版 Runner 将串口出球封装成一个方法的接口。
            for index, command in enumerate(commands):
                send(command)
                wait = waits[index] if index < len(waits) else default_wait
                if wait:
                    r.spin(wait)
            return True, 'simulated'
        r.scan_to, r.rear_visual_scoop = scan, scoop
        r.aligned_intake_approach, r.intake_text = scoop, send
        r.mechanism_release = mechanism_release
        plan = RoundRoute(r, cfg, number, task, 'cycle', emit=lambda m: None)
        assert plan.execute()
        assert r.pose == pytest.approx(origin)
        assert stages.load_session(number, task)['next_step'] == 4
        normalized = []
        for e in r.events:
            if e[0] in ('move', 'scan'):
                x, y = plan.frame.nominal(e[1:3])
                normalized.append((e[0], x, y))
                if e[0] == 'scan':
                    normalized.append(('turn', wrap(e[3]-origin[2])))
            elif e[0] == 'turn':
                normalized.append(('turn', wrap(e[1]-origin[2])))
            else:
                normalized.append(e)
        paths.append(normalized)
        assert [e[1] for e in r.events if e[0] == 'text'] == cfg['release'][task]['commands']*2
    assert len(paths[0]) == len(paths[1])
    for a, b in zip(*paths):
        assert a[0] == b[0]
        if a[0] in ('move', 'scan'):
            assert b[1:] == pytest.approx((a[1], -a[2]))
        elif a[0] == 'turn':
            assert wrap(a[1]+b[1]) == pytest.approx(0.)
        else:
            assert a == b


def test_concurrent_session_writes_use_distinct_temporary_paths(tmp_path,monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    monkeypatch.setattr(stages,'STATE_DIR',tmp_path)
    def write(index):
        stages.save_session(dict(version=2,round=2,task='pass',start_pose=[1.,2.,0.],marker=index))
    with ThreadPoolExecutor(max_workers=8) as pool:list(pool.map(write,range(80)))
    assert stages.load_session(2,'pass')['marker'] in range(80)
    assert not list(tmp_path.glob('*.tmp'))


def test_production_b_adjustments_only_shift_middle_and_pass():
    a = load_config(start_point='A')
    b = load_config(start_point='B')
    expected = mirror_to_b(a)
    expected['middle']['y'] += .24
    expected['release']['pass']['point'][1] += .24
    assert b == expected
    assert a['middle']['y'] == pytest.approx(.9)
    assert a['release']['pass']['point'] == pytest.approx([3.616, 1.275])
    assert b['middle']['y'] == pytest.approx(-.66)
    assert b['release']['pass']['point'] == pytest.approx([3.616, -1.035])
    for task in ('pass', 'shoot'):
        for number in (1, 2, 3):
            for cycle in (0, 1):
                for area in ('middle', 'arc', 'inner'):
                    ap = search_plan(a, area, cycle, number, task)
                    bp = search_plan(b, area, cycle, number, task)
                    assert len(ap) == len(bp)
                    for ((ax, ay), ah), ((bx, by), bh) in zip(ap, bp):
                        assert (bx, by) == pytest.approx((ax, -ay + (.24 if area == 'middle' else 0.)))
                        assert wrap(ah + bh) == pytest.approx(0.)


def test_missing_b_adjustments_preserves_pure_mirror(tmp_path):
    config = Path(__file__).resolve().parents[3] / 'tools' / 'match_rounds.json'
    raw = json.loads(config.read_text())
    raw.pop('b_y_adjustments_m', None)
    path = tmp_path / 'rounds.json'
    path.write_text(json.dumps(raw))
    assert load_config(path, start_point='B') == mirror_to_b(load_config(path, start_point='A'))
