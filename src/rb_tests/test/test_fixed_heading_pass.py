"""传球固定角度、三个回合完整流程和后备搜索；不连接ROS或硬件。"""
import json
import math
from pathlib import Path
import shutil
import subprocess

import pytest

from test_match_round_route import fake_runner
from match_round_route import RoundRoute, StartFrame, load_config, validate_mode
import match_round_steps as stages


@pytest.mark.parametrize('number', [1, 2, 3])
@pytest.mark.parametrize('heading', [49., -31.5])
def test_formal_pass_two_releases_use_configured_start_relative_heading_and_return_home(
        tmp_path, monkeypatch, number, heading):
    import match_round_route
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    monkeypatch.setattr(match_round_route, 'align_pillar',
                        lambda *a, **kw: pytest.fail('固定角度不调用视觉对柱'))
    cfg = load_config()
    cfg['release']['pass']['search_heading_deg'] = heading
    r, events = fake_runner(); origin = r.pose; r.args.route_intake = True
    launch_yaws = []
    def send(command):
        events.append(('text', command))
        if command == 'vesc_shoot 3':
            launch_yaws.append(r.pose[2])
        return True, 'mock'
    r.intake_text = send
    assert RoundRoute(r, cfg, number, 'pass', 'cycle', emit=lambda msg: None).execute()
    assert launch_yaws == pytest.approx([StartFrame(origin, cfg['start']).yaw(math.radians(heading))]*2)
    commands = [e[1] for e in events if e[0] == 'text']
    assert commands == cfg['release']['pass']['commands']*2
    assert len([e for e in events if e[0] == 'scoop']) == (1 if number == 1 else 2)
    assert r.pose == pytest.approx(origin)
    assert stages.workflow_state(number, 'pass')['finished']


def test_fixed_heading_rotation_failure_stops_before_release(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); r, events = fake_runner(); r.args.route_intake = True
    expected = StartFrame(r.pose, cfg['start']).yaw(math.radians(49))
    turn = r.rotate_to
    r.rotate_to = lambda yaw, **kw: False if abs(yaw-expected) < 1e-6 else turn(yaw, **kw)
    assert not RoundRoute(r, cfg, 1, 'pass', 'cycle', emit=lambda msg: None).execute()
    assert not [e for e in events if e[0] == 'text']
    assert stages.workflow_state(1, 'pass')['next_step'] == 1
    assert events[-1] == ('stop',)


def test_coordinate_target_mode_still_requires_aim_and_visual_alignment(monkeypatch):
    cfg = load_config(); cfg['release']['pass']['aim_mode'] = 'target'
    with pytest.raises(ValueError, match='aim'):
        validate_mode(cfg, 'pass', 'cycle')
    cfg['release']['pass']['aim'] = [5., 7.4]
    r, events = fake_runner(); r.args.route_intake = True
    import match_round_route
    monkeypatch.setattr(match_round_route, 'align_pillar', lambda *a, **kw: False)
    plan = RoundRoute(r, cfg, 2, 'pass', 'cycle', emit=lambda msg: None)
    assert not plan.action_point()
    assert not plan.release()
    assert not [e for e in events if e[0] == 'text']


@pytest.mark.parametrize('number,areas', [(2, ['middle', 'arc']), (3, ['arc', 'inner'])])
@pytest.mark.parametrize('empty', [False, True])
def test_formal_route_falls_back_or_stops_without_fake_release(tmp_path, monkeypatch, number, areas, empty):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    r, events = fake_runner(); origin = r.pose; r.args.route_intake = False
    cfg = load_config(); seen = []; active = {}
    original = RoundRoute.search_area
    def search(self, area, cycle, search_only=False):
        active['area'] = area; seen.append((cycle, area))
        return original(self, area, cycle, search_only)
    monkeypatch.setattr(RoundRoute, 'search_area', search)
    def scan(x, y, heading, **kw):
        r.pose = (x, y, heading)
        return ('found' if not empty and active['area'] == areas[-1] else 'empty'), None
    r.scan_to = scan
    assert RoundRoute(r, cfg, number, 'pass', 'cycle', emit=lambda msg: None).execute() is (not empty)
    assert seen == [(cycle, area) for cycle in (range(1) if empty else range(2)) for area in areas]
    if empty:
        assert stages.workflow_state(number, 'pass')['next_step'] == 1
    else:
        assert r.pose == pytest.approx(origin)
    assert not [e for e in events if e[0] == 'text']


@pytest.mark.parametrize('number', [2, 3])
def test_round_shell_preflight_accepts_fixed_heading_without_target_coordinate(tmp_path, number):
    source = Path(__file__).resolve().parents[3] / 'tools'
    tools = tmp_path / 'tools'; tools.mkdir(); (tmp_path / 'install').mkdir()
    (tmp_path / 'install/setup.bash').write_text('')
    for name in (f'run_round{number}.sh', 'match_rounds.json', 'match_round_route.py', 'visual_target_alignment.py'):
        shutil.copy(source / name, tools / name)
    script = (source / 'run_round_test.sh').read_text().replace('source /opt/ros/humble/setup.bash', ':')
    (tools / 'run_round_test.sh').write_text(script)
    (tools / 'test_perimeter_ball.py').write_text('import sys,json\nprint(json.dumps(sys.argv[1:]))\n')
    result = subprocess.run(['bash', str(tools / f'run_round{number}.sh'), '--mechanism'],
                            capture_output=True, text=True, check=True)
    args = json.loads(result.stdout.splitlines()[-1])
    assert args[args.index('--route')+1] == f'round{number}'
    assert '--mechanism' in args
    assert args[len(args)-1-args[::-1].index('--round-release-kind')+1] == 'match'


def test_fixed_heading_release_retry_restores_angle_before_sending(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); r, events = fake_runner(); r.args.route_intake = True
    origin = r.pose
    plan = RoundRoute(r, cfg, 1, 'pass', 'collect', emit=lambda msg: None)
    assert not plan.release()  # 未到角度不能直接发射。
    r.intake_text = lambda command: (False, 'mock disconnected')
    assert not stages.execute_step(r, cfg, 1, 'pass', 1, 'match', emit=lambda msg: None)
    assert stages.load_session(1, 'pass')['active_action'] == 'release'
    r.pose = (*r.pose[:2], 0.)
    yaws = []
    r.intake_text = lambda command: yaws.append(r.pose[2]) or (True, 'mock')
    assert stages.execute_step(r, cfg, 1, 'pass', 1, 'match', emit=lambda msg: None)
    expected = StartFrame(origin, cfg['start']).yaw(math.radians(49))
    assert yaws == pytest.approx([expected]*3)
