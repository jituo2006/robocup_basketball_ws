"""整轮与阶段共用动作、两次取球后回位、失败不跳过；全部为模拟调用。"""
import math
import subprocess
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from test_match_round_route import fake_runner, console
from match_round_route import RoundRoute, load_config
import match_round_steps as stages

WS = Path(__file__).resolve().parents[3]


def test_round1_pickup_stops_in_passing_zone_with_intake_held_and_no_release(monkeypatch):
    cfg = load_config(); cfg['round1_first_ball'] = 'search'
    r, events = fake_runner(); r.pose = (1.5, 8., 0.)
    def detect(dets, centered=True):
        assert not centered
        assert r.pose == pytest.approx((1.5, 4.8, -math.pi/2))
        return NS()
    r.centered_target_candidates = detect
    import match_round_route
    monkeypatch.setattr(match_round_route, 'align_pillar', lambda *a, **k: pytest.fail('先测取球到位，不等待柱对准'))
    assert RoundRoute(r, cfg, 1, 'pass', 'collect', release_kind='drop', emit=lambda m: None).execute()
    assert r.pose[:2] == pytest.approx((3.6, 4.5))
    assert ('scoop', 'ball_basketball', .70) in events
    assert r.keep_intake_after_scoop
    assert not any(e[0] == 'text' for e in events)
    assert events[-1] == ('stop',)


@pytest.mark.parametrize('number', [1, 2, 3])
@pytest.mark.parametrize('task,label', [('pass', 'ball_basketball'), ('shoot', 'ball_volleyball')])
def test_whole_round_handles_two_targets_then_returns_to_original_pose(tmp_path, monkeypatch, number, task, label):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); cfg['round1_first_ball'] = 'search'
    r, events = fake_runner(); origin = r.pose; messages = []
    assert RoundRoute(r, cfg, number, task, 'cycle', release_kind='drop', emit=messages.append).execute()
    scoops = [e for e in events if e[0] == 'scoop']
    assert len(scoops) == 2 and all(e[1] == label for e in scoops)
    assert events.count(('text', 'MOVE 1 0 5')) == 2
    assert r.pose == pytest.approx(origin)
    state = stages.load_session(number, task)
    assert state['start_pose'] == list(origin) and state['next_step'] == 4
    assert not state['last_error'] and events[-1] == ('stop',)
    assert any('整轮进度 3/3' in message for message in messages)
    assert not any('等待下一阶段按钮' in message for message in messages)
    if number == 3:
        # 第3轮直接去三分区，不能进入中线置球区。
        middle_start = RoundRoute(r, cfg, number, task, 'collect', release_kind='drop').frame.point((4.4, 4.8))
        assert not any(e[0] == 'move' and e[1:] == middle_start for e in events)


def test_whole_and_three_buttons_execute_identical_movements(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); cfg['round1_first_ball'] = 'search'
    whole, whole_events = fake_runner()
    assert RoundRoute(whole, cfg, 2, 'pass', 'cycle', release_kind='drop', emit=lambda m: None).execute()
    stages.clear_session(2, 'pass')
    stepped, step_events = fake_runner()
    for index in (1, 2, 3):
        assert stages.execute_step(stepped, cfg, 2, 'pass', index, emit=lambda m: None)
    kinds = ('move', 'turn', 'align', 'scoop', 'text')
    assert [e for e in whole_events if e[0] in kinds] == [e for e in step_events if e[0] in kinds]


def test_second_release_failure_does_not_report_complete_or_run_home(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); cfg['round1_first_ball'] = 'search'
    r, events = fake_runner(); messages = []; calls = []
    def send(command):
        calls.append(command)
        return (True, 'sent') if len(calls) == 1 else (False, '串口未打开')
    r.intake_text = send
    monkeypatch.setattr(RoundRoute, 'home', lambda self: pytest.fail('第二次放球失败不能直接回位'))
    assert not RoundRoute(r, cfg, 2, 'pass', 'cycle', release_kind='drop', emit=messages.append).execute()
    state = stages.workflow_state(2, 'pass')
    assert state['next_step'] == 2 and not state['finished']
    assert '串口未打开' in state['last_error']
    assert any('整轮在第2阶段停止' in message for message in messages)


def test_first_competition_round_can_skip_all_mechanisms_but_complete_entire_route(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    import match_round_route
    monkeypatch.setattr(match_round_route, 'align_pillar', lambda *a, **k: pytest.fail('机构关闭，不等发射柱对准'))
    cfg = load_config(); r, events = fake_runner(); origin = r.pose
    r.args.route_intake = False
    r.intake_text = lambda command: pytest.fail('机构开关关闭，不能下发任何机构指令')
    assert RoundRoute(r, cfg, 1, 'pass', 'cycle', release_kind='match', emit=lambda m: None).execute()
    assert len([e for e in events if e[0] == 'scoop']) == 1  # 首球预装，只找第二球。
    assert len([e for e in events if e[0] == 'align']) == 1
    assert r.pose == pytest.approx(origin)
    assert stages.workflow_state(1, 'pass')['finished']
    assert not [e for e in events if e[0] == 'text']


def test_first_competition_button_with_mechanism_off_accepts_unconfigured_release():
    mod = console(); calls = []; unused = NS()
    tool = NS(start=lambda key, args: calls.append(args) or (True, 'started'))
    client = mod.make_app(unused, unused, tool, unused, unused, unused, {}).test_client()
    args = {'whole_round': True, 'first_ball': 'preloaded', 'release_kind': 'match', 'start_intake': False}
    assert client.post('/api/action', json={'name': 'TEST_ROUND1', 'args': args}).get_json()['ok']
    assert '--no-route-intake' in calls[0]
    assert calls[0][calls[0].index('--round-first-ball')+1] == 'preloaded'
    args['start_intake'] = True
    assert not client.post('/api/action', json={'name': 'TEST_ROUND1', 'args': args}).get_json()['ok']


@pytest.mark.parametrize('number', [1, 2, 3])
def test_whole_button_forces_cycle_even_if_old_single_ball_mode_selected(number):
    mod = console(); calls = []; unused = NS()
    tool = NS(start=lambda key, args: calls.append((key, args)) or (True, 'started'))
    client = mod.make_app(unused, unused, tool, unused, unused, unused, {}).test_client()
    response = client.post('/api/action', json={'name': f'TEST_ROUND{number}',
        'args': {'mode': 'collect', 'whole_round': True}}).get_json()
    assert response['ok']
    key, args = calls[0]
    assert key == f'TEST_ROUND{number}' and args[args.index('--round-mode')+1] == 'cycle'
    assert '--round-step' not in args


def test_whole_action_rejects_accidental_single_stage():
    mod = console(); unused = NS()
    tool = NS(start=lambda *args: pytest.fail('冲突入口不能启动运动'))
    client = mod.make_app(unused, unused, tool, unused, unused, unused, {}).test_client()
    response = client.post('/api/action', json={'name': 'TEST_ROUND2',
        'args': {'whole_round': True, 'step': 1}}).get_json()
    assert not response['ok'] and '不能指定单个阶段' in response['msg']


def test_startup_shell_dispatches_selected_round_without_ros_or_hardware(tmp_path):
    root = tmp_path / 'workspace'; (root / 'tools').mkdir(parents=True)
    (root / 'install').mkdir(); (root / 'install/setup.bash').write_text('export ROS_DOMAIN_ID=99\n')
    script = (WS / 'tools/run_round_test.sh').read_text().replace('source /opt/ros/humble/setup.bash', ':')
    (root / 'tools/run_round_test.sh').write_text(script)
    # 用空 CLI 打印参数；不加载真实 ROS、不发布速度。
    (root / 'tools/test_perimeter_ball.py').write_text('import sys,os,json\nprint(json.dumps([sys.argv[1:],os.environ.get("ROS_DOMAIN_ID")]))\n')
    result = subprocess.run(['bash', str(root / 'tools/run_round_test.sh'), '3', 'shoot',
        '--round-first-ball', 'preloaded', '--no-route-intake'], text=True, capture_output=True, check=True)
    import json
    args, domain = json.loads(result.stdout.splitlines()[-1])
    assert domain == '2' and args[args.index('--route')+1] == 'round3'
    assert args[args.index('--round-task')+1] == 'shoot'
    assert args[args.index('--round-mode')+1] == 'cycle'
    assert '--no-route-intake' in args and '--round-step' not in args
    pickup = subprocess.run(['bash', str(root / 'tools/run_round_test.sh'), '1', 'pass', '--pickup-only'],
                            text=True, capture_output=True, check=True)
    pickup_args, _ = json.loads(pickup.stdout.splitlines()[-1])
    assert pickup_args[pickup_args.index('--round-mode')+1] == 'collect'
    assert '--pickup-only' not in pickup_args
    assert subprocess.run(['bash', str(root / 'tools/run_round_test.sh'), '2', '--round-step', '1'],
                          capture_output=True).returncode == 2
    first_script = root / 'tools/run_first_round.sh'
    first_script.write_text((WS / 'tools/run_first_round.sh').read_text())
    first = subprocess.run(['bash', str(first_script)], text=True, capture_output=True, check=True)
    first_args, domain = json.loads(first.stdout.splitlines()[-1])
    def last_value(flag):
        return first_args[len(first_args)-1-first_args[::-1].index(flag)+1]
    assert last_value('--round-first-ball') == 'preloaded'
    assert last_value('--round-release-kind') == 'match'
    assert '--no-mechanism' in first_args
    enabled = subprocess.run(['bash', str(first_script), '--mechanism'], text=True, capture_output=True, check=True)
    enabled_args, _ = json.loads(enabled.stdout.splitlines()[-1])
    assert '--mechanism' in enabled_args and '--no-mechanism' not in enabled_args
