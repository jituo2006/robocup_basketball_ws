"""实车日志对应的首球误解、串口失败卡阶段和纯运动联调回归。"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS / 'tools'))
from match_round_route import load_config, RoundRoute
import match_round_steps as stages
from test_match_round_route import fake_runner, console


def test_ground_first_ball_reaches_right_endpoint_then_turns_and_approaches(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); cfg['round1_first_ball'] = 'search'
    r, events = fake_runner(); r.pose = (1.5, 8., 0.)
    def detect(dets):
        assert r.pose == pytest.approx((4.4, 4.8, -math.pi/2))
        events.append(('detect',)); return NS()
    r.centered_target_candidates = detect
    assert stages.execute_step(r, cfg, 1, 'pass', 1, emit=lambda msg: None)
    assert events.index(('move', 4.4, 4.8)) < events.index(('detect',))
    assert events.index(('detect',)) < events.index(('turn', math.pi/2))
    assert ('align',) in events
    assert ('scoop', 'ball_basketball', .45) in events
    assert stages.workflow_state(1, 'pass')['next_step'] == 2


@pytest.mark.parametrize('number', [1, 2, 3])
def test_three_phases_with_actuators_disabled_all_finish_without_serial(tmp_path, monkeypatch, number):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); cfg['round1_first_ball'] = 'search'
    r, events = fake_runner(); origin = r.pose
    for index in (1, 2, 3):
        current = r.pose; r, events = fake_runner(); r.pose = current
        r.args.route_intake = False
        r.intake_text = lambda command: pytest.fail('明确关闭机构时不发串口指令')
        assert stages.execute_step(r, cfg, number, 'pass', index, emit=lambda msg: None)
        assert stages.workflow_state(number, 'pass')['next_step'] == index+1
    assert r.pose == pytest.approx(origin)


def test_serial_failure_reason_is_visible_and_retry_only_retries_release(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); r, events = fake_runner()
    r.intake_text = lambda command: (False, '串口未打开')
    assert not stages.execute_step(r, cfg, 1, 'pass', 1, emit=lambda msg: None)
    state = stages.workflow_state(1, 'pass')
    assert state['next_step'] == 1 and '串口未打开' in state['last_error']
    r.goto = lambda *args, **kwargs: pytest.fail('失败重试放球不应重复走入场路线')
    r.intake_text = lambda command: (True, 'sent')
    assert stages.execute_step(r, cfg, 1, 'pass', 1, emit=lambda msg: None)
    assert not stages.workflow_state(1, 'pass')['last_error']


def test_movement_exception_is_saved_as_phase_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); r, events = fake_runner()
    def fail(*args, **kwargs): raise RuntimeError('定位源数据过期')
    r.rotate_to = fail
    assert not stages.execute_step(r, cfg, 1, 'pass', 1, emit=lambda msg: None)
    assert '定位源数据过期' in stages.workflow_state(1, 'pass')['last_error']


def test_reset_and_first_ball_choice_reach_api_and_cli(tmp_path, monkeypatch):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(); r, events = fake_runner()
    assert stages.execute_step(r, cfg, 1, 'pass', 1, emit=lambda msg: None)
    mod = console(); calls=[]; unused=NS()
    tool=NS(running=False, start=lambda key,args: calls.append(args) or (True,'started'))
    client=mod.make_app(unused,unused,tool,unused,unused,unused,{}).test_client()
    assert client.post('/api/round-reset',json={'round':1,'task':'pass'}).get_json()['ok']
    assert client.post('/api/action',json={'name':'TEST_ROUND1','args':{'step':1,'first_ball':'search'}}).get_json()['ok']
    assert calls[0][calls[0].index('--round-first-ball')+1] == 'search'
    response=client.get('/api/round-state?task=pass&first_ball=search').get_json()
    assert '取第1球' in response['rounds'][0]['steps'][0]['label']


def test_formal_round1_rejects_ground_first_ball_before_moving():
    cfg=load_config(); cfg['round1_first_ball']='search'
    cfg['release']['shoot']['commands']=['TEST_RELEASE']
    r, events=fake_runner()
    with pytest.raises(ValueError,match='首球须预装'):
        RoundRoute(r,cfg,1,'shoot','cycle')
    assert events == []
