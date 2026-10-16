"""Exercise production status/admission logic; subprocesses are harmless Python prints."""
import json
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).parents[3]/'tools'))
from pass_health import PassHealth
from pass_console import PassRuns, make_app


def healthy(clock=lambda: 100.):
    h = PassHealth(clock=clock, can_present=lambda: True)
    h.observe_session(json.dumps(dict(ready=True, reason='stable')))
    h.observe_chassis(json.dumps(dict(ready=True, dry_run=False, reason='armed')))
    h.observe_ready(True)
    h.observe_localization(True)
    return h


@pytest.mark.parametrize('side', ['A', 'B'])
@pytest.mark.parametrize('number', [1, 2, 3])
def test_default_app_rejects_every_round_until_real_status_arrives(monkeypatch, side, number):
    import pass_console
    monkeypatch.setattr(pass_console.subprocess, 'Popen', lambda *a, **k: pytest.fail('must not run a route'))
    client = make_app().test_client()
    result = client.post('/api/pass/run', json=dict(side=side, round=number))
    assert result.status_code == 409
    assert not result.json['hardware']['can_run']
    assert '雷达' in result.json['error'] and '底盘' in result.json['error']
    assert client.get('/api/pass/state').json['status'] == 'idle'


@pytest.mark.parametrize('lidar,chassis', [(False, True), (True, False), (False, False), (True, True)])
def test_both_devices_required_and_reasons_are_returned(lidar, chassis):
    h = healthy()
    h.observe_session(json.dumps(dict(ready=lidar, reason='等待PTP同步')))
    h.observe_chassis(json.dumps(dict(ready=chassis, dry_run=False, reason='四路电调反馈不齐')))
    state = h.snapshot()
    assert state['can_run'] is (lidar and chassis)
    if not lidar: assert 'PTP' in state['lidar']['reason']
    if not chassis: assert '电调反馈不齐' in state['chassis']['reason']


def test_live_node_without_recent_board_ack_cannot_authorize_a_new_round():
    h = healthy()
    # /chassis/ready remains true under the no-motion-timeout policy.
    h.observe_chassis(json.dumps(dict(ready=False, dry_run=False, reason='大疆板状态应答中断')))
    assert not h.snapshot()['can_run']
    assert '应答中断' in h.snapshot()['chassis']['reason']


def test_status_staleness_and_recovery_never_call_motion_or_stop():
    now = [100.]
    h = healthy(lambda: now[0])
    assert h.snapshot()['can_run']
    now[0] = 103
    assert not h.snapshot()['can_run']
    assert '更新已中断' in h.snapshot()['lidar']['reason']
    h.observe_session(json.dumps(dict(ready=True)))
    h.observe_chassis(json.dumps(dict(ready=True, dry_run=False)))
    h.observe_ready(True)
    h.observe_localization(True)
    assert h.snapshot()['can_run']


def test_dry_run_missing_can_and_missing_localization_are_not_healthy():
    h = healthy()
    h.observe_chassis(json.dumps(dict(ready=True, dry_run=True)))
    assert '模拟模式' in h.snapshot()['chassis']['reason']
    h = healthy(); h.can_present = lambda: False
    assert 'CAN适配器' in h.snapshot()['chassis']['reason']
    h = healthy(); h.observe_localization(False)
    assert '定位确认' in h.snapshot()['lidar']['reason']
    assert not h.snapshot()['can_run']


def test_link_loss_and_monitor_error_override_previous_healthy_samples():
    h = healthy()
    assert not h.snapshot(dict(power=dict(carrier=False)))['can_run']
    assert '网口未连接' in h.snapshot(dict(power=dict(carrier=False)))['lidar']['reason']
    h.set_error('ROS订阅失败')
    assert not h.snapshot()['can_run']
    assert 'ROS订阅失败' in h.snapshot()['chassis']['reason']


@pytest.mark.parametrize('text', ['broken JSON', '[]', '{"ready":1}', '{"ready":"true"}'])
def test_invalid_session_data_revokes_admission(text):
    h = healthy(); h.observe_session(text)
    assert not h.snapshot()['can_run']
    assert '数据无效' in h.snapshot()['lidar']['reason']


@pytest.mark.parametrize('text', ['{}', '{"ready":true}', '{"ready":true,"dry_run":0}'])
def test_old_or_invalid_chassis_status_is_not_trusted(text):
    h = healthy(); h.observe_chassis(text)
    assert not h.snapshot()['can_run']


def test_health_api_and_409_share_the_same_concrete_reason():
    h = healthy(); h.observe_ready(False)
    client = make_app(health=h).test_client()
    result = client.post('/api/pass/run', json=dict(side='B', round=2))
    assert result.status_code == 409
    assert result.json['hardware'] == client.get('/api/hardware/state').json
    assert result.json['hardware'] == client.get('/api/pass/state').json['hardware']
    assert client.get('/api/pass/state').headers['Cache-Control'] == 'no-store'


def test_healthy_equipment_allows_only_the_fake_command():
    runs = PassRuns(lambda side, number: [sys.executable, '-c', 'print("simulated route")'])
    client = make_app(runs, health=healthy()).test_client()
    result = client.post('/api/pass/run', json=dict(side='A', round=1))
    assert result.status_code == 202
    deadline = time.monotonic()+5
    while runs.snapshot()['status'] == 'running' and time.monotonic() < deadline: time.sleep(.01)
    assert runs.snapshot()['status'] == 'succeeded'
    assert 'simulated route' in runs.snapshot()['lines']


def test_fault_during_route_handoff_is_checked_again_before_spawn():
    h = healthy()
    runs = PassRuns(lambda side, number: [sys.executable, '-c', 'print("must not execute")'])
    client = make_app(runs, health=h).test_client()
    with runs.execution_lock:
        response = client.post('/api/pass/run', json=dict(side='A', round=3))
        assert response.status_code == 202
        h.observe_ready(False)
    deadline = time.monotonic()+5
    while runs.snapshot()['status'] == 'running' and time.monotonic() < deadline: time.sleep(.01)
    assert runs.snapshot()['status'] == 'failed'
    assert runs.runs[response.json['id']]['process'] is None
    assert any('尚未收到底盘就绪确认' in line for line in runs.snapshot()['lines'])
