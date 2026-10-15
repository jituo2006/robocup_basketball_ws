"""Exercise intake sequencing and front-end actions without real actuators."""
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

WS=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(WS/'tools'))
from intake_control import start_intake, stop_intake


def console():
    spec=importlib.util.spec_from_file_location('intake_console',WS/'tools/console_app.py')
    mod=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_intake_sequence_waits_for_initialization_and_never_uses_binary_commands():
    events=[]
    ok,_=start_intake(lambda cmd:events.append(cmd) or (True,'sent'),
                      wait=lambda sec:events.append(sec))
    assert ok
    assert events==['dji_stop',.3,'dji_zero',.8,'dji_move 1 90 5']


def test_intake_sequence_stops_on_failed_initialization():
    events=[]
    def send(cmd):
        events.append(cmd)
        return cmd!='dji_zero','result'
    assert not start_intake(send,wait=lambda sec:None)[0]
    assert events==['dji_stop','dji_zero']


def test_intake_buttons_only_call_text_service_and_do_not_start_route():
    mod=console();events=[]
    mod.start_intake=lambda send:start_intake(send,wait=lambda sec:None)
    mod.stop_intake=lambda send:stop_intake(send,wait=lambda sec:None)
    bridge=NS(launcher_text=lambda cmd:events.append(cmd) or (True,'sent'))
    unused=NS()
    app=mod.make_app(bridge,unused,unused,unused,unused,unused,{})
    client=app.test_client()
    assert client.post('/api/action',json={'name':'INTAKE_START'}).get_json()['ok']
    assert client.post('/api/action',json={'name':'INTAKE_STOP'}).get_json()['ok']
    assert events==['dji_stop','dji_zero','dji_move 1 90 5','dji_move 1 0 5']


def test_stop_intake_only_sends_move_zero_without_waiting():
    events=[]
    assert stop_intake(lambda cmd:events.append(cmd) or (True,'sent'),
                       wait=lambda sec:events.append(sec))[0]
    assert events==['dji_move 1 0 5']


def test_failed_stop_has_no_additional_commands():
    events=[]
    def send(cmd):
        events.append(cmd)
        return cmd!='dji_move 1 0 5','result'
    assert not stop_intake(send,wait=lambda sec:None)[0]
    assert events==['dji_move 1 0 5']


def test_route_ui_parameters_reach_script_without_requiring_unplugged_mechanism():
    args=console().fixed_seek_tool_args({'scoop_distance':.4,'scoop_speed':.08})
    assert args==['--route-scoop-distance','0.4','--route-scoop-speed','0.08',
                  '--route-scoop-timeout','15.0','--route-intake']
    assert console().fixed_seek_tool_args({'start_intake':False})[-1]=='--no-route-intake'
    assert console().fixed_seek_tool_args({'start_intake':True})[-1]=='--route-intake'
    defaults=console().fixed_seek_tool_args({})
    assert defaults[defaults.index('--route-scoop-distance')+1]=='0.7'


@pytest.mark.parametrize('args',[{'scoop_distance':float('nan')}, {'scoop_distance':-.4},
                                 {'scoop_speed':0}, {'scoop_speed':'$(touch /tmp/x)'},
                                 {'start_intake':'false'}])
def test_route_ui_rejects_invalid_parameters(args):
    with pytest.raises((ValueError,TypeError)):
        console().fixed_seek_tool_args(args)


def test_fixed_route_ui_enables_automatic_scoop_by_default_but_retains_skip_switch():
    ui = (WS/'tools/console_ui.html').read_text()
    assert 'id="routeIntake" type="checkbox" checked' in ui
    assert '等1秒放下' in ui
    assert "start_intake: $('routeIntake').checked" in ui


@pytest.mark.parametrize('failed_command', [None, 'vesc_stop', 'vesc_home'])
def test_manual_pass_sends_configured_sequence_and_aborts_on_failure(failed_command, monkeypatch):
    mod = console(); events = []
    waits = []
    monkeypatch.setattr(mod.time, "sleep", waits.append)
    def send(command):
        events.append(command)
        return command != failed_command, 'result'
    bridge = NS(launcher_text=send)
    unused = NS()
    client = mod.make_app(bridge, unused, unused, unused, unused, unused, {}).test_client()
    response = client.post('/api/action', json={'name': 'LAUNCH_PASS'}).get_json()
    expected = ['vesc_stop', 'vesc_home', 'vesc_shoot 3', 'vesc_stop', 'vesc_reset']
    assert mod.load_round_config()['release']['pass']['commands'] == expected
    assert response['ok'] == (failed_command is None)
    assert events == (expected if failed_command is None else expected[:expected.index(failed_command)+1])
    if failed_command is None:
        assert waits == [1., 1., 2.]


def test_manual_lift_sends_full_initialization_before_move():
    mod = console(); events = []
    mod.start_intake = lambda send: start_intake(send, wait=lambda seconds: None)
    unused = NS()
    bridge = NS(launcher_text=lambda command: events.append(command) or (True, 'sent'))
    client = mod.make_app(bridge, unused, unused, unused, unused, unused, {}).test_client()
    assert client.post('/api/action', json={'name': 'LAUNCH_GRAB'}).get_json()['ok']
    assert events == ['dji_stop', 'dji_zero', 'dji_move 1 90 5']
