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
    assert events==['dji_stop',.3,'dji_zero',.3,'dji_move 1 90 5']


def test_intake_sequence_stops_on_failed_initialization():
    events=[]
    def send(cmd):
        events.append(cmd)
        return cmd!='dji_zero','result'
    assert not start_intake(send,wait=lambda sec:None)[0]
    assert events==['dji_stop','dji_zero']


def checked_bridge():
    from test_mechanism_control import FakeMCU
    m=FakeMCU()
    return NS(mechanism_client=NS(controller=m.c)),m


def test_intake_buttons_wait_for_completion_without_rezeroing():
    mod=console();bridge,m=checked_bridge();unused=NS()
    client=mod.make_app(bridge,unused,unused,unused,unused,unused,{}).test_client()
    assert client.post('/api/action',json={'name':'INTAKE_START'}).json['ok']
    assert m.position==90
    assert client.post('/api/action',json={'name':'INTAKE_STOP'}).json['ok']
    assert m.position==0 and not any(c=='dji_zero' for _,c in m.sent)


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
    ui = (WS/'tools/console_debug_ui.html').read_text()
    assert 'id="routeIntake" type="checkbox" checked' in ui
    assert '上升DONE后等1秒' in ui
    assert '50ms重发最多20次' in ui
    assert "start_intake: $('routeIntake').checked" in ui


@pytest.mark.parametrize('failed_command',[None,'vesc_stop','vesc_home'])
def test_manual_pass_uses_checked_actions_and_aborts_on_refusal(failed_command):
    mod=console();bridge,m=checked_bridge();unused=NS()
    if failed_command:m.overrides[failed_command]=lambda:m.rx('ERR FAULT')
    client=mod.make_app(bridge,unused,unused,unused,unused,unused,{}).test_client()
    response=client.post('/api/action',json={'name':'LAUNCH_PASS'}).json
    assert response['ok']==(failed_command is None)
    if failed_command:assert not any(c=='vesc_shoot 3' for _,c in m.sent)
    else:
        assert sum(c=='vesc_shoot 3' for _,c in m.sent)==1
        assert sum(c=='vesc_stop' for _,c in m.sent)==3
        assert sum(c=='vesc_home' for _,c in m.sent)==2
        # 用户指定的末尾vesc_stop会清除home；不再要求复位后的home保留。
        assert m.home==0


def test_manual_lift_uses_existing_zero_and_confirms_target():
    mod=console();bridge,m=checked_bridge();unused=NS()
    client=mod.make_app(bridge,unused,unused,unused,unused,unused,{}).test_client()
    assert client.post('/api/action',json={'name':'LAUNCH_GRAB'}).json['ok']
    assert m.position==90 and not any(c=='dji_zero' for _,c in m.sent)


def test_manual_init_is_the_explicit_zeroing_action():
    mod=console();bridge,m=checked_bridge();unused=NS();m.zero=0
    client=mod.make_app(bridge,unused,unused,unused,unused,unused,{}).test_client()
    assert client.post('/api/action',json={'name':'LAUNCH_INIT'}).json['ok']
    assert [c for _,c in m.sent][:2]==['dji_stop','dji_zero']
