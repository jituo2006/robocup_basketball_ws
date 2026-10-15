"""上电自动第一回合：用模拟电源/进程验证，绝不连接真实硬件。"""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from match_round_route import start_wait_seconds, load_config

WS = Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('auto_round1', WS/'tools/auto_round1.py')
auto=importlib.util.module_from_spec(spec); spec.loader.exec_module(auto)


@pytest.mark.parametrize('samples,expected', [
    ([True,True,True], []), ([False,True,True,False,True],[1]),
    ([False,False,True],[2]), ([False,False],[])])
def test_only_first_off_on_edge_triggers(samples,expected):
    edge=auto.PowerEdge(off_seconds=0.)
    assert [i for i,on in enumerate(samples) if edge.update(on)] == expected


@pytest.mark.parametrize('powered,now,expected', [(None,20,5),(10,12,3),(10,15,0),(10,30,0)])
def test_start_delay_counts_power_wait_once(powered,now,expected):
    assert start_wait_seconds({'start_delay_s':5}, NS(power_on_monotonic=powered),now)==expected


@pytest.mark.parametrize('powered', [-1, float('nan'),float('inf'),21])
def test_bad_power_clock_rejected(powered):
    with pytest.raises(ValueError): start_wait_seconds({'start_delay_s':5},NS(power_on_monotonic=powered),20)


@pytest.fixture
def simulation(tmp_path,monkeypatch):
    cfg=dict(workspace=str(WS),state_dir=str(tmp_path),power_probe_ip='192.168.1.138',
             can_interface='can0',poll_seconds=.25,power_loss_seconds=2.,power_off_seconds=.5,delay_seconds=5.)
    clock=[0.]; launches=[]; stops=[]; saved=[]
    class Process:
        def __init__(self,kind): self.kind=kind;self.returncode=None;self.pid=12345
        def poll(self): return self.returncode
    def popen(cmd,**kw):
        state=json.loads((tmp_path/'state.json').read_text())
        saved.append(state['status'])
        p=Process('route' if 'run_first_round.sh' in cmd[1] else 'stack')
        launches.append((clock[0],cmd,p));return p
    monkeypatch.setattr(auto,'stop_process',lambda p:stops.append(p.kind) if p is not None else None)
    return cfg,clock,launches,stops,saved,popen


def test_full_round_launches_once_after_five_seconds_then_stops_controller_before_stack(simulation):
    cfg,clock,launches,stops,saved,popen=simulation
    def sleep(seconds):
        clock[0]+=seconds
        if launches and launches[-1][2].kind=='route': launches[-1][2].returncode=0
    assert auto.monitor(cfg,probe=lambda ip:clock[0]>=1,now=lambda:clock[0],sleep=sleep,
                        popen=popen,ready=lambda:True,busy=lambda:False,can_present=lambda:True)==0
    assert len(launches)==2 and launches[0][0]==1 and launches[1][0]>=6
    assert launches[0][1][-1]=='start_mission:=false'
    args=launches[1][1]
    assert '--mechanism' in args and args[-2]=='--power-on-monotonic' and float(args[-1])==1
    assert saved==['power_seen','running'] and stops==['route','stack']
    assert json.loads((Path(cfg['state_dir'])/'state.json').read_text())['status']=='complete'
    assert auto.monitor(cfg,probe=lambda ip:pytest.fail('锁定后不再探测/发车'),popen=popen)==0
    assert len(launches)==2


def test_waits_for_can_and_startup_without_restarting_or_adding_second_five_second_delay(simulation):
    cfg,clock,launches,stops,saved,popen=simulation
    def sleep(seconds):
        clock[0]+=seconds
        if launches and launches[-1][2].kind=='route': launches[-1][2].returncode=0
    assert auto.monitor(cfg,probe=lambda ip:clock[0]>=1,now=lambda:clock[0],sleep=sleep,
                        popen=popen,ready=lambda:clock[0]>=10,busy=lambda:False,
                        can_present=lambda:clock[0]>=3)==0
    assert launches[0][0]==3 and launches[1][0]==10
    assert start_wait_seconds({'start_delay_s':5},NS(power_on_monotonic=1),10)==0


def test_power_loss_aborts_without_launching_route_or_retrying(simulation):
    cfg,clock,launches,stops,saved,popen=simulation
    assert auto.monitor(cfg,probe=lambda ip:1<=clock[0]<2,now=lambda:clock[0],
                        sleep=lambda sec:clock.__setitem__(0,clock[0]+sec),
                        popen=popen,ready=lambda:True,busy=lambda:False,can_present=lambda:True)==1
    assert len(launches)==1 and stops==['stack']
    assert json.loads((Path(cfg['state_dir'])/'state.json').read_text())['status']=='failed'


def test_busy_controller_prevents_stack_cleanup_or_motion(simulation):
    cfg,clock,launches,stops,saved,popen=simulation
    assert auto.monitor(cfg,probe=lambda ip:clock[0]>=1,now=lambda:clock[0],
                        sleep=lambda sec:clock.__setitem__(0,clock[0]+sec),
                        popen=popen,ready=lambda:True,busy=lambda:True,can_present=lambda:True)==1
    assert launches==[]


def test_startup_already_online_never_triggers(simulation):
    cfg,clock,launches,stops,saved,popen=simulation
    assert auto.monitor(cfg,probe=lambda ip:True,now=lambda:clock[0],
                        sleep=lambda sec:clock.__setitem__(0,clock[0]+sec),
                        popen=popen,stopping=lambda:clock[0]>2)==0
    assert not launches


def test_automatic_wait_requires_live_front_rear_streams_and_launcher_without_sending(monkeypatch):
    import sys
    from types import ModuleType
    m=ModuleType('rb_msgs.srv');m.SerialText=object;monkeypatch.setitem(sys.modules,'rb_msgs.srv',m)
    b=ModuleType('std_msgs.msg');b.Bool=object;monkeypatch.setitem(sys.modules,'std_msgs.msg',b)
    from test_perimeter_ball import Runner
    r=NS(det_stamp=1.,rear_det_stamp=0.)
    counts=[0]; client=NS(wait_for_service=lambda **kw:counts[0]>=3)
    callbacks=[]; destroyed=[]
    r.node=NS(create_client=lambda *a:client, create_subscription=lambda typ,topic,cb,qos:callbacks.append(cb),
              destroy_subscription=lambda sub:destroyed.append(True), count_publishers=lambda topic:1)
    r.rclpy=NS(ok=lambda:counts[0]<5)
    def spin(sec):
        counts[0]+=1;r.rear_det_stamp=1. if counts[0]>=2 else 0.
        callbacks[0](NS(data=counts[0]>=3))
    r.spin=spin;r.require_fresh_pose=lambda:None;r.detection_fresh=lambda stamp:stamp==1.
    Runner.wait_autostart_inputs(r)
    assert counts[0]==3 and destroyed == [True]


def test_launch_mission_default_remains_enabled_but_auto_command_disables_it():
    launch=(WS/'src/rb_bringup/launch/bringup_lidar_vision.launch.py').read_text()
    assert "DeclareLaunchArgument('start_mission', default_value='true'" in launch
    assert "condition=IfCondition(LaunchConfiguration('start_mission'))" in launch
    assert '--mechanism' in auto.route_command({'workspace':str(WS)},10)


def test_route_failure_is_latched_and_never_retries_same_power(simulation):
    cfg,clock,launches,stops,saved,popen=simulation
    def sleep(seconds):
        clock[0]+=seconds
        if launches and launches[-1][2].kind=='route': launches[-1][2].returncode=7
    assert auto.monitor(cfg,probe=lambda ip:clock[0]>=1,now=lambda:clock[0],sleep=sleep,
                        popen=popen,ready=lambda:True,busy=lambda:False,can_present=lambda:True)==7
    assert len(launches)==2
    assert auto.monitor(cfg,probe=lambda ip:pytest.fail('不应自动重试'))==0


def test_stop_before_power_event_leaves_no_latch_and_sends_no_commands(simulation):
    cfg,clock,launches,stops,saved,popen=simulation
    assert auto.monitor(cfg,probe=lambda ip:False,now=lambda:clock[0],
                        sleep=lambda seconds:clock.__setitem__(0,clock[0]+seconds),
                        popen=popen,stopping=lambda:clock[0]>1)==0
    assert not launches and not (Path(cfg['state_dir'])/'state.json').exists()


def test_config_validation_is_read_only_and_can_target_named_workspace(tmp_path):
    cfg=json.loads((WS/'tools/auto_round1.json').read_text())
    cfg.update(workspace=str(WS),state_dir=str(tmp_path/'not_created'))
    path=tmp_path/'config.json';path.write_text(json.dumps(cfg))
    assert auto.load_config(path)['delay_seconds']==5
    assert not Path(cfg['state_dir']).exists()
    cfg['power_probe_ip']='not-an-ip';path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError):auto.load_config(path)


def test_short_ping_failure_does_not_arm_startup():
    edge=auto.PowerEdge(off_seconds=3.)
    assert not edge.update(True,0)
    assert not edge.update(False,1)
    assert not edge.update(True,2)
    assert not edge.update(True,5)
    assert not edge.update(False,6)
    assert edge.update(True,9)
    assert not edge.update(True,10)
