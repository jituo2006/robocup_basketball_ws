"""Chassis power cycling is explicit readiness/I/O recovery, never a silent-age stop."""
import json
from types import SimpleNamespace as NS
import pytest
from configure_chassis_can import configure
from test_seek_observations import load


class Commands:
    def __init__(self, *, restart_ms=0, state='ERROR-ACTIVE', configured=False, deny=False):
        self.calls=[]
        self.deny=deny
        self.link={'flags':['UP'], 'txqlen':16 if configured else 1000,
                   'linkinfo':{'info_kind':'can','info_data':{
                       'state':state,'restart_ms':restart_ms,'bittiming':{'bitrate':1000000}}}}
        self.qdiscs=([{'root':True,'kind':'pfifo_head_drop','options':{'limit':16}}]
                     if configured else [{'root':True,'kind':'pfifo_fast'}])
    def __call__(self, args, **kwargs):
        self.calls.append(args)
        if args[:3]==['ip','-details','-json']:
            return NS(returncode=0,stdout=json.dumps([self.link]))
        if args[:3]==['tc','-json','qdisc']:
            return NS(returncode=0,stdout=json.dumps(self.qdiscs))
        return NS(returncode=1 if self.deny else 0,stdout='',stderr='permission denied')


def test_boot_configures_auto_restart_and_bounded_drop_oldest_queue():
    commands=Commands()
    configure(run=commands)
    assert ['ip','link','set','dev','can0','down'] in commands.calls
    assert ['ip','link','set','dev','can0','type','can','bitrate','1000000','restart-ms','100'] in commands.calls
    assert ['ip','link','set','dev','can0','txqueuelen','16'] in commands.calls
    assert ['tc','qdisc','replace','dev','can0','root','pfifo_head_drop','limit','16'] in commands.calls


def test_already_configured_interface_is_not_bounced_or_flushed():
    commands=Commands(restart_ms=100,configured=True)
    configure(run=commands)
    assert len(commands.calls)==2  # Only the two read-only state queries.


def test_bus_off_requests_restart_without_restarting_lidar_or_ros():
    commands=Commands(restart_ms=100,state='BUS-OFF',configured=True)
    configure(run=commands)
    assert ['ip','link','set','dev','can0','type','can','restart'] in commands.calls
    assert not any('down' in c or 'systemctl' in c for c in commands.calls)


def test_configuration_failure_propagates_instead_of_starting_unprotected():
    with pytest.raises(RuntimeError,match='permission denied'):
        configure(run=Commands(deny=True))


def test_task_waits_indefinitely_for_board_then_sends_a_new_zero():
    module=load();r=module.Runner.__new__(module.Runner)
    r.chassis_ready=False;r.args=NS(obstacle_guard=False)
    elapsed=[0.];stops=[]
    def spin(seconds):
        elapsed[0]+=seconds
        if elapsed[0]>=120.:r.chassis_ready=True
    r.spin=spin;r.rclpy=NS(ok=lambda:True)
    r.require_fresh_pose=lambda:False
    r.stop=lambda:stops.append(r.chassis_ready)
    r.wait_motion_ready()
    assert elapsed[0]>=120. and stops[-1] is True and False in stops


def test_wait_for_board_can_be_cancelled():
    module=load();r=module.Runner.__new__(module.Runner)
    r.chassis_ready=False;r.args=NS(obstacle_guard=False)
    elapsed=[0.];r.spin=lambda s:elapsed.__setitem__(0,elapsed[0]+s)
    r.rclpy=NS(ok=lambda:elapsed[0]<.2)
    r.require_fresh_pose=lambda:False;r.stop=lambda:None
    with pytest.raises(RuntimeError,match='ROS 已停止'):r.wait_motion_ready()


def test_reported_loss_cancels_active_route_and_cannot_auto_resume_it():
    module=load();r=module.Runner.__new__(module.Runner)
    r.chassis_ready=True;r.motion_guard=True;r.motion_fault=None
    r._on_chassis_ready(NS(data=False))
    assert not r.chassis_ready and r.motion_fault
    r._on_chassis_ready(NS(data=True))
    assert r.motion_fault  # A new ready report never clears the previous run's fault.
    stops=[];r.stop=lambda:stops.append(True)
    with pytest.raises(RuntimeError,match='底盘'):r.require_fresh_pose()
    assert stops==[True]


def test_initial_offline_report_is_not_an_active_route_failure():
    module=load();r=module.Runner.__new__(module.Runner)
    r.chassis_ready=False;r.motion_guard=False;r.motion_fault=None
    r._on_chassis_ready(NS(data=False))
    assert r.motion_fault is None
