from collections import deque
from types import ModuleType, SimpleNamespace as NS
import sys
import pytest
from test_seek_startup import load


@pytest.mark.parametrize('route,dry,step,expected', [
    ('first-pass', False, None, True), ('fixed', False, None, True),
    ('round1', False, None, True), ('round2', False, None, True),
    ('round3', False, None, True), ('round1', False, 'seek', False),
    ('first-pass', True, None, False), ('perimeter', False, None, False)])
def test_only_full_experiments_reset(route, dry, step, expected):
    assert load().needs_experiment_reset(NS(route=route, dry=dry, round_step=step)) == expected


@pytest.mark.parametrize('mode', ['success', 'missing', 'failure', 'timeout'])
def test_reset_clears_cache_only_after_success_and_stays_stopped(monkeypatch, mode):
    mod = load(); run = mod.Runner.__new__(mod.Runner)
    srv = ModuleType('std_srvs.srv'); srv.Trigger = NS(Request=lambda: NS())
    monkeypatch.setitem(sys.modules, 'std_srvs.srv', srv)
    result = NS(success=mode != 'failure', message='reset')
    ticks=[0]
    client = NS(wait_for_service=lambda **kw: mode != 'missing' or ticks[0]>=200,
                call_async=lambda req: NS(done=lambda: mode != 'timeout' or ticks[0]>=200, result=lambda: result))
    destroyed = []; stops = []
    run.node = NS(create_client=lambda *a: client, destroy_client=destroyed.append,
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=101000000000)))

    run.rclpy = NS(ok=lambda: True, spin_once=lambda *a, **kw:ticks.__setitem__(0,ticks[0]+1))
    run.stop = lambda: stops.append(True)
    run.pose = (3., 4., 1.); run.odom_cam = run.cloud = object()
    run.pose_history = deque([(100., run.pose)])
    run.obstacle_cloud_history = deque([object()])
    run.dets = [object()]; run.dets_rear = [object()]
    run.motion_fault = 'old jump'
    if mode != 'failure':
        run.reset_experiment_start()
        assert run.pose is run.cloud is run.odom_cam is None
        assert not run.pose_history and not run.obstacle_cloud_history
        assert not run.dets and not run.dets_rear
        assert run.motion_fault is None and run.pose_stamp == run.odom_stamp == 0.
        assert run.experiment_cutoff == 101.
        old = NS(header=NS(stamp=NS(sec=100, nanosec=0)), detections=[object()])
        run._on_dets(old); run._on_dets_rear(old); run._on_cloud(old); run._on_odom_cam(old)
        assert not run.dets and not run.dets_rear and run.cloud is run.odom_cam is None
        old.header.stamp.sec = 102
        run._on_dets(old)
        assert run.det_stamp == 102. and run.dets
    else:
        with pytest.raises(RuntimeError):
            run.reset_experiment_start()
    assert stops == [True] and destroyed == [client]


def test_late_mechanism_reply_waits_on_same_request(monkeypatch):
    mod=load();run=mod.Runner.__new__(mod.Runner)
    srv=ModuleType('rb_msgs.srv');srv.SerialText=NS(Request=lambda:NS())
    monkeypatch.setitem(sys.modules,'rb_msgs.srv',srv)
    ticks=[0];sent=[]
    future=NS(done=lambda:ticks[0]>=200,result=lambda:NS(success=True,message='sent'))
    run.intake_client=NS(wait_for_service=lambda **kw:True,
                         call_async=lambda req:sent.append(req.command) or future)
    run.rclpy=NS(ok=lambda:True,spin_once=lambda *a,**kw:ticks.__setitem__(0,ticks[0]+1))
    run.node=NS()
    assert run.intake_text('vesc_shoot 3')==(True,'sent')
    assert sent==['vesc_shoot 3'] and ticks[0]==200  # 6 seconds, beyond the old 3-second deadline.
