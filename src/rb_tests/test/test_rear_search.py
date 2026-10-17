import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import pytest


def make_search_runner(monkeypatch, heading=0., visible_at=None, stream=True, fresh=True, moving=True):
    path=Path(__file__).resolve().parents[3]/'tools/test_perimeter_ball.py'
    spec=importlib.util.spec_from_file_location('rear_search_tool',path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    run=mod.Runner.__new__(mod.Runner)
    now=[10.];commands=[];positions=[];approaches=[];last=[0.]
    monkeypatch.setattr(mod.time,'monotonic',lambda:now[0])
    run.args=NS(route_rear_align_speed=.04,pose_timeout=1.,route_heading_tol=.1,route_rear_center_tolerance=.025)
    run.pose=(0.,0.,heading);run.pose_stamp=10.;run.rear_det_stamp=10.
    run.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(now[0]*1e9))))
    run.detection_fresh=lambda stamp:stream
    run._Twist=lambda:NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
    run.stop=lambda:last.__setitem__(0,0.)
    def spin(dt):
        now[0]+=dt
        x,y,h=run.pose
        if moving:
            run.pose=(x-last[0]*dt*math.sin(h),y+last[0]*dt*math.cos(h),h)
        if fresh:run.pose_stamp=now[0]
        positions.append(-(run.pose[0])*math.sin(heading)+run.pose[1]*math.cos(heading))
    def publish(tw):
        commands.append((now[0],tw.linear.x,tw.linear.y,tw.angular.z))
        last[0]=tw.linear.y
    run.spin=spin;run.publish_command=publish
    run.centered_target_candidates=lambda *a,**kw: NS(x_min=.45,x_max=.55,confidence=.9,stamp=NS(sec=10,nanosec=0)) if visible_at and now[0]>=visible_at else None
    run.aligned_intake_approach=lambda: approaches.append(True) or True
    return mod,run,commands,positions,approaches,now


@pytest.mark.parametrize('heading',[0.,math.pi/2,-1.2])
def test_missing_ball_returns_to_search_origin_then_approaches(monkeypatch,heading):
    m,r,cmd,pos,approach,now=make_search_runner(monkeypatch,heading)
    assert r._rear_visual_scoop()
    assert cmd and cmd[0][0]>=10.6
    assert all(x==0 and z==0 and abs(y)<=.04 for _,x,y,z in cmd)
    assert max(pos)>=.239 and min(pos)<=-.239
    assert max(abs(p) for p in pos)<=.251 and abs(pos[-1])<=.01
    assert approach == [True]


def test_reacquisition_stops_search_and_allows_centered_approach(monkeypatch):
    m,r,cmd,pos,approach,now=make_search_runner(monkeypatch,visible_at=12.)
    assert r._rear_visual_scoop()
    assert cmd and approach==[True]
    assert now[0]<12.1


@pytest.mark.parametrize('kwargs',[{'stream':False},{'fresh':False},{'moving':False}])
def test_missing_feedback_or_stall_cannot_move_forever(monkeypatch,kwargs):
    m,r,cmd,pos,approach,now=make_search_runner(monkeypatch,**kwargs)
    assert not r._rear_visual_scoop()
    assert now[0]<45.1 and not approach
    if not kwargs.get('stream',True):assert not cmd
    if not kwargs.get('fresh',True):assert all(t<=11.01 for t,*_ in cmd)


def test_active_reverse_ignores_camera_loss_through_real_command_gate(monkeypatch):
    from test_fixed_seek_route import load, make_runner
    m=load();r=make_runner(m)
    r.pose=(0.,0.,0.);r.pose_stamp=10.;r.yaw_rate=0.
    r.visual_guard=True;r.motion_guard=False
    r.args.route_intake=False
    r.dets=[];r.dets_rear=[]
    r.best_ball=lambda:pytest.fail('distance pickup must not consult visual stop gate')
    r.centered_target_candidates=lambda *a,**kw:pytest.fail('distance pickup must not re-enter alignment')
    r._Twist=lambda:NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
    commands=[];events=[];last=[0.]
    def publish(tw):
        commands.append(tw.linear.x);last[0]=tw.linear.x
    r.cmd_pub=NS(publish=publish)
    def spin(dt):
        r.pose=(r.pose[0]+last[0]*dt,0.,0.)
        assert len(commands)<1000, 'distance pickup must finish'
    r.spin=spin
    r.stop=lambda:(last.__setitem__(0,0.),events.append('stop'))
    monkeypatch.setattr(m.Runner, '_timed_pickup_sequence', lambda self:events.append('pickup') or True)
    r.args.route_intake=True
    assert r.aligned_intake_approach()
    assert commands and all(v<0 for v in commands)
    assert -r.pose[0]>=r.args.route_scoop_distance-r.args.step_tol
    assert events[-1]=='pickup' and r.visual_guard is True


def test_active_reverse_keeps_motion_fault_and_restores_guard():
    from test_fixed_seek_route import load, make_runner
    m=load();r=make_runner(m)
    r.visual_guard=True;r.motion_guard=True
    r.pose=(0.,0.,0.);r.yaw_rate=0.
    r.spin=lambda dt:None;r.stop=lambda:None
    r.motion_fault='simulated localization failure'
    r._Twist=lambda:NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
    r.cmd_pub=NS(publish=lambda tw:pytest.fail('fault must prevent reverse command'))
    with pytest.raises(RuntimeError,match='simulated localization failure'):
        r.rear_distance_scoop()
    assert r.visual_guard is True


def test_stage_alignment_fallback_returns_success_only_after_returning_to_origin(monkeypatch):
    m,r,cmd,pos,approach,now=make_search_runner(monkeypatch)
    assert r._rear_visual_scoop(approach=False)
    assert max(pos)>=.239 and min(pos)<=-.239 and abs(pos[-1])<=.01
    assert not approach  # 整轮由下一个 approach 动作完成距离后退，不能重复后退。
