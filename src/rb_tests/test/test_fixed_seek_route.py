"""固定路线：方向、动作顺序、检测即停车、失败不转身后退。"""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


def load():
    path = Path(__file__).resolve().parents[3] / 'tools/test_perimeter_ball.py'
    spec = importlib.util.spec_from_file_location('fixed_seek', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def make_runner(mod):
    run = mod.Runner.__new__(mod.Runner)
    run.pose = (1.5, 8.0, 0.0)
    run.dets = []
    run.args = NS(ball_label='ball_basketball', route_forward=3.55,
                  route_scan_start_offset=2.9, route_scan_distance=3.8,
                  route_forward_speed=.12, route_scan_speed=.08, route_scoop_speed=.08,
                  route_scoop_distance=.20, route_forward_timeout=60., route_scan_timeout=60.,
                  route_scoop_timeout=15., route_heading_tol=.10, route_ball_tol_deg=6.,
                  route_center_tolerance=.04, ball_confidence=.5,
                  route_rear_center_tolerance=.025, route_rear_center_frames=3,
                  route_ball_min_distance=.45, route_ball_max_distance=1.0,
                  speed=.25, approach_speed=.12, step_tol=.02, yaw_tol=.015,
                  kp=1.5, kp_yaw=2., kd_yaw=.25, max_ang=.3)
    return run


def test_route_is_right_turn_then_forward_then_body_left():
    mod = load()
    heading, near, axis, end = mod.fixed_route_plan((1.5, 8., 0.), 3.55, 3.)
    assert heading == pytest.approx(-math.pi/2)
    assert near == pytest.approx((1.5, 4.45))
    assert axis == pytest.approx((1., 0.))
    assert end == pytest.approx((4.5, 4.45))
    # 右转后的 +车体y 是 +场地x，不是朝场地左侧(-x)。
    assert (-math.sin(heading), math.cos(heading)) == pytest.approx(axis)


def test_route_respects_initial_heading_without_resetting_localization():
    mod = load()
    heading, near, axis, _ = mod.fixed_route_plan((1., 2., math.pi/2), 3., 2.)
    assert heading == pytest.approx(0.)
    assert near == pytest.approx((4., 2.))
    assert axis == pytest.approx((0., 1.))


@pytest.mark.parametrize('yaw', [0., math.pi/2, -1.2])
def test_middle_preposition_and_scan_use_rule_diagram_directions_in_any_local_frame(yaw):
    mod = load()
    heading, near, begin, axis, end = mod.fixed_middle_route_plan((1.5, 8., yaw), 3.2, 2.9, 3.8)
    c, s = math.cos(yaw), math.sin(yaw)
    assert near == pytest.approx((1.5+3.2*s, 8.-3.2*c))
    assert begin == pytest.approx((near[0]+2.9*c, near[1]+2.9*s))
    assert end == pytest.approx((near[0]-.9*c, near[1]-.9*s))
    assert axis == pytest.approx((-c, -s))
    # 向图左端扫在右转后的车体坐标中是右移，不能沿用旧的左移符号。
    assert math.cos(heading)*axis[0]+math.sin(heading)*axis[1] == pytest.approx(0.)
    assert -math.sin(heading)*axis[0]+math.cos(heading)*axis[1] == pytest.approx(-1.)


def mock_motion(run, fail=None):
    actions = []
    def turn(yaw, **kw):
        actions.append(('turn', yaw))
        if fail == 'turn':
            return False
        run.pose = (*run.pose[:2], yaw)
        return True
    def goto(x, y, **kw):
        actions.append(('move', x, y, run.args.speed, kw['tolerance']))
        if fail == 'goto':
            return False
        run.pose = (x, y, run.pose[2])
        return True
    run.rotate_to, run.goto = turn, goto
    def preposition(tx, ty, heading, **kwargs):
        return ('empty', None) if run.goto(tx, ty, tolerance=run.args.step_tol) else ('failed', None)
    run.scan_to = preposition
    def rear():
        x,y,yaw=run.pose
        saved=run.args.speed;run.args.speed=run.args.route_scoop_speed
        result=goto(x-run.args.route_scoop_distance*math.cos(yaw),y-run.args.route_scoop_distance*math.sin(yaw),tolerance=run.args.step_tol)
        run.args.speed=saved;run.stop();return result
    run.rear_visual_scoop=rear
    run.stop = lambda: actions.append(('stop',))
    def scan(*a):
        actions.append(('scan',))
        return (.0, .70, 3)
    run.fixed_left_scan = scan
    return actions


def test_fixed_sequence_turns_180_and_backs_towards_ball_at_low_speed():
    mod = load()
    run = make_runner(mod)
    actions = mock_motion(run)
    assert run.fixed_seek_and_scoop()
    assert [a[0] for a in actions] == ['turn', 'move', 'move', 'scan', 'turn', 'move', 'stop']
    assert actions[0][1] == pytest.approx(-math.pi/2)
    assert actions[4][1] == pytest.approx(math.pi/2)
    assert actions[1][1:3] == pytest.approx((1.5, 4.45))
    assert actions[2][1:3] == pytest.approx((4.4, 4.45))
    assert actions[2][3] == .08
    assert actions[5][1:3] == pytest.approx((4.4, 4.25))
    assert actions[5][3] == .08 and actions[5][4] == .02
    assert run.args.speed == .25 and run.args.approach_speed == .12


@pytest.mark.parametrize('failure', ['turn', 'goto'])
def test_route_failure_never_starts_search_or_scoop(failure):
    mod = load()
    run = make_runner(mod)
    actions = mock_motion(run, fail=failure)
    assert not run.fixed_seek_and_scoop()
    assert not any(a[0] == 'scan' for a in actions)
    assert run.args.speed == .25


def test_preposition_failure_never_reads_yolo_or_turns_to_scoop():
    mod = load(); run = make_runner(mod)
    events = mock_motion(run)
    goto = run.goto
    def fail_at_right_end(x, y, **kwargs):
        if x == pytest.approx(4.4):
            events.append(('preposition_failed',))
            return False
        return goto(x, y, **kwargs)
    run.goto = fail_at_right_end
    run.centered_basketball = lambda: pytest.fail('preposition must not inspect basketballs')
    assert not run.fixed_seek_and_scoop()
    assert events[-1] == ('preposition_failed',)
    assert not any(e[0] == 'scan' for e in events)
    assert run.args.speed == .25 and run.args.approach_speed == .12


@pytest.mark.parametrize('found', [None])
def test_unconfirmed_off_axis_or_out_of_range_ball_does_not_trigger_turn_or_scoop(found):
    mod = load()
    run = make_runner(mod)
    actions = mock_motion(run)
    run.fixed_left_scan = lambda *a: found
    assert not run.fixed_seek_and_scoop()
    assert sum(a[0] == 'turn' for a in actions) == 1
    assert sum(a[0] == 'move' for a in actions) == 2


def test_detection_stops_without_waiting_for_center_or_confirmation():
    mod = load()
    run = make_runner(mod)
    events = []
    run.spin = lambda sec: None
    det = NS(x_min=.65,x_max=.75,confidence=.9, stamp=NS(sec=10,nanosec=0))
    def detect(dets, centered=True):
        assert not centered
        return det
    run.centered_target_candidates = detect
    run.stop = lambda: events.append('stop')
    run.measure_ball_stable = lambda **kw: events.append('confirm') or (0., .7, 3)
    assert run.fixed_left_scan(-math.pi/2, (1, 0)) is det
    assert events == ['stop']


def test_scan_command_is_body_left_with_zero_forward_speed():
    mod = load()
    run = make_runner(mod)
    run.pose, run.yaw_rate = (1.5, 4.45, -math.pi/2), 0.
    run.args.route_scan_distance = .1
    speeds, stops = [], []
    run.spin = lambda sec: None
    run.centered_target_candidates = lambda *a, **kw: None
    run.stop = lambda: stops.append(True)
    run._Twist = lambda: NS(linear=NS(x=0., y=0.), angular=NS(z=0.))
    def command(msg):
        speeds.append((msg.linear.x, msg.linear.y, msg.angular.z))
        run.pose = (run.pose[0]+.1, run.pose[1], run.pose[2])
    run.publish_command = command
    assert run.fixed_left_scan(-math.pi/2, (1., 0.)) is None
    assert speeds[0] == pytest.approx((0., .08, 0.))
    assert stops


def test_rule_left_scan_commands_body_right_and_stops_at_end_tolerance():
    mod = load(); run = make_runner(mod)
    run.pose, run.yaw_rate = (4.4, 4.8, -math.pi/2), 0.
    run.args.route_scan_distance = .1
    run.spin = lambda sec: None
    run.centered_target_candidates = lambda *a, **kw: None
    run.stop = lambda: None
    run._Twist = lambda: NS(linear=NS(x=0., y=0.), angular=NS(z=0.))
    speeds = []
    def command(msg):
        speeds.append((msg.linear.x, msg.linear.y))
        # 停在终点容差内，不必依赖超时或越过末端才停。
        run.pose = (4.315, 4.8, run.pose[2])
    run.publish_command = command
    assert run.fixed_left_scan(-math.pi/2, (-1., 0.)) is None
    assert speeds == [pytest.approx((0., -.08))]

@pytest.mark.parametrize('label,center,source,fresh,conf,expected', [
    ('ball_basketball',.5,'camera',True,.9,True),
    ('ball_basketball',.7,'camera',True,.9,False),
    ('ball_volleyball',.5,'camera',True,.9,False),
    ('ball_basketball',.5,'camera',False,.9,False),
    ('ball_basketball',.5,'lidar',True,.9,False),
    ('ball_basketball',.5,'camera',True,.2,False),
])
def test_center_trigger_requires_fresh_confident_camera_basketball(label,center,source,fresh,conf,expected):
    mod=load();run=make_runner(mod)
    run.dets=[NS(label=label,source=source,confidence=conf,x_min=center-.05,
        x_max=center+.05,y_min=.4,y_max=.6,stamp=NS(sec=10,nanosec=0))]
    run.detection_fresh=lambda stamp:fresh
    assert (run.centered_basketball() is not None)==expected


@pytest.mark.parametrize('axis', [(1., 0.), (-1., 0.)])
def test_off_center_ball_stops_both_middle_sweep_directions_before_any_more_motion(axis):
    mod = load(); run = make_runner(mod)
    run.pose = (2., 4.8, -math.pi/2)
    run.dets = [NS(label='ball_basketball', source='camera', confidence=.9,
                   x_min=.65, x_max=.75, y_min=.4, y_max=.6, stamp=NS(sec=10, nanosec=0))]
    run.detection_fresh = lambda stamp: True
    run.spin = lambda seconds: None
    stops = []; run.stop = lambda: stops.append(True)
    run.publish_command = lambda tw: pytest.fail('first eligible ball must stop before another velocity')
    status, det = run.scan_to(run.pose[0]+axis[0], 4.8, -math.pi/2, timeout=1., speed=.25)
    assert status == 'found' and det is run.dets[0] and stops == [True]


@pytest.mark.parametrize('distance', [.45, .70])
def test_fixed_first_sweep_interrupts_before_right_end_turns_and_completes_distance_approach(distance):
    mod = load(); run = make_runner(mod)
    run.args.route_forward=3.2;run.args.route_scoop_distance=distance;run.yaw_rate=0.;run.pose_stamp=10.
    run._Twist=lambda:NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
    run.spin=lambda seconds:None;run.stop=lambda:None
    turns, commands, aligned = [], [], []
    def turn(yaw, **kwargs):
        turns.append(yaw);run.pose=(*run.pose[:2],yaw);return True
    def goto(x,y,**kwargs):
        assert (x,y)==pytest.approx((1.5,4.8))
        run.pose=(x,y,run.pose[2]);return True
    def command(tw):
        commands.append((tw.linear.x,tw.linear.y))
        x,y,yaw=run.pose
        run.pose=(x+.2*(math.cos(yaw)*tw.linear.x-math.sin(yaw)*tw.linear.y),
                  y+.2*(math.sin(yaw)*tw.linear.x+math.cos(yaw)*tw.linear.y),yaw)
    ball=NS(label='ball_basketball',source='camera',confidence=.9,
            x_min=.65,x_max=.75,y_min=.4,y_max=.6,stamp=NS(sec=10,nanosec=0))
    choose=run.centered_target_candidates
    def target(dets, centered=True, **kwargs):
        assert not aligned, 'distance approach must not read YOLO'
        return choose([ball],centered=centered) if run.pose[0]>=2. else None
    def scoop():
        aligned.append(run.pose)
        return run.rear_distance_scoop()
    run.rotate_to=turn;run.goto=goto;run.publish_command=command
    run.detection_fresh=lambda stamp:True;run.centered_target_candidates=target
    run.rear_visual_scoop=scoop
    run.fixed_left_scan=lambda *a:pytest.fail('first sweep found a ball; never continue to left sweep')
    assert run.fixed_seek_and_scoop()
    assert turns==pytest.approx([-math.pi/2,math.pi/2])
    assert 2. <= aligned[0][0] < 4.4
    assert any(vx<0 and vy==0 for vx,vy in commands)
    assert aligned[0][1]-run.pose[1]>=distance-run.args.step_tol
    assert run.args.speed==.25


def test_cli_reaches_node_check_after_manual_obstacle_parameter_removal(monkeypatch):
    mod=load()
    monkeypatch.setattr(mod.subprocess,'run',lambda *a,**kw:NS(returncode=1))
    # 所有节点被模拟为未启动：只能正常返回1，不能在参数校验阶段 AttributeError。
    assert mod.main(['--dry','--yes','--no-mechanism'])==1


def timed_rear_samples(mod, run, monkeypatch, samples, cancel_after=30.):
    clock = [0.]
    monkeypatch.setattr(mod.time, 'monotonic', lambda: clock[0])
    def spin(seconds):
        clock[0] += seconds
        if clock[0] > cancel_after:
            raise KeyboardInterrupt()
    run.spin = spin
    observations = iter(samples)
    run.centered_target_candidates = lambda *a, **kw: next(observations, None)


def test_rear_lost_detection_waits_stopped_until_cancel_without_blind_backing(monkeypatch):
    mod=load();run=make_runner(mod)
    run.args.route_rear_timeout=.1;run.args.route_rear_stop_height=.45
    run.args.route_rear_align_speed=.04
    d=NS(x_min=.45,x_max=.55,y_min=.4,y_max=.6,confidence=.9,stamp=NS(sec=10,nanosec=0))
    events=[]
    timed_rear_samples(mod, run, monkeypatch, [d,None], cancel_after=.2)
    run.stop=lambda:events.append('stop')
    run._Twist=lambda:NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
    run.publish_command=lambda tw:events.append((tw.linear.x,tw.linear.y))
    with pytest.raises(KeyboardInterrupt):
        run.rear_visual_scoop()
    assert len(events)>=3 and set(events)=={'stop'}

@pytest.mark.parametrize('cx,expected',[(.7,(0.,.04)),(.3,(0.,-.04)),(.5,(-.08,0.))])
def test_rear_camera_corrects_body_lateral_sign_and_backs_only_when_centered(cx,expected,monkeypatch):
    mod=load();run=make_runner(mod)
    run.args.route_rear_timeout=.1;run.args.route_rear_stop_height=.45
    run.args.route_rear_align_speed=.04
    d=NS(x_min=cx-.05,x_max=cx+.05,y_min=.4,y_max=.6,confidence=.9,stamp=NS(sec=10,nanosec=0))
    commands=[]
    timed_rear_samples(mod, run, monkeypatch, [d,None], cancel_after=.2)
    run.stop=lambda:None
    run._Twist=lambda:NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
    run.publish_command=lambda tw:commands.append((tw.linear.x,tw.linear.y))
    with pytest.raises(KeyboardInterrupt):
        run.rear_visual_scoop()
    assert commands==([] if cx==.5 else [expected])


def test_rear_reacquires_after_logged_drop_and_completes_45cm_without_more_yolo(monkeypatch):
    mod=load();run=make_runner(mod)
    run.args.route_rear_timeout=1.;run.args.route_rear_align_speed=.04
    run.args.route_scoop_distance=.45;run.args.route_intake=False
    run.pose=(0.,0.,math.pi/2);run.pose_stamp=10.;run.yaw_rate=0.
    def det(cx, stamp):
        return NS(x_min=cx-.05,x_max=cx+.05,confidence=.9,
                  stamp=NS(sec=stamp,nanosec=0))
    # Same off-centre location as the failed field run, then one missing frame.
    samples=[det(.263,10),None,det(.3,11),det(.5,12),det(.5,12),det(.5,13),det(.5,14)]
    timed_rear_samples(mod, run, monkeypatch, samples)
    commands=[];run.stop=lambda:None
    run._Twist=lambda:NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
    def publish(tw):
        commands.append((tw.linear.x,tw.linear.y))
        x,y,yaw=run.pose
        run.pose=(x+.03*(tw.linear.x*math.cos(yaw)-tw.linear.y*math.sin(yaw)),
                  y+.03*(tw.linear.x*math.sin(yaw)+tw.linear.y*math.cos(yaw)),yaw)
    run.publish_command=publish
    run.intake_text=lambda cmd:pytest.fail('disabled mechanism must not block approach')
    original_approach=run.rear_distance_scoop
    def approach():
        run.centered_target_candidates=lambda *a,**kw:pytest.fail('approach must not read YOLO')
        return original_approach()
    run.rear_distance_scoop=approach
    assert run.rear_visual_scoop()
    assert sum(vy<0 for vx,vy in commands)==2
    assert all(vy==0 for vx,vy in commands if vx<0)
    assert -run.pose[1] == pytest.approx(.45,abs=run.args.step_tol)


def test_rear_missing_frame_resets_center_count_and_cached_frame_cannot_finish_alignment(monkeypatch):
    mod=load();run=make_runner(mod)
    run.args.route_rear_timeout=1.;run.args.route_rear_align_speed=.04
    def det(stamp):
        return NS(x_min=.45,x_max=.55,confidence=.9,stamp=NS(sec=stamp,nanosec=0))
    samples=iter([det(10),det(11),None,det(11),det(12),det(13),det(14)])
    timed_rear_samples(mod,run,monkeypatch,samples)
    run.stop=lambda:None
    def approach():
        assert next(samples,None) is None, 'three distinct centered frames required after drop'
        return True
    run.aligned_intake_approach=approach
    assert run.rear_visual_scoop()

@pytest.mark.parametrize('height',[.2,.6])
def test_rear_requires_unique_center_frames_then_approaches_without_more_yolo(height):
    mod=load();run=make_runner(mod)
    run.args.route_rear_timeout=.2;run.args.route_rear_align_speed=.04
    def det(stamp):
        return NS(x_min=.45,x_max=.55,y_min=.2,y_max=.2+height,
                  confidence=.9,stamp=NS(sec=stamp,nanosec=0))
    samples=iter([det(10),det(10),det(11),det(12)])
    events=[];run.spin=lambda sec:None
    run.centered_target_candidates=lambda *a,**kw:next(samples)
    run.stop=lambda:events.append('stop')
    run.rear_distance_scoop=lambda:events.append('distance') or True
    assert run.rear_visual_scoop()
    assert events==['stop','stop','stop','stop','stop','distance','stop']
    assert not run.obstacle_scoop


def test_distance_scoop_moves_when_yolo_is_missing_and_stops_at_distance():
    mod=load();run=make_runner(mod)
    run.pose=(0.,0.,0.);run.pose_stamp=10.;run.yaw_rate=0.
    run.spin=lambda sec:None
    run.centered_target_candidates=lambda *a,**kw:pytest.fail('approach must not read YOLO')
    run.stop=lambda:None
    run._Twist=lambda:NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))
    commands=[]
    def publish(t):
        commands.append(t.linear.x)
        run.pose=(run.pose[0]+t.linear.x,0.,0.)
    run.publish_command=publish
    assert run.rear_distance_scoop()
    assert commands and all(-.08 <= v < 0 for v in commands)
    assert -run.pose[0] >= run.args.route_scoop_distance-run.args.step_tol


def test_distance_scoop_stops_when_heading_recovery_fails():
    mod=load();run=make_runner(mod)
    run.pose=(0.,0.,0.);run.pose_stamp=10.;run.yaw_rate=0.
    run.spin=lambda sec:setattr(run,'pose',(0.,0.,.3))
    stops=[];run.stop=lambda:stops.append(True)
    run.publish_command=lambda tw:pytest.fail('wrong heading must stop before command')
    run.rotate_to=lambda heading, **kw: False
    assert not run.rear_distance_scoop()
    assert stops


def test_distance_scoop_recovers_heading_and_keeps_original_distance_origin():
    mod = load(); run = make_runner(mod)
    run.pose = (0., 0., 0.); run.pose_stamp = 10.; run.yaw_rate = 0.
    run.stop = lambda: None
    run._Twist = lambda: NS(linear=NS(x=0., y=0.), angular=NS(z=0.))
    turns, commands = [], []
    def spin(dt):
        if not turns:
            run.pose = (-.05, 0., .3)
    def turn(heading, **kw):
        turns.append(heading)
        run.pose = (run.pose[0], run.pose[1], heading)
        return True
    def publish(tw):
        commands.append(tw.linear.x)
        run.pose = (run.pose[0]+tw.linear.x, 0., 0.)
    run.spin, run.rotate_to, run.publish_command = spin, turn, publish
    assert run.rear_distance_scoop()
    assert turns == [0.] and commands
    assert run.args.route_scoop_distance-run.args.step_tol <= -run.pose[0] < .23


def test_rear_center_confirmation_is_not_blocked_by_former_lateral_cap():
    mod = load(); run = make_runner(mod)
    run.args.route_rear_align_speed = .04
    run.args.route_rear_timeout = .001
    run.args.route_rear_align_distance = .01
    count = [0]
    def spin(dt):
        count[0] += 1
        run.pose = (1.5, 8.5, 0.)  # 已横移50cm，仍允许完成居中确认。
    run.spin = spin
    run.centered_target_candidates = lambda *a, **kw: NS(
        x_min=.45, x_max=.55, confidence=.9, stamp=NS(sec=count[0], nanosec=0))
    run.stop = lambda: None
    run.aligned_intake_approach = lambda: True
    assert run.rear_visual_scoop()
    assert count[0] == 3


@pytest.mark.parametrize('heading', [0., math.pi/2, -1.2])
def test_distance_scoop_finishes_with_simulated_low_speed_deadzone(monkeypatch, heading):
    mod = load(); run = make_runner(mod)
    run.pose = (0., 0., heading); run.pose_stamp = 10.; run.yaw_rate = 0.
    run.args.route_scoop_distance = .7
    run.args.route_scoop_timeout = .01  # 旧总时限不再中断持续有进展的后退。
    run.args.route_scoop_min_speed = .05
    now, commands, stops = [0.], [], []
    monkeypatch.setattr(mod.time, 'monotonic', lambda: now[0])
    run._Twist = lambda: NS(linear=NS(x=0., y=0.), angular=NS(z=0.))
    run.stop = lambda: stops.append(now[0])
    def spin(dt):
        now[0] += dt
        # 模拟小于0.04m/s时底盘无法克服摩擦，其余速度按时间积分。
        vx = commands[-1] if commands and abs(commands[-1]) >= .04 else 0.
        run.pose = (run.pose[0]+vx*dt*math.cos(heading),
                    run.pose[1]+vx*dt*math.sin(heading), heading)
    run.spin = spin
    run.publish_command = lambda tw: commands.append(tw.linear.x)
    assert run.rear_distance_scoop()
    assert now[0] > run.args.route_scoop_timeout
    assert all(.05 <= abs(v) <= .08 for v in commands)
    assert stops[-1] == now[0]
    assert math.hypot(*run.pose[:2]) == pytest.approx(.68, abs=.003)


def test_distance_scoop_respects_low_speed_cap_and_reports_stalled_timeout(monkeypatch):
    mod = load(); run = make_runner(mod)
    run.pose = (0., 0., 0.); run.pose_stamp = 10.; run.yaw_rate = 0.
    run.args.route_scoop_speed = .02
    run.args.route_scoop_min_speed = .05
    run.args.motion_stall_seconds = .15
    now, commands, stops, messages = [0.], [], [], []
    monkeypatch.setattr(mod.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(mod, 'log', lambda tag, msg, *a: messages.append(msg))
    run.spin = lambda dt: now.__setitem__(0, now[0]+dt)
    run._Twist = lambda: NS(linear=NS(x=0., y=0.), angular=NS(z=0.))
    run.stop = lambda: stops.append(now[0])
    run.publish_command = lambda tw: commands.append(tw.linear.x)
    assert not run.rear_distance_scoop()
    assert commands and all(v == -.02 for v in commands)
    assert stops[-1] == now[0]
    assert '后退距离持续无进展' in messages[-1]
    assert float(messages[-1].split('已走 ')[1].split('m')[0]) == 0.
    assert '剩余 0.200m' in messages[-1]
    assert '最后指令速度 0.020m/s' in messages[-1]


def test_rear_accepts_observed_two_percent_offset_then_starts_approach():
    mod=load();run=make_runner(mod)
    run.args.route_rear_timeout=.2
    samples=iter([NS(x_min=.469,x_max=.569,confidence=.918,
                     stamp=NS(sec=10+i,nanosec=0)) for i in range(3)])
    run.spin=lambda sec:None
    run.centered_target_candidates=lambda *a,**kw:next(samples)
    run.stop=lambda:None
    events=[]
    run.rear_distance_scoop=lambda:events.append('approach') or True
    assert run.rear_visual_scoop()
    assert events==['approach']


def intake_runner():
    mod = load()
    run = make_runner(mod)
    run.args.route_intake = True
    events = []
    run.intake_text = lambda cmd: events.append(cmd) or (True, 'sent')
    run.spin = lambda sec: events.append(('wait', sec))
    run.stop = lambda: events.append('chassis_stop')
    run.rear_distance_scoop = lambda: events.append('approach') or True
    return run, events


def test_automatic_scoop_runs_after_approach_then_lowers_after_exactly_one_second():
    run, events = intake_runner()
    assert run.aligned_intake_approach()
    assert events == ['approach', 'chassis_stop', 'dji_stop', ('wait', .3),
                      'dji_zero', ('wait', .8), 'dji_move 1 90 5', ('wait', 1.), 'dji_move 1 0 5']


def test_approach_failure_never_sends_shovel_commands():
    run, events = intake_runner()
    run.rear_distance_scoop = lambda: events.append('approach') or False
    assert not run.aligned_intake_approach()
    assert events == ['approach', 'chassis_stop']


def test_mechanism_switch_off_keeps_approach_but_skips_every_serial_command():
    run, events = intake_runner()
    run.args.route_intake = False
    assert run.aligned_intake_approach()
    assert events == ['approach', 'chassis_stop']


def test_unplugged_intake_is_reported_after_approach_without_claiming_scoop_success():
    run, events = intake_runner()
    run.intake_text = lambda cmd: events.append(cmd) or (False, '串口未打开')
    assert not run.aligned_intake_approach()
    assert events == ['approach', 'chassis_stop', 'dji_stop', 'dji_move 1 0 5']


def test_failed_lift_skips_one_second_hold_and_attempts_lowering():
    run, events = intake_runner()
    run.intake_text = lambda cmd: events.append(cmd) or (cmd != 'dji_move 1 90 5', 'result')
    assert not run.aligned_intake_approach()
    assert ('wait', 1.) not in events
    assert events[-2:] == ['dji_move 1 90 5', 'dji_move 1 0 5']


def test_failed_lowering_does_not_complete_the_pickup_action():
    run, events = intake_runner()
    run.intake_text = lambda cmd: events.append(cmd) or (cmd != 'dji_move 1 0 5', 'result')
    assert not run.aligned_intake_approach()
    assert events[-2:] == [('wait', 1.), 'dji_move 1 0 5']


def test_interrupted_hold_still_lowers_without_suppressing_cancellation():
    run, events = intake_runner()
    def wait(sec):
        events.append(('wait', sec))
        if sec == 1.:
            raise KeyboardInterrupt()
    run.spin = wait
    with pytest.raises(KeyboardInterrupt):
        run.aligned_intake_approach()
    assert events[-1] == 'dji_move 1 0 5'


def test_resumed_pickup_already_at_distance_still_scoops_without_driving_again():
    run, events = intake_runner()
    run.rear_distance_scoop = lambda: pytest.fail('already approached: no repeat movement')
    assert run.aligned_intake_approach(approach=False)
    assert events == ['chassis_stop', 'dji_stop', ('wait', .3), 'dji_zero', ('wait', .8),
                      'dji_move 1 90 5', ('wait', 1.), 'dji_move 1 0 5']
