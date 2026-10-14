"""2026 回合差异、实际搜索中断、来路退出、机构失败和 UI 分发。无实物动作。"""
import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from types import MethodType

import pytest

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS/'tools'))
from match_round_route import RoundRoute, StartFrame, load_config, search_order, search_plan, validate_mode
from test_perimeter_ball import Runner


def test_custom_routes_and_tuning_are_not_rejected_by_former_arbitrary_ranges(tmp_path):
    import json
    cfg = load_config()
    cfg['middle'].update(y=2., left_x=2., right_x=6., scoop_m=2.1)
    cfg['arc'].update(center=[7.,3.], radius=1., angles_deg=[0.,90.])
    cfg['inner'].update(x=5., top_y=6., bottom_y=.2)
    cfg['pillar_alignment'].update(stable_frames=15, timeout_s=30., kp=4.,
                                    camera_yaw_offset_rad=2*math.pi,
                                    launcher_yaw_offset_deg=360.)
    cfg['release']['pass'].update(wait_s=60., search_heading_deg=360.)
    path = tmp_path/'route.json'
    path.write_text(json.dumps(cfg))
    assert load_config(path) == cfg


def fake_runner():
    events = []
    r = NS(pose=(0., 0., math.pi/2), dets=[], yaw_rate=0.,
           args=NS(speed=.25, step_tol=.02, route_scoop_distance=.4,
                   route_scoop_timeout=15., route_scoop_speed=.08,
                   route_heading_tol=.1, kp=1.5, kp_yaw=2., kd_yaw=.25, max_ang=.3))
    r.progress_watchdog = MethodType(Runner.progress_watchdog, r)
    def goto(x, y, **kw):
        events.append(('move', x, y))
        r.pose = (x, y, r.pose[2])
        return True
    def turn(yaw, **kw):
        events.append(('turn', yaw))
        r.pose = (*r.pose[:2], yaw)
        return True
    def scoop(approach=True):
        if not approach:
            events.append(('align',))
            return True
        events.append(('scoop', r.args.ball_label, r.args.route_scoop_distance))
        x, y, yaw = r.pose
        r.pose = (x-.2*math.cos(yaw), y-.2*math.sin(yaw), yaw)
        return True
    def send(command):
        events.append(('text', command))
        return True, 'sent'
    r.goto, r.rotate_to, r.rear_visual_scoop, r.intake_text = goto, turn, scoop, send
    r.aligned_intake_approach = scoop
    r.stop = lambda: events.append(('stop',))
    r.spin = lambda seconds: None
    r.detection_fresh = lambda stamp: True
    det = NS(label='pillar', source='camera', confidence=.9, bearing_rad=0.,
             x_min=.45, x_max=.55, y_min=.2, y_max=.8, stamp=NS(sec=10, nanosec=0))
    r.dets = [det]
    def spin(seconds):
        det.stamp.nanosec += 1_000_000
    r.spin = spin
    r._Twist = lambda: NS(linear=NS(x=0., y=0.), angular=NS(z=0.))
    def publish(tw):
        x, y, yaw = r.pose
        r.pose = (x+.2*(math.cos(yaw)*tw.linear.x-math.sin(yaw)*tw.linear.y),
                  y+.2*(math.sin(yaw)*tw.linear.x+math.cos(yaw)*tw.linear.y),
                  yaw+.2*tw.angular.z)
    r.publish_command = publish
    r.centered_target_candidates = lambda dets, **kwargs: NS()
    r.scan_to = MethodType(Runner.scan_to, r)
    return r, events


def test_each_round_has_own_start_frame_without_resetting_localization():
    f = StartFrame((0., 0., math.pi/2), (1.5, 8.))
    assert f.point((1.5, 4.8)) == pytest.approx((3.2, 0.))
    assert f.yaw(-math.pi/2) == pytest.approx(0.)
    assert f.point((1.5, 8.)) == pytest.approx((0., 0.))
    assert f.nominal(f.point((8.2, 1.3))) == pytest.approx((8.2, 1.3))


def test_rule_target_regions_differ_by_round_and_task():
    assert search_order(2, 'pass') == search_order(2, 'shoot') == ['middle', 'arc']
    assert search_order(3, 'pass') == ['arc', 'inner']
    assert search_order(3, 'shoot') == ['inner', 'arc']


@pytest.mark.parametrize('cycle', [0, 1])
def test_middle_always_starts_at_diagram_right_end_before_scanning_left(cycle):
    points = search_plan(load_config(), 'middle', cycle)
    assert [p for p, yaw in points] == [(4.4, 4.8), (.6, 4.8)]
    assert all(yaw == pytest.approx(-math.pi/2) for p, yaw in points)


def test_middle_first_outward_sweep_stops_before_right_end_and_remembers_actual_pickup_point():
    cfg = load_config(); r, events = fake_runner(); r.pose = (1.5, 8., 0.)
    plan = RoundRoute(r, cfg, 2, 'pass', 'collect', emit=lambda msg: None)
    def detect(dets, centered=True):
        assert not centered
        assert r.pose == pytest.approx((1.5, 4.8, -math.pi/2))
        events.append(('detect',))
        return NS()
    r.centered_target_candidates = detect
    assert plan.search_area('middle', 0, search_only=True) == 'found'
    assert [e[1:] for e in events if e[0] == 'move'] == [(1.5, 6.8), (1.5, 4.8)]
    assert events.index(('turn', -math.pi/2)) < events.index(('move', 1.5, 4.8))
    assert plan.trail[:3] == [(1.5, 6.8), (1.5, 4.8), (1.5, 4.8)]


def test_middle_scan_velocity_is_towards_diagram_left_after_preposition():
    cfg = load_config(); r, events = fake_runner(); r.pose = (1.5, 8., 0.)
    plan = RoundRoute(r, cfg, 2, 'pass', 'collect', emit=lambda msg: None)
    commands = []; publish = r.publish_command
    searching_left = [False]
    def command(tw):
        commands.append((tw.linear.x, tw.linear.y))
        if tw.linear.y < 0:
            searching_left[0] = True
        publish(tw)
    r.publish_command = command
    r.centered_target_candidates = lambda dets, **kwargs: NS() if searching_left[0] else None
    assert plan.search_area('middle', 1, search_only=True) == 'found'
    assert commands[0] == pytest.approx((0., .25))
    assert any(c == pytest.approx((0., -.25)) for c in commands)
    assert plan.search_origin[0] < 4.4


def test_middle_first_sweep_motion_failure_does_not_acquire_target():
    cfg = load_config(); r, events = fake_runner(); r.pose = (1.5, 8., 0.)
    plan = RoundRoute(r, cfg, 2, 'pass', 'collect', emit=lambda msg: None)
    r.scan_to = lambda *args, **kwargs: ('failed', None)
    assert plan.search_area('middle', 0) == 'failed'
    assert not any(e[0] == 'scoop' for e in events)


@pytest.mark.parametrize('number,task,area,distance', [
    (2, 'pass', 'middle', .70), (2, 'shoot', 'middle', .70),
    (3, 'pass', 'arc', .25), (3, 'shoot', 'inner', .15)])
def test_collect_uses_correct_class_then_unwinds_to_entry_and_stops_at_action(number, task, area, distance):
    cfg = load_config(); r, events = fake_runner()
    plan = RoundRoute(r, cfg, number, task, 'collect', emit=lambda msg: None)
    assert plan.execute()
    scoop = next(e for e in events if e[0] == 'scoop')
    assert scoop == ('scoop', 'ball_basketball' if task == 'pass' else 'ball_volleyball', distance)
    assert not [e for e in events if e[0] == 'text']
    # 吸球位移后第一个回撤点是原检测点，而非直接跨过置球线到动作点。
    pickup_index = events.index(scoop)
    next_move = next(e for e in events[pickup_index+1:] if e[0] == 'move')
    # 中线允许在向右端第一遍横移中提前停车，返程必须回实际检测点。
    expected = plan.search_origin[:2]
    assert next_move[1:] == pytest.approx(expected)
    assert r.pose[:2] == pytest.approx(plan.frame.point(cfg['release'][task]['point']))
    # 单次默认不会回起点或继续取第二颗。
    assert sum(e[0] == 'scoop' for e in events) == 1


@pytest.mark.parametrize('number,task', [(2,'pass'), (3,'pass'), (3,'shoot')])
def test_empty_field_drive_returns_to_start_without_any_actuator(number, task):
    r, events = fake_runner(); start = r.pose
    r.centered_target_candidates = lambda dets, **kwargs: pytest.fail('drive must not inspect targets')
    r.rear_visual_scoop = lambda: pytest.fail('drive must not collect')
    r.intake_text = lambda command: pytest.fail('drive must not actuate')
    assert RoundRoute(r, load_config(), number, task, 'drive', emit=lambda msg: None).execute()
    assert r.pose == pytest.approx(start)


def test_missing_middle_targets_falls_back_to_arc_but_motion_failure_does_not():
    r, events = fake_runner()
    plan = RoundRoute(r, load_config(), 2, 'pass', 'collect', emit=lambda msg: None)
    areas = []
    plan.search_area = lambda area, cycle: areas.append(area) or ('empty' if area=='middle' else 'found')
    assert plan.execute()
    assert areas == ['middle', 'arc']
    areas.clear()
    plan.search_area = lambda area, cycle: areas.append(area) or 'failed'
    assert not plan.execute()
    assert areas == ['middle']


def test_actual_search_center_detection_stops_before_next_velocity():
    r, events = fake_runner(); cfg = load_config()
    plan = RoundRoute(r, cfg, 2, 'pass', 'collect', emit=lambda msg: None)
    r.publish_command = lambda tw: pytest.fail('centered target must stop immediately')
    assert plan.scan((4.4, 4.8), -math.pi/2) == 'found'
    assert events == [('stop',)]


def test_wrong_heading_does_not_translate_when_recovery_fails():
    r, events = fake_runner()
    r.centered_target_candidates = lambda dets, **kwargs: None
    plan = RoundRoute(r, load_config(), 3, 'shoot', 'collect', emit=lambda msg: None)
    r.publish_command = lambda tw: pytest.fail('wrong heading must not move')
    r.rotate_to = lambda *args, **kwargs: False
    assert plan.scan((8.2, 1.3), 0.3) == 'failed'


def test_cycle_requires_real_release_configuration_before_motion():
    r, events = fake_runner()
    with pytest.raises(ValueError, match='出球'):
        RoundRoute(r, load_config(), 2, 'pass', 'cycle')
    assert events == []


@pytest.mark.parametrize('succeeds', [True, False])
def test_two_cycles_return_home_only_after_commands_and_abort_on_release_failure(succeeds):
    cfg = load_config();cfg['release']['pass'].update(aim=[5., 7.4], commands=['TEST_RELEASE'])
    r, events = fake_runner(); start = r.pose
    def send(command):
        events.append(('text', command))
        return succeeds, 'mock'
    r.intake_text = send
    plan = RoundRoute(r, cfg, 2, 'pass', 'cycle', emit=lambda msg: None)
    assert plan.execute() is succeeds
    assert sum(e[0]=='scoop' for e in events) == (2 if succeeds else 1)
    assert sum(e[0]=='text' for e in events) == (2 if succeeds else 1)
    if succeeds:
        assert r.pose == pytest.approx(start)


def console():
    spec = importlib.util.spec_from_file_location('round_console', WS/'tools/console_app.py')
    mod = importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod


def test_frontend_reads_current_config_defaults_without_actuating():
    mod=console(); unused=NS()
    client=mod.make_app(unused,unused,unused,unused,unused,unused,{}).test_client()
    cfg=client.get('/api/round-config').get_json()
    assert cfg['ok'] and cfg['scoop_m']=={'middle':.70,'arc':.25,'inner':.15}
    assert cfg['release_ready']=={'pass':False,'shoot':False}
    assert cfg['forward_m']==pytest.approx(3.2)
    assert cfg['middle_scan']==pytest.approx({'start_x':4.4, 'end_x':.6,
                                             'preposition_m':2.9, 'distance_m':3.8})


@pytest.mark.parametrize('name', ['TEST_ROUND2', 'TEST_ROUND3'])
def test_frontend_routes_call_tool_with_task_mode_and_distances(name):
    mod = console(); calls=[]
    tool = NS(start=lambda key,args: calls.append((key,args)) or (True,'started'))
    unused=NS()
    client=mod.make_app(unused,unused,tool,unused,unused,unused,{}).test_client()
    response = client.post('/api/action',json={'name':name,'args':{'task':'shoot','mode':'collect',
        'inner_scoop':.2,'scoop_speed':.06,'start_intake':False}}).get_json()
    assert response['ok']
    key,args=calls[0]
    assert key==name and args[:4]==['--round-task','shoot','--round-mode','collect']
    assert args[args.index('--round-inner-scoop')+1]=='0.2'
    assert '--no-route-intake' in args


@pytest.mark.parametrize('args', [{'task':'basketball'}, {'mode':'arbitrary'},
    {'inner_scoop':float('nan')}, {'arc_scoop':-1}, {'scoop_speed':0},
    {'mode':'cycle','release_kind':'match'}])
def test_bad_ui_parameters_never_launch_motion(args):
    mod=console();tool=NS(start=lambda *args:pytest.fail('invalid settings cannot start'))
    unused=NS()
    client=mod.make_app(unused,unused,tool,unused,unused,unused,{}).test_client()
    assert not client.post('/api/action',json={'name':'TEST_ROUND3','args':args}).get_json()['ok']


def test_volleyball_mode_uses_same_filter_on_front_and_rear_and_ignores_basketballs():
    spec=importlib.util.spec_from_file_location('round_fixed',WS/'tools/test_perimeter_ball.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    r=mod.Runner.__new__(mod.Runner)
    r.args=NS(ball_label='ball_volleyball',ball_confidence=.5,route_center_tolerance=.04)
    r.detection_fresh=lambda stamp:True
    def det(label):
        return NS(label=label,source='camera',confidence=.9,x_min=.45,x_max=.55,
                  y_min=.4,y_max=.6,stamp=NS(sec=10,nanosec=0))
    basketball,volleyball=det('ball_basketball'),det('ball_volleyball')
    assert r.centered_target_candidates([basketball,volleyball]) is volleyball
    assert r.centered_target_candidates([basketball],centered=False) is None


def test_nonzero_or_zero_sensor_origin_uses_nominal_field_bounds_without_changing_pose():
    spec=importlib.util.spec_from_file_location('round_bounds',WS/'tools/test_perimeter_ball.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    r=mod.Runner.__new__(mod.Runner)
    r.round_start_frame=StartFrame((0.,0.,0.),(1.5,8.))
    r.pose=(*r.round_start_frame.point((8.2,1.3)),0.)
    original=r.pose
    r.field_l,r.field_w=14.,7.5
    r.pose_stamp=r.odom_stamp=10.
    r.args=NS(pose_timeout=.5)
    r.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)))
    r.stop=lambda:pytest.fail('valid nominal field pose must not stop')
    r.require_fresh_pose()
    assert r.pose==original


def test_round1_preloaded_release_then_only_one_middle_pickup_and_return():
    cfg=load_config();r,events=fake_runner();start=r.pose
    plan=RoundRoute(r,cfg,1,'pass','cycle',release_kind='drop',emit=lambda msg:None)
    assert plan.execute()
    release_indices=[i for i,e in enumerate(events) if e==('text','dji_move 1 0 5')]
    scoop_indices=[i for i,e in enumerate(events) if e[0]=='scoop']
    assert len(release_indices)==2 and len(scoop_indices)==1
    assert release_indices[0] < scoop_indices[0] < release_indices[1]
    assert r.pose==pytest.approx(start)


@pytest.mark.parametrize('round_number',[1,2,3])
def test_every_stage_returns_stopped_and_preserves_initial_frame_across_new_runner(tmp_path,monkeypatch,round_number):
    import match_round_steps as stages
    monkeypatch.setattr(stages,'STATE_DIR',tmp_path)
    cfg=load_config();r,events=fake_runner();start=r.pose
    for index,(kind,label) in enumerate(stages.steps(round_number),1):
        # 每个按钮都起新进程，新 Runner 从当前位姿开始，但规划起点必须继承。
        current=r.pose
        r,events=fake_runner();r.pose=current
        assert stages.execute_step(r,cfg,round_number,'pass',index,emit=lambda msg:None)
        assert events[-1]==('stop',)
        state=stages.load_session(round_number,'pass')
        assert state['start_pose']==list(start) and state['next_step']==index+1
        assert r.round_start_frame.pose==start
        if kind in ('first','second'):
            assert [e for e in events if e[0]=='text']==[('text','dji_move 1 0 5')]
            if kind=='second' or round_number!=1:
                assert ('align',) in events and any(e[0]=='scoop' for e in events)
    assert r.pose==pytest.approx(start)
    assert stages.workflow_state(round_number,'pass')['finished']


def test_step_order_rejects_out_of_sequence_and_restart_invalidates_old_progress(tmp_path,monkeypatch):
    import match_round_steps as stages
    monkeypatch.setattr(stages,'STATE_DIR',tmp_path)
    cfg=load_config();r,events=fake_runner()
    with pytest.raises(ValueError,match='第1阶段'):
        stages.validate_step(2,'pass',2,cfg,'drop')
    assert stages.execute_step(r,cfg,2,'pass',1,emit=lambda msg:None)
    with pytest.raises(ValueError,match='第2阶段'):
        stages.validate_step(2,'pass',3,cfg,'drop')
    r.goto=lambda *args,**kw:False
    assert not stages.execute_step(r,cfg,2,'pass',1,emit=lambda msg:None)
    assert stages.load_session(2,'pass')['next_step']==1


def test_failed_stage_does_not_advance_or_release_ball(tmp_path,monkeypatch):
    import match_round_steps as stages
    monkeypatch.setattr(stages,'STATE_DIR',tmp_path)
    cfg=load_config();r,events=fake_runner()
    assert stages.execute_step(r,cfg,2,'pass',1,emit=lambda msg:None)
    events.clear()
    r.goto=lambda *args,**kw:False
    assert not stages.execute_step(r,cfg,2,'pass',2,emit=lambda msg:None)
    assert stages.load_session(2,'pass')['next_step']==2
    assert not [e for e in events if e[0]=='text']


def test_approach_retry_only_completes_remaining_distance_and_never_reads_yolo(tmp_path,monkeypatch):
    import match_round_steps as stages
    monkeypatch.setattr(stages,'STATE_DIR',tmp_path)
    cfg=load_config();r,events=fake_runner()
    r.aligned_intake_approach=lambda:False
    assert not stages.execute_step(r,cfg,2,'pass',1,emit=lambda msg:None)
    original_start=stages.load_session(2,'pass')['start_pose']
    x,y,yaw=r.pose
    r.pose=(x-.12*math.cos(yaw),y-.12*math.sin(yaw),yaw)
    r.centered_target_candidates=lambda *args:pytest.fail('approach must not read YOLO')
    distances=[]
    r.aligned_intake_approach=lambda:distances.append(r.args.route_scoop_distance) or True
    assert stages.execute_step(r,cfg,2,'pass',1,emit=lambda msg:None)
    assert distances==pytest.approx([.58])  # 0.70m总行程减去已经后退的0.12m。
    assert stages.load_session(2,'pass')['start_pose']==original_start


def test_round_state_api_and_round1_first_button(tmp_path,monkeypatch):
    import match_round_steps as stages
    monkeypatch.setattr(stages,'STATE_DIR',tmp_path)
    mod=console();calls=[];unused=NS()
    tool=NS(start=lambda key,args:calls.append((key,args)) or (True,'started'))
    client=mod.make_app(unused,unused,tool,unused,unused,unused,{}).test_client()
    status=client.get('/api/round-state?task=pass').get_json()
    assert status['ok'] and [len(r['steps']) for r in status['rounds']]==[3,3,3]
    response=client.post('/api/action',json={'name':'TEST_ROUND1','args':{'step':1}}).get_json()
    assert response['ok'] and '--round-step' in calls[0][1]
