"""Exact two-pickup order, persistent frame, and failure isolation; no ROS motion."""
import math
import subprocess
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from first_pass_route import execute, load_config as saved_config, validate_release
from match_round_route import StartFrame, load_config as load_round

WS = Path(__file__).parents[3]


def load_config(*args):
    # Preserve regression coverage of the optional legacy base-reference route;
    # radar-reference has separate tests.
    cfg = saved_config(*args)
    cfg.update(target_reference='base',coordinate_frame='legacy_x_basket',pass_point=[3.6,4.5],return_point=[3.725,4.8],second_scan_end=[3.725,0.])
    return cfg


def runner(outcomes=('found', 'found'), origin=(1.5, 8., 0.), mechanism=False):
    events, results = [], iter(outcomes)
    r = NS(pose=origin, args=NS(speed=.25, step_tol=.02, route_scoop_distance=.4,
                               route_scoop_timeout=15., route_scoop_speed=.08,
                               route_intake=mechanism))
    r.stop = lambda: events.append(('stop',))
    r.spin = lambda seconds: events.append(('wait', seconds))
    def move(x, y, **kwargs):
        events.append(('move', x, y))
        r.pose = (x, y, r.pose[2])
        return True
    def turn(yaw, **kwargs):
        events.append(('turn', yaw))
        r.pose = (*r.pose[:2], yaw)
        return True
    def scan(x, y, heading, **kwargs):
        assert kwargs['detect'] and not kwargs['centered']
        events.append(('scan', x, y, heading, r.args.ball_label))
        status = next(results)
        # A found ball stops partway, rather than pretending to reach the endpoint.
        r.pose = ((r.pose[0]+x)/2, (r.pose[1]+y)/2, heading) if status == 'found' else (x, y, heading)
        return status, NS()
    def scoop():
        events.append(('scoop', r.args.route_scoop_distance))
        return True
    def send(command):
        events.append(('text', command))
        return True, 'sent'
    r.goto, r.rotate_to, r.scan_to, r.rear_visual_scoop, r.intake_text = move, turn, scan, scoop, send
    return r, events


@pytest.mark.parametrize('yaw', [0., math.pi/2, -.9])
def test_two_pickups_use_requested_points_and_one_start_frame(yaw):
    origin = (2., 3., yaw)
    r, events = runner(origin=origin)
    base, cfg = load_round(), load_config()
    frame = StartFrame(origin, base['start'])
    assert execute(r, base, cfg, emit=lambda message: None)
    route = [e for e in events if e[0] in ('move', 'scan', 'scoop')]
    assert [e[0] for e in route] == ['move', 'scan', 'scoop', 'move', 'move', 'scan', 'scoop', 'move']
    assert route[0][1:] == pytest.approx(frame.point((1.5, 4.8)))
    assert route[3][1:] == pytest.approx(frame.point((3.6, 4.5)))
    assert route[4][1:] == pytest.approx(frame.point((3.725, 4.8)))
    assert route[5][1:3] == pytest.approx(frame.point((3.725, 0.)))
    assert route[5][-1] == 'ball_basketball'
    assert route[5][3] == pytest.approx(frame.yaw(0.))
    assert r.pose[:2] == pytest.approx(frame.point((3.6, 4.5)))
    assert r.args.route_scoop_distance == .4 and events[-1] == ('stop',)


def test_empty_first_sweep_continues_left_then_finds_first_ball():
    r, events = runner(('empty', 'found', 'found'))
    assert execute(r, load_round(), load_config(), emit=lambda m: None)
    scans = [e for e in events if e[0] == 'scan']
    assert [s[1] for s in scans] == [4.4, .6, 3.725]


def test_no_first_ball_never_shovels_or_starts_second_search():
    r, events = runner(('empty', 'empty'))
    assert not execute(r, load_round(), load_config(), emit=lambda m: None)
    assert not [e for e in events if e[0] == 'scoop']
    assert len([e for e in events if e[0] == 'scan']) == 2
    assert events[-1] == ('stop',)


def test_no_second_ball_stops_at_exact_requested_endpoint_without_second_shovel():
    r, events = runner(('found', 'empty'))
    assert not execute(r, load_round(), load_config(), emit=lambda m: None)
    assert r.pose[:2] == pytest.approx((3.725, 0.))
    assert len([e for e in events if e[0] == 'scoop']) == 1


def test_failed_first_release_prevents_second_search():
    r, events = runner(mechanism=True)
    cfg = load_config(); cfg['first_release'] = 'drop'
    r.intake_text = lambda command: (False, '串口断开')
    assert not execute(r, load_round(), cfg, emit=lambda m: None)
    assert len([e for e in events if e[0] == 'scan']) == 1


def test_first_drop_runs_once_and_second_point_has_no_automatic_release():
    r, events = runner(mechanism=True)
    cfg = load_config(); cfg['first_release'] = 'drop'
    assert execute(r, load_round(), cfg, emit=lambda m: None)
    assert [e for e in events if e[0] == 'text'] == [('text', 'dji_move 1 0 5')]


def test_default_reserves_release_interface_and_match_needs_verified_commands():
    base, cfg = load_round(), load_config()
    validate_release(base, cfg, True)
    validate_release(base, cfg, False)
    cfg['first_release'] = 'match'
    with pytest.raises(ValueError, match='已验证'):
        validate_release(base, cfg, True)


def test_default_mechanism_pickups_continue_without_any_passing_serial_command():
    r, events = runner(mechanism=True)
    r.intake_text = lambda command: pytest.fail('传球串口接口尚未启用')
    assert execute(r, load_round(), load_config(), emit=lambda m: None)
    assert len([e for e in events if e[0] == 'scoop']) == 2
    assert r.pose[:2] == pytest.approx((3.6, 4.5))


def test_cancellation_stops_and_restores_scoop_parameters():
    r, events = runner()
    r.rear_visual_scoop = lambda: (_ for _ in ()).throw(KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        execute(r, load_round(), load_config(), emit=lambda m: None)
    assert events[-1] == ('stop',) and r.args.route_scoop_distance == .4


def test_plan_wrapper_runs_without_ros_and_prints_exact_coordinates():
    result = subprocess.run(['bash', str(WS/'tools/run_first_pass.sh'), '--plan'],
                            cwd=WS, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    import json
    plan = json.loads(result.stdout)
    assert plan['return_point'] == [0., 4.8]
    assert plan['second_scan_end'] == [0., 0.]






def test_modern_two_pickups_end_with_radar_at_confirmed_passing_point():
    r,events=runner();cfg=saved_config();assert 'pillar_required' not in cfg
    assert execute(r,load_round(),cfg,emit=lambda s:None)
    radar=(r.pose[0]+.3*math.cos(r.pose[2]),r.pose[1]+.3*math.sin(r.pose[2]))
    assert r.round_start_frame.nominal(radar)==pytest.approx([-1.9,3.45])
    assert len([e for e in events if e[0]=='scoop'])==2
    scans=[e for e in events if e[0]=='scan']
    x,y,yaw=scans[-1][1:4]
    radar_endpoint=(x+.3*math.cos(yaw),y+.3*math.sin(yaw))
    assert r.round_start_frame.nominal(radar_endpoint)==pytest.approx([0.,0.])
