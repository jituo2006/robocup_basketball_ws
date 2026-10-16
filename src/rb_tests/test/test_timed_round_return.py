"""Deadline return, cooperative interruption and mechanism completion; no ROS/hardware."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from match_round_route import RoundRoute, load_config
import match_round_steps as stages
from round_return_budget import RoundReturnBudget, ReturnRequested, check_runner, translation_seconds
from test_perimeter_ball import Runner
from test_round_mirror import runner as basic_runner


def make_round_fixture(tmp_path, monkeypatch, side='A', number=3, enabled=True):
    monkeypatch.setattr(stages, 'STATE_DIR', tmp_path)
    cfg = load_config(start_point=side)
    cfg['timed_return']['enabled'] = enabled
    r = basic_runner((12., -7., .37))
    r.args.route_intake = True
    r.args.route_heading_tol = .1
    r.args.route_scoop_speed = .08
    r.args.route_scoop_distance = .44
    r.args.route_scoop_timeout = 15.
    clock = [0.]
    real_budget = stages.RoundReturnBudget
    monkeypatch.setattr(stages, 'RoundReturnBudget', lambda *a, **kw: real_budget(*a, **kw, clock=lambda: clock[0]))
    r.intake_text = lambda command: r.events.append(('text', command)) or (True, 'simulated')
    def scan(x, y, yaw, **kwargs):
        r.pose = (x, y, yaw)
        return 'found', NS(bearing_rad=0.)
    def scoop(approach=True):
        r.events.append(('pickup', approach))
        return True
    r.scan_to = scan
    r.rear_visual_scoop = r.aligned_intake_approach = scoop
    return r, cfg, clock


@pytest.mark.parametrize('side', ['A', 'B'])
@pytest.mark.parametrize('number', [1, 2, 3])
@pytest.mark.parametrize('interrupt', ['travel', 'scan', 'rear', 'approach'])
def test_deadline_unwinds_to_entry_turn_start_with_original_frame(tmp_path, monkeypatch, side, number, interrupt):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch, side, number)
    origin = r.pose
    def request():
        clock[0] = 85.
        check_runner(r)
    if interrupt == 'travel':
        goto = r.goto
        calls = [0]
        def move(x, y, **kwargs):
            calls[0] += 1
            if calls[0] == 1:
                request()
            return goto(x, y, **kwargs)
        r.goto = move
    elif interrupt == 'scan':
        r.scan_to = lambda *a, **kw: request()
    elif interrupt == 'rear':
        r.rear_visual_scoop = lambda **kw: request()
    else:
        r.aligned_intake_approach = lambda **kw: request()
    assert RoundRoute(r, cfg, number, 'shoot', 'cycle', emit=lambda m: None).execute()
    state = stages.load_session(number, 'shoot')
    assert state['returned_early'] and state['timed_return']
    assert state['start_pose'] == list(origin)
    expected_releases = 1 if number == 1 and interrupt != 'travel' else 0
    assert state.get('completed_releases', 0) == expected_releases
    workflow = stages.workflow_state(number, 'shoot')
    assert not workflow['steps'][1]['done']
    assert workflow['steps'][2]['done']
    frame = r.round_start_frame
    motion = [e for e in r.events if e[0] in ('move', 'turn')]
    assert motion[-4:] == [
        ('move', *frame.point(cfg['corridor'])),
        ('move', *frame.point(cfg['entry'])),
        ('turn', frame.yaw(0.)),
        ('move', *frame.point(cfg['start']))]
    assert r.pose == pytest.approx(origin)
    assert r.round_return_budget is None


def test_dynamic_return_budget_includes_slow_speed_and_entry_to_start(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    profile = cfg['speed_profiles']['3']
    profile.update(travel_mps=.25, carry_mps=.25)
    budget = RoundReturnBudget(r, cfg, 3, 'shoot', clock=lambda: clock[0])
    r.pose = (*budget.frame.point(cfg['release']['shoot']['point']), .37)
    assert budget.return_seconds() > 50.
    clock[0] = 120.-budget.return_seconds()
    with pytest.raises(ReturnRequested):
        budget.check()


def test_translation_model_matches_terminal_slowdown():
    assert translation_seconds(10., 1.) == pytest.approx(12.33, abs=.05)
    assert translation_seconds(.02, 1.) == 0.
    assert translation_seconds(10., .65) > translation_seconds(10., 1.)


def test_pickup_finishes_exact_fixed_sequence_before_return(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    budget = RoundReturnBudget(r, cfg, 3, 'shoot', clock=lambda: clock[0])
    r.round_return_budget = budget
    r.rear_distance_scoop = lambda: True
    r.spin = lambda seconds: (clock.__setitem__(0, clock[0]+seconds), check_runner(r))
    send = r.intake_text
    def slow_send(command):
        if command == 'dji_move 1 90 5':
            clock[0] = 84.
        return send(command)
    r.intake_text = slow_send
    with pytest.raises(ReturnRequested):
        Runner.aligned_intake_approach(r)
    assert [e[1] for e in r.events if e[0] == 'text'] == [
        'dji_stop', 'dji_zero', 'dji_move 1 90 5', 'dji_move 1 0 5', 'dji_stop']
    assert r.round_pickup_completed
    assert clock[0] == pytest.approx(94.)
    assert budget.deferred == 0


def test_no_new_pickup_when_sequence_would_consume_reserve(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    r.round_return_budget = RoundReturnBudget(r, cfg, 3, 'shoot', clock=lambda: clock[0])
    r.rear_distance_scoop = lambda: True
    clock[0] = 80.
    with pytest.raises(ReturnRequested):
        Runner.aligned_intake_approach(r)
    assert not any(e[0] == 'text' for e in r.events)


def test_shot_finishes_reset_before_return(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    r.round_return_budget = RoundReturnBudget(r, cfg, 3, 'shoot', clock=lambda: clock[0])
    r.spin = lambda seconds: (clock.__setitem__(0, clock[0]+seconds), check_runner(r))
    send = r.intake_text
    def slow_send(command):
        if command == 'vesc_shoot 3':
            clock[0] = 84.
        return send(command)
    r.intake_text = slow_send
    with pytest.raises(ReturnRequested):
        RoundRoute(r, cfg, 3, 'shoot', 'collect', emit=lambda m: None).release()
    assert [e[1] for e in r.events if e[0] == 'text'] == cfg['release']['shoot']['commands']
    assert r.round_release_completed


@pytest.mark.parametrize('action', ['pickup', 'release'])
def test_completed_sequence_at_deadline_preserves_payload_and_action_count(tmp_path, monkeypatch, action):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    if action == 'pickup':
        def pickup(**kwargs):
            r.round_pickup_completed = True
            clock[0] = 86.
            check_runner(r)
        r.aligned_intake_approach = pickup
    else:
        send = r.intake_text
        def finish(command):
            if command == 'vesc_reset':
                clock[0] = 86.
            return send(command)
        r.intake_text = finish
    assert RoundRoute(r, cfg, 3, 'shoot', 'cycle', emit=lambda m: None).execute()
    state = stages.load_session(3, 'shoot')
    assert state['carrying'] == (action == 'pickup')
    assert state.get('completed_releases', 0) == (1 if action == 'release' else 0)
    assert state['returned_early']


def test_deadline_before_second_release_does_not_count_previous_release_twice(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    original_action = RoundRoute.action_point
    visits = [0]
    def action(plan, *args, **kwargs):
        ok = original_action(plan, *args, **kwargs)
        visits[0] += 1
        if visits[0] == 2:
            clock[0] = 86.
        return ok
    monkeypatch.setattr(RoundRoute, 'action_point', action)
    assert RoundRoute(r, cfg, 3, 'shoot', 'cycle', emit=lambda m: None).execute()
    state = stages.load_session(3, 'shoot')
    assert state['completed_releases'] == 1
    assert state['carrying']
    assert not stages.workflow_state(3, 'shoot')['steps'][1]['done']


def test_serial_failure_during_deferred_sequence_is_not_converted_to_return(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    r.round_return_budget = RoundReturnBudget(r, cfg, 3, 'shoot', clock=lambda: clock[0])
    def fail(command):
        clock[0] = 86.
        return False, 'serial failed'
    r.intake_text = fail
    assert not RoundRoute(r, cfg, 3, 'shoot', 'collect', emit=lambda m: None).release()
    assert not r.events
    assert not getattr(r, 'round_release_completed', False)


def test_unknown_service_result_stops_instead_of_driving_home(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    budget = RoundReturnBudget(r, cfg, 3, 'shoot', clock=lambda: clock[0])
    with budget.mechanism(3.):
        clock[0] = 86.
        with pytest.raises(RuntimeError, match='机构服务结果未确认'):
            budget.check_service_wait()


@pytest.mark.parametrize('available', [True, False])
def test_real_serial_wait_observes_budget_and_stops_on_unknown_result(tmp_path, monkeypatch, available):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    budget = RoundReturnBudget(r, cfg, 3, 'shoot', clock=lambda: clock[0])
    r.round_return_budget = budget
    monkeypatch.setitem(sys.modules, 'rb_msgs', NS())
    monkeypatch.setitem(sys.modules, 'rb_msgs.srv', NS(SerialText=NS(Request=NS)))
    r.intake_client = NS(wait_for_service=lambda **kw: available,
                         call_async=lambda req: NS(done=lambda: False))
    r.rclpy = NS(ok=lambda: True, spin_once=lambda *a, **kw: clock.__setitem__(0, 86.))
    r.node = NS()
    with budget.mechanism(3.):
        with pytest.raises(RuntimeError, match='机构服务结果未确认'):
            Runner.intake_text(r, 'vesc_home')
    assert not r.events


def test_motion_fault_precedes_deadline_at_command_outlet(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    r.round_return_budget = RoundReturnBudget(r, cfg, 3, 'shoot', clock=lambda: clock[0])
    clock[0] = 86.
    r.motion_guard = True
    def fault():
        raise RuntimeError('localization fault')
    r.require_fresh_pose = fault
    r.cmd_pub = NS(publish=lambda msg: pytest.fail('fault must prevent motion'))
    with pytest.raises(RuntimeError, match='localization fault'):
        Runner.publish_command(r, NS())


def test_normal_home_is_not_restarted_when_clock_passes_cutoff(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    original_home = RoundRoute.home
    called = []
    def home(plan):
        called.append(True)
        clock[0] = 119.
        check_runner(r)
        return original_home(plan)
    monkeypatch.setattr(RoundRoute, 'home', home)
    assert RoundRoute(r, cfg, 3, 'shoot', 'cycle', emit=lambda m: None).execute()
    assert len(called) == 1
    state = stages.load_session(3, 'shoot')
    assert not state.get('timed_return')


@pytest.mark.parametrize('fault', ['localization failed', 'invalid_pose'])
def test_fault_at_deadline_never_starts_return_motion(tmp_path, monkeypatch, fault):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    def scan(*a, **kw):
        if fault == 'invalid_pose':
            r.pose = None
        else:
            r.motion_fault = fault
        clock[0] = 85.
        check_runner(r)
    r.scan_to = scan
    assert not RoundRoute(r, cfg, 3, 'shoot', 'cycle', emit=lambda m: None).execute()
    state = stages.load_session(3, 'shoot')
    assert state['next_step'] == 3 and state['last_error']
    assert not state.get('returned_early')


def test_real_spin_wait_checks_budget_without_motion(tmp_path, monkeypatch):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch)
    r.round_return_budget = RoundReturnBudget(r, cfg, 3, 'shoot', clock=lambda: clock[0])
    clock[0] = 84.99
    r.motion_guard = False
    r.rclpy = NS(spin_once=lambda *a, **kw: clock.__setitem__(0, clock[0]+.02))
    r.node = NS()
    with pytest.raises(ReturnRequested):
        Runner.spin(r, .2)
    assert not r.events


@pytest.mark.parametrize('task,enabled', [('pass', True), ('shoot', False)])
def test_opt_out_and_passing_unchanged(tmp_path, monkeypatch, task, enabled):
    r, cfg, clock = make_round_fixture(tmp_path, monkeypatch, enabled=enabled)
    r.round_return_budget = RoundReturnBudget(r, cfg, 3, task, clock=lambda: clock[0])
    clock[0] = 500.
    check_runner(r)
    assert not r.round_return_budget.requested


@pytest.mark.parametrize('key,value', [('round_duration_s', float('nan')), ('return_reserve_s', 120.),
                                      ('margin_s', -1.), ('enabled', 'yes'), ('tasks', ['other'])])
def test_invalid_budget_configuration_rejected(tmp_path, key, value):
    import json
    cfg = load_config(start_point='A')
    cfg['timed_return'][key] = value
    path = tmp_path/'config.json'
    path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError):
        load_config(path)
