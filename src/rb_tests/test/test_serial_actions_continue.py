"""Continue after ordinary MCU errors; simulated events only, no hardware."""
from types import SimpleNamespace as NS
import pytest
from test_serial_actions import MCU, SHOOT, commands


@pytest.mark.parametrize('failed', ['vesc_stop', 'vesc_home', 'vesc_shoot2 3', 'vesc_reset'])
@pytest.mark.parametrize('reply', ['', 'ERR ARG'])
def test_each_ack_failure_does_not_skip_later_steps(failed, reply):
    m = MCU()
    m.overrides[failed] = lambda: m.rx(reply) if reply else None
    report = m.c.shoot(SHOOT)
    assert not report.confirmed and len(report.steps) == 6
    for step in report.steps:
        assert step.attempts == (20 if step.command == failed else 1)
    assert list(dict.fromkeys(commands(m))) == list(dict.fromkeys(SHOOT))
    assert commands(m)[-1] == 'vesc_stop'
    assert commands(m).count('vesc_shoot2 3') <= 20
    assert m.t < 8.


def test_all_commands_error_twenty_times_each_then_sequence_returns():
    m = MCU()
    for c in set(SHOOT):
        m.overrides[c] = lambda: m.rx('ERR FAULT')
    report = m.c.shoot(SHOOT)
    assert len(report.steps) == 6 and not report.confirmed
    assert all(s.attempts == 20 for s in report.steps)
    assert len(m.sent) == 120
    assert m.t == pytest.approx(6., abs=.01)


def test_actual_encoder_invalid_log_still_attempts_reset_and_final_stop():
    m = MCU()
    def shot():
        m.rx('--SHOT2 starting both; feedback=MT6816; 0 A at 850 ms.')
        m.later(.612, lambda: m.rx('RESULT FINISHED run=1 reason=shaft encoder invalid/stale timing_speed=INVALID'))
        m.later(.7, lambda: m.rx('VERIFY actual stroke/ball departure/battery current: UNVERIFIED. RAM only; export trace.'))
    m.overrides['vesc_shoot2 3'] = shot
    report = m.c.shoot(SHOOT)
    assert not report.steps[2].confirmed
    assert 'shaft encoder invalid/stale' in report.steps[2].detail
    assert commands(m) == SHOOT
    assert report.steps[4].confirmed and report.steps[5].confirmed


@pytest.mark.parametrize('case', ['malformed_run', 'fault_result', 'missing_report_tail'])
def test_bad_shot_report_never_skips_reset(case):
    m = MCU()
    def shot():
        m.rx('--SHOT2 starting both; feedback=MT6816')
        run = 'bad' if case == 'malformed_run' else '1'
        m.rx(f'RESULT FINISHED run={run} reason=timed release timing_speed=PASS')
        if case == 'fault_result':
            m.rx('VERIFY fault_id=2 fault=failed')
        if case != 'missing_report_tail':
            m.rx('VERIFY actual stroke/ball departure/battery current: UNVERIFIED. RAM only; export trace.')
    m.overrides['vesc_shoot2 3'] = shot
    report = m.c.shoot(SHOOT)
    assert not report.confirmed and commands(m) == SHOOT and m.t < 8.


@pytest.mark.parametrize('number', [1, 2, 3])
def test_round_release_returns_to_route_even_with_all_step_errors(number):
    from test_match_round_route import fake_runner
    from match_round_route import RoundRoute, load_config
    r, _ = fake_runner()
    r.args.route_intake = True
    m = MCU()
    for c in set(SHOOT):
        m.overrides[c] = lambda: m.rx('ERR BUSY')
    r.serial_actions = NS(controller=m.c)
    plan = RoundRoute(r, load_config(), number, 'shoot', 'collect', emit=lambda s: None)
    plan.heading_ready = plan.target_aligned = True
    assert plan.release()
    assert r.round_release_completed and not r.round_release_confirmed
    assert not plan.carrying and not plan.last_error
    assert len(r.shoot_report.steps) == 6


def test_legacy_five_step_config_also_gets_final_stop():
    m = MCU()
    report = m.c.shoot(SHOOT[:-1])
    assert report.confirmed and commands(m) == SHOOT


def test_user_cancellation_is_not_swallowed_as_an_mcu_error():
    m = MCU()
    pump = m.pump
    def cancel(seconds):
        pump(seconds)
        if m.t > .2:
            raise KeyboardInterrupt
    m.c.pump = cancel
    with pytest.raises(KeyboardInterrupt):
        m.c.shoot(SHOOT)
    assert 'vesc_reset' not in commands(m)
