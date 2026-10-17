"""Fake clock + UART events only; no ROS or hardware I/O."""
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from serial_actions import SerialActions, ProtocolError
from mechanism_control import ReplyInbox

SHOOT = ['vesc_stop', 'vesc_home', 'vesc_shoot2 3', 'vesc_stop', 'vesc_reset', 'vesc_stop']


class MCU:
    def __init__(self):
        self.t = 0.
        self.seq = self.tx = 0
        self.epoch = 'test'
        self.inbox = ReplyInbox()
        self.sent, self.scheduled, self.overrides, self.logs = [], [], {}, []
        self.position = self.home = self.shot_run = self.reset_run = 0
        self.c = SerialActions(self.send, self.pump, self.inbox,
                               clock=lambda: self.t, log=self.logs.append)

    def emit(self, kind, line, tx=None):
        self.seq += 1
        self.inbox.feed(dict(kind=kind, line=line, seq=self.seq,
                            tx_id=self.tx if tx is None else tx,
                            epoch=self.epoch, monotonic_ns=int(self.t * 1e9)))

    def rx(self, line):
        self.emit('rx', line)

    def later(self, seconds, callback):
        self.scheduled.append((self.t + seconds, callback))

    def pump(self, seconds):
        self.t += seconds
        ready = sorted((x for x in self.scheduled if x[0] <= self.t + 1e-10), key=lambda x: x[0])
        for item in ready:
            self.scheduled.remove(item)
            item[1]()

    def status(self):
        self.rx(f'--VESC dual IDLE bus=CAN2 ids=1,2 poles=7 home={self.home} diag=10')
        self.rx('CAN_ESR=0x00000000 fault_id=0 fault=none (STM32 checks only)')
        for i in (1, 2):
            self.rx(f'MOTOR id={i} rx=100 age=1 ms actual=0 ERPM current=0 mA duty=0/1000')
            self.rx(f'CONTROL id={i} ref=0 ERPM cmd=0 mA tx_queued=100 tx_fail=0 pending=0x0')

    def reply(self, command):
        if command in self.overrides:
            return self.overrides[command]()
        if command == 'dji_stop':
            self.rx('ACK DJI_STOP')
        elif command == 'dji_zero':
            self.rx('ACK DJI_ZERO')
        elif command.startswith('dji_move'):
            self.rx('ACK DJI_MOVE 1 MOVING')
            def done():
                self.position = int(command.split()[2])
                self.rx('DONE')
            self.later(.2, done)
        elif command == 'vesc_stop':
            self.home = 0
            self.rx('--VESC stop: both 0 A mailboxes cleared; coast (no active brake).')
        elif command == 'vesc_status':
            self.status()
        elif command == 'vesc_home':
            self.home = 1
            self.rx('--Manual start-position confirmation accepted; encoder_home=1.')
        elif command == 'vesc_shoot2 3':
            self.home = 0
            self.shot_run += 1
            self.rx('--SHOT2 starting both; feedback=MT6816; 0 A at 850 ms.')
            def finished():
                self.rx(f'RESULT FINISHED run={self.shot_run} reason=timed release timing_speed=PASS')
                self.rx('VERIFY fault_id=0 VESC motor faults/physical synchrony UNREAD')
                self.rx('VERIFY actual stroke/ball departure/battery current: UNVERIFIED. RAM only; export trace.')
            self.later(.4, finished)
        elif command == 'vesc_reset':
            self.reset_run += 1
            self.rx('--RESET top assumed; fixed reverse=300 RPM (-2100 ERPM), duration=1200 ms; vesc_stop aborts.')
            self.later(1.2, lambda: self.rx(f'RESET_RESULT FINISHED run={self.reset_run} valid=YES reason=timed release target_rpm=-300 target_erpm=-2100 duration=1200 ms home=0 home_pending=1 fault_id=0 TOP_ASSUMED BOTTOM_TIME_ASSUMED'))
            def home():
                self.home = 1
                self.rx('--RESET bottom zero established; home=1 (timed return assumption).')
            self.later(1.4, home)
        else:
            raise AssertionError(command)

    def send(self, command, deadline):
        assert self.t < deadline
        self.tx += 1
        self.emit('tx', command)
        meta = f'EPOCH={self.epoch} TX_ID={self.tx} SEQ={self.seq}'
        self.sent.append((self.t, command))
        self.reply(command)
        return True, meta


def commands(m):
    return [s for _, s in m.sent]


def test_pickup_waits_done_holds_one_second_and_never_queries_status():
    m = MCU()
    report = m.c.pickup()
    assert report.confirmed, report.message
    assert commands(m) == ['dji_stop', 'dji_zero', 'dji_move 1 90 5', 'dji_move 1 0 5', 'dji_stop']
    times = {c: t for t, c in m.sent}
    assert times['dji_move 1 0 5'] - times['dji_move 1 90 5'] == pytest.approx(1.2, abs=.015)
    assert m.position == 0


@pytest.mark.parametrize('failing', ['dji_stop', 'dji_zero', 'dji_move 1 90 5', 'dji_move 1 0 5'])
@pytest.mark.parametrize('reply', ['', 'ERR BUSY', 'ERR ARG', 'WRONG ACK'])
def test_twenty_attempts_at_50ms_then_skip_and_continue(failing, reply):
    m = MCU()
    m.overrides[failing] = lambda: m.rx(reply) if reply else None
    report = m.c.pickup()
    assert not report.confirmed and m.t < 5.
    times = [t for t, c in m.sent if c == failing]
    if failing == 'dji_stop':
        times = times[:20]  # final stop is another exchange
    assert len(times) == 20
    assert times == pytest.approx([times[0] + .05*i for i in range(20)], abs=.006)
    assert commands(m)[-1] == 'dji_stop' and 'dji_move 1 0 5' in commands(m)


def test_delayed_ack_retries_share_tx_group_then_stop_retrying():
    m = MCU()
    m.overrides['dji_zero'] = lambda: m.later(.12, lambda: m.rx('ACK DJI_ZERO'))
    report = m.c.pickup()
    assert report.confirmed, report.message
    assert commands(m).count('dji_zero') == 3
    assert commands(m).count('dji_move 1 90 5') == 1


@pytest.mark.parametrize('state', ['MOVING', 'HOMING', 'HOLD'])
def test_ack_done_same_packet_and_idempotent_hold(state):
    m = MCU()
    def respond():
        m.rx('ACK DJI_MOVE 1 ' + state)
        if state != 'HOLD':
            m.rx('DONE')
    m.overrides['dji_move 1 90 5'] = respond
    assert m.c.pickup().confirmed


def test_done_without_ack_is_not_acceptance():
    m = MCU()
    m.overrides['dji_move 1 90 5'] = lambda: m.rx('DONE')
    report = m.c.pickup()
    assert not report.steps[2].confirmed
    assert commands(m).count('dji_move 1 90 5') == 20


@pytest.mark.parametrize('failure', ['missing', 'ERR FAULT CAN'])
def test_done_independent_deadline_and_fault_does_not_hang(failure):
    m = MCU()
    def respond():
        m.rx('ACK DJI_MOVE 1 MOVING')
        if failure != 'missing':
            m.later(.1, lambda: m.rx(failure))
    m.overrides['dji_move 1 90 5'] = respond
    report = m.c.pickup()
    assert not report.steps[2].confirmed
    assert commands(m).count('dji_move 1 90 5') == 1
    assert commands(m)[-1] == 'dji_stop' and m.t < 17.


def test_cancel_during_hold_never_lowers_or_runs_cleanup_motion():
    m = MCU()
    base = m.pump
    def pump(seconds):
        base(seconds)
        if m.t >= .4:
            raise KeyboardInterrupt
    m.c.pump = pump
    with pytest.raises(KeyboardInterrupt):
        m.c.pickup()
    assert 'dji_move 1 0 5' not in commands(m)


def test_normal_shoot2_reset_and_final_stop_without_status_or_home_gate():
    m = MCU()
    report = m.c.shoot(SHOOT)
    assert report.confirmed, report.message
    assert commands(m) == SHOOT
    assert commands(m).count('vesc_shoot2 3') == 1 and commands(m).count('vesc_reset') == 1
    assert commands(m).count('vesc_stop') == 3 and m.home == 0 and m.t < 3.
    assert m.c.shoot(SHOOT).confirmed


@pytest.mark.parametrize('command,ack', [
    ('vesc_stop', '--VESC stop: both 0 A mailboxes cleared; coast (no active brake).'),
    ('vesc_home', '--Manual start-position confirmation accepted; encoder_home=1.'),
    ('vesc_shoot2 3', '--SHOT2 starting both; feedback=MT6816; 0 A at 850 ms.'),
    ('vesc_reset', '--RESET top assumed; fixed reverse=300 RPM')])
def test_all_shoot_steps_retry_at_50ms_until_ack(command, ack):
    m = MCU()
    m.overrides[command] = lambda: m.later(.12, lambda: m.rx(ack))
    ex, result = m.c.accept(command, lambda line: line == ack)
    assert result.confirmed and result.attempts == 3
    assert [t for t, _ in m.sent] == pytest.approx([0., .05, .1], abs=.006)
    assert ex.ack == ack


@pytest.mark.parametrize('reply', [
    '--Home refused: healthy CAN and fresh stationary feedback required.',
    '--Manual start-position confirmation accepted for both motors; encoder_home=0; no automatic return.'])
def test_bad_home_is_recorded_but_shoot_reset_and_final_stop_are_attempted(reply):
    m = MCU()
    m.overrides['vesc_home'] = lambda: m.rx(reply)
    report = m.c.shoot(SHOOT)
    assert not report.confirmed
    assert 'vesc_shoot2 3' in commands(m) and 'vesc_reset' in commands(m)
    assert commands(m)[-1] == 'vesc_stop'
    assert commands(m).count('vesc_home') == (20 if 'refused:' in reply else 1)


@pytest.mark.parametrize('header', [
    'RESULT FINISHED run=1 reason=shaft encoder invalid/stale timing_speed=INVALID',
    'RESULT FINISHED run=1 reason=user abort timing_speed=PASS',
    'RESULT FINISHED run=1 reason=timed release timing_speed=INVALID'])
def test_failed_shot_is_recorded_then_stop_reset_and_stop_continue(header):
    m = MCU()
    def shoot():
        m.rx('--SHOT2 starting both; feedback=MT6816')
        m.later(.1, lambda: m.rx(header))
        m.later(.12, lambda: m.rx('VERIFY actual stroke/ball departure/battery current: UNVERIFIED. RAM only; export trace.'))
    m.overrides['vesc_shoot2 3'] = shoot
    report = m.c.shoot(SHOOT)
    assert not report.confirmed and not report.steps[2].confirmed
    assert commands(m) == SHOOT
    assert commands(m).count('vesc_shoot2 3') == 1 and commands(m).count('vesc_reset') == 1


def test_missing_shot_completion_is_bounded_then_reset_is_attempted_without_refiring():
    m = MCU()
    m.overrides['vesc_shoot2 3'] = lambda: m.rx('--SHOT2 starting both; feedback=MT6816')
    report = m.c.shoot(SHOOT)
    assert not report.confirmed and m.t < 8.
    assert commands(m) == SHOOT


@pytest.mark.parametrize('kind', ['no_result', 'invalid', 'no_home', 'dropped_home_notice'])
def test_reset_errors_and_missing_home_never_skip_final_stop(kind):
    m = MCU()
    def reset():
        m.rx('--RESET top assumed; fixed reverse=300 RPM')
        if kind != 'no_result':
            m.later(1.2, lambda: m.rx('RESET_RESULT FINISHED run=1 valid=' +
                ('NO' if kind == 'invalid' else 'YES') + ' reason=timed release home=0 home_pending=1 fault_id=0'))
        if kind == 'dropped_home_notice':
            m.later(1.4, lambda: setattr(m, 'home', 1))
    m.overrides['vesc_reset'] = reset
    report = m.c.shoot(SHOOT)
    assert report.confirmed == (kind in ('no_home', 'dropped_home_notice')), report.message
    assert commands(m) == SHOOT and m.t < 8.


def test_stale_result_is_unconfirmed_but_does_not_prevent_reset():
    m = MCU()
    m.rx('DONE')
    m.c.last_shot_run = 5
    report = m.c.shoot(SHOOT)
    assert not report.confirmed and commands(m) == SHOOT


@pytest.mark.parametrize('mode', ['disconnect', 'foreign', 'gap', 'restart'])
def test_transport_disruption_fails_exchange(mode):
    m = MCU()
    def response():
        if mode == 'disconnect':
            m.emit('disconnect', 'lost')
        elif mode == 'foreign':
            m.emit('tx', 'operator motion', tx=999)
        elif mode == 'gap':
            m.seq += 1
            m.rx('ACK DJI_ZERO')
        else:
            m.epoch = 'restarted'
            m.rx('ACK DJI_ZERO')
    m.overrides['dji_zero'] = response
    with pytest.raises(ProtocolError):
        m.c.accept('dji_zero', lambda s: s == 'ACK DJI_ZERO')


def test_missing_service_returns_within_one_second_per_step():
    m = MCU()
    def unavailable(command, deadline):
        m.pump(deadline - m.t)
        return False, 'service unavailable'
    m.c.send = unavailable
    report = m.c.pickup()
    assert not report.confirmed and m.t == pytest.approx(5.) and not m.sent


def test_foreign_tx_does_not_allow_later_automatic_motion():
    m = MCU()
    m.overrides['dji_zero'] = lambda: m.emit('tx', 'operator stop', tx=999)
    report = m.c.pickup()
    assert not report.confirmed
    assert commands(m) == ['dji_stop', 'dji_zero']


def test_observed_previous_run_is_recorded_as_unconfirmed_without_stopping_sequence():
    m = MCU()
    m.rx('RESULT FINISHED run=1 reason=timed release timing_speed=PASS')
    report = m.c.shoot(SHOOT)
    assert not report.confirmed and commands(m) == SHOOT


@pytest.mark.parametrize('unavailable', ['service', 'publisher', 'future'])
def test_ros_adapter_bounds_service_wait_and_never_queues_unknown_requests(monkeypatch, unavailable):
    from types import SimpleNamespace as NS
    import serial_actions as module
    clock = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    adapter = module.RosSerialActions.__new__(module.RosSerialActions)
    adapter.unknown = False
    adapter.pending = None
    adapter.request_type = NS
    adapter.node = NS(count_publishers=lambda _: 0 if unavailable == 'publisher' else 1)
    adapter._pump = lambda s: clock.__setitem__(0, clock[0] + s)
    requests = []
    def call(req):
        requests.append(req.command)
        return NS(done=lambda: False)
    adapter.client = NS(service_is_ready=lambda: unavailable != 'service', call_async=call)
    ok, _ = adapter.send('dji_stop', 1.)
    assert not ok and clock[0] == pytest.approx(1.)
    assert requests == (['dji_stop'] if unavailable == 'future' else [])
    if unavailable == 'future':
        ok, _ = adapter.send('dji_zero', 2.)
        assert not ok and requests == ['dji_stop']


def test_no_extra_status_query_blocks_shoot_sequence():
    m = MCU()
    m.overrides['vesc_status'] = lambda: pytest.fail('automatic shoot no longer adds status gates')
    report = m.c.shoot(SHOOT)
    assert report.confirmed and commands(m) == SHOOT


@pytest.mark.parametrize('start_reply', [
    '--SHOT2 starting both; feedback=MT6816 common_shaft single_PI equal_current; 0 A at 850 ms.',
    '--SHOT2 starting right only (left=0 A); feedback=MT6816 common_shaft single_PI current_split; beta=0%; 0 A at 550 ms.',
])
@pytest.mark.parametrize('ack_delay', [0., .12])
@pytest.mark.parametrize('timing_speed', ['PASS', 'INVALID'])
def test_shoot2_start_reply_variants_stop_retries_and_continue_reset(start_reply, ack_delay, timing_speed):
    m = MCU()
    started = False

    def shot():
        nonlocal started
        m.later(ack_delay, lambda: m.rx(start_reply))
        if not started:
            started = True
            def finished():
                m.rx('RESULT FINISHED run=1 reason=timed release timing_speed=' + timing_speed)
                m.rx('VERIFY fault_id=0 VESC motor faults/physical synchrony UNREAD')
                m.rx('VERIFY actual stroke/ball departure/battery current: UNVERIFIED. RAM only; export trace.')
            m.later(.65, finished)

    def reset():
        m.rx('--RESET top assumed; fixed reverse=300 RPM (-2100 ERPM), duration=600 ms; vesc_stop aborts.')
        m.later(.6, lambda: m.rx('RESET_RESULT FINISHED run=1 valid=YES reason=timed release target_rpm=-300 target_erpm=-2100 duration=600 ms home=0 home_pending=1 fault_id=0 TOP_ASSUMED BOTTOM_TIME_ASSUMED'))

    m.overrides['vesc_shoot2 3'] = shot
    m.overrides['vesc_reset'] = reset
    report = m.c.shoot(SHOOT)
    attempts = 1 if ack_delay == 0 else 3
    assert commands(m) == SHOOT[:2] + ['vesc_shoot2 3'] * attempts + SHOOT[3:]
    assert report.steps[2].attempts == attempts
    assert report.steps[2].confirmed == (timing_speed == 'PASS')
    assert all(step.confirmed for step in report.steps[3:])
    shot_times = [t for t, command in m.sent if command == 'vesc_shoot2 3']
    assert shot_times == pytest.approx([.05 * i for i in range(attempts)], abs=.006)
    assert m.sent[2 + attempts][0] >= .65
