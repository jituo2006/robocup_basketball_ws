"""Deterministic simulated MCU; never opens hardware."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools'))
from mechanism_control import MechanismController, ReplyInbox, MechanismError, Token


class FakeMCU:
    def __init__(self):
        self.time = 0.
        self.seq = self.tx = 0
        self.inbox = ReplyInbox()
        self.sent, self.events = [], []
        self.target = self.position = 0
        self.move_until = 0
        self.zero = self.online = self.home = 1
        self.dji_fault = 'NONE'
        self.run = self.reset_run = 0
        self.active = False
        self.shot_reason = 'timed release'
        self.reset_valid = 'YES'
        self.no_auto_home = False
        self.overrides = {}
        self.c = MechanismController(self.send, self.pump, self.inbox, clock=lambda: self.time,
            ack_timeout=.5, move_timeout=2., shot_timeout=2., reset_timeout=2.)

    def emit(self, kind, line, tx=None):
        self.seq += 1
        self.inbox.feed(dict(epoch='test', seq=self.seq, tx_id=self.tx if tx is None else tx,
                             kind=kind, line=line))

    def rx(self, line):
        self.emit('rx', line)

    def later(self, delay, function):
        self.events.append((self.time+delay, function))

    def pump(self, delay):
        self.time += max(delay, 1e-6)
        due = [e for e in self.events if e[0] <= self.time+1e-8]
        self.events = [e for e in self.events if e not in due]
        for _, function in sorted(due, key=lambda e:e[0]):function()

    def result(self):
        self.rx(f'RESULT FINISHED run={self.run} reason={self.shot_reason} timing_speed=PASS')
        self.rx('VERIFY actual stroke/ball departure/battery current: UNVERIFIED. RAM only; export trace.')

    def send(self, command):
        self.sent.append((self.time, command))
        self.tx += 1
        self.emit('tx', command)
        boundary = self.seq
        if command in self.overrides:self.overrides[command]()
        elif command == 'dji_status':
            moving = self.time < self.move_until
            if not moving:self.position = self.target
            self.rx(f'STATUS {"MOVING" if moving else "HOLD"} MODE=1 '
                f'L={self.position} R={-self.position} LV={2 if moving else 0} RV={-2 if moving else 0} '
                f'LO={self.online} RO={self.online} ZERO={self.zero} FAULT={self.dji_fault} CAN=0')
        elif command.startswith('dji_move'):
            self.target = int(command.split()[2]); self.move_until = self.time+.5
            self.rx('ACK DJI_MOVE 1 MOVING'); self.rx('DONE')
        elif command == 'dji_stop':self.rx('ACK DJI_STOP')
        elif command == 'dji_zero':
            self.zero = 1; self.rx('ACK DJI_ZERO')
        elif command == 'vesc_status':
            for line in [f'--VESC dual {"RUN" if self.active else "IDLE"} bus=CAN2 ids=1,2 poles=7 home={self.home} diag=10',
                         'CAN_ESR=0x00000000 fault_id=0 fault=none (STM32 checks only)',
                         'CAN live bus_off=0 TEC=0 REC=0 LEC=0']:self.rx(line)
            for i in (1,2):
                self.rx(f'MOTOR id={i} rx=100 age=1 ms actual=0 ERPM current=0 mA duty=0/1000')
                self.rx(f'CONTROL id={i} ref=0 ERPM cmd=0 mA tx_queued=100 tx_fail=0 pending=0x0')
        elif command == 'vesc_stop':
            self.home = 0
            self.rx('--VESC stop: both 0 A mailboxes cleared; coast (no active brake).')
        elif command == 'vesc_home':
            self.home = 1; self.rx('--Manual start-position confirmation accepted; encoder_home=1.')
        elif command == 'vesc_result':
            if self.run:self.result()
            else:self.rx('--No shot result.')
        elif command.startswith(('vesc_shoot ', 'vesc_shoot2 ')):
            self.run += 1; self.home = 0; self.active = True
            self.rx('--SHOT starting both once; 0 A at 600 ms; vesc_stop aborts both.')
            def finish():
                self.active = False; self.result()
            self.later(.6, finish)
        elif command == 'vesc_reset_result':
            self.rx('--No reset result.' if not self.reset_run else
                    f'RESET_RESULT FINISHED run={self.reset_run} valid=YES fault_id=0')
        elif command == 'vesc_reset':
            self.active = True; self.reset_run += 1
            self.rx('--RESET top assumed; fixed reverse=300 RPM (-2100 ERPM), duration=1200 ms; vesc_stop aborts.')
            def finish():
                self.active = False
                self.rx(f'RESET_RESULT FINISHED run={self.reset_run} valid={self.reset_valid} '
                        'reason=duration home=0 home_pending=1 fault_id=0 TOP_ASSUMED BOTTOM_TIME_ASSUMED')
                if not self.no_auto_home:self.later(.2, lambda: setattr(self, 'home', 1))
            self.later(1.2, finish)
        else:self.rx('ERR ARG')
        return True, f'written EPOCH=test TX_ID={self.tx} SEQ={boundary}'


def commands(m):return [c for _,c in m.sent]


def test_pickup_waits_for_targets_and_five_seconds_after_arrival():
    m=FakeMCU(); ok,msg=m.c.pickup(5.)
    assert ok,msg
    moves=[(t,c) for t,c in m.sent if c.startswith('dji_move')]
    assert [c for _,c in moves]==['dji_move 1 90 5','dji_move 1 0 5']
    assert moves[1][0]-moves[0][0]>=5.5 and m.time-moves[1][0]>=.5
    assert not any(c in ('dji_stop','dji_zero') for c in commands(m))


@pytest.mark.parametrize('reply',['ERR BUSY','ERR NOT_STILL','ERR FAULT'])
def test_rejected_lift_returns_without_extra_stop_or_lowering(reply):
    m=FakeMCU();m.overrides['dji_move 1 90 5']=lambda:m.rx(reply)
    ok,msg=m.c.pickup(0.)
    assert not ok and reply in msg
    assert commands(m)[-1]=='dji_move 1 90 5'
    assert 'dji_stop' not in commands(m) and 'dji_move 1 0 5' not in commands(m)


def test_no_ack_does_not_resend_motion():
    m=FakeMCU();m.overrides['dji_move 1 90 5']=lambda:None
    assert not m.c.pickup(0.)[0]
    assert commands(m).count('dji_move 1 90 5')==1


def test_early_done_or_wrong_target_does_not_count_as_arrival():
    m=FakeMCU()
    m.overrides['dji_move 1 90 5']=lambda:(m.rx('ACK DJI_MOVE 1 MOVING'),m.rx('DONE'))
    assert not m.c.pickup(0.)[0] and 'dji_move 1 0 5' not in commands(m)


@pytest.mark.parametrize('attribute,value',[('zero',0),('online',0),('dji_fault','CAN')])
def test_bad_dji_status_blocks_motion(attribute,value):
    m=FakeMCU();setattr(m,attribute,value)
    assert not m.c.pickup()[0] and not any(c.startswith('dji_move') for c in commands(m))


def test_lowering_rejection_is_not_success():
    m=FakeMCU();m.overrides['dji_move 1 0 5']=lambda:m.rx('ERR BUSY')
    assert not m.c.pickup(0.)[0] and commands(m).count('dji_move 1 0 5')==1


def test_initialization_is_explicit():
    m=FakeMCU();m.zero=0
    assert m.c.initialize_dji()[0] and commands(m)[:2]==['dji_stop','dji_zero']


def test_release_waits_three_seconds_and_reset_home():
    m=FakeMCU();ok,msg=m.c.release(['vesc_stop','vesc_home','vesc_shoot 3','vesc_stop','vesc_reset'])
    assert ok,msg
    shot=next(t for t,c in m.sent if c=='vesc_shoot 3')
    assert [t for t,c in m.sent if c=='vesc_stop'][1]-shot>=3.
    reset=next(t for t,c in m.sent if c=='vesc_reset')
    assert m.time-reset>=1.4 and m.home==1


@pytest.mark.parametrize('command,reply',[
 ('vesc_stop','ERR FAULT'),('vesc_home','--Home refused: feedback missing.'),
 ('vesc_shoot 3','--Shoot refused: no home.'),('vesc_reset','--Reset refused: not stationary.')])
def test_release_rejection_prevents_advance(command,reply):
    m=FakeMCU();m.overrides[command]=lambda:m.rx(reply)
    ok,msg=m.c.release(['vesc_stop','vesc_home','vesc_shoot 3','vesc_stop','vesc_reset'])
    assert not ok and reply in msg
    if command!='vesc_reset':assert 'vesc_reset' not in commands(m)
    assert commands(m).count('vesc_shoot 3')<=1


@pytest.mark.parametrize('reason',['user abort','feedback timeout','coast timeout/unconfirmed stop'])
def test_abnormal_shot_result_prevents_reset(reason):
    m=FakeMCU();m.shot_reason=reason
    assert not m.c.release(['vesc_shoot 3','vesc_reset'])[0] and 'vesc_reset' not in commands(m)


def test_current_high_finished_result_does_not_block_following_reset():
    m=FakeMCU();messages=[];m.c.log=messages.append
    m.shot_reason='motor current feedback high'
    def result():
        m.rx(f'RESULT FINISHED run={m.run} reason=motor current feedback high timing_speed=INVALID;')
        m.rx('VERIFY actual stroke/ball departure/battery current: UNVERIFIED. RAM only; export trace.')
    m.result=result
    ok,message=m.c.release(['vesc_stop','vesc_home','vesc_shoot 3','vesc_stop','vesc_reset'])
    assert ok,message
    assert commands(m).count('vesc_shoot 3')==1
    assert 'vesc_reset' in commands(m) and m.home==1
    assert any('motor current feedback high timing_speed=INVALID;' in line for line in messages)


@pytest.mark.parametrize('fault',['invalid_reset','missing_home'])
def test_reset_invalid_or_missing_home_fails(fault):
    m=FakeMCU();m.home=0;m.reset_valid='NO' if fault=='invalid_reset' else 'YES'
    m.no_auto_home=fault=='missing_home'
    assert not m.c.release(['vesc_reset'])[0]


def test_old_result_run_cannot_complete_new_shot():
    m=FakeMCU();m.run=9
    m.overrides['vesc_shoot 3']=lambda:(m.rx('--SHOT starting both once;'),
        m.rx('RESULT FINISHED run=9 reason=timed release timing_speed=PASS'))
    assert not m.c.release(['vesc_shoot 3'])[0] and commands(m).count('vesc_shoot 3')==1


@pytest.mark.parametrize('kind,line',[('disconnect','USB removed'),('tx','vesc_shoot 6')])
def test_disconnect_or_external_tx_aborts(kind,line):
    m=FakeMCU();m.overrides['dji_move 1 90 5']=lambda:(m.rx('ACK DJI_MOVE 1 MOVING'),m.emit(kind,line))
    assert not m.c.pickup(0.)[0]


def test_old_node_without_tx_metadata_fails():
    m=FakeMCU();m.c.send=lambda command:(True,'sent')
    assert not m.c.pickup()[0]


def test_lost_event_fails_closed():
    b=ReplyInbox();b.feed(dict(epoch='e',seq=1,tx_id=1,kind='tx',line='a'))
    b.feed(dict(epoch='e',seq=3,tx_id=1,kind='rx',line='DONE'))
    with pytest.raises(MechanismError):b.after(Token('e',1,1))


def test_restart_cannot_reuse_reply():
    b=ReplyInbox();b.feed(dict(epoch='old',seq=1,tx_id=1,kind='tx',line='a'))
    b.feed(dict(epoch='new',seq=1,tx_id=1,kind='rx',line='DONE'))
    with pytest.raises(MechanismError):b.after(Token('old',1,1))


def test_interrupt_propagates_without_extra_stop_or_lowering():
    m=FakeMCU()
    def interrupt():raise KeyboardInterrupt
    m.overrides['dji_move 1 90 5']=interrupt
    with pytest.raises(KeyboardInterrupt):m.c.pickup(0.)
    assert commands(m)[-1]=='dji_move 1 90 5'
    assert 'dji_stop' not in commands(m) and 'dji_move 1 0 5' not in commands(m)


@pytest.mark.parametrize('group',['dji','vesc'])
@pytest.mark.parametrize('error_type',[MechanismError,KeyboardInterrupt])
def test_failure_cleanup_never_sends_extra_mechanism_command(group,error_type):
    m=FakeMCU();messages=[];m.c.log=messages.append
    def fail():raise error_type('simulated fault')
    if error_type is KeyboardInterrupt:
        with pytest.raises(KeyboardInterrupt):m.c._run(group,fail)
    else:
        ok,message=m.c._run(group,fail)
        assert not ok and 'simulated fault' in message
    assert commands(m)==[]
    assert any('未额外发送机构停止指令' in line for line in messages)
    assert m.c._run(group,lambda:None)[0], 'failure must release action lock'
