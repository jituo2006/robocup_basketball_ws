"""Bounded UART handshakes for automatic pickup and shoot2 (no serial reader).

Transport is rb_launcher/text + rx_event. Own retry TXs form one exchange;
external writes, event loss and reconnects invalidate it. MCU has no wire IDs:
host TX boundaries reduce stale replies but cannot prove mechanical position.
"""
from dataclasses import dataclass, field
import math
import time

from mechanism_control import ReplyInbox, fields

ACK_SECONDS = 1.0
RETRY_SECONDS = .05
MAX_ATTEMPTS = 20
MOVE_SECONDS = 15.0
SHOT_SECONDS = 5.0
RESET_SECONDS = 5.0
HOME_SECONDS = 5.0
PICKUP_RESERVE_SECONDS = 36.0
SHOOT_RESERVE_SECONDS = 32.0


class ProtocolError(RuntimeError):
    pass


@dataclass
class Exchange:
    command: str
    tokens: list = field(default_factory=list)
    lines: list = field(default_factory=list)
    cursor: int = 0
    ack_seq: int = 0
    ack: str = ''


@dataclass
class StepResult:
    command: str
    confirmed: bool
    detail: str
    attempts: int = 0


@dataclass
class ActionReport:
    steps: list = field(default_factory=list)

    @property
    def confirmed(self):
        return bool(self.steps) and all(s.confirmed for s in self.steps)

    @property
    def message(self):
        return '; '.join(f'{s.command}: {s.detail}' for s in self.steps)


class SerialActions:
    def __init__(self, send, pump, inbox, *, clock=time.monotonic, log=lambda message: None):
        # send(command, absolute_deadline) must itself obey that deadline.
        self.send, self.pump, self.inbox = send, pump, inbox
        self.clock, self.log = clock, log
        self.last_shot_run = 0
        self.last_reset_run = 0
        self.transport_error = None

    def _pause(self, seconds):
        end = self.clock() + seconds
        while self.clock() < end:
            self.pump(min(.01, end-self.clock()))

    def _invalidate(self, message):
        self.transport_error = message
        raise ProtocolError(message)

    def _read(self, exchange):
        if self.transport_error:
            raise ProtocolError(self.transport_error)
        if not exchange.tokens:
            return exchange.lines
        with self.inbox.lock:
            if self.inbox.broken:
                self._invalidate(self.inbox.broken)
            events = list(self.inbox.events)
        epoch = exchange.tokens[0][0]
        allowed = {t[1] for t in exchange.tokens}
        if events and events[-1]['epoch'] != epoch:
            self._invalidate('串口节点重启/重连')
        if events and events[0]['epoch'] == epoch and events[0]['seq'] > exchange.cursor+1:
            self._invalidate('串口回复缓存溢出')
        for event in events:
            if event['epoch'] != epoch or event['seq'] <= exchange.cursor:
                continue
            if event['kind'] in ('connect', 'disconnect'):
                self._invalidate('串口连接已改变')
            if event['tx_id'] not in allowed:
                self._invalidate('动作期间有其他串口命令插入')
            if event['kind'] == 'rx':
                exchange.lines.append((event['seq'], event['line']))
            exchange.cursor = event['seq']
        return exchange.lines

    def accept(self, command, match):
        if self.transport_error:
            raise ProtocolError(self.transport_error)
        with self.inbox.lock:
            if self.inbox.broken:
                self._invalidate(self.inbox.broken)
        exchange = Exchange(command)
        start = self.clock()
        deadline, next_send, attempts = start+ACK_SECONDS, start, 0
        last_error = '未收到有效回复'
        seen = 0
        while self.clock() < deadline:
            # Check buffered replies before another retry. ACK+DONE may be together.
            for seq, line in self._read(exchange)[seen:]:
                if line.startswith('==VESC'):
                    self._invalidate('主板重启，禁止自动继续动作')
                if match(line):
                    exchange.ack_seq, exchange.ack = seq, line
                    self.log(f'{command}: 已确认（发送{attempts}次） {line}')
                    return exchange, StepResult(command, True, line, attempts)
                if line.startswith('ERR ') or 'refused:' in line.lower():
                    last_error = line
                    self.log(f'{command}: {line}；继续本次确认窗口')
            seen = len(exchange.lines)
            now = self.clock()
            if attempts < MAX_ATTEMPTS and now+1e-9 >= next_send:
                attempts += 1
                sent_at = self.clock()
                ok, message = self.send(command, deadline)
                if ok:
                    meta = fields(message)
                    try:
                        token = (str(meta['EPOCH']), int(meta['TX_ID']), int(meta['SEQ']))
                    except (KeyError, ValueError):
                        self._invalidate('串口节点缺少TX元数据，请使用新版rb_launcher')
                    if not exchange.tokens:
                        exchange.cursor = token[2]
                    exchange.tokens.append(token)
                else:
                    last_error = message
                # Never catch up with a burst after a slow service response.
                next_send = max(start+attempts*RETRY_SECONDS, sent_at+RETRY_SECONDS)
                continue
            self._pause(min(.005, deadline-self.clock()))
        # Include replies delivered by the final spin at the deadline.
        for seq, line in self._read(exchange)[seen:]:
            if match(line):
                exchange.ack_seq, exchange.ack = seq, line
                return exchange, StepResult(command, True, line, attempts)
        result = StepResult(command, False, f'1秒确认超时/{attempts}次；{last_error}', attempts)
        self.log(f'{command}: {result.detail}，跳过本步骤')
        return exchange, result

    def wait(self, exchange, predicate, seconds, *, ignore_busy=False):
        deadline = self.clock()+seconds
        seen = 0
        while True:
            for seq, line in self._read(exchange)[seen:]:
                if seq <= exchange.ack_seq:
                    continue
                if predicate(line):
                    return line
                if line.startswith('ERR ') and not (ignore_busy and line.startswith('ERR BUSY')):
                    raise ProtocolError('主板动作失败: '+line)
                if line.startswith('==VESC') or line.startswith('--VESC fault'):
                    raise ProtocolError('主板重启/故障: '+line)
            seen = len(exchange.lines)
            if self.clock() >= deadline:
                raise ProtocolError(f'动作完成等待超时（{seconds:g}秒）')
            self._pause(min(.01, deadline-self.clock()))

    def pickup(self):
        report = ActionReport()
        def step(command, ack, move=False):
            try:
                exchange, result = self.accept(command, ack)
                report.steps.append(result)
                if move and result.confirmed and not exchange.ack.endswith(' HOLD'):
                    try:
                        self.wait(exchange, lambda line: line == 'DONE', MOVE_SECONDS, ignore_busy=True)
                        result.detail += ' -> DONE'
                    except ProtocolError as exc:
                        result.confirmed, result.detail = False, str(exc)
                return result.confirmed
            except ProtocolError as exc:
                report.steps.append(StepResult(command, False, str(exc)))
                return False
        step('dji_stop', lambda line: line == 'ACK DJI_STOP')
        step('dji_zero', lambda line: line == 'ACK DJI_ZERO')
        raised = step('dji_move 1 90 5', self._move_ack, True)
        if raised:
            self.log('上升完成，等待1秒后下降')
            self._pause(1.)
        step('dji_move 1 0 5', self._move_ack, True)
        step('dji_stop', lambda line: line == 'ACK DJI_STOP')
        self.log(('捡球全部确认；' if report.confirmed else '捡球含跳过/未确认步骤；')+'继续后续任务，未检测球是否入仓')
        return report

    @staticmethod
    def _move_ack(line):
        return line in ('ACK DJI_MOVE 1 MOVING', 'ACK DJI_MOVE 1 HOMING', 'ACK DJI_MOVE 1 HOLD')

    def _stop(self):
        ex, result = self.accept('vesc_stop', lambda s: s.startswith('--VESC stop: both 0 A mailboxes cleared;'))
        if not result.confirmed:
            raise ProtocolError(result.detail)
        # Stop reply proves TX completion, not standstill. Query only while idle.
        deadline = self.clock()+3.
        stable_since = None
        while self.clock() < deadline:
            query, status = self.accept('vesc_status', lambda s: s.startswith('--VESC dual IDLE '))
            if not status.confirmed:
                raise ProtocolError('停止后状态未确认: '+status.detail)
            self.wait(query, lambda s: s.startswith('CONTROL id=2 '), max(.001, min(1., deadline-self.clock())))
            lines = [line for _, line in self._read(query)]
            header = next((fields(s) for s in lines if s.startswith('--VESC dual ')), {})
            fault = next((fields(s) for s in lines if s.startswith('CAN_ESR=')), {})
            motors = {fields(s).get('id'): fields(s) for s in lines if s.startswith('MOTOR id=')}
            controls = {fields(s).get('id'): fields(s) for s in lines if s.startswith('CONTROL id=')}
            try:
                poles = int(header['poles'])
                healthy = (1 <= poles <= 100 and fault['fault_id'] == '0' and
                           int(fault['CAN_ESR'], 0) & 4 == 0)
                for motor_id in ('1', '2'):
                    m, c = motors[motor_id], controls[motor_id]
                    nums = [float(m[k]) for k in ('age', 'actual')]
                    healthy = healthy and all(math.isfinite(v) for v in nums)
                    healthy = healthy and 0 <= nums[0] < 20 and abs(nums[1]) <= 5*poles
                    healthy = healthy and int(m['rx']) > 0 and int(c['cmd']) == 0 and int(c['pending'], 0) == 0
            except (KeyError, ValueError):
                healthy = False
            if healthy:
                if stable_since is None:
                    stable_since = self.clock()
                if self.clock()-stable_since >= .15:
                    return result
            else:
                stable_since = None
            self._pause(.05)
        raise ProtocolError('停止后3秒内未确认双机新鲜静止反馈')

    def _observed_run(self, prefix):
        with self.inbox.lock:
            lines = [e['line'] for e in self.inbox.events if e['kind'] == 'rx']
        values = [int(fields(s)['run']) for s in lines
                  if s.startswith(prefix) and fields(s).get('run', '').isdigit()]
        return max(values, default=0)

    def shoot(self, commands):
        expected = ['vesc_stop', 'vesc_home', 'vesc_shoot2 3', 'vesc_stop', 'vesc_reset']
        report = ActionReport()
        if list(commands) != expected:
            report.steps.append(StepResult('shoot配置', False, '投篮配置必须为 stop/home/shoot2 3/stop/reset'))
            return report
        stage = 'vesc_stop'
        try:
            report.steps.append(self._stop())
            stage = 'vesc_home'
            ex, result = self.accept(stage, lambda s: s.startswith('--Manual start-position confirmation accepted')
                                     and fields(s).get('encoder_home', '').rstrip('.') == '1')
            report.steps.append(result)
            if not result.confirmed:
                raise ProtocolError('起点确认失败，跳过发射')
            stage = 'vesc_shoot2 3'
            shot_baseline = max(self.last_shot_run, self._observed_run('RESULT '))
            ex, result = self.accept(stage, lambda s: s.startswith('--SHOT2 starting both;'))
            report.steps.append(result)
            if not result.confirmed:
                raise ProtocolError('发射是否启动未确认，停止，不自动再次发射或复位')
            finish_deadline = self.clock()+SHOT_SECONDS
            header = self.wait(ex, lambda s: s.startswith('RESULT FINISHED '), SHOT_SECONDS)
            data = fields(header)
            run = int(data.get('run', '0'))
            # Drain the multi-line result before sending the next command.
            self.wait(ex, lambda s: s.startswith('VERIFY actual stroke/'), max(0., finish_deadline-self.clock()))
            if run <= shot_baseline or 'reason=timed release ' not in header or data.get('timing_speed') not in ('PASS', 'FAIL'):
                raise ProtocolError('本次发射未正常完成: '+header)
            self.last_shot_run = run
            self.log('发射完成: '+header)
            if any('fault_id=' in s and fields(s).get('fault_id') != '0' for _, s in ex.lines):
                raise ProtocolError('发射结果包含故障')
            result.detail += ' -> '+header
            stage = 'vesc_stop'
            report.steps.append(self._stop())
            stage = 'vesc_reset'
            reset_baseline = max(self.last_reset_run, self._observed_run('RESET_RESULT '))
            ex, result = self.accept(stage, lambda s: s.startswith('--RESET top assumed;'))
            report.steps.append(result)
            if not result.confirmed:
                raise ProtocolError('复位启动未确认')
            header = self.wait(ex, lambda s: s.startswith('RESET_RESULT FINISHED '), RESET_SECONDS)
            data = fields(header)
            run = int(data.get('run', '0'))
            if (run <= reset_baseline or data.get('valid') != 'YES' or data.get('fault_id') != '0'
                    or 'reason=timed release ' not in header):
                raise ProtocolError('定时复位未正常完成: '+header)
            self.last_reset_run = run
            self.log('复位结束: '+header)
            result.detail += ' -> '+header
            # The asynchronous home notice can be dropped by MCU output congestion.
            # Once RESET_RESULT finished, status queries are permitted again.
            end = self.clock()+HOME_SECONDS
            home = data.get('home') == '1'
            while not home and self.clock() < end:
                self._read(ex)
                home = any(seq > ex.ack_seq and s.startswith('--RESET bottom zero established; home=1')
                           for seq, s in ex.lines)
                if home:
                    break
                if self.clock()+ACK_SECONDS > end:
                    self._pause(min(.01, end-self.clock()))
                    continue
                query, status = self.accept('vesc_status', lambda s: s.startswith('--VESC dual IDLE '))
                if status.confirmed:
                    self.wait(query, lambda s: s.startswith('CONTROL id=2 '), min(1., end-self.clock()))
                    home = fields(status.detail).get('home') == '1'
                # Queries now own the TX boundary; old reset exchange must not be reread.
                ex = query
                self._pause(.05)
            if not home:
                raise ProtocolError('复位后5秒未确认home=1；未证明到底')
            result.detail += ' -> home=1（定时假定到底）'
            self.log('投篮与定时复位已确认；不追加会清除home的vesc_stop')
        except (ProtocolError, ValueError) as exc:
            if report.steps and report.steps[-1].command == stage:
                report.steps[-1].confirmed, report.steps[-1].detail = False, str(exc)
            else:
                report.steps.append(StepResult(stage, False, str(exc)))
            self.log('投篮流程失败: '+str(exc))
            # Cleanup only requests zero current; never auto-shoot or auto-reset.
            try:
                report.steps.append(self._stop())
            except ProtocolError as stop_exc:
                report.steps.append(StepResult('vesc_stop', False, str(stop_exc)))
        return report


class RosSerialActions:
    def __init__(self, node, pump, *, ok=lambda: True, log=lambda message: None):
        from std_msgs.msg import String
        from rb_msgs.srv import SerialText
        self.node, self.pump, self.ok = node, pump, ok
        self.request_type = SerialText.Request
        self.inbox = ReplyInbox()
        self.subscription = node.create_subscription(String, '/rb_launcher/rx_event',
                                                       lambda msg: self.inbox.feed(msg.data), 512)
        self.client = node.create_client(SerialText, '/rb_launcher/text')
        self.pending = None
        self.unknown = False
        self.controller = SerialActions(self.send, self._pump, self.inbox, log=log)

    def _pump(self, seconds):
        if not self.ok():
            raise KeyboardInterrupt
        self.pump(seconds)

    def send(self, command, deadline):
        if self.unknown:
            return False, '此前串口服务超时，发送结果未知；本进程不再排队新动作'
        while not self.client.service_is_ready() or self.node.count_publishers('/rb_launcher/rx_event') < 1:
            if time.monotonic() >= deadline:
                return False, '串口服务或rx_event未就绪'
            self._pump(min(.01, max(0., deadline-time.monotonic())))
        request = self.request_type()
        request.command = command
        self.pending = self.client.call_async(request)
        while not self.pending.done():
            if time.monotonic() >= deadline:
                self.unknown = True
                return False, '串口服务响应超时，发送结果未知'
            self._pump(min(.01, max(0., deadline-time.monotonic())))
        try:
            response = self.pending.result()
            return bool(response.success), response.message
        except Exception as exc:
            self.unknown = True
            return False, '串口服务异常: '+str(exc)
        finally:
            self.pending = None


def runner_actions(runner):
    client = getattr(runner, 'serial_actions', None)
    if client is None:
        def pump(seconds):
            fault = getattr(runner, 'motion_fault', None)
            if fault:
                raise RuntimeError(fault)
            runner.rclpy.spin_once(runner.node, timeout_sec=seconds)
        client = RosSerialActions(runner.node, pump, ok=runner.rclpy.ok,
                                  log=lambda s: runner.node.get_logger().info(s))
        runner.serial_actions = client
    return client.controller
