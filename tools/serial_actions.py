"""Bounded UART handshakes for automatic pickup and shoot2 (no serial reader).

Transport is rb_launcher/text + rx_event. Own retry TXs form one exchange;
external writes, event loss and reconnects invalidate it. MCU has no wire IDs:
host TX boundaries reduce stale replies but cannot prove mechanical position.
"""
from dataclasses import dataclass, field
import time

from mechanism_control import ReplyInbox, fields

ACK_SECONDS = 1.0
RETRY_SECONDS = .05
MAX_ATTEMPTS = 20
MOVE_SECONDS = 15.0
SHOT_SECONDS = 5.0
RESET_SECONDS = 5.0
PICKUP_RESERVE_SECONDS = 36.0
SHOOT_RESERVE_SECONDS = 16.0


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

    def _observed_run(self, prefix):
        with self.inbox.lock:
            lines = [e['line'] for e in self.inbox.events if e['kind'] == 'rx']
        values = [int(fields(s)['run']) for s in lines
                  if s.startswith(prefix) and fields(s).get('run', '').isdigit()]
        return max(values, default=0)

    def shoot(self, commands):
        """Try every configured step; diagnostics never gate the next command."""
        sequence = list(commands)
        if not sequence or sequence[-1].strip().lower() != 'vesc_stop':
            sequence.append('vesc_stop')
        report = ActionReport()
        for command in sequence:
            name = command.strip().lower().split()[0] if command.strip() else ''
            result = StepResult(command, False, '未确认')
            report.steps.append(result)
            matchers = {
                'vesc_stop': lambda s: s.startswith('--VESC stop: both 0 A mailboxes cleared;'),
                'vesc_home': lambda s: s.startswith('--Manual start-position confirmation accepted'),
                'vesc_shoot2': lambda s: s.startswith('--SHOT2 starting both;'),
                'vesc_reset': lambda s: s.startswith('--RESET top assumed;'),
            }
            if name not in matchers:
                result.detail = '未配置该命令的确认格式，跳过并继续'
                self.log(command+': '+result.detail)
                continue
            try:
                baseline = (max(self.last_shot_run, self._observed_run('RESULT '))
                            if name == 'vesc_shoot2' else
                            max(self.last_reset_run, self._observed_run('RESET_RESULT ')))
                exchange, accepted = self.accept(command, matchers[name])
                result.confirmed, result.detail, result.attempts = (
                    accepted.confirmed, accepted.detail, accepted.attempts)
                if not accepted.confirmed:
                    continue
                if name == 'vesc_home':
                    result.confirmed = fields(exchange.ack).get('encoder_home', '').rstrip('.') == '1'
                elif name == 'vesc_shoot2':
                    deadline = self.clock()+SHOT_SECONDS
                    header = self.wait(exchange, lambda s: s.startswith('RESULT FINISHED '), SHOT_SECONDS)
                    result.detail += ' -> '+header
                    data = fields(header)
                    run = int(data.get('run', '0'))
                    result.confirmed = (run > baseline and 'reason=timed release ' in header
                                        and data.get('timing_speed') in ('PASS', 'FAIL'))
                    self.last_shot_run = max(self.last_shot_run, run)
                    # Drain the report, including an abnormal result, before the next command.
                    self.wait(exchange, lambda s: s.startswith('VERIFY actual stroke/'),
                              max(0., deadline-self.clock()))
                    if any('fault_id=' in s and fields(s).get('fault_id') != '0'
                           for _, s in exchange.lines):
                        result.confirmed = False
                    self.log('发射结果（仅记录，不阻止复位）: '+header)
                elif name == 'vesc_reset':
                    header = self.wait(exchange, lambda s: s.startswith('RESET_RESULT FINISHED '), RESET_SECONDS)
                    result.detail += ' -> '+header
                    data = fields(header)
                    run = int(data.get('run', '0'))
                    result.confirmed = (run > baseline and data.get('valid') == 'YES'
                                        and data.get('fault_id') == '0' and 'reason=timed release ' in header)
                    self.last_reset_run = max(self.last_reset_run, run)
                    self.log('复位结果（仅记录，继续末尾停止）: '+header)
            except (ProtocolError, ValueError) as exc:
                result.confirmed = False
                result.detail += '；'+str(exc)
            finally:
                self.log(f'{command}: '+('已确认' if result.confirmed else '异常/未确认，继续下一步')
                         +'；'+result.detail)
        self.log('投篮命令序列已走完，继续后续任务；不将串口确认当作实际命中/到底')
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
