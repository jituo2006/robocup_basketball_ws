"""Reply-driven mechanism actions. Pure Python core; no serial device is opened here.

The ROS serial node is the only UART reader. Untagged MCU replies are paired with
host TX boundaries AND fresh status/target checks, never with a bare old DONE.
"""
from collections import deque
from dataclasses import dataclass
import json
import math
import re
import threading
import time


class MechanismError(RuntimeError):
    pass


@dataclass(frozen=True)
class Token:
    epoch: str
    tx_id: int
    seq: int


class ReplyInbox:
    def __init__(self):
        self.events = deque(maxlen=4096)
        self.lock = threading.Lock()
        self.last = None
        self.broken = None

    def feed(self, payload):
        try:
            event = json.loads(payload) if isinstance(payload, str) else dict(payload)
            event['epoch'] = str(event['epoch'])
            for key in ('seq', 'tx_id'):
                event[key] = int(event[key])
            if event['kind'] not in ('rx', 'tx', 'connect', 'disconnect'):
                raise ValueError('unknown RX event kind')
            if not isinstance(event['line'], str):
                raise ValueError('invalid RX line')
        except (ValueError, KeyError, TypeError) as exc:
            with self.lock:
                self.broken = 'RX事件格式错误: ' + str(exc)
            return
        with self.lock:
            if self.last and event['epoch'] == self.last['epoch']:
                if event['seq'] != self.last['seq'] + 1:
                    self.broken = 'RX事件丢失或乱序，禁止继续动作'
            self.last = event
            self.events.append(event)

    def after(self, token):
        with self.lock:
            if self.broken:
                raise MechanismError(self.broken)
            if self.last and self.last['epoch'] != token.epoch:
                raise MechanismError('串口节点重启或重连代次改变')
            events = list(self.events)
        matching = [e for e in events if e['epoch'] == token.epoch and e['seq'] > token.seq]
        if events and events[0]['epoch'] == token.epoch and events[0]['seq'] > token.seq + 1:
            raise MechanismError('回复缓存溢出')
        lines = []
        for e in matching:
            if e['kind'] == 'disconnect':
                raise MechanismError('串口断开: ' + e['line'])
            if e['kind'] in ('tx', 'connect') or e['tx_id'] != token.tx_id:
                raise MechanismError('动作期间串口被其他命令占用或重新连接')
            if e['kind'] == 'rx':
                line = e['line']
                if (line.startswith('ERR ') or 'refused:' in line.lower()
                        or line.startswith('--VESC fault') or line.startswith('==VESC')):
                    raise MechanismError('主板拒绝/故障: ' + line)
                lines.append(line)
        return lines


def fields(line):
    return dict(re.findall(r'([A-Za-z_][A-Za-z_0-9]*)=([^\s]+)', line))


def number(data, key):
    try:
        value = float(data[key])
        if not math.isfinite(value):
            raise ValueError(key)
        return value
    except (KeyError, ValueError, TypeError) as exc:
        raise MechanismError('主板状态字段缺失或非法: ' + key) from exc


class MechanismController:
    def __init__(self, send, pump, inbox, *, clock=time.monotonic, log=lambda message: None,
                 ack_timeout=2., move_timeout=20., shot_timeout=20., reset_timeout=8.):
        self.send = send
        self.pump = pump
        self.inbox = inbox
        self.clock = clock
        self.log = log
        self.ack_timeout = ack_timeout
        self.move_timeout = move_timeout
        self.shot_timeout = shot_timeout
        self.reset_timeout = reset_timeout
        self.lock = threading.Lock()

    def _send(self, command):
        self.log('TX ' + command)
        ok, message = self.send(command)
        if not ok:
            raise MechanismError(command + ': ' + message)
        match = re.search(r'EPOCH=(\S+) TX_ID=(\d+) SEQ=(\d+)', message)
        if not match:
            raise MechanismError('串口节点尚未升级：缺少发送编号，不能确认动作')
        return Token(match[1], int(match[2]), int(match[3]))

    def _wait(self, token, predicate, timeout, description):
        deadline = self.clock() + timeout
        while True:
            lines = self.inbox.after(token)
            value = predicate(lines)
            if value is not None:
                return value
            if self.clock() >= deadline:
                tail = ' / '.join(lines[-3:]) or '无主板回复'
                raise MechanismError(description + '超时；' + tail)
            self.pump(min(.02, max(0., deadline-self.clock())))

    def _reply(self, command, prefix, timeout=None):
        token = self._send(command)
        return self._wait(token, lambda lines: next((s for s in lines if s.startswith(prefix)), None),
                          self.ack_timeout if timeout is None else timeout, command + '等待回复')

    def _pause(self, seconds):
        deadline = self.clock() + seconds
        while self.clock() < deadline:
            self.pump(min(.02, deadline-self.clock()))

    def _run(self, group, operation):
        if not self.lock.acquire(blocking=False):
            return False, '机构动作正在执行，未发送新运动指令'
        try:
            operation()
            return True, '机构动作及状态确认完成'
        except BaseException as exc:
            # 异常只记录并向调用方报告，不额外发送机构停止/复位/置零指令。
            note = '未额外发送机构停止指令'
            self.log(str(exc) + '; ' + note)
            if not isinstance(exc, Exception):
                raise
            return False, str(exc) + '; ' + note
        finally:
            self.lock.release()

    def _dji_status(self):
        line = self._reply('dji_status', 'STATUS ')
        state = line.split()[1]
        data = fields(line)
        if state == 'FAULT' or data.get('FAULT') != 'NONE':
            raise MechanismError('DJI故障: ' + line)
        if number(data, 'LO') != 1 or number(data, 'RO') != 1 or number(data, 'CAN') != 0:
            raise MechanismError('DJI反馈离线或CAN故障: ' + line)
        if number(data, 'ZERO') != 1:
            raise MechanismError('DJI零点无效，请在确认机构位于实际初始位置后手动初始化')
        return state, data

    @staticmethod
    def _dji_at(state, data, target):
        return (state == 'HOLD' and number(data, 'MODE') == 1
                and abs(number(data, 'L')-target) <= 1.
                and abs(number(data, 'R')+target) <= 1.
                and abs(number(data, 'LV')) <= 1. and abs(number(data, 'RV')) <= 1.)

    def _dji_move(self, target):
        if target not in (0, 90):
            raise MechanismError('自动铲球只支持已有的0/90度目标')
        state, data = self._dji_status()
        if state not in ('READY', 'HOLD') or abs(number(data, 'LV')) > 1 or abs(number(data, 'RV')) > 1:
            raise MechanismError('DJI尚未静止，未发送下一条运动指令')
        self._reply(f'dji_move 1 {target} 5', 'ACK DJI_MOVE 1 ')
        deadline = self.clock() + self.move_timeout
        while True:
            state, data = self._dji_status()
            if self._dji_at(state, data, target):
                self.log(f'DJI已到位: target={target}, L={data["L"]}, R={data["R"]}')
                return
            if self.clock() >= deadline:
                raise MechanismError(f'DJI到位超时: target={target}, state={state}, L={data["L"]}, R={data["R"]}')
            self._pause(.15)

    def initialize_dji(self):
        """Explicit console action: operator must first confirm the physical zero."""
        def action():
            self._reply('dji_stop', 'ACK DJI_STOP')
            self._pause(.3)
            self._reply('dji_zero', 'ACK DJI_ZERO')
            self._dji_status()
        return self._run('dji', action)

    def pickup(self, hold_s=5.):
        def action():
            if not math.isfinite(hold_s) or hold_s < 0:
                raise MechanismError('铲起后保持时间非法')
            state, data = self._dji_status()
            if (state not in ('READY', 'HOLD') or abs(number(data, 'L')) > 1
                    or abs(number(data, 'R')) > 1):
                raise MechanismError('接球前铲球机构不在已有零位，需现场检查；不会自动重新置零')
            self._dji_move(90)
            # Monitor health through the dwell as well. No blind five-second sleep.
            deadline = self.clock() + hold_s
            while self.clock() < deadline:
                state, data = self._dji_status()
                if not self._dji_at(state, data, 90):
                    raise MechanismError('保持阶段铲球机构离开目标位置')
                self._pause(min(.15, max(0., deadline-self.clock())))
            self._dji_move(0)
        return self._run('dji', action)

    def _vesc_status(self):
        token = self._send('vesc_status')
        def complete(lines):
            header = next((s for s in lines if s.startswith('--VESC dual ')), None)
            fault = next((s for s in lines if s.startswith('CAN_ESR=')), None)
            live = next((s for s in lines if s.startswith('CAN live ')), None)
            motors = {int(fields(s)['id']): fields(s) for s in lines if s.startswith('MOTOR id=')}
            controls = {int(fields(s)['id']): fields(s) for s in lines if s.startswith('CONTROL id=')}
            if header and fault and live and set(motors) == {1, 2} and set(controls) == {1, 2}:
                if ' fault=none ' not in fault or number(fields(live), 'bus_off') != 0:
                    raise MechanismError('VESC故障: ' + fault + ' / ' + live)
                return header, motors, controls
            return None
        return self._wait(token, complete, self.ack_timeout, 'VESC完整状态')

    def _idle(self, require_home=False, timeout=5.):
        deadline = self.clock()+timeout
        stable_since = None
        while True:
            header, motors, controls = self._vesc_status()
            h = fields(header)
            poles = number(h, 'poles')
            if poles < 1 or poles > 100:
                raise MechanismError('VESC极对数非法')
            ready = header.startswith('--VESC dual IDLE ') and all(
                number(motors[i], 'rx') > 0 and 0 <= number(motors[i], 'age') < 20
                and abs(number(motors[i], 'actual')) <= 5*poles
                and number(controls[i], 'cmd') == 0
                and int(controls[i]['pending'], 0) == 0 for i in (1, 2))
            ready = ready and (not require_home or h.get('home') == '1')
            if ready:
                if stable_since is None:
                    stable_since = self.clock()
                elif self.clock()-stable_since >= .15:
                    return
            else:
                stable_since = None
            if self.clock() >= deadline:
                raise MechanismError('VESC未满足新鲜反馈、静止、零电流' + ('及home=1' if require_home else '') + '条件')
            self._pause(.15)

    def _stop(self):
        self._reply('vesc_stop', '--VESC stop: both 0 A mailboxes cleared;')
        self._idle()

    def _home(self):
        self._idle()
        self._reply('vesc_home', '--Manual start-position confirmation accepted')
        self._idle(require_home=True)

    def _result_run(self, command, prefix):
        token = self._send(command)
        line = self._wait(token, lambda lines: next((s for s in lines if s.startswith(prefix) or
                         s.startswith('--No ')), None), self.ack_timeout, command)
        if prefix == 'RESULT ' and not line.startswith('--No '):
            self._wait(token, lambda lines: next((s for s in lines if s.startswith(
                'VERIFY actual stroke/ball departure/')), None), self.ack_timeout, '发射结果完整接收')
        return int(fields(line).get('run', 0))

    def _shoot(self, command, minimum_wait):
        self._idle(require_home=True)
        previous = self._result_run('vesc_result', 'RESULT ')
        token = self._send(command)
        self._wait(token, lambda lines: next((s for s in lines if s.startswith('--SHOT starting ') or
                   s.startswith('--SHOT2 starting ')), None), self.ack_timeout, '发射接受确认')
        started = self.clock()
        def finished(lines):
            for line in lines:
                if line.startswith('RESULT FINISHED ') and int(fields(line).get('run', 0)) > previous:
                    reason = re.search(r' reason=(.*?) timing_speed=', line)
                    if reason and reason[1] == 'motor current feedback high':
                        # 本次已结束的电流偏高反馈仅记录；继续原有复位和后续路线。
                        self.log('发射结束电流反馈（仅记录，继续流程）: ' + line)
                    elif not reason or reason[1] != 'timed release':
                        raise MechanismError('发射结束异常: ' + line)
                    return line
            return None
        self._wait(token, finished, self.shot_timeout, '发射完成')
        self._wait(token, lambda lines: next((s for s in lines if s.startswith(
            'VERIFY actual stroke/ball departure/')), None), self.ack_timeout, '发射结果完整接收')
        # Check all accumulated lines before issuing the next query.
        self.inbox.after(token)
        remaining = minimum_wait-(self.clock()-started)
        if remaining > 0:
            deadline = self.clock()+remaining
            while self.clock() < deadline:
                self.inbox.after(token)
                self._pause(min(.02, deadline-self.clock()))
        self._idle()

    def _reset(self):
        self._idle()
        previous = self._result_run('vesc_reset_result', 'RESET_RESULT ')
        token = self._send('vesc_reset')
        self._wait(token, lambda lines: next((s for s in lines if s.startswith('--RESET top assumed;')), None),
                   self.ack_timeout, '复位启动确认')
        def finished(lines):
            for line in lines:
                if line.startswith('RESET_RESULT FINISHED ') and int(fields(line).get('run', 0)) > previous:
                    if fields(line).get('valid') != 'YES' or fields(line).get('fault_id') != '0':
                        raise MechanismError('复位失败: ' + line)
                    return line
            return None
        self._wait(token, finished, self.reset_timeout, '定时复位完成')
        self._idle(require_home=True)
        self.log('定时复位、静止及home已确认；机械最低点未检测')

    def _execute(self, command, minimum_wait=0.):
        command = ' '.join(command.lower().split())
        if command == 'vesc_stop':
            self._stop()
        elif command == 'vesc_home':
            self._home()
        elif re.fullmatch(r'vesc_shoot2? \d+(?:\.\d+)?', command):
            self._shoot(command, max(3., minimum_wait))
            return
        elif command == 'vesc_reset':
            self._reset()
        elif command == 'dji_move 1 0 5':
            self._dji_move(0)
        elif command == 'dji_move 1 90 5':
            self._dji_move(90)
        elif command == 'vesc_status':
            self._vesc_status()
        else:
            raise MechanismError('自动流程不支持未定义确认规则的指令: ' + command)
        self._pause(minimum_wait)

    def release(self, commands, waits=None, default_wait=0.):
        commands = list(commands)
        group = 'vesc' if any(str(c).lower().startswith('vesc_') for c in commands) else 'dji'
        def action():
            if not commands:
                raise MechanismError('机构动作序列为空，不能报告完成')
            for index, command in enumerate(commands):
                delay = waits[index] if waits and index < len(waits) else default_wait
                if not math.isfinite(delay) or delay < 0:
                    raise MechanismError('机构等待参数非法')
                self._execute(command, delay)
        return self._run(group, action)


class RosMechanismClient:
    """Same core for route's spin_once and console's existing executor thread."""
    def __init__(self, node, pump, *, ok=lambda: True, log=lambda message: None):
        from std_msgs.msg import String
        from rb_msgs.srv import SerialText
        self.node, self.pump, self.ok = node, pump, ok
        self.request_type = SerialText.Request
        self.inbox = ReplyInbox()
        self.subscription = node.create_subscription(String, '/rb_launcher/rx_event',
                                                      lambda msg: self.inbox.feed(msg.data), 512)
        self.client = node.create_client(SerialText, '/rb_launcher/text')
        self.controller = MechanismController(self.raw_send, self._pump, self.inbox, log=log)

    def _pump(self, seconds):
        if not self.ok():
            raise KeyboardInterrupt
        self.pump(seconds)

    def raw_send(self, command):
        deadline = time.monotonic()+2.
        while not self.client.service_is_ready() or self.node.count_publishers('/rb_launcher/rx_event') < 1:
            if time.monotonic() >= deadline:
                return False, '机构串口服务或回复发布器未就绪，请启动新版rb_launcher'
            self._pump(.02)
        request = self.request_type()
        request.command = command
        future = self.client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                self.client.remove_pending_request(future)
                return False, '串口服务响应超时，发送结果未知，禁止自动重发'
            self._pump(.02)
        response = future.result()
        return bool(response.success), response.message
