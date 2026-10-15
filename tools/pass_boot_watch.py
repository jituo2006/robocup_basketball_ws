#!/usr/bin/env python3
"""共用总开关上电后启动 boot；独立服务持有整车进程，不自动执行比赛路线。"""
from __future__ import annotations

import argparse
from collections import deque
import json
import os
from pathlib import Path
import subprocess
import threading
import time

WS = Path(__file__).resolve().parent.parent
CONFIG = Path(__file__).with_name('pass_boot.json')


def load_settings(path=CONFIG):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def shared_power(config, net_root=Path('/sys/class/net'), run=subprocess.run):
    """仅将共用电源上的雷达用作上电信号，不将 can0 UP 当作电机回执。"""
    try:
        carrier = (net_root / config['power_interface'] / 'carrier').read_text().strip() == '1'
    except OSError:
        carrier = False
    reachable = False
    if carrier:
        try:
            reachable = run(['ping', '-n', '-c', '1', '-W', '1', config['power_host']],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            timeout=2).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            pass
    return dict(online=carrier and reachable, carrier=carrier, reachable=reachable)


def existing_stack(proc_root=Path('/proc')):
    """精确匹配本工作空间 boot 或整车 launch；不匹配检查命令本身。"""
    try:
        entries = list(proc_root.iterdir())
    except OSError:
        return None
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            args = [a.decode(errors='replace') for a in (entry/'cmdline').read_bytes().split(b'\0') if a]
        except OSError:
            continue
        if str(WS/'tools/boot.sh') in args:
            return int(entry.name)
        if 'tools/boot.sh' in args:
            try:
                if (entry/'cwd').resolve() == WS:
                    return int(entry.name)
            except OSError:
                pass
        for index in range(len(args)-2):
            if (args[index:index+2] == ['launch', 'rb_bringup']
                    and args[index+2] in ('bringup_lidar_vision.launch.py', 'bringup_full.launch.py')):
                return int(entry.name)
    return None


def unit_status(unit):
    result = subprocess.run(['systemctl', '--user', 'show', unit,
                             '--property=ActiveState', '--property=ExecMainStatus',
                             '--property=MainPID'], capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or '无法查询整车启动服务')
    return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)


def start_unit(unit):
    result = subprocess.run(['systemctl', '--user', 'start', '--no-block', unit],
                            capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or '无法启动 boot 服务')


def restart_unit(unit):
    result = subprocess.run(['systemctl', '--user', 'restart', '--no-block', unit],
                            capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or '无法重新启动 boot 服务')


class BootWatch:
    def __init__(self, config=None, probe=shared_power, find_stack=existing_stack,
                 status=unit_status, launch=start_unit, restart=restart_unit, clock=time.monotonic):
        self.config = config if config is not None else load_settings()
        self.probe, self.find_stack, self.status, self.launch = probe, find_stack, status, launch
        self.restart, self.clock = restart, clock
        self.link_down_since = None
        self.power_rearm = False
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.lines = deque(maxlen=100)
        self.state = dict(status='waiting', message='等待总开关上电，后台持续检测',
                          power={}, pid=None, returncode=None, checked_at=None)

    def update(self, status, message, **fields):
        with self.lock:
            if (status, message) != (self.state['status'], self.state['message']):
                line = f'{time.strftime("%H:%M:%S")} {message}'
                self.lines.append(line)
                print(f'[自动启动] {line}', flush=True)
            self.state.update(status=status, message=message, **fields)

    def step(self):
        power = self.probe(self.config)
        common = dict(power=power, checked_at=time.time())
        state = self.status(self.config['stack_unit'])
        if power.get('carrier') is False:
            if self.link_down_since is None:
                self.link_down_since = self.clock()
            if self.clock()-self.link_down_since >= self.config.get('power_off_confirm_s', 3.):
                self.power_rearm = True
        else:
            self.link_down_since = None
        if self.power_rearm:
            if not power['online']:
                self.update('waiting', '已检测到总电源断开，等待重新上电后执行 boot.sh', **common)
                return
            if state.get('ActiveState') in ('active', 'activating', 'reloading', 'deactivating'):
                self.restart(self.config['stack_unit'])
            else:
                self.launch(self.config['stack_unit'])
            self.power_rearm = False
            self.update('starting', '共用总电源重新上电，重新执行 boot.sh 使能底盘',
                        pid=None, returncode=None, **common)
            return
        if state.get('ActiveState') in ('active', 'activating', 'reloading', 'deactivating'):
            self.update('running', 'boot.sh 整车启动进程运行中',
                        pid=int(state.get('MainPID', 0)) or None, returncode=None, **common)
            return
        pid = self.find_stack()
        if pid:
            self.update('existing', '已有整车启动进程，复用当前实例', pid=pid, returncode=None, **common)
            return
        if not power['online']:
            self.update('waiting', '等待总开关上电，后台持续检测', pid=None,
                        returncode=int(state.get('ExecMainStatus', 0)), **common)
            return
        code = int(state.get('ExecMainStatus', 0))
        if state.get('ActiveState') == 'failed':
            self.update('retrying', f'boot.sh 上次退出码 {code}，上电状态有效，重新启动',
                        returncode=code, **common)
        self.launch(self.config['stack_unit'])
        self.update('starting', '已检测到共用总电源上电，自动执行 boot.sh',
                    pid=None, returncode=None, **common)

    def _loop(self):
        while not self.stop_event.is_set():
            started = time.monotonic()
            try:
                self.step()
            except Exception as exc:
                self.update('error', f'自动启动检查失败，将继续重试：{exc}', checked_at=time.time())
            elapsed = time.monotonic()-started
            self.stop_event.wait(max(0., self.config['poll_interval_s']-elapsed))

    def start(self):
        with self.lock:
            if self.thread is not None:
                return
            self.thread = threading.Thread(target=self._loop, daemon=True, name='pass-boot-watch')
            self.thread.start()

    def snapshot(self):
        with self.lock:
            return dict(self.state, lines=list(self.lines))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-stack', action='store_true')
    args = parser.parse_args(argv)
    if args.run_stack:
        config = load_settings()
        os.chdir(WS)
        command = config['boot_argv']
        os.execvpe(command[0], command, os.environ)
    parser.error('整车服务入口需要 --run-stack')


if __name__ == '__main__':
    main()
