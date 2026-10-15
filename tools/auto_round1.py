#!/usr/bin/env python3
"""Monitor the shared chassis/sensor power switch and run the first passing round once.

The LiDAR's network presence is a PROXY for the shared switch, not motor-board feedback.
No CAN frames, velocity or mechanism commands are sent by this monitor itself.
"""
import argparse
import fcntl
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

CONFIG = Path(__file__).with_suffix('.json')


def log(message):
    print(time.strftime('%Y-%m-%d %H:%M:%S'), message, flush=True)


def load_config(path):
    cfg = json.loads(Path(path).read_text())
    ipaddress.IPv4Address(cfg['power_probe_ip'])
    for key in ('poll_seconds', 'power_off_seconds'):
        value = cfg[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key}须为正有限数')
    for key in ('workspace', 'state_dir'):
        if not isinstance(cfg[key], str) or not Path(cfg[key]).is_absolute():
            raise ValueError(f'{key}须为绝对路径')
    interface = cfg['can_interface']
    if not isinstance(interface, str) or not interface or '/' in interface or len(interface) > 15:
        raise ValueError('can_interface须为明确的CAN接口名称')
    workspace = Path(cfg['workspace'])
    for name in ('boot.sh', 'run_first_round.sh', 'match_rounds.json'):
        if not (workspace/'tools'/name).is_file():
            raise ValueError(f'缺少工作空间文件: {name}')
    # Reuse production validation, without importing ROS or touching hardware.
    import sys
    sys.path.insert(0, str(workspace/'tools'))
    from match_round_route import load_config as route_config, validate_mode
    route = route_config(workspace/'tools/match_rounds.json')
    if route.get('round1_first_ball', 'preloaded') != 'preloaded':
        raise ValueError('自动第一回合首球须预装')
    validate_mode(route, 'pass', 'cycle', 'match', mechanism_enabled=True)
    cfg['delay_seconds'] = route['start_delay_s']
    return cfg


def power_online(ip):
    try:
        return subprocess.run(['ping', '-n', '-c', '1', '-W', '1', ip],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              timeout=1.5).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


class PowerEdge:
    """Require an observed OFF before ON; a daemon restart while ON is not a start."""
    def __init__(self, off_seconds=3.):
        self.off_seconds = off_seconds
        self.off_since = None
        self.saw_off = False
        self.triggered = False

    def update(self, online, now=0.):
        if not online:
            self.off_since = now if self.off_since is None else self.off_since
        if self.off_since is not None and now-self.off_since >= self.off_seconds:
            self.saw_off = True
        if online:
            self.off_since = None
        if online and self.saw_off and not self.triggered:
            self.triggered = True
            return True
        return False


def save_state(directory, status, **fields):
    directory = Path(directory)
    temp = directory/'state.tmp'
    with temp.open('w') as stream:
        json.dump({'status': status, **fields}, stream, ensure_ascii=False)
        stream.flush()
        os.fsync(stream.fileno())
    temp.replace(directory/'state.json')


def stop_process(process):
    if process is None:
        return
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass


def nodes_started():
    # Process existence only synchronizes launch; the runner separately waits on real input timestamps.
    return all(subprocess.run(['pgrep', '-f', pattern], stdout=subprocess.DEVNULL).returncode == 0
               for pattern in ('[r]b_chassis_node', '[l]ocalization_node',
                               '[p]erception_node', '[r]b_launcher_node'))


def controller_busy():
    return subprocess.run(['pgrep', '-f', '[t]est_perimeter_ball.py|[m]atch_run.py|[t]eleop_chassis.py'],
                          stdout=subprocess.DEVNULL).returncode == 0


def route_command(cfg, powered_at):
    return ['bash', str(Path(cfg['workspace'])/'tools/run_first_round.sh'),
            '--mechanism', '--power-on-monotonic', str(powered_at)]


def monitor(cfg, *, probe=power_online, now=time.monotonic, sleep=time.sleep,
            popen=subprocess.Popen, ready=nodes_started, busy=controller_busy,
            can_present=None, stopping=lambda: False):
    directory = Path(cfg['state_dir'])
    if (directory/'inhibit').exists():
        log('急停禁止自动执行；解除急停后复位。')
        return 0
    edge = PowerEdge(cfg['power_off_seconds'])
    stack = route = None
    powered_at = None
    can_present = can_present or (lambda: Path('/sys/class/net', cfg['can_interface']).exists())
    log('等待总开关：先观测雷达离线，再观测上线。启动时已经在线不会触发。')
    try:
        while not stopping() and not (directory/'inhibit').exists():
            # Ping only detects the initial power edge; it never aborts an active round.
            online = probe(cfg['power_probe_ip']) if powered_at is None else True
            moment = now()
            if powered_at is None:
                if not edge.update(online, moment):
                    sleep(cfg['poll_seconds'])
                    continue
                powered_at = moment
                # Diagnostic record only; restarting still requires a newly observed power edge.
                save_state(directory, 'power_seen', power_on_monotonic=powered_at)
                log('检测到共电雷达上线；开始准备软件，上电等待与准备同时计时。')
                if busy():
                    save_state(directory, 'failed', reason='已有其它运动控制程序')
                    return 1
            if stack is None and online and can_present():
                stack = popen(['bash', str(Path(cfg['workspace'])/'tools/boot.sh'), 'start_mission:=false'],
                              cwd=cfg['workspace'], start_new_session=True)
                log('已启动整套软件，不启动独立mission。等待CAN、真实定位、相机和机构。')
            if stack is not None and stack.poll() is not None:
                save_state(directory, 'failed', reason='整套软件提前退出', code=stack.returncode)
                return 1
            if (route is None and stack is not None and online
                    and moment-powered_at >= cfg['delay_seconds'] and ready()):
                if busy():
                    save_state(directory, 'failed', reason='倒计时中出现其它运动程序')
                    return 1
                save_state(directory, 'running', power_on_monotonic=powered_at)
                route = popen(route_command(cfg, powered_at), cwd=cfg['workspace'], start_new_session=True)
                log('上电等待已满足，第一回合完整传球已启动；数据未就绪时仍原地等待。')
            if route is not None and route.poll() is not None:
                code = route.returncode
                save_state(directory, 'complete' if code == 0 else 'failed', code=code)
                log(f'第一回合退出 code={code}；下次服务启动仍需重新观测断电→上电。')
                return code
            sleep(cfg['poll_seconds'])
        if powered_at is not None:
            save_state(directory, 'stopped', reason='用户停止或急停')
        return 0
    finally:
        # Stop the controller first so its existing finally sends zero, then shut down the stack.
        stop_process(route)
        stop_process(stack)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=CONFIG)
    parser.add_argument('--check', action='store_true', help='只读检查配置和代理信号，不启动软件')
    parser.add_argument('--reset', action='store_true', help='停止服务后清除记录和急停禁止，下次须先断电再上电')
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    directory = Path(cfg['state_dir'])
    if args.check:
        print(json.dumps({'config':cfg, 'power_proxy_online':power_online(cfg['power_probe_ip']),
                          'has_history':(directory/'state.json').exists(),
                          'inhibited':(directory/'inhibit').exists()}, ensure_ascii=False, indent=2))
        return 0
    directory.mkdir(parents=True, exist_ok=True)
    with (directory/'monitor.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('监视器已在运行；复位前须先停止服务')
        if args.reset:
            for name in ('state.json', 'inhibit'):
                (directory/name).unlink(missing_ok=True)
            log('已复位；启动服务后须观测一次断电→上电才会发车。')
            return 0
        cancelled = [False]
        def cancel(_sig, _frame): cancelled[0] = True
        signal.signal(signal.SIGTERM, cancel)
        signal.signal(signal.SIGINT, cancel)
        return monitor(cfg, stopping=lambda: cancelled[0])


if __name__ == '__main__':
    raise SystemExit(main())
