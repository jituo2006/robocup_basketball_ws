"""Cold-start prerequisites and bounded retry policy, independent of ROS."""
from dataclasses import dataclass
from pathlib import Path
import json
import math
import subprocess


@dataclass(frozen=True)
class StartupState:
    ready: bool
    phase: str
    reason: str


def startup_state(interface, host_ip, ptp_mode='auto', *, run=subprocess.run,
                  net_root=Path('/sys/class/net'),
                  ptp_marker=Path('/etc/robocup/livox_ptp_utc_offset')):
    try:
        if (net_root/interface/'carrier').read_text().strip() != '1':
            return StartupState(False, 'waiting_network', '等待雷达网口链路')
        result = run(['ip', '-4', '-json', 'addr', 'show', 'dev', interface],
                     capture_output=True, text=True, timeout=.3)
        links = json.loads(result.stdout) if result.returncode == 0 else []
        if not any(a.get('family') == 'inet' and a.get('local') == host_ip
                   for link in links for a in link.get('addr_info', [])):
            return StartupState(False, 'waiting_network', '等待主机雷达网口地址 '+host_ip)
        if ptp_mode == 'on' or ptp_mode == 'auto' and ptp_marker.is_file():
            result = run(['systemctl', 'is-active',
                          f'robocup-ptp4l@{interface}.service',
                          f'robocup-phc2sys@{interface}.service'],
                         capture_output=True, text=True, timeout=.3)
            # is-active exits successfully when *any* named unit is active.
            # Both ptp4l and phc2sys must be running before launching Livox.
            if result.returncode or result.stdout.split() != ['active', 'active']:
                return StartupState(False, 'waiting_ptp', '等待PTP服务运行；仍需真实数据验证同步')
        return StartupState(True, 'waiting_inputs', '网络已准备，等待点云和IMU有效数据')
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        return StartupState(False, 'waiting_network', '启动条件暂未就绪：'+str(exc))


class Backoff:
    def __init__(self, initial=1., maximum=15.):
        self.initial, self.maximum = initial, maximum
        self.delay = initial

    def take(self):
        delay = self.delay
        self.delay = min(self.maximum, delay*2)
        return delay

    def reset(self):
        self.delay = self.initial


@dataclass(frozen=True)
class RecoveryPolicy:
    startup_grace: float = 60.
    silence_grace: float = 10.
    invalid_grace: float = 30.
    mapping_grace: float = 30.

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (
                self.startup_grace, self.silence_grace, self.invalid_grace, self.mapping_grace)):
            raise ValueError('恢复等待参数必须为有限正数')

    def driver_problem(self, health, started, now):
        for name in ('imu', 'lidar'):
            received = health.input_received[name]
            progressed = health.last_valid[name]
            if received is None:
                if now-started >= self.startup_grace:
                    return name+'启动后一直无数据'
            elif now-received >= self.silence_grace:
                return name+'持续断流'
            if progressed is None:
                if now-started >= self.startup_grace:
                    return name+'启动后一直无有效源时间'
            elif now-progressed >= self.invalid_grace:
                return name+'源时间长期无有效进展'
        return ''

    def mapping_problem(self, health, started, now):
        progressed = health.last_valid['odom']
        if progressed is None and now-started >= self.mapping_grace:
            return 'FAST-LIO启动后一直无有效里程计'
        if progressed is not None and now-progressed >= self.silence_grace:
            return 'FAST-LIO里程计长期无有效进展'
        return ''
