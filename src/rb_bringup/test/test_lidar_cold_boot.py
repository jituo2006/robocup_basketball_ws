"""Replay the production manager loop with logical time and fake OS/ROS boundaries."""
import json
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
import lidar_session


def replay(monkeypatch, scenario, duration):
    rt = NS(t=100., tick=0, processes=[], events=[], messages=[], callbacks={},
            attempts={'driver': 0, 'mapping': 0}, probes=[])
    def elapsed(): return rt.t-100.
    def wall(): return rt.t+(37. if scenario == 'clock_step' and elapsed() >= 8 else 0.)
    def link(): return scenario != 'never_link' and (scenario != 'cold_boot' or elapsed() >= 3)

    class Proc:
        def __init__(self, cmd, **kwargs):
            assert kwargs['start_new_session'] is True
            self.kind = 'mapping' if 'fast_lio' in cmd else 'driver'
            rt.attempts[self.kind] += 1
            self.number = rt.attempts[self.kind]
            rt.events.append(('attempt', self.kind, rt.t))
            if (scenario == self.kind+'_spawn_error' and self.number <= 2 or
                    scenario == 'persistent_spawn_error' and self.kind == 'driver'):
                raise OSError('temporarily unavailable')
            self.pid, self.stopped = 1000+len(rt.processes), False
            rt.processes.append(self)
            rt.events.append(('start', self.kind, rt.t))
        def poll(self):
            crashed = scenario == 'driver_exit' and self.kind == 'driver' and self.number == 1 and elapsed() >= 3
            return 1 if self.stopped or crashed else None
        def wait(self, timeout): self.stopped = True

    def kill(pid, sig):
        p = next(p for p in rt.processes if p.pid == pid)
        if not p.stopped:
            rt.events.append(('stop', p.kind, rt.t))
        p.stopped = True
    def active(kind):
        return next((p for p in reversed(rt.processes) if p.kind == kind and p.poll() is None), None)
    class Carrier:
        def __init__(self, *args): pass
        def read_text(self): return '1' if link() else '0'

    def prerequisites(*args):
        rt.probes.append(rt.t)
        phase = ('waiting_network' if elapsed() < 8 else 'waiting_ptp')
        ready = scenario != 'cold_boot' or elapsed() >= 12
        return NS(ready=ready, phase='waiting_inputs' if ready else phase, reason=phase)

    class Node:
        def create_publisher(self, *args):
            return NS(publish=lambda msg: rt.messages.append((rt.t, json.loads(msg.data))))
        def create_subscription(self, typ, topic, cb, qos, **kwargs):
            if topic == '/livox/lidar':
                assert kwargs['raw']
                def send(msg):
                    stamp = msg.header.stamp
                    cb(b'\x00\x01\x00\x00'+struct.pack('<iI', stamp.sec, stamp.nanosec))
                rt.callbacks[topic] = send
            else:
                rt.callbacks[topic] = cb
            return cb
        def create_timer(self, period, cb): rt.timer = cb
        def get_clock(self): return NS(now=lambda: NS(nanoseconds=round(wall()*1e9)))
        def get_logger(self): return NS(info=lambda *args: None)
        def destroy_node(self): pass

    def message(stamp=None):
        stamp = wall() if stamp is None else stamp
        ns = round(stamp*1e9)
        return NS(header=NS(stamp=NS(sec=ns//10**9, nanosec=ns%10**9)),
                  pose=NS(pose=NS(position=NS(x=0, y=0, z=0))))

    def spin(*args, **kwargs):
        rt.tick += 1
        rt.t = 100+rt.tick*.1
        driver, mapping = active('driver'), active('mapping')
        if driver:
            first = driver.number == 1
            silent = (scenario == 'driver_stuck' and first or
                      scenario == 'cold_boot' and elapsed() < 24 or
                      scenario == 'long_silence' and first and 3 < elapsed() < 16)
            if not silent:
                stamp = 103. if scenario == 'duplicates' and first and elapsed() > 3 else wall()
                if scenario == 'invalid_time' and first:
                    stamp -= 37.
                if not (scenario == 'one_stream' and first):
                    rt.callbacks['/livox/imu'](message(stamp))
                rt.callbacks['/livox/lidar'](message(stamp))
        if mapping and not (scenario == 'mapping_stuck' and mapping.number == 1):
            rt.callbacks['/Odometry'](message())
        rt.timer()

    node = Node()
    modules = {
        'rclpy': dict(init=lambda: None, create_node=lambda *a: node,
                     ok=lambda: elapsed() < duration, spin_once=spin, shutdown=lambda: None),
        'rclpy.qos': dict(qos_profile_sensor_data=object()),
        'livox_ros_driver2.msg': dict(CustomMsg=object),
        'nav_msgs.msg': dict(Odometry=object), 'sensor_msgs.msg': dict(Imu=object),
        'std_msgs.msg': dict(String=lambda: NS(data='')),
    }
    for name, attrs in modules.items():
        mod = ModuleType(name)
        mod.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.setattr(lidar_session, 'Path', Carrier)
    monkeypatch.setattr(lidar_session, 'startup_state', prerequisites)
    monkeypatch.setattr(lidar_session.subprocess, 'Popen', Proc)
    monkeypatch.setattr(lidar_session.os, 'killpg', kill, raising=False)
    monkeypatch.setattr(lidar_session.signal, 'SIGKILL', 9, raising=False)
    monkeypatch.setattr(lidar_session, 'time', NS(monotonic=lambda: rt.t, time_ns=lambda: 123))
    monkeypatch.setattr(sys, 'argv', ['lidar_session.py', '--stable-seconds', '.2'])
    lidar_session.main()
    assert all(p.stopped for p in rt.processes), 'process groups must be reclaimed on shutdown'
    return rt


def starts(rt, kind): return [t for a, k, t in rt.events if a == 'start' and k == kind]
def ready(rt): return [(t, s) for t, s in rt.messages if s['ready']]


def test_simultaneous_power_waits_for_ip_ptp_and_slow_lidar(monkeypatch):
    rt = replay(monkeypatch, 'cold_boot', 40)
    assert len(starts(rt, 'driver')) == len(starts(rt, 'mapping')) == 1
    assert starts(rt, 'driver')[0] >= 112
    assert starts(rt, 'mapping')[0] >= 124.2
    assert ready(rt)[0][0] > starts(rt, 'mapping')[0]
    phases = {s['phase'] for t, s in rt.messages}
    assert {'waiting_network', 'waiting_ptp', 'waiting_inputs', 'waiting_odometry', 'ready'} <= phases


@pytest.mark.parametrize('scenario', ['driver_stuck', 'one_stream', 'invalid_time'])
def test_live_driver_with_missing_valid_inputs_restarts_and_recovers(monkeypatch, scenario):
    rt = replay(monkeypatch, scenario, 80)
    assert len(starts(rt, 'driver')) == 2
    assert starts(rt, 'driver')[1] >= 161
    assert len(starts(rt, 'mapping')) == 1
    assert ready(rt)


@pytest.mark.parametrize('scenario', ['long_silence', 'duplicates', 'clock_step', 'driver_exit'])
def test_post_start_fault_withdraws_old_session_then_recovers(monkeypatch, scenario):
    rt = replay(monkeypatch, scenario, 50)
    output = ready(rt)
    assert len(starts(rt, 'mapping')) == 2
    assert len(starts(rt, 'driver')) == (1 if scenario == 'clock_step' else 2)
    assert output[-1][1]['generation'] > output[0][1]['generation']
    restart = starts(rt, 'mapping')[-1]
    assert any(not s['ready'] and t < restart and s['generation'] > output[0][1]['generation']
               for t, s in rt.messages)
    for t, session in output:
        if t > restart:
            assert session['cutoff'] >= restart+(37 if scenario == 'clock_step' else 0)


def test_mapping_alive_without_output_restarts_only_mapping(monkeypatch):
    rt = replay(monkeypatch, 'mapping_stuck', 45)
    assert len(starts(rt, 'driver')) == 1
    assert len(starts(rt, 'mapping')) == 2
    assert starts(rt, 'mapping')[1]-starts(rt, 'mapping')[0] >= 31
    assert ready(rt)


@pytest.mark.parametrize('scenario', ['driver_spawn_error', 'mapping_spawn_error', 'already_powered'])
def test_start_failures_are_retryable_and_lidar_may_precede_host(monkeypatch, scenario):
    rt = replay(monkeypatch, scenario, 12)
    assert ready(rt)
    assert len(starts(rt, 'driver')) == len(starts(rt, 'mapping')) == 1
    if scenario != 'already_powered':
        kind = scenario.split('_')[0]
        assert rt.attempts[kind] == 3
        attempts = [t for action, k, t in rt.events if action == 'attempt' and k == kind]
        assert attempts[1]-attempts[0] >= 1
        assert attempts[2]-attempts[1] >= 2


def test_persistent_error_keeps_retrying_with_bounded_rate(monkeypatch):
    rt = replay(monkeypatch, 'persistent_spawn_error', 80)
    attempts = [t for action, kind, t in rt.events if action == 'attempt']
    assert 5 < len(attempts) < 12
    assert attempts[-1] > 165
    assert not ready(rt)


def test_can_cancel_while_waiting_for_network_without_starting_driver(monkeypatch):
    rt = replay(monkeypatch, 'never_link', 10)
    assert not rt.processes and not ready(rt)
