"""Cold-boot prerequisites and recovery budgets; no ROS or hardware."""
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from lidar_startup import startup_state, RecoveryPolicy, Backoff
from lidar_health import LidarHealth


@pytest.fixture
def probe(tmp_path):
    network = tmp_path/'net'
    (network/'eth0').mkdir(parents=True)
    carrier = network/'eth0'/'carrier'
    carrier.write_text('1')
    marker = tmp_path/'ptp_enabled'
    marker.write_text('37')
    state = NS(ip='192.168.1.110', services='active\nactive\n', calls=[], error=None)

    def run(cmd, **kwargs):
        state.calls.append(cmd)
        assert kwargs['timeout'] <= .3
        if state.error:
            raise state.error
        if cmd[0] == 'ip':
            return NS(returncode=0, stdout=json.dumps([
                {'addr_info': [{'family': 'inet', 'local': state.ip}]}]))
        # systemctl can return 0 with only ONE active service.
        return NS(returncode=0, stdout=state.services)

    state.check = lambda mode='auto': startup_state(
        'eth0', '192.168.1.110', mode, run=run, net_root=network, ptp_marker=marker)
    state.carrier, state.marker = carrier, marker
    return state


def test_network_address_and_both_ptp_services_may_appear_late(probe):
    probe.carrier.unlink()
    assert not probe.check().ready
    assert not probe.calls
    probe.carrier.write_text('0')
    assert not probe.check().ready
    probe.carrier.write_text('1')
    probe.ip = '192.168.1.111'
    assert probe.check().phase == 'waiting_network'
    probe.ip = '192.168.1.110'
    for statuses in ('inactive\nactive\n', 'active\ninactive\n', 'active\n'):
        probe.services = statuses
        assert probe.check().phase == 'waiting_ptp'
    probe.services = 'active\nactive\n'
    assert probe.check().ready


def test_auto_marker_and_explicit_modes(probe):
    probe.services = 'inactive\ninactive\n'
    assert not probe.check().ready
    probe.marker.unlink()
    assert probe.check().ready
    assert not probe.check('on').ready
    probe.marker.write_text('37')
    assert probe.check('off').ready


@pytest.mark.parametrize('error', [OSError('not yet installed'),
    ValueError('invalid JSON'), subprocess.TimeoutExpired('ip', .3)])
def test_probe_failures_remain_retryable(probe, error):
    probe.error = error
    assert not probe.check().ready
    probe.error = None
    assert probe.check().ready


def test_recovery_distinguishes_slow_start_silence_and_invalid_source():
    h, policy = LidarHealth(), RecoveryPolicy()
    assert not policy.driver_problem(h, 100, 159.9)
    assert policy.driver_problem(h, 100, 160)
    for name in ('imu', 'lidar'):
        h.observe(name, 112, 112, 112)
    assert not policy.driver_problem(h, 100, 121.9)
    assert '断流' in policy.driver_problem(h, 100, 122)
    # Transport remains active; duplicate/stale measurements do not renew progress.
    for name in ('imu', 'lidar'):
        h.observe(name, 112, 142, 142)
    assert '无有效进展' in policy.driver_problem(h, 100, 142)
    assert not policy.mapping_problem(h, 100, 129.9)
    assert policy.mapping_problem(h, 100, 130)


@pytest.mark.parametrize('step', [-37., 37.])
def test_wall_clock_step_invalidates_previous_stability(step):
    h = LidarHealth(.2)
    assert not h.check_clock(1000, 100)
    for i in range(5):
        for name in ('imu', 'lidar', 'odom'):
            h.observe(name, 1000+i*.1, 1000+i*.1, 100+i*.1)
    assert h.inputs_stable(1000.4, 100.4)
    assert h.check_clock(1000.5+step, 100.5)
    assert not h.inputs_stable(1000.5+step, 100.5)
    assert not h.odom_stable(1000.5+step, 100.5)
    assert h.last_valid == dict(imu=None, lidar=None, odom=None)
    assert not h.check_clock(1000.6+step, 100.6)


def test_retry_is_bounded_and_resets_after_success():
    b = Backoff()
    assert [b.take() for _ in range(8)] == [1, 2, 4, 8, 15, 15, 15, 15]
    b.reset()
    assert b.take() == 1


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_invalid_recovery_budgets_are_rejected(value):
    with pytest.raises(ValueError):
        RecoveryPolicy(startup_grace=value)
