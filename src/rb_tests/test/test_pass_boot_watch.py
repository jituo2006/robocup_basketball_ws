"""模拟电源和服务验证自动 boot，不连接设备、不执行真实 boot.sh。"""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools'))
from pass_boot_watch import BootWatch, existing_stack, shared_power, WS
from pass_console import make_app

CONFIG = dict(power_interface='eth-test', power_host='192.0.2.1',
              poll_interval_s=.01, stack_unit='test-stack.service')


@pytest.mark.parametrize('carrier,ping_ok,expected', [('0', True, False), ('1', False, False), ('1', True, True)])
def test_power_requires_carrier_and_reachable_lidar(tmp_path, carrier, ping_ok, expected):
    iface = tmp_path/'eth-test'; iface.mkdir(); (iface/'carrier').write_text(carrier)
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        return NS(returncode=0 if ping_ok else 1)
    assert shared_power(CONFIG, tmp_path, run)['online'] is expected
    assert bool(calls) is (carrier == '1')
    if calls:
        assert calls[0][0] == 'ping' and calls[0][-1] == CONFIG['power_host']


def test_missing_interface_never_triggers_boot(tmp_path):
    assert not shared_power(CONFIG, tmp_path, lambda *a, **k: pytest.fail('no link means no ping'))['online']


def build_watch(online=False, active=False, existing=None):
    hardware = dict(online=online)
    unit = dict(ActiveState='active' if active else 'inactive', MainPID='123' if active else '0', ExecMainStatus='0')
    launches = []
    def launch(name):
        launches.append(name)
        unit.update(ActiveState='activating', MainPID='123')
    watch = BootWatch(CONFIG, probe=lambda config: dict(hardware),
                      find_stack=lambda: existing, status=lambda name: dict(unit), launch=launch)
    return watch, hardware, unit, launches


def test_repeated_offline_checks_then_power_on_starts_once():
    watch, hardware, unit, launches = build_watch()
    for _ in range(4): watch.step()
    assert launches == [] and watch.snapshot()['status'] == 'waiting'
    hardware['online'] = True
    watch.step()
    assert launches == ['test-stack.service']
    for _ in range(4): watch.step()
    assert len(launches) == 1 and watch.snapshot()['status'] == 'running'


def test_already_powered_at_ui_start_does_not_require_power_cycle():
    watch, _, _, launches = build_watch(online=True)
    watch.step()
    assert launches == ['test-stack.service']


def test_ui_restart_adopts_service_without_restarting_stack():
    watch, _, _, launches = build_watch(online=True, active=True)
    watch.step()
    assert launches == [] and watch.snapshot()['pid'] == 123


def test_existing_manual_stack_is_preserved():
    watch, _, _, launches = build_watch(online=True, existing=456)
    watch.step()
    assert launches == []
    assert watch.snapshot()['status'] == 'existing' and watch.snapshot()['pid'] == 456


def test_ping_loss_after_start_does_not_stop_or_restart_nodes():
    watch, hardware, unit, launches = build_watch(online=True)
    watch.step()
    hardware['online'] = False
    for _ in range(5): watch.step()
    assert len(launches) == 1 and watch.snapshot()['status'] == 'running'


def test_failed_boot_retries_when_hardware_remains_powered():
    watch, _, unit, launches = build_watch(online=True)
    watch.step()
    unit.update(ActiveState='failed', MainPID='0', ExecMainStatus='1')
    watch.step()
    assert len(launches) == 2


def test_offline_after_failed_boot_waits_for_power():
    watch, hardware, unit, launches = build_watch(online=True)
    watch.step()
    hardware['online'] = False
    unit.update(ActiveState='failed', MainPID='0', ExecMainStatus='1')
    watch.step()
    assert len(launches) == 1
    assert watch.snapshot()['status'] == 'waiting' and watch.snapshot()['returncode'] == 1


@pytest.mark.parametrize('command', [
    ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'rb_bringup', 'bringup_lidar_vision.launch.py'],
    ['bash', str(WS/'tools/boot.sh')], ['bash', 'tools/boot.sh'],
])
def test_exact_existing_process_detection(tmp_path, command):
    proc = tmp_path/'123'; proc.mkdir()
    (proc/'cmdline').write_bytes(b'\0'.join(a.encode() for a in command))
    (proc/'cwd').symlink_to(WS, target_is_directory=True)
    assert existing_stack(tmp_path) == 123


def test_inspection_command_is_not_mistaken_for_running_stack(tmp_path):
    proc = tmp_path/'123'; proc.mkdir()
    (proc/'cmdline').write_bytes(b'bash\0-c\0pgrep -f "ros2 launch rb_bringup bringup_lidar_vision.launch.py"\0')
    assert existing_stack(tmp_path) is None


def test_api_adds_boot_status_without_changing_route_contract():
    watch, _, _, _ = build_watch()
    watch.step()
    client = make_app(boot_watch=watch).test_client()
    state = client.get('/api/pass/state').json
    assert state['status'] == 'idle' and state['cursor'] == 0
    assert state['boot']['status'] == 'waiting'
    assert client.get('/api/boot/state').json['status'] == 'waiting'


def test_thread_keeps_polling_and_start_is_idempotent():
    import threading
    calls = []
    signal = threading.Event()
    watch, _, _, launches = build_watch()
    def probe(config):
        calls.append(True)
        if len(calls) >= 3: signal.set()
        return {'online': False}
    watch.probe = probe
    watch.start(); first = watch.thread; watch.start()
    assert watch.thread is first
    try:
        assert signal.wait(2) and launches == []
    finally:
        watch.stop_event.set(); watch.thread.join(2)


@pytest.mark.parametrize('owned', [True, False])
def test_confirmed_power_cycle_reboots_even_when_old_ros_process_is_alive(owned):
    watch, hardware, unit, launches = build_watch(active=owned, existing=456)
    now = [0.]
    watch.clock = lambda: now[0]
    restarts = []
    watch.restart = lambda name: restarts.append(name)
    hardware.update(carrier=False, online=False)
    watch.step()
    now[0] = 3.
    watch.step()
    assert watch.snapshot()['status'] == 'waiting'
    assert launches == restarts == []
    hardware.update(carrier=True, online=True)
    watch.step()
    assert (restarts if owned else launches) == ['test-stack.service']
    assert watch.snapshot()['status'] == 'starting'
    watch.step()
    assert len(launches)+len(restarts) == 1


def test_brief_link_flap_and_ping_only_loss_do_not_rearm():
    watch, hardware, unit, launches = build_watch(active=True)
    now = [0.]
    watch.clock = lambda: now[0]
    watch.restart = lambda name: pytest.fail('brief link or ICMP loss must not restart')
    hardware.update(carrier=False, online=False)
    watch.step()
    now[0] = 1.
    hardware.update(carrier=True, online=False)
    watch.step()
    now[0] = 10.
    watch.step()
    hardware['online'] = True
    watch.step()
    assert not watch.power_rearm and not launches
