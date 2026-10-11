"""Prevent duplicate real chassis starts and misleading enable status."""
from collections import deque
from types import SimpleNamespace as NS
import io
from test_console_app import _load


def test_external_node_blocks_duplicate_start(monkeypatch):
    mod = _load()
    monkeypatch.setattr(mod.subprocess, 'run', lambda *a, **k: NS(stdout='ros2\nrb_chassis_node\n'))
    monkeypatch.setattr(mod.subprocess, 'Popen', lambda *a, **k: (_ for _ in ()).throw(AssertionError('duplicate start')))
    manager = mod.ChassisManager(deque(), None)
    ok, message = manager.start()
    assert not ok and '不会重复启动' in message
    ok, message = manager.stop()
    assert not ok and '外部' in message


def test_in_progress_start_cannot_be_reentered():
    mod = _load(); manager = mod.ChassisManager(deque(), None)
    manager.state['flow'] = 'running'
    assert manager.start() == (False, '底盘启动流程正在执行，请等待')


def test_can_write_error_invalidates_enable_status():
    mod = _load(); manager = mod.ChassisManager(deque(), None)
    lines = ('底盘使能指令已入队（不代表电机板确认）\n'
             'CAN TX failed: can0 errno=105 (No buffer space available)\n')
    proc = NS(stdout=io.BytesIO(lines.encode()), poll=lambda: 0)
    manager._pump(proc)
    assert not manager.enabled_line and manager.state['flow'] == 'failed'
    assert 'errno=105' in manager.state['step']
