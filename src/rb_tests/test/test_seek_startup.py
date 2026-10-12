"""Late localisation must wait, never start motion from missing data."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


def load():
    path = Path(__file__).parents[3]/'tools/test_perimeter_ball.py'
    spec = importlib.util.spec_from_file_location('seek_startup', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_wait_pose_accepts_initialisation_longer_than_eight_seconds():
    mod = load()
    run = mod.Runner.__new__(mod.Runner)
    run.pose = None
    clock = NS(elapsed=0.)
    def spin(node, timeout_sec):
        clock.elapsed += timeout_sec
        if clock.elapsed >= 30.:
            run.pose = (1.5, 8., 0.)
    run.node = NS()
    run.rclpy = NS(ok=lambda:clock.elapsed<40., spin_once=spin)
    assert run.wait_pose()
    assert clock.elapsed>=30.


def test_wait_pose_can_be_cancelled_without_motion():
    mod = load()
    run = mod.Runner.__new__(mod.Runner)
    run.pose = None
    clock = NS(elapsed=0.)
    run.node = NS()
    run.rclpy = NS(ok=lambda:clock.elapsed<.2,
                   spin_once=lambda node,timeout_sec:setattr(clock,'elapsed',clock.elapsed+timeout_sec))
    assert not run.wait_pose()


def test_motion_readiness_waits_for_late_odometry_and_can_cancel():
    mod = load()
    run = mod.Runner.__new__(mod.Runner)
    clock = NS(elapsed=0.)
    run.args = NS(obstacle_guard=False)
    run.spin = lambda seconds:setattr(clock,'elapsed',clock.elapsed+seconds)
    run.rclpy = NS(ok=lambda:clock.elapsed<40.)
    def require():
        if clock.elapsed<25.:
            raise RuntimeError('雷达里程计尚未就绪')
    run.require_fresh_pose = require
    run.wait_motion_ready()
    assert clock.elapsed>=25.
    clock.elapsed = 0.
    run.rclpy.ok = lambda:clock.elapsed<.2
    with pytest.raises(RuntimeError, match='ROS 已停止'):
        run.wait_motion_ready()


def test_click_before_fastlio_starts_reaches_pose_wait(monkeypatch,tmp_path):
    mod = load()
    patterns = []
    def processes(cmd,**kwargs):
        patterns.append(cmd[-1])
        return NS(returncode=0 if cmd[-1] in ('[c]hassis_node','[l]ocalization_node','[p]erception_node') else 1,
                  stdout='')
    reached = []
    class Runner:
        def __init__(self,args):pass
        def wait_pose(self):reached.append('wait');return False  # Simulate explicit cancellation.
        def close(self):pass
    monkeypatch.setattr(mod.subprocess,'run',processes)
    monkeypatch.setattr(mod,'Runner',Runner)
    assert mod.main(['--dry','--yes','--no-mechanism','--log-file',str(tmp_path/'test.jsonl')])==1
    assert reached==['wait']
    assert '[f]astlio_mapping' not in patterns
