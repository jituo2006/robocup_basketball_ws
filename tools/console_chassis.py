#!/usr/bin/env python3
"""Shared chassis driver supervisor; does not depend on a lidar connection."""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import uuid

ROOT = Path(__file__).resolve().parent.parent


def main():
    from chassis_owner import acquire
    from configure_chassis_can import configure
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    state_path = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')) / 'robocup-console-chassis.json'
    generation = uuid.uuid4().hex
    child = owner = None
    code = 0
    exit_reason = '共享底盘驱动已停止'
    def report(error=None):
        temporary = state_path.with_suffix('.new')
        temporary.write_text(json.dumps(dict(pid=child.pid if child is not None else None,
                                             generation=generation, error=error), ensure_ascii=False))
        temporary.replace(state_path)
    try:
        owner = acquire('driver')
        # Refuse a leftover unmanaged node instead of starting two CAN senders.
        for entry in Path('/proc').glob('[0-9]*'):
            try:
                args = (entry/'cmdline').read_bytes().split(b'\0')
                if args and Path(os.fsdecode(args[0])).name == 'rb_chassis_node':
                    raise RuntimeError('已有独立底盘节点运行，请先退出该程序')
            except (OSError, ValueError):
                pass
        while not stop.is_set():
            try:
                configure()
                break
            except (RuntimeError, OSError) as exc:
                report('CAN连接尚未就绪：' + str(exc))
                stop.wait(2.)
        if stop.is_set():
            return 0
        child = subprocess.Popen(['ros2', 'run', 'rb_chassis', 'rb_chassis_node', '--ros-args',
                                  '--params-file', str(ROOT/'src/rb_chassis/config/chassis.yaml')],
                                 start_new_session=True)
        report()
        while not stop.wait(.1):
            if child.poll() is not None:
                code = 1
                exit_reason = '底盘驱动已退出，正在等待服务重新启动'
                report(exit_reason)
                break
    except Exception as exc:
        exit_reason = '共享底盘驱动异常：' + str(exc)
        report(exit_reason)
        print(str(exc), flush=True)
        code = 1
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        report(exit_reason)
        if owner is not None:
            owner.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
