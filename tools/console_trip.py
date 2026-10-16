"""Single-console arbitration and finite chassis-only round trip."""
import json
import os
from pathlib import Path
import threading
import time

from fallback_motion import Motion


class TaskBusy(RuntimeError):
    pass


def driver_state():
    path = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')) / 'robocup-console-chassis.json'
    try:
        data = json.loads(path.read_text())
        if data.get('error'):
            return None, data['error']
        pid = data.get('pid')
        if not isinstance(pid, int) or pid <= 1:
            return None, '共享底盘驱动尚未启动'
        os.kill(pid, 0)
        return data['generation'], None
    except (OSError, ValueError, KeyError):
        return None, '共享底盘驱动未运行，请重新打开投篮控制界面'


class ConsoleTasks:
    def __init__(self, runs, health, config, driver=driver_state, clock=time.monotonic):
        self.runs, self.health = runs, health
        self.motion = Motion(config)
        self.driver, self.clock = driver, clock
        self.lock = threading.RLock()
        self.publish = None
        self.other_publishers = False
        self.generation = None
        self.maintenance = False

    def shoot_running(self):
        with self.runs.lock:
            return any(run['status'] == 'running' for run in self.runs.runs.values())

    def activity(self):
        with self.lock:
            if self.maintenance:
                return 'maintenance'
            return ('shoot' if self.shoot_running() else
                    'chassis_trip' if self.motion.snapshot()['status'] == 'running' else 'idle')

    def snapshot(self):
        with self.lock:
            chassis = self.health.snapshot()['chassis']
            generation, error = self.driver()
            if error:
                chassis = dict(ready=False, reason=error)
            elif self.publish is None:
                chassis = dict(ready=False, reason='底盘运动通信尚未启动')
            elif self.other_publishers and not self.shoot_running():
                chassis = dict(ready=False, reason='检测到其他速度控制程序，请先退出遥控或独立任务')
            activity = self.activity()
            reason = ('控制界面正在更新，暂时不能执行任务' if activity == 'maintenance' else
                      '投篮回合正在执行，请等待结束' if activity == 'shoot' else
                      '底盘往返正在执行' if activity == 'chassis_trip' else chassis['reason'])
            return dict(can_run=chassis['ready'] and activity == 'idle', chassis=chassis,
                        reason=reason, activity=activity, motion=self.motion.snapshot(),
                        settings=dict(self.motion.config))

    def start_shoot(self, side, number):
        with self.lock:
            if self.activity() != 'idle':
                raise TaskBusy('当前任务正在执行，请等待结束后再选择任务')
            return self.runs.start(side, number)

    def start_trip(self):
        with self.lock:
            state = self.snapshot()
            if not state['can_run']:
                raise TaskBusy(state['reason'])
            self.generation, error = self.driver()
            if error:
                raise TaskBusy(error)
            self.motion.start(self.clock())
            return self.snapshot()

    def stop_trip(self, reason='已手动停止底盘往返', failed=False):
        with self.lock:
            running = self.motion.snapshot()['status'] == 'running'
            self.motion.stop(reason, failed)
            if running and self.publish is not None:
                self.publish(0.)
            return self.snapshot()

    def board_ready(self, ready):
        if not ready:
            self.stop_trip('底盘报告未就绪，本次往返已结束；恢复后需重新点击执行', True)

    def attach(self, publish):
        with self.lock:
            self.publish = publish

    def detach(self):
        with self.lock:
            self.stop_trip('投篮控制已关闭，底盘往返已结束', True)
            self.publish = None

    def tick(self, other_publishers=False):
        with self.lock:
            self.other_publishers = other_publishers
            if self.motion.snapshot()['status'] != 'running':
                return  # Never publish idle zeros over a shooting route's commands.
            generation, error = self.driver()
            if error or generation != self.generation:
                self.stop_trip(error or '底盘驱动已重新启动，本次往返已结束', True)
                return
            if other_publishers:
                self.stop_trip('出现其他速度控制程序，本次往返已结束', True)
                return
            velocity = self.motion.tick(self.clock())
            if velocity is not None:
                self.publish(velocity)
