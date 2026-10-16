"""Finite body-frame lateral motion; distances are commanded, not measured."""
import math
import threading


class Profile:
    def __init__(self, distance, speed, acceleration):
        self.ramp = min(speed / acceleration, math.sqrt(distance / acceleration))
        self.peak = acceleration * self.ramp
        self.cruise = max(0., distance / self.peak - self.ramp)
        self.duration = 2 * self.ramp + self.cruise
        self.acceleration = acceleration

    def velocity(self, elapsed):
        if elapsed < 0 or elapsed >= self.duration:
            return 0.
        return min(self.peak, self.acceleration * elapsed,
                   self.acceleration * (self.duration - elapsed))


def validate(config):
    bounds = dict(distance_m=(2., 2.), speed_mps=(.05, .25), acceleration_mps2=(.05, .5),
                  right_scale=(.5, 2.), left_scale=(.5, 2.), pause_s=(.5, 3.))
    for key, (low, high) in bounds.items():
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError('无效的底盘往返参数：' + key)
    return config


class Motion:
    def __init__(self, config):
        self.config = validate(config)
        self.lock = threading.RLock()
        self.state = dict(status='idle', phase='待执行', message='等待底盘就绪后点击执行往返', run_id=0)
        self.started = None

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def start(self, now):
        with self.lock:
            if self.state['status'] == 'running':
                raise RuntimeError('底盘往返任务正在执行，请勿重复启动')
            self.started = now
            self.right = Profile(2. / self.config['right_scale'], self.config['speed_mps'], self.config['acceleration_mps2'])
            self.left = Profile(2. / self.config['left_scale'], self.config['speed_mps'], self.config['acceleration_mps2'])
            self.state.update(status='running', phase='准备', message='发送零速，准备右移', run_id=self.state['run_id'] + 1)

    def stop(self, reason='已手动停止', failed=False):
        with self.lock:
            if self.state['status'] == 'running':
                self.state.update(status='failed' if failed else 'stopped', phase='已停止', message=reason)

    def tick(self, now):
        with self.lock:
            if self.state['status'] != 'running':
                return None  # Idle console must not overwrite another operator's commands.
            t = max(0., now - self.started)
            stages = [(.6, 0, '准备', '发送零速，准备右移'),
                      (self.right.duration, -1, '右移', '向车体右侧移动，目标 2 米'),
                      (self.config['pause_s'], 0, '换向', '右移结束，停车后准备返回'),
                      (self.left.duration, 1, '返回', '向车体左侧移动，目标 2 米'),
                      (1., 0, '停车', '返回结束，发送零速')]
            for duration, direction, phase, message in stages:
                if t < duration:
                    self.state.update(phase=phase, message=message)
                    profile = self.right if direction < 0 else self.left
                    return direction * profile.velocity(t) if direction else 0.
                t -= duration
            self.state.update(status='succeeded', phase='完成', message='底盘往返任务已结束，底盘已停车')
            return 0.
