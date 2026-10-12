"""连续输入检测；不依赖 ROS，方便重放断电和时间跳变。"""
import math


class Stream:
    def __init__(self, max_gap):
        self.max_gap = max_gap
        self.stamp = self.received = self.since = None

    def clear(self):
        self.stamp = self.received = self.since = None

    def observe(self, stamp, wall, mono):
        if not math.isfinite(stamp) or not -.05 <= wall-stamp <= .5:
            self.clear()
            return False
        if self.stamp is not None:
            if stamp == self.stamp:
                return False  # 重发旧帧不延长连续输出时间。
            if not 0 < stamp-self.stamp <= self.max_gap or mono-self.received > self.max_gap:
                self.clear()
                self.stamp, self.received, self.since = stamp, mono, mono
                return False
        if self.since is None:
            self.since = mono
        self.stamp, self.received = stamp, mono
        return True

    def live(self, wall, mono):
        return (self.stamp is not None and -.05 <= wall-self.stamp <= .5
                and mono-self.received <= self.max_gap)


class LidarHealth:
    def __init__(self, stable_seconds=1.):
        self.stable_seconds = stable_seconds
        self.imu = Stream(.2)
        self.lidar = Stream(.5)
        self.odom = Stream(.5)
        self.last_position = None
        self.fault_serial = 0
        self.reason = '等待雷达和 IMU'

    def observe(self, name, stamp, wall, mono, position=None):
        stream = getattr(self, name)
        previous = stream.stamp
        if not stream.observe(stamp, wall, mono):
            if previous is not None and stamp != previous:
                self.fault_serial += 1
                self.reason = name+' 时间不连续'
            return False
        if name == 'odom' and position is not None:
            if not all(math.isfinite(v) for v in position):
                self.fault_serial += 1
                self.reason = '里程计非有限数'
                stream.clear()
                return False
            if previous is not None and self.last_position is not None:
                dt = stamp-previous
                distance = math.dist(position, self.last_position)
                if distance > .25+dt:  # 当前车速 .25m/s；允许 1m/s + .25m 测量余量。
                    self.fault_serial += 1
                    self.reason = '里程计位置跳变'
                    stream.clear()
                    return False
            self.last_position = position
        return True

    def inputs_live(self, wall, mono):
        return (self.imu.live(wall, mono) and self.lidar.live(wall, mono)
                and abs(self.imu.stamp-self.lidar.stamp) <= .25)

    def inputs_stable(self, wall, mono):
        return (self.inputs_live(wall, mono)
                and mono-max(self.imu.since, self.lidar.since) >= self.stable_seconds)

    def odom_stable(self, wall, mono):
        return (self.odom.live(wall, mono)
                and mono-self.odom.since >= self.stable_seconds)

    def reset_odom(self):
        self.odom.clear()
        self.last_position = None
