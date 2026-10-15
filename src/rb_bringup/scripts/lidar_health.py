"""连续输入检测；不依赖 ROS，方便重放断电和时间跳变。"""
import math
import struct


def cdr_header_stamp(data):
    """只读取 ROS2 CDR 的 Header 时间，不展开后面的点云点数组。"""
    if len(data) < 12 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('无效的 CDR Header')
    sec, nanosec = struct.unpack_from('<iI' if data[1] == 1 else '>iI', data, 4)
    if nanosec >= 1000000000:
        raise ValueError('无效的 Header 纳秒')
    return sec + nanosec * 1e-9


class Stream:
    def __init__(self, max_gap):
        self.max_gap = max_gap
        self.stamp = self.received = self.since = None

    def clear(self):
        self.stamp = self.received = self.since = None

    def observe(self, stamp, wall, mono):
        if not math.isfinite(stamp) or not -.05 <= wall-stamp <= 1.:
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
        return (self.stamp is not None and -.05 <= wall-self.stamp <= 1.
                and mono-self.received <= self.max_gap)


class LidarHealth:
    def __init__(self, stable_seconds=1.):
        self.stable_seconds = stable_seconds
        self.imu = Stream(1.)
        self.lidar = Stream(1.)
        self.odom = Stream(1.)
        self.last_position = None
        self.fault_serial = 0
        self.reason = '等待雷达和 IMU'
        self.input_received = {'imu': None, 'lidar': None}

    def observe(self, name, stamp, wall, mono, position=None):
        if name in self.input_received:
            self.input_received[name] = mono  # 传输活跃与源时间是否有效分别判断。
        stream = getattr(self, name)
        previous = stream.stamp
        if not stream.observe(stamp, wall, mono):
            if (previous is not None and math.isfinite(stamp) and stamp < previous
                    and -.05 <= wall-stamp <= 1.):
                self.fault_serial += 1
                self.reason = name+' 源时间倒退'
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

    def inputs_receiving(self, mono, max_silence=2.):
        """Only actual delivery silence restarts the driver, not PTP settling."""
        return all(received is not None and mono-received <= max_silence
                   for received in self.input_received.values())

    def reset_inputs(self):
        self.imu.clear()
        self.lidar.clear()
        self.input_received = {'imu': None, 'lidar': None}

    def inputs_live(self, wall, mono):
        # 点云 header 是整帧扫描的起始采样时间，IMU 则逐样本发布。
        # 组帧/投递时 IMU 已经前进，二者的“最新帧”不是同一次采样。
        # 比较这两个 header 会把正常的点云延迟误判成失同步，反复杀掉
        # FAST-LIO。各自检查真实源时间及连续性；PTP 时基由驱动保障，
        # 同一扫描所需的历史 IMU 由 FAST-LIO 按采样时间匹配。
        return self.imu.live(wall, mono) and self.lidar.live(wall, mono)

    def inputs_stable(self, wall, mono):
        return (self.inputs_live(wall, mono)
                and mono-max(self.imu.since, self.lidar.since) >= self.stable_seconds)

    def odom_stable(self, wall, mono):
        return (self.odom.live(wall, mono)
                and mono-self.odom.since >= self.stable_seconds)

    def reset_odom(self):
        self.odom.clear()
        self.last_position = None
