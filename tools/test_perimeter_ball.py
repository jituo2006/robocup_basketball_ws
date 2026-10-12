#!/usr/bin/env python3
"""默认固定路线：右转90° → 到中线附近 → 左移找篮球 → 停车 → 转180° → 后退。

--route legacy 运行旧绕场/视觉伺服流程。

旧流程的转身和后摄复核/后退需显式传入 --turn-back。

设计要点
--------
* **独立脚本，不改状态机**：直接发 /cmd_vel。但 rb_mission 在 IDLE 时也会 20Hz
  发零速，会和本脚本抢话题 —— 所以要先让 mission 闭嘴。用 `--takeover` 自动
  SIGTERM 掉它（优雅退出），也可手动 `pkill -f '[m]ission_node'`。
  不动底盘/定位/感知，它们照常工作。
* **复用经过测试的运动学**：到点用 `rb_mission.geometry.goto_command`
  （任务侧在用的同一份，坐标系/单位约定一致）。
* **安全兜底**：每步都有超时；超时/异常/Ctrl-C 一律发零速再退出。

用法
----
    rbws
    python3 tools/test_perimeter_ball.py --dry              # 只查前置条件，不动车
    python3 tools/test_perimeter_ball.py --takeover --yes   # 完整跑
    python3 tools/test_perimeter_ball.py --skip-perimeter   # 跳过绕场，直接找球

    # 调参：
    python3 tools/test_perimeter_ball.py --takeover --speed 0.20 --inset 0.8
    python3 tools/test_perimeter_ball.py --takeover --ball-label ball_basketball
"""

from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path
import math
import os
import signal
import subprocess
import sys
import time

WS = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, str(Path(__file__).resolve().parent))
import front_obstacle_guard as obstacle_guard
from intake_control import start_intake, stop_intake
_TRACE = None
C_OK, C_BAD, C_WARN, C_DIM, C_OFF = (
    "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m")


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def bearing_to_world(yaw: float, bearing_rad: float, cam_yaw_offset: float = 0.0) -> float:
    """相机方位角 → 场地系方位角。

    ⚠️ 符号是本文件最容易错的地方：`Detection.bearing_rad` 是相机约定
    **右正左负**（顺时针为正），而场地系是 **y 朝左、逆时针为正**，两者相反。
    所以是 `yaw - bearing`，不是 `yaw + bearing`。
    （与 rb_localization.ekf2d.camera_bearing_to_world 同一约定，那边有单测锁住。）
    """
    return wrap_pi(yaw - bearing_rad + cam_yaw_offset)


def yaw_to_put_ball_behind(yaw: float, bearing_rad: float,
                          cam_yaw_offset: float = 0.0) -> float:
    """算出"让【背面】对着球"时车头该朝哪。

    球在场地系方位是 B；车头朝 B+π 时，车头背对球 → 背面正对球。
    """
    return wrap_pi(bearing_to_world(yaw, bearing_rad, cam_yaw_offset) + math.pi)


def depth_to_slant(depth_m: float, bearing_rad: float,
                   max_bearing_deg: float = 78.0) -> float:
    """单目【光轴深度】→ 斜距；不可信时返回 nan。

    ⚠️ Detection.distance_m 是**沿相机光轴的深度 Z**（感知 = fx·real_size/pixel_size，
    docs/13 明确"按光轴深度处理"），不是直线斜距。水平偏移 = Z·tan(b)，
    所以 r = √(Z²+(Z·tan b)²) = Z/cos(b)。直接拿 Z 当斜距会让球越偏离画面中心
    位置越错（偏 45° 少算 29%）。b 接近 ±90° 时斜距发散，判不可信。
    """
    if not (depth_m == depth_m) or depth_m <= 0:
        return float("nan")
    if not (bearing_rad == bearing_rad):
        return float("nan")
    if abs(math.degrees(bearing_rad)) > max_bearing_deg:
        return float("nan")
    c = math.cos(bearing_rad)
    return depth_m / c if c > 1e-3 else float("nan")


def ball_xy(x: float, y: float, yaw: float, bearing_rad: float, distance: float,
            cam_yaw_offset: float = 0.0) -> tuple[float, float]:
    """用「自车位置 + 方位角 + 光轴深度」反算球在场地系下的坐标。

    `distance` 是光轴深度，先换算成斜距（见 depth_to_slant）；不可信时抛 ValueError。
    """
    slant = depth_to_slant(distance, bearing_rad)
    if not (slant == slant):
        raise ValueError(
            f"方位角 {math.degrees(bearing_rad):.0f}° 或深度 {distance} 不可信，无法定位")
    ang = bearing_to_world(yaw, bearing_rad, cam_yaw_offset)
    return x + slant * math.cos(ang), y + slant * math.sin(ang)


def median_ball(samples):
    """把若干 (bearing, distance) 采样合成一个稳健估计。

    为什么用中位数：单目检测偶尔会把干扰物/局部特征当球，给出离谱的距离或方位；
    中位数对这类离群值免疫，平均会被带偏（现场"找球非常不精准"的一部分原因）。

    返回 (bearing, distance, n) 或 None（样本不足/全非法）。
    """
    ok = [(float(b), float(d)) for b, d in (samples or [])
          if float(b) == float(b) and float(d) == float(d) and float(d) > 0.05]
    if len(ok) < 2:
        return None
    bs = sorted(x[0] for x in ok)
    ds = sorted(x[1] for x in ok)
    mid = len(ok) // 2
    return (bs[mid], ds[mid], len(ok))


def approach_axis(yaw: float, bearing_rad: float) -> float:
    """球在场地系的方位角。

    bearing 是【右正左负】（相机约定），场地系是逆时针为正 ⇒ 方位 = yaw − bearing。
    沿这个角的单位向量 (cos, sin) 就是从车指向球的水平方向。
    """
    return wrap_pi(yaw - bearing_rad)


def ball_body_from_rear(rear_offset_x: float, bearing_rad: float, distance_m: float):
    """后摄检测 → 球在【车体坐标】的位置（旋转中心为原点，+x 前、+y 左）。

    ⚠️ 后摄朝 -x、顶朝上 ⇒ 图像右侧对应车的【左侧(+y)】
       ⇒ 球相对车体的方向 = (-cos b, +sin b)（b 是右正左负的 bearing）
    ⚠️ distance_m 是【光轴深度】，要 /cos(b) 才是斜距（和任务侧同一套换算）。
    """
    b = float(bearing_rad)
    d = float(distance_m)
    if not (b == b and d == d) or d <= 0.0:
        return None
    slant = d / max(math.cos(b), 1e-3)
    return (-float(rear_offset_x) - slant * math.cos(b), slant * math.sin(b))


def ball_world_from_rear(x: float, y: float, yaw: float, rear_offset_x: float,
                         bearing_rad: float, distance_m: float):
    """后摄检测 → 球在【场地坐标】的位置。"""
    pb = ball_body_from_rear(rear_offset_x, bearing_rad, distance_m)
    if pb is None:
        return None
    bx, by = pb
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * bx - s * by, y + s * bx + c * by)


def rear_world_angle(yaw: float, bearing_rad: float) -> float:
    """后摄：球在场地系的方位角 = yaw + bearing（前摄是 yaw − bearing，符号相反）。"""
    return wrap_pi(yaw + bearing_rad)


def fixed_route_plan(start_pose, forward_m: float, scan_m: float):
    """相对出发朝向：右转后前进，随后车体左移（沿原出发朝向）。"""
    x, y, yaw = start_pose
    right_yaw = wrap_pi(yaw - math.pi / 2)
    near_line = (x + forward_m * math.cos(right_yaw),
                 y + forward_m * math.sin(right_yaw))
    left_axis = (math.cos(yaw), math.sin(yaw))
    scan_end = (near_line[0] + scan_m * left_axis[0],
                near_line[1] + scan_m * left_axis[1])
    return right_yaw, near_line, left_axis, scan_end


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_at(history, stamp: float, max_gap: float = 0.15):
    """只在源时间戳两侧位姿间插值，跨 ±π 时走最短角，不外推。"""
    for t, pose in history:
        if abs(t - stamp) < 1e-6:
            return pose
    for (ta, a), (tb, b) in zip(history, list(history)[1:]):
        if ta < stamp < tb and tb - ta <= max_gap:
            k = (stamp - ta) / (tb - ta)
            return (a[0] + k * (b[0] - a[0]), a[1] + k * (b[1] - a[1]),
                    wrap_pi(a[2] + k * wrap_pi(b[2] - a[2])))
    return None


def trace(event, **data):
    if _TRACE is not None:
        _TRACE.write(json.dumps(dict(time=time.time(), event=event, **data),
                                ensure_ascii=False, allow_nan=False) + "\n")
        _TRACE.flush()


def log(tag: str, msg: str, color: str = "") -> None:
    trace("phase", tag=tag, message=msg)
    print(f"  {C_DIM}{time.strftime('%H:%M:%S')}{C_OFF} "
          f"{color}{tag:10}{C_OFF} {msg}", flush=True)


class Runner:
    """管 ROS 连接、位姿/检测缓存、以及三个基本动作（到点 / 转向 / 找球）。"""

    def __init__(self, args) -> None:
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from rb_msgs.msg import DetectionArray
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from rclpy.qos import qos_profile_sensor_data

        self.args = args
        self.rclpy = rclpy
        self._Twist = Twist
        rclpy.init()
        self.node = rclpy.create_node("perimeter_ball_test")
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.pose: tuple[float, float, float] | None = None
        self.dets: list = []
        self.last_det_time = 0.0
        self.pose_stamp = self.odom_stamp = 0.0
        self.det_stamp = self.rear_det_stamp = 0.0
        self.motion_fault = None
        self.last_obstacle_tf_error = ""
        self.motion_guard = False
        self.visual_guard = False
        self.obstacle_body_cloud = None
        self.obstacle_cloud_stamp = 0.0
        self.obstacle_cloud_status = "not_received"
        self.obstacle_scoop = False
        self.obstacle_cloud_history = deque(maxlen=4)
        from tf2_ros import Buffer, TransformListener
        self.obstacle_tf = Buffer()
        self.obstacle_tf_listener = TransformListener(self.obstacle_tf, self.node)
        self.pose_history = deque(maxlen=250)
        self.yaw_rate = float("inf")

        self.node.create_subscription(PoseWithCovarianceStamped, "/localization/pose",
                                      self._on_pose, 10)
        self.node.create_subscription(DetectionArray, "/perception/detections",
                                      self._on_dets, 10)
        # 雷达兜底找球：需要点云 + camera_init 位姿（把点云转到车体系）
        self.cloud = None          # Nx3 numpy（camera_init 系）
        self.odom_cam = None       # (x, y, yaw) in camera_init
        self.node.create_subscription(PointCloud2, self.args.lidar_cloud,
                                      self._on_cloud, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, self.args.lidar_odom,
                                      self._on_odom_cam, 10)

        # 后摄像头（车正后方）：转 180° 之后靠它做精准定位
        self.dets_rear = []
        self.node.create_subscription(
            DetectionArray, self.args.rear_topic, self._on_dets_rear, 10)

        # 延迟导入任务侧的运动学（与任务用同一份，避免公式两套）
        sys.path.insert(0, f"{WS}/build/rb_mission")
        from rb_mission.geometry import goto_command
        self._goto_command = goto_command

    # -- 回调 ---------------------------------------------------------------
    def _on_pose(self, m) -> None:
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        if math.hypot(q.x, q.y, q.z, q.w) > 1e-6:
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        else:
            yaw = p.z
        stamp = stamp_seconds(m.header.stamp)
        pose = (p.x, p.y, yaw)
        if not all(math.isfinite(v) for v in (*pose, stamp)) or stamp <= self.pose_stamp:
            return
        if self.pose is not None and self.pose_stamp > 0:
            dt = stamp-self.pose_stamp
            jump = math.hypot(pose[0]-self.pose[0],pose[1]-self.pose[1])
            if jump > getattr(self.args,"pose_jump_margin",.25) + getattr(self.args,"pose_max_speed",1.0)*dt:
                self.motion_fault = f"定位跳变 {jump:.2f}m/{dt:.3f}s，需重新初始化定位"
                trace("pose_fault", reason=self.motion_fault, previous=list(self.pose), received=list(pose))
                return
            self.yaw_rate = wrap_pi(yaw - self.pose[2]) / dt
        self.pose, self.pose_stamp = pose, stamp
        self.pose_history.append((stamp, pose))

    def _load_lidar_cfg(self) -> dict:
        """从 perception.yaml 读 lidar_ball 参数（读不到就用模块默认值）。"""
        cfg = {"enabled": True}
        try:
            import yaml
            from pathlib import Path as _P
            ws = _P(__file__).resolve().parents[1]
            f = ws / "install/rb_perception/share/rb_perception/config/perception.yaml"
            if not f.is_file():
                f = ws / "src/rb_perception/config/perception.yaml"
            d = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
            cfg.update(((d.get("perception") or {}).get("lidar_ball") or {}))
        except Exception:  # noqa: BLE001
            pass
        cfg["enabled"] = True     # 兜底用，强制打开
        return cfg

    def _on_cloud(self, m) -> None:
        try:
            pts = obstacle_guard.pointcloud_xyz(m)
            self.cloud = pts
            if pts is not None:
                self.obstacle_cloud_history.append((
                    pts, m.header.stamp, m.header.frame_id))
        except Exception as exc:  # noqa: BLE001
            self.obstacle_cloud_status = "parse_error: " + str(exc)

    def _on_odom_cam(self, m) -> None:
        p = m.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_cam = (p.position.x, p.position.y, yaw)
        self.odom_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def lidar_find_ball(self, seconds: float | None = None):
        """雷达兜底找球：返回场地系候选列表 [(fx, fy, bearing_deg, dist), ...]。

        为什么值得先跑这步：雷达约 360° 覆盖、6m 范围，**不用转圈**就知道球
        的大致方位；前摄只有约 ±40° 视野，必须转着找。所以合理顺序是
        "雷达给大致位置 → 摄像头凑近确认"，而不是让摄像头盲转。

        返回的 bearing_deg 是【车体系】方位（右正左负，和相机一致）。
        """
        if self.args.no_lidar_fallback:
            return []
        secs = float(seconds if seconds is not None else self.args.lidar_seconds)
        log("雷达找球", f"等点云/位姿（{secs:.0f}s）…")
        t0 = time.time()
        best = []
        while time.time() - t0 < secs:
            self.spin(0.1)
            if self.cloud is None or self.odom_cam is None or self.pose is None:
                continue
            try:
                from rb_perception.lidar_ball import body_frame_cloud, find_ball_candidates
                body = body_frame_cloud(self.cloud, self.odom_cam)
                cands = find_ball_candidates(body, self.args.lidar_cfg)
            except Exception as e:  # noqa: BLE001
                log("雷达找球", f"失败: {e}", C_WARN)
                return []
            if cands:
                best = cands
                break
        if not best:
            log("雷达找球", "附近没找到球形候选（地面杂点/球太远/球在盲区）", C_WARN)
            return []
        # 车体系 → 场地系（用当前场地位姿；+y 是左）
        px, py, pyaw = self.pose
        out = []
        for c in best:
            bx, by = float(c["x"]), float(c["y"])
            c_, s_ = math.cos(pyaw), math.sin(pyaw)
            fx = px + c_ * bx - s_ * by
            fy = py + s_ * bx + c_ * by
            dist = math.hypot(bx, by)
            # 车体系方位：右正左负（和相机 bearing 约定一致）
            b = -math.degrees(math.atan2(by, bx))
            out.append((fx, fy, b, dist))
        out.sort(key=lambda o: o[3])
        for fx, fy, b, d in out[:3]:
            log("雷达候选", f"场地({fx:.2f},{fy:.2f}) 车体系方位{b:+.1f}° 距离{d:.2f}m", C_OK)
        return out

    def _on_dets_rear(self, m) -> None:
        # ⚠️ 这个 Runner 不用锁 —— 回调就是直接赋值（和 _on_dets 一样）。
        #    我第一版写了 `with self.lock:`，而类里根本没有 lock 属性 →
        #    回调抛 AttributeError，直接打断 ros2 执行器（现象是"找球"刚开始就崩）。
        self.dets_rear = list(m.detections)
        self.rear_det_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def best_ball_rear(self):
        """后摄里置信度最高的目标球检测。"""
        best = None
        for d in list(getattr(self, "dets_rear", []) or []):
            if (d.label != self.args.ball_label or not self.detection_fresh(stamp_seconds(d.stamp))
                    or not math.isfinite(d.confidence) or d.confidence < self.args.ball_confidence
                    or not math.isfinite(d.bearing_rad) or not math.isfinite(d.distance_m)
                    or d.distance_m <= 0.05):
                continue
            if best is None or d.confidence > best.confidence:
                best = d
        return best

    def measure_rear_stable(self, timeout: float = 6.0, n: int | None = None):
        """后摄多帧取中位数（同前摄的稳健做法）。"""
        self.stop()
        self.spin(self.args.settle_seconds)
        cutoff = self.node.get_clock().now().nanoseconds * 1e-9
        want = max(3, int(n or self.args.stable_frames))
        got = []
        last_stamp = None
        t0 = time.time()
        while len(got) < want and time.time() - t0 < timeout:
            d = self.best_ball_rear()
            stamp = stamp_seconds(d.stamp) if d is not None else 0.0
            if d is not None and stamp > cutoff and stamp != last_stamp:
                got.append((float(d.bearing_rad), float(d.distance_m)))
                last_stamp = stamp
            self.spin(0.05)
        return median_ball(got) if len(got) >= want else None

    def _on_dets(self, m) -> None:
        self.dets = list(m.detections)
        self.last_det_time = time.time()
        self.det_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def detection_fresh(self, stamp: float) -> bool:
        now = self.node.get_clock().now().nanoseconds * 1e-9
        return math.isfinite(stamp) and stamp > 0 and -0.05 <= now - stamp <= getattr(getattr(self, "args", None), "detection_timeout", 0.5)

    def spin(self, sec: float) -> None:
        t0 = time.time()
        while time.time() - t0 < sec:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
        if self.motion_guard:
            self.require_fresh_pose()

    def require_fresh_pose(self) -> None:
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if getattr(self,"motion_fault",None):
            self.stop()
            raise RuntimeError(self.motion_fault)
        if self.pose is not None and hasattr(self,"field_l") and not (
                -2 <= self.pose[0] <= self.field_l+2 and -2 <= self.pose[1] <= self.field_w+2):
            self.stop()
            raise RuntimeError("定位已超出场地及出发区范围，需重新初始化定位")
        if self.pose is None or not all(math.isfinite(v) for v in self.pose):
            self.stop()
            raise RuntimeError("位姿缺失或非有限数，已请求停车")
        for name, stamp in (("定位", self.pose_stamp), ("雷达里程计", self.odom_stamp)):
            if not math.isfinite(stamp) or stamp <= 0 or not -0.05 <= now - stamp <= self.args.pose_timeout:
                self.stop()
                raise RuntimeError(f"{name}源时间戳过期，已请求停车")

    def stop(self) -> None:
        trace("stop")
        for _ in range(5):
            self.cmd_pub.publish(self._Twist())
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_pose(self, timeout: float = 8.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and self.pose is None:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.pose is not None

    def wait_motion_ready(self, timeout: float = 8.0) -> None:
        deadline = time.monotonic() + timeout
        while True:
            self.spin(0.05)
            try:
                self.require_fresh_pose()
                if getattr(self.args,"obstacle_guard",False):
                    now=self.node.get_clock().now().nanoseconds*1e-9
                    if not obstacle_guard.fresh(self.det_stamp,now,self.args.obstacle_timeout):
                        raise RuntimeError("前摄新数据尚未就绪，保持停车")
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise

    def observation_pose(self, det):
        return pose_at(self.pose_history, stamp_seconds(det.stamp), self.args.pose_match_gap)

    def publish_command(self, msg):
        if getattr(self, "motion_guard", False):
            self.require_fresh_pose()
        if getattr(self, "visual_guard", False) and self.best_ball() is None:
            self.stop()
            raise RuntimeError("接近期间球观测失效，已停车；需要重新确认")
        if getattr(self.args, "obstacle_guard", False) and not getattr(self, "obstacle_scoop", False):
            self.check_obstacle(msg)
        self.cmd_pub.publish(msg)
        trace("command", pose=list(self.pose) if self.pose else None,
              pose_stamp=self.pose_stamp, vx=float(msg.linear.x), vy=float(msg.linear.y),
              wz=float(msg.angular.z), yaw_rate=self.yaw_rate if math.isfinite(self.yaw_rate) else None)

    def update_obstacle_cloud(self, now):
        # TF for a new cloud can arrive a little later. Try recent source frames,
        # never the latest TF with an old cloud and never renew its source stamp.
        for pts, stamp_msg, frame in reversed(getattr(self,"obstacle_cloud_history", [])):
            stamp = stamp_seconds(stamp_msg)
            if not obstacle_guard.fresh(stamp,now,self.args.obstacle_timeout):
                continue
            if stamp == self.obstacle_cloud_stamp and self.obstacle_body_cloud is not None:
                return
            try:
                from rclpy.time import Time
                tf = self.obstacle_tf.lookup_transform("base_link",frame,Time.from_msg(stamp_msg))
                t,q = tf.transform.translation,tf.transform.rotation
                self.obstacle_body_cloud = obstacle_guard.transform_points(
                    pts,(t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
                self.obstacle_cloud_stamp = stamp
                self.obstacle_cloud_status = "transformed"
                return
            except Exception as exc:
                self.last_obstacle_tf_error = str(exc)
                continue

    def check_obstacle(self, msg):
        vx, vy = float(msg.linear.x), float(msg.linear.y)
        if not all(math.isfinite(v) for v in (vx,vy)):
            self.stop()
            raise RuntimeError("避障停车: 非有限速度")
        # Front guard does not claim rear coverage or block in-place turns.
        if vx < -.001 or math.hypot(vx,vy) < .001:
            return
        now = self.node.get_clock().now().nanoseconds*1e-9
        reason = None
        if not obstacle_guard.fresh(self.det_stamp, now, self.args.obstacle_timeout):
            reason = "camera:stale_stream"
        else:
            reason = obstacle_guard.camera_hazard(self.dets,now,vx,vy,self.args)
        self.update_obstacle_cloud(now)
        lidar_status = getattr(self,"obstacle_cloud_status","not_received")
        if self.obstacle_body_cloud is not None and obstacle_guard.fresh(
                self.obstacle_cloud_stamp,now,self.args.obstacle_timeout):
            hazard, lidar_status = obstacle_guard.lidar_hazard(
                self.obstacle_body_cloud,vx,vy,self.args)
            reason = reason or hazard
        else:
            lidar_status = "unavailable_or_stale"
        # Sideways travel cannot be protected by a forward camera alone.
        if abs(vy) > .001 and abs(vy)>abs(vx) and lidar_status != "usable":
            reason = reason or "lidar:lateral_coverage_unavailable"
        trace("obstacle_guard", reason=reason, lidar_status=lidar_status,
              cloud_stamp=self.obstacle_cloud_stamp, tf_error=getattr(self,"last_obstacle_tf_error",""),
              camera_age=now-self.det_stamp, vx=vx, vy=vy)
        if reason:
            self.stop()
            log("避障停车", reason+"；本次运动中止，清障后重新启动", C_WARN)
            raise RuntimeError("避障停车: "+reason)

    # -- 基本动作 -----------------------------------------------------------
    def goto(self, tx: float, ty: float, timeout: float, tolerance: float | None = None) -> bool:
        """平移到 (tx, ty)。用任务侧同一套 goto_command。"""
        t0 = time.time()
        last_log = 0.0
        while time.time() - t0 < timeout:
            self.spin(0.05)
            if self.pose is None:
                self.stop()
                return False
            x, y, yaw = self.pose
            vx, vy, _wz, dist = self._goto_command(
                x, y, yaw, tx, ty,
                self.args.kp, self.args.kp_yaw, self.args.speed, self.args.max_ang,
                face_travel=False,
                approach_radius=self.args.approach_radius,
                approach_speed=(self.args.approach_speed if getattr(self, "visual_guard", False)
                                else self.args.speed))
            if dist < (self.args.tol if tolerance is None else tolerance):
                self.stop()
                log("到点", f"({tx:.2f},{ty:.2f}) 剩余 {dist*100:.1f}cm", C_OK)
                return True
            t = self._Twist()
            t.linear.x, t.linear.y = vx, vy
            self.publish_command(t)
            if time.time() - last_log > 1.0:
                last_log = time.time()
                log("移动", f"→({tx:.2f},{ty:.2f}) 剩余 {dist:.2f}m  "
                            f"v={math.hypot(vx, vy):.2f}")
        self.stop()
        log("超时", f"到 ({tx:.2f},{ty:.2f}) 未完成", C_BAD)
        return False

    def rotate_to(self, target_yaw: float, timeout: float) -> bool:
        t0 = time.time()
        stable_since = None
        while time.time() - t0 < timeout:
            self.spin(0.05)
            if self.pose is None:
                self.stop()
                return False
            err = wrap_pi(target_yaw - self.pose[2])
            if abs(err) < self.args.yaw_tol:
                self.stop()
                if abs(self.yaw_rate) <= self.args.settle_yaw_rate:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= self.args.settle_seconds:
                        log("转向到位", f"yaw 误差 {math.degrees(err):+.1f}°，已稳定", C_OK)
                        return True
                else:
                    stable_since = None
                continue
            stable_since = None
            t = self._Twist()
            wz = self.args.kp_yaw * err - self.args.kd_yaw * self.yaw_rate
            if not math.isfinite(wz):
                self.stop()
                continue
            t.angular.z = max(-self.args.max_ang, min(self.args.max_ang, wz))
            trace("rotation", target_yaw=target_yaw, error=err)
            self.publish_command(t)
        self.stop()
        log("超时", "转向未完成", C_BAD)
        return False

    # -- 视觉 ---------------------------------------------------------------
    def best_ball(self):
        """当前最好的目标球检测（有有效距离的优先，再按置信度）。"""
        cands = [d for d in self.dets if d.label == self.args.ball_label
                 and getattr(d, "source", "camera") == "camera"
                 and math.isfinite(d.confidence) and d.confidence >= self.args.ball_confidence
                 and math.isfinite(d.bearing_rad) and abs(d.bearing_rad) < math.radians(78)
                 and math.isfinite(d.distance_m) and d.distance_m > 0.05
                 and self.detection_fresh(stamp_seconds(d.stamp))
                 and self.observation_pose(d) is not None]
        return max(cands, key=lambda d: d.confidence) if cands else None

    def ball_bearing_world(self, det) -> float:
        """球在场地系里的方位角。

        ⚠️ bearing_rad 是"右正左负"（相机约定），场地系是 y 朝左、逆时针为正，
        两者相反 → 用 yaw - bearing（与 localization.camera_bearing_to_world 一致）。
        """
        pose = self.observation_pose(det)
        return bearing_to_world(pose[2], det.bearing_rad, self.args.cam_yaw_offset) if pose else float("nan")

    def ball_field_xy(self, det) -> tuple[float, float] | None:
        pose = self.observation_pose(det)
        if pose is None or not math.isfinite(det.distance_m) or det.distance_m <= 0:
            return None
        return ball_xy(*pose, det.bearing_rad, det.distance_m, self.args.cam_yaw_offset)

    def measure_ball_stable(self, timeout: float = 6.0, n: int | None = None):
        """连续采几帧取【中位数】。

        为什么要中位数不要平均：单帧偶尔会把干扰物当成球（距离/方位离谱），
        平均会被它带偏；中位数对离群值免疫。
        """
        self.stop()
        # 只接受停车后的新曝光；旧缓存和停车前画面均不算确认帧。
        self.spin(self.args.settle_seconds)
        cutoff = self.node.get_clock().now().nanoseconds * 1e-9
        want = max(3, int(n or self.args.stable_frames))
        got, stamps = [], set()
        previous = None
        last_observation = cutoff
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.spin(0.05)
            d = self.best_ball()
            if d is None or abs(self.yaw_rate) > self.args.settle_yaw_rate:
                got, stamps, previous = [], set(), None
                continue
            stamp = stamp_seconds(d.stamp)
            if stamp <= cutoff or stamp <= last_observation or stamp in stamps:
                continue
            # 停车后同一目标应连续一致，拒绝在两颗球之间跳变。
            world = self.ball_field_xy(d)
            if world is None or self.pose is None:
                continue
            dx, dy = world[0] - self.pose[0], world[1] - self.pose[1]
            b = wrap_pi(self.pose[2] + self.args.cam_yaw_offset - math.atan2(dy, dx))
            # 将曝光时的方位/深度补偿到当前停车位姿，再做稳健确认。
            current = (b, math.hypot(dx, dy) * math.cos(b))
            if current[1] <= 0.05 or abs(b) >= math.radians(78):
                got, stamps, previous = [], set(), None
                continue
            if stamp - last_observation > self.args.detection_timeout:
                got, stamps, previous = [], set(), None
            last_observation = stamp
            if previous and (abs(current[0] - previous[0]) > math.radians(self.args.track_bearing_deg)
                             or abs(current[1] - previous[1]) > self.args.track_distance):
                got, stamps = [], set()
            previous = current
            got.append(current)
            stamps.add(stamp)
            trace("observation", stamp=stamp, age=self.node.get_clock().now().nanoseconds * 1e-9-stamp,
                  label=d.label, confidence=float(d.confidence), bearing=current[0], distance=current[1],
                  pose=list(self.observation_pose(d)))
            if len(got) >= want:
                return median_ball(got)
        return None

    def servo_to_ball(self, target_d: float, tol_b_deg: float, tol_d: float,
                      steps: int, max_move: float, max_turn_deg: float):
        """闭环把车摆到"正对球、且旋转中心距球 target_d"。

        每一步都重新量，不靠一次性解算 —— 单目测距在近距离本来就有几个百分点
        误差，只有闭环才能收敛到"正对着"。
        返回 (ok, 最后一次测量)
        """
        last = None
        for i in range(1, steps + 1):
            m = self.measure_ball_stable()
            if m is None:
                log("丢球", f"第 {i}/{steps} 步：看不到球了，先找", C_WARN)
                if self.find_ball(timeout=12.0) is None:
                    return False, last
                continue
            bearing, dist, nf = m
            last = m
            db = math.degrees(bearing)
            dd = dist - target_d
            log("伺服", f"第 {i}/{steps} 步：方位 {db:+.2f}°（容差 ±{tol_b_deg:.1f}°）、"
                        f"距离 {dist:.3f}m（目标 {target_d:.2f}±{tol_d:.2f}，"
                        f"差 {dd:+.3f}） [{nf} 帧中位数]")
            if abs(db) <= tol_b_deg and abs(dd) <= tol_d:
                log("对准完成", f"方位 {db:+.2f}°，距离 {dist:.3f}m —— 摄像头正对球", C_OK)
                return True, m

            # 先转：把球拉到画面正中（bearing → 0）
            # bearing 右正左负，ROS yaw 逆时针为正，右转必须减小 yaw。
            turn = max(-math.radians(max_turn_deg),
                       min(math.radians(max_turn_deg), -bearing))
            if abs(db) > tol_b_deg:
                if self.pose is not None:
                    if not self.rotate_to(wrap_pi(self.pose[2] + turn), timeout=15.0):
                        return False, last
                    # 转动后重新测量，不能再按旧方位角平移。
                    continue

            # 再走：朝球方向把距离补上（正数=还远，前进；负数=太近，后退）
            move = max(-max_move, min(max_move, dd))
            if abs(move) > 0.01 and self.pose is not None:
                yaw = self.pose[2]
                # 球在世界系的方位 = yaw - bearing（右正左负）
                ang = wrap_pi(yaw - bearing)
                x, y, _ = self.pose
                self.visual_guard = True
                try:
                    if not self.goto(x + move * math.cos(ang), y + move * math.sin(ang),
                                     timeout=20.0, tolerance=self.args.step_tol):
                        return False, last
                finally:
                    self.visual_guard = False
        log("未收敛", f"伺服 {steps} 步没进容差，以最后一次为准", C_WARN)
        return False, last

    def backup_along(self, dist: float, timeout: float = 25.0) -> bool:
        """沿【车头方向】平移 dist（正=前进；负=后退）。用里程计闭环。"""
        if self.pose is None:
            return False
        x0, y0, yaw = self.pose
        tx = x0 + dist * math.cos(yaw)
        ty = y0 + dist * math.sin(yaw)
        log("平移", f"{'前进' if dist > 0 else '后退'} {abs(dist):.2f}m "
                    f"→ ({tx:.2f},{ty:.2f})")
        return self.goto(tx, ty, timeout=timeout)

    def centered_basketball_candidates(self, detections, centered=True):
        """中心竖带中的新鲜视觉篮球，不依赖单目距离估计。"""
        candidates = []
        for d in detections:
            if d.label != 'ball_basketball' or getattr(d, 'source', 'camera') != 'camera':
                continue
            values = [float(getattr(d, k, float('nan'))) for k in
                      ('confidence', 'x_min', 'y_min', 'x_max', 'y_max')]
            confidence, x1, y1, x2, y2 = values
            if not all(math.isfinite(v) for v in values):
                continue
            if not self.args.ball_confidence <= confidence <= 1:
                continue
            if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
                continue
            if not self.detection_fresh(stamp_seconds(d.stamp)):
                continue
            if not centered or abs((x1+x2)/2 - .5) <= self.args.route_center_tolerance:
                candidates.append(d)
        return max(candidates, key=lambda d: d.confidence) if candidates else None

    def centered_basketball(self):
        return self.centered_basketball_candidates(self.dets)

    def rear_visual_scoop(self):
        self.obstacle_scoop = True
        try:
            return self._rear_visual_scoop()
        finally:
            self.obstacle_scoop = False

    def intake_text(self, command):
        from rb_msgs.srv import SerialText
        if not hasattr(self, 'intake_client'):
            self.intake_client = self.node.create_client(SerialText, '/rb_launcher/text')
        if not self.intake_client.wait_for_service(timeout_sec=1.):
            return False, '机构服务未启动'
        req = SerialText.Request()
        req.command = command
        future = self.intake_client.call_async(req)
        deadline = time.monotonic()+3.
        while not future.done() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=.03)
        if not future.done():
            self.intake_client.remove_pending_request(future)
            return False, '机构指令无响应'
        response = future.result()
        return bool(response.success), response.message

    def aligned_intake_approach(self):
        use_intake = getattr(self.args, 'route_intake', False)
        if not use_intake:
            return self.rear_distance_scoop()
        try:
            ok, message = start_intake(self.intake_text,
                                      wait=lambda delay: self.spin(delay))
            log('启动吸球', message, C_OK if ok else C_BAD)
            if not ok:
                return False
            reached = self.rear_distance_scoop()
            if reached:
                self.spin(2.)
            return reached
        finally:
            self.stop()
            ok, message = stop_intake(self.intake_text)
            log('停止吸球', message, C_OK if ok else C_BAD)

    def _rear_visual_scoop(self):
        """YOLO is used only to align laterally; approach then uses pose distance."""
        self.stop()
        origin = self.pose[:2]
        deadline = time.monotonic() + self.args.route_rear_timeout
        seen = False
        centered_frames = 0
        last_center_stamp = None
        while time.monotonic() < deadline:
            self.spin(.03)
            d = self.centered_basketball_candidates(getattr(self, 'dets_rear', []), centered=False)
            if d is None:
                self.stop()
                if seen:
                    log('后摄停车', '横移对准期间篮球丢失或过期，保持停车', C_WARN)
                    return False
                continue
            seen = True
            error = (float(d.x_min)+float(d.x_max))/2 - .5
            displacement = math.hypot(self.pose[0]-origin[0], self.pose[1]-origin[1])
            trace('rear_align', cx=error+.5, displacement=displacement,
                  confidence=float(d.confidence), stamp=stamp_seconds(d.stamp))
            if displacement >= getattr(self.args,'route_rear_align_distance',.4):
                self.stop()
                log('后摄停车', '横移纠偏已达位移上限', C_WARN)
                return False
            if abs(error) <= self.args.route_rear_center_tolerance:
                self.stop()
                stamp = stamp_seconds(d.stamp)
                if last_center_stamp is None or stamp > last_center_stamp:
                    centered_frames += 1
                    last_center_stamp = stamp
                if centered_frames < self.args.route_rear_center_frames:
                    continue
                log('后摄对准', '连续新帧居中，切换定位距离后退；接近期间不再要求YOLO')
                return self.aligned_intake_approach()
            centered_frames = 0
            last_center_stamp = None
            tw = self._Twist()
            # Rear image right corresponds to body +y.
            # 5 mm/s often cannot overcome chassis friction near the centre.
            speed = min(self.args.route_rear_align_speed, max(.015, abs(error)*.4))
            tw.linear.y = math.copysign(speed, error)
            self.publish_command(tw)
        self.stop()
        log('后摄停车', '等待篮球或横移对准超时', C_WARN)
        return False

    def rear_distance_scoop(self):
        """After alignment: bounded reverse displacement, independent of YOLO."""
        self.stop()
        x0,y0,heading = self.pose
        log('靠近篮球', f'后摄已对正，向中线方向后退 {self.args.route_scoop_distance:.2f}m，'
                        f'速度 {self.args.route_scoop_speed:.2f}m/s；不再读取YOLO')
        deadline = time.monotonic() + self.args.route_scoop_timeout
        while time.monotonic() < deadline:
            self.spin(.03)
            x,y,yaw = self.pose
            dx,dy = x-x0,y-y0
            travelled = -dx*math.cos(heading)-dy*math.sin(heading)
            remaining = self.args.route_scoop_distance-travelled
            err = wrap_pi(heading-yaw)
            if remaining <= self.args.step_tol:
                self.stop()
                log('铲球停车', f'后退距离完成 {travelled:.3f}m；是否入仓需实物确认')
                return True
            if abs(err) > self.args.route_heading_tol:
                self.stop()
                log('铲球停车', '后退时车头偏离对准方向', C_WARN)
                return False
            tw = self._Twist()
            tw.linear.x = -min(self.args.route_scoop_speed,self.args.kp*remaining)
            wz = self.args.kp_yaw*err-self.args.kd_yaw*self.yaw_rate
            tw.angular.z = max(-self.args.max_ang,min(self.args.max_ang,wz)) if math.isfinite(wz) else 0.
            self.publish_command(tw)
            trace('rear_distance_scoop', travelled=travelled, remaining=remaining,
                  vx=tw.linear.x, pose_stamp=self.pose_stamp)
        self.stop()
        log('铲球停车', '距离控制后退超时', C_WARN)
        return False

    def fixed_left_scan(self, heading, axis):
        """车体左移，篮球框中心进入中央竖带即停车。"""
        origin = self.pose[:2]
        deadline = time.monotonic() + self.args.route_scan_timeout
        while time.monotonic() < deadline:
            self.spin(0.05)
            det = self.centered_basketball()
            if det is not None:
                self.stop()
                trace('center_trigger', cx=(det.x_min+det.x_max)/2,
                      confidence=float(det.confidence), stamp=stamp_seconds(det.stamp))
                log("路线停车", "篮球进入画面中心，已发零速，立即转身180°")
                return (float(det.bearing_rad), float(det.distance_m), 1)
            px, py, yaw = self.pose
            travelled = (px-origin[0])*axis[0] + (py-origin[1])*axis[1]
            err = wrap_pi(heading-yaw)
            if travelled >= self.args.route_scan_distance:
                break
            if abs(err) > self.args.route_heading_tol:
                self.stop()
                log("路线中止", "平移时车头偏离固定朝向，保持停车", C_WARN)
                return None
            # 世界方向固定；车体系转换，保留小角度朝向纠偏。
            speed = min(self.args.route_scan_speed,
                        max(0.0, self.args.kp * (self.args.route_scan_distance-travelled)))
            vx, vy = speed*axis[0], speed*axis[1]
            tw = self._Twist()
            tw.linear.x = math.cos(yaw)*vx + math.sin(yaw)*vy
            tw.linear.y = -math.sin(yaw)*vx + math.cos(yaw)*vy
            wz = self.args.kp_yaw*err - self.args.kd_yaw*self.yaw_rate
            tw.angular.z = max(-self.args.max_ang, min(self.args.max_ang, wz)) if math.isfinite(wz) else 0.0
            self.publish_command(tw)
        self.stop()
        log("路线中止", "左移搜索到达距离/时间上限，未找到篮球", C_WARN)
        return None

    def fixed_seek_and_scoop(self):
        """固定步骤：右90° → 到中线附近 → 左移找篮球 → 停 → 转180° → 后退。"""
        if self.args.ball_label != "ball_basketball":
            raise ValueError("固定铲球路线只搜索篮球")
        start = self.pose
        heading, near_line, axis, endpoint = fixed_route_plan(
            start, self.args.route_forward, self.args.route_scan_distance)
        trace("fixed_route", start=list(start), right_yaw=heading,
              near_line=list(near_line), scan_end=list(endpoint))
        log("固定路线", f"右转90° → 前进{self.args.route_forward:.2f}m → 左移找篮球 "
                        f"(最多{self.args.route_scan_distance:.2f}m) → 停 → 转180° → 后退")
        if not self.rotate_to(heading, timeout=30.0):
            return False
        saved_speed, saved_approach = self.args.speed, self.args.approach_speed
        try:
            self.args.speed = self.args.route_forward_speed
            self.args.approach_speed = min(saved_approach, self.args.route_forward_speed)
            if not self.goto(*near_line, timeout=self.args.route_forward_timeout,
                             tolerance=self.args.step_tol):
                return False
        finally:
            self.args.speed, self.args.approach_speed = saved_speed, saved_approach
        if abs(wrap_pi(heading-self.pose[2])) > self.args.yaw_tol:
            if not self.rotate_to(heading, timeout=15.0):
                return False
        found = self.fixed_left_scan(heading, axis)
        if found is None:
            return False
        bearing, distance, frames = found
        yaw_before = self.pose[2]
        if not self.rotate_to(wrap_pi(yaw_before+math.pi), timeout=45.0):
            return False
        return self.rear_visual_scoop()

    def _scan_for_ball(self, seconds: float, wz: float, tag: str):
        """原地按给定角速度转，边转边看摄像头。找到返回检测。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.spin(0.05)
            det = self.best_ball()
            if det is not None:
                confirmed = self.measure_ball_stable(timeout=min(6.0, max(0.0, t0 + seconds - time.monotonic())))
                if confirmed is not None:
                    det = self.best_ball()
                    if det is not None:
                        log("找到球", f"[{tag}] 停车后独立帧确认 {det.label}", C_OK)
                        return det
            # 短转一段就停，让检测延迟不会累积成大角度超调。
            until = min(time.monotonic() + self.args.scan_step_seconds, t0 + seconds)
            while time.monotonic() < until:
                tw = self._Twist()
                tw.angular.z = wz
                self.publish_command(tw)
                self.spin(0.05)
                if self.best_ball() is not None:
                    break
            self.stop()
            self.spin(self.args.scan_pause_seconds)
        self.stop()
        return None

    def move_into_field(self, dist: float, timeout: float = 30.0) -> bool:
        """朝【场地中心】方向走 dist 米（三级都找不到时，往场内挪一点换个视角）。"""
        if dist <= 0.0:
            return True
        if self.pose is None:
            return False
        cx, cy = self.field_l / 2.0, self.field_w / 2.0
        px, py, _ = self.pose
        vx, vy = cx - px, cy - py
        n = math.hypot(vx, vy)
        if n < 1e-6:
            return False
        tx, ty = px + vx / n * dist, py + vy / n * dist
        m = 0.35                                    # 别跑出场
        tx = min(max(tx, m), self.field_l - m)
        ty = min(max(ty, m), self.field_w - m)
        log("进场", f"朝场地中心挪 {dist:.2f}m → ({tx:.2f},{ty:.2f})")
        return self.goto(tx, ty, timeout=timeout)

    def find_ball(self, timeout: float):
        """三级找球：① 前摄转圈 → ② 雷达 360° 粗定位 → ③ 进场 + 顺时针旋转。

        为什么不让摄像头一直盲转：
          前摄视野只有约 ±40°，必须转着找；雷达约 360°、6m 范围，
          【不用转】就能给出球的大致方位。所以合理顺序是
          "雷达先给大致位置 → 摄像头转过去精确确认"，比盲转快得多。
        雷达只做粗定位（地面上一个小点簇，容易有杂点），
        最终对准仍然靠摄像头 —— 两者各司其职。
        """
        # ── ① 前摄原地转圈找 ──
        log("找球①", f"前摄原地搜索 {self.args.ball_label}（{timeout:.0f}s）")
        det = self._scan_for_ball(timeout, self.args.search_wz, "前摄")
        if det is not None:
            return det

        # ── ② 雷达兜底：不用转圈就知道球的大致方位 ──
        if not self.args.no_lidar_fallback:
            log("找球②", "前摄没看到 → 用雷达找大致位置（360°，不用转圈）")
            for fx, fy, bdeg, dist in self.lidar_find_ball()[:3]:
                if self.pose is None:
                    break
                # 车体系方位右正左负 ⇒ 世界角 = yaw − bearing
                want = wrap_pi(self.pose[2] - math.radians(bdeg))
                log("转向候选", f"车体系方位 {bdeg:+.1f}° → 世界角 "
                                f"{math.degrees(want):+.1f}°（场地 {fx:.2f},{fy:.2f}）")
                if not self.rotate_to(want, timeout=20.0):
                    log("转向候选", "转向失败，试下一个", C_WARN)
                    continue
                # 转过去后用摄像头确认（雷达是粗定位，必须摄像头复核）
                det = self._scan_for_ball(3.0, 0.0, f"雷达候选{dist:.1f}m")
                if det is not None:
                    return det
                log("雷达候选", "转过去后摄像头没确认到球（可能是地面杂点）", C_WARN)
        else:
            log("找球②", "雷达兜底已关闭", C_DIM)

        # ── ③ 都没有：先向场内挪，再顺时针旋转找 ──
        log("找球③", f"雷达也没找到 → 向场内移动 {self.args.into_field:.2f}m，"
                     f"再顺时针旋转找球 {self.args.cw_scan_sec:.0f}s")
        self.move_into_field(self.args.into_field)
        self.spin(0.3)
        det = self._scan_for_ball(self.args.cw_scan_sec, self.args.cw_wz, "顺时针")
        if det is not None:
            return det

        self.stop()
        log("找球失败", "三级都试过：前摄转圈 / 雷达 360° / 进场顺时针 —— 都没找到球", C_BAD)
        return None

    def close(self) -> None:
        try:
            self.rclpy.shutdown()
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass


def takeover_mission() -> bool:
    """SIGTERM 掉 mission 节点（它现在会优雅退出并发零速）。"""
    r = subprocess.run(["pgrep", "-f", "[m]ission_node"], capture_output=True, text=True)
    pids = [p for p in r.stdout.split() if p.isdigit()]
    if not pids:
        print(f"      {C_DIM}mission 没在跑，不用接管{C_OFF}")
        return True
    for p in pids:
        try:
            os.kill(int(p), signal.SIGTERM)
        except Exception as e:  # noqa: BLE001
            print(f"      {C_BAD}kill {p} 失败: {e}{C_OFF}")
            return False
    time.sleep(2.0)
    left = subprocess.run(["pgrep", "-f", "[m]ission_node"], capture_output=True, text=True)
    ok = not left.stdout.strip()
    print(f"      {C_OK if ok else C_BAD}"
          f"{'已接管 /cmd_vel（mission 已退出）' if ok else 'mission 没退干净'}{C_OFF}")
    return ok


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="整车测试：绕场 → 寻球 → 对准停车；转身需 --turn-back")
    ap.add_argument("--route", choices=["fixed", "legacy"], default="fixed", help="默认固定出发寻球铲球路线")
    ap.add_argument("--dry", action="store_true", help="只检查前置条件，不动车")
    ap.add_argument("--yes", action="store_true", help="跳过确认")
    ap.add_argument("--takeover", action="store_true",
                    help="自动 SIGTERM mission 节点（否则要求你先手动停它）")
    ap.add_argument("--skip-perimeter", action="store_true", help="跳过绕场，直接找球")
    ap.add_argument("--ball-label", default="ball_basketball", help="目标球标签")
    ap.add_argument("--speed", type=float, default=0.25, help="平移速度上限 m/s")
    ap.add_argument("--pose-timeout", type=float, default=0.5,
                    help="定位及雷达源时间戳最大年龄 s，过期即停车")
    ap.add_argument('--pose-jump-margin',type=float,default=.25,help='定位跳变基础容差 m')
    ap.add_argument('--pose-max-speed',type=float,default=1.0,help='定位连续性允许速度 m/s')
    ap.add_argument("--kp", type=float, default=1.5)
    ap.add_argument("--kp-yaw", type=float, default=2.0)
    ap.add_argument("--kd-yaw", type=float, default=0.25, help="实测角速度阻尼系数")
    ap.add_argument("--max-ang", type=float, default=0.3, help="角速度上限 rad/s")
    ap.add_argument("--yaw-tol", type=float, default=0.015, help="朝向容差 rad")
    ap.add_argument("--tol", type=float, default=0.10, help="到点容差 m")
    ap.add_argument("--approach-radius", type=float, default=0.45)
    ap.add_argument("--approach-speed", type=float, default=0.12)
    ap.add_argument("--inset", type=float, default=0.70, help="绕场时离边界距离 m")
    ap.add_argument("--search-timeout", type=float, default=90.0, help="分段搜索总时长 s")
    ap.add_argument("--search-wz", type=float, default=0.12, help="找球自转角速度 rad/s")
    ap.add_argument("--cam-yaw-offset", type=float, default=0.0,
                    help="相机光轴相对车头的偏置 rad")
    # ── 现场规格：对准 → 停在球前 70cm → 转 180° → 后退 20cm ──
    # ── 三级找球 ──
    ap.add_argument("--no-lidar-fallback", action="store_true",
                    help="关掉雷达兜底找球（默认开）")
    ap.add_argument("--lidar-cloud", default="/cloud_registered",
                    help="雷达点云话题（车体系变换用）")
    ap.add_argument("--lidar-odom", default="/Odometry",
                    help="camera_init 系位姿（把点云转到车体系用）")
    ap.add_argument("--lidar-seconds", type=float, default=4.0,
                    help="雷达找球累计时长")
    ap.add_argument("--into-field", type=float, default=0.0,
                    help="三级都找不到时，先朝场内走多少米（默认关闭，设正数启用）")
    ap.add_argument("--cw-scan-sec", type=float, default=40.0,
                    help="进场后顺时针旋转找球时长")
    ap.add_argument("--cw-wz", type=float, default=-0.12,
                    help="顺时针旋转角速度 rad/s（【负】= 顺时针）")
    ap.add_argument("--rear-offset", type=float, default=0.25,
                    help="后摄到旋转中心的距离 m（装完量一下；后摄在车后填正数）")
    ap.add_argument("--rear-topic", default="/perception/detections_rear",
                    help="后摄检测话题")
    ap.add_argument("--align-dist", type=float, default=0.70,
                    help="对准到位时【车旋转中心】到球的距离 m（现场要求 70cm）")
    ap.add_argument("--align-tol-deg", type=float, default=1.5,
                    help="摄像头对准球的方位角容差（度）。越小越正对，但更难收敛")
    ap.add_argument("--align-dist-tol", type=float, default=0.05,
                    help="距离容差 m")
    ap.add_argument("--servo-steps", type=int, default=14,
                    help="视觉伺服最多迭代几步")
    ap.add_argument("--servo-move", type=float, default=0.12,
                    help="每步最大平移 m（小步逼近，避免冲过）")
    ap.add_argument("--servo-turn-deg", type=float, default=12.0,
                    help="每步最大转角（度）")
    ap.add_argument("--stable-frames", type=int, default=5,
                    help="取几帧做中位数（抗抖动/漏检）")
    ap.add_argument("--turn-deg", type=float, default=180.0,
                    help="对准后原地转多少度（现场要求 180）")
    ap.add_argument("--backup", type=float, default=0.20,
                    help="转身后退多少 m（现场要求 20cm；朝球方向退）")
    ap.add_argument("--stop-distance", type=float, default=0.60,
                    help="走到离球多远停下 m")
    ap.add_argument("--max-approaches", type=int, default=3, help="接近球最多试几次")
    ap.add_argument("--turn-back", action="store_true", help="对准成功后执行转身及后摄复核/后退；默认停在球前")
    ap.add_argument("--detection-timeout", type=float, default=0.5)
    ap.add_argument("--ball-confidence", type=float, default=0.5)
    ap.add_argument("--pose-match-gap", type=float, default=0.15)
    ap.add_argument("--track-bearing-deg", type=float, default=6.0)
    ap.add_argument("--track-distance", type=float, default=0.25)
    ap.add_argument("--scan-step-seconds", type=float, default=1.0)
    ap.add_argument("--scan-pause-seconds", type=float, default=0.5)
    ap.add_argument("--settle-seconds", type=float, default=0.3)
    ap.add_argument("--settle-yaw-rate", type=float, default=0.08)
    ap.add_argument("--step-tol", type=float, default=0.02)
    ap.add_argument("--log-file", default="", help="JSONL 日志路径；默认 log/seek_ball_时间.jsonl")
    # 默认 A 出发区 y=8.0；前进3.2m到y=4.8，位于黄线y=3.75北侧1.05m。
    ap.add_argument("--route-forward", type=float, default=3.2, help="右转后到中线附近的前进距离 m")
    ap.add_argument("--route-forward-speed", type=float, default=0.25)
    ap.add_argument("--route-forward-timeout", type=float, default=60.0)
    ap.add_argument("--route-scan-distance", type=float, default=3.0, help="车体左移搜索距离上限 m")
    ap.add_argument("--route-scan-speed", type=float, default=0.25)
    ap.add_argument("--route-scan-timeout", type=float, default=60.0)
    ap.add_argument("--route-heading-tol", type=float, default=0.10, help="平移时允许的朝向偏差 rad")
    ap.add_argument("--route-ball-tol-deg", type=float, default=6.0, help="停车后球偏离铲口轴线最大角度")
    ap.add_argument("--route-ball-min-distance", type=float, default=0.45)
    ap.add_argument("--route-ball-max-distance", type=float, default=1.0)
    ap.add_argument("--route-scoop-distance", type=float, default=0.40, help="后摄居中后独立后退距离 m，不含横移纠偏")
    ap.add_argument("--route-scoop-speed", type=float, default=0.08)
    ap.add_argument("--route-scoop-timeout", type=float, default=15.0)
    ap.add_argument('--route-intake', action=argparse.BooleanOptionalAction, default=True,
                    help='对正后启动文本吸球机构再靠近；--no-route-intake 仅测试靠近')
    ap.add_argument("--route-center-tolerance", type=float, default=0.04,
                    help="篮球框中心距画面中心允许偏差（画面宽度比例）")
    ap.add_argument('--route-rear-timeout', type=float, default=15.0)
    ap.add_argument('--route-rear-align-distance', type=float, default=.40, help='横移纠偏总位移上限 m')
    ap.add_argument('--route-rear-align-speed', type=float, default=.04)
    ap.add_argument('--route-rear-stop-height', type=float, default=.45,
                    help='兼容旧参数；距离铲球模式不再使用球框高度停车')
    ap.add_argument('--route-rear-center-tolerance', type=float, default=.025,
                    help='后摄篮球中心容差，画面宽度比例')
    ap.add_argument('--route-rear-center-frames', type=int, default=3,
                    help='停车居中确认需要的连续新检测帧数')
    ap.add_argument('--obstacle-guard', action=argparse.BooleanOptionalAction, default=False,
                    help='前摄/雷达近障碍停车，默认关闭；--obstacle-guard开启')
    ap.add_argument('--obstacle-stop-gap', type=float, default=.25,
                    help='车体包络外停车余量 m')
    ap.add_argument('--obstacle-half-length', type=float, default=.50)
    ap.add_argument('--obstacle-half-width', type=float, default=.50)
    ap.add_argument('--obstacle-side-margin', type=float, default=.15)
    ap.add_argument('--obstacle-timeout', type=float, default=.50)
    ap.add_argument('--obstacle-confidence', type=float, default=.50)
    ap.add_argument('--obstacle-camera-x', type=float, default=.33,
                    help='前摄相对车中心向前偏移 m，需与实物一致')
    ap.add_argument('--obstacle-depth-scale', type=float, default=.80,
                    help='单目距离保守缩放系数，不是精确测距')
    ap.add_argument('--obstacle-box-size', type=float, default=.22,
                    help='前方球框宽或高达到画面比例时停车')
    ap.add_argument('--obstacle-deceleration', type=float, default=.30,
                    help='规划减速度 m/s²，需实车验证停车距离')
    ap.add_argument('--obstacle-lidar-cells', type=int, default=3,
                    help='危险区域内不同6cm体素数量阈值')
    args = ap.parse_args(argv)
    if any(not math.isfinite(v) or v <= 0 for v in (
            args.obstacle_stop_gap,args.obstacle_half_length,args.obstacle_half_width,
            args.obstacle_side_margin,args.obstacle_timeout,args.obstacle_deceleration)):
        ap.error('避障包络、余量、超时和减速度须为正有限数')
    if not all(math.isfinite(v) and 0<v<=1 for v in (
            args.obstacle_confidence,args.obstacle_depth_scale,args.obstacle_box_size)):
        ap.error('避障置信度、距离系数、框阈值须在(0,1]')
    if not math.isfinite(args.obstacle_camera_x) or args.obstacle_lidar_cells<1:
        ap.error('相机偏移须有限，雷达体素阈值至少1')
    if not 0 < args.route_rear_center_tolerance < .5 or args.route_rear_center_frames < 1:
        ap.error('后摄中心容差须在0和0.5之间，确认帧数须至少1')
    if not 0 < args.route_rear_stop_height <= 1:
        ap.error('后摄停止框高度须在0和1之间')
    if not all(math.isfinite(v) and v > 0 for v in (args.route_rear_timeout, args.route_rear_align_speed, args.route_rear_align_distance)):
        ap.error('后摄超时和对准速度须为正且有限')
    if not math.isfinite(args.route_center_tolerance) or not 0 < args.route_center_tolerance < .5:
        ap.error("中心容差必须在0和0.5之间")
    positive = ("pose_jump_margin", "pose_max_speed", "speed", "pose_timeout", "max_ang", "yaw_tol", "detection_timeout",
                "pose_match_gap", "track_bearing_deg", "track_distance", "scan_step_seconds",
                "scan_pause_seconds", "settle_seconds", "settle_yaw_rate", "step_tol",
                "servo_move", "servo_turn_deg", "align_dist", "align_dist_tol", "align_tol_deg",
                "kp", "kp_yaw", "servo_steps", "cw_scan_sec", "lidar_seconds", "search_timeout")
    positive += ("route_forward", "route_forward_speed", "route_forward_timeout",
                 "route_scan_distance", "route_scan_speed", "route_scan_timeout",
                 "route_heading_tol", "route_ball_tol_deg", "route_ball_min_distance",
                 "route_ball_max_distance", "route_scoop_distance", "route_scoop_speed",
                 "route_scoop_timeout")
    if args.route_ball_min_distance >= args.route_ball_max_distance:
        ap.error("铲球距离窗口的下限须小于上限")
    if args.route_scoop_distance <= args.step_tol:
        ap.error("铲球后退距离须大于到点容差")
    if any(not math.isfinite(getattr(args, k)) or getattr(args, k) <= 0 for k in positive):
        ap.error("控制参数必须为正且有限")
    if not math.isfinite(args.kd_yaw) or args.kd_yaw < 0:
        ap.error("kd-yaw 必须非负且有限")
    if not 0 < args.ball_confidence <= 1 or args.stable_frames < 3:
        ap.error("ball-confidence 须在 (0,1]，stable-frames 至少为 3")
    if args.servo_move > 0.2 or args.step_tol >= args.servo_move:
        ap.error("servo-move 最大 0.2m，step-tol 必须小于移动步长")
    if any(not math.isfinite(v) for v in (args.search_wz, args.cw_wz, args.cam_yaw_offset,
                                        args.into_field, args.turn_deg, args.backup)):
        ap.error("转速、相机偏置、进场距离、转角及后退距离必须有限")
    if abs(args.search_wz) > args.max_ang or abs(args.cw_wz) > args.max_ang:
        ap.error("搜索转速不得超过 max-ang")


    os.chdir(WS)
    print("🤖 固定路线：右转90° → 到中线附近 → 左移找篮球 → 停车 → 转180° → 后退铲球" if args.route == "fixed" else "🤖 绕场/视觉伺服测试")

    # ── [1/5] 节点 ──────────────────────────────────────────────────────
    print("\n[1/5] 检查节点")
    need = {"底盘": "[c]hassis_node", "定位": "[l]ocalization_node",
            "感知": "[p]erception_node", "FAST-LIO": "[f]astlio_mapping"}
    miss = []
    for n, pat in need.items():
        ok = subprocess.run(["pgrep", "-f", pat], capture_output=True).returncode == 0
        print(f"      {C_OK+'✓'+C_OFF if ok else C_BAD+'✗'+C_OFF} {n}")
        if not ok:
            miss.append(n)
    if miss:
        print(f"\n  {C_BAD}缺: {', '.join(miss)}{C_OFF} —— 先跑  rbws && bash tools/boot.sh")
        return 1

    # ── [2/5] 接管 /cmd_vel ─────────────────────────────────────────────
    print("\n[2/5] 接管 /cmd_vel（mission 在 IDLE 时也会发零速，会抢话题）")
    mission_alive = subprocess.run(["pgrep", "-f", "[m]ission_node"],
                                   capture_output=True, text=True).stdout.strip() != ""
    if mission_alive:
        if args.takeover and not args.dry:
            if not takeover_mission():
                return 1
        else:
            print(f"      {C_WARN}mission 还在跑{C_OFF} —— 加 --takeover 自动停，"
                  f"或手动：pkill -f '[m]ission_node'")
            if not args.dry:
                return 1
    else:
        print(f"      {C_OK}✓{C_OFF} mission 没在跑，可安全接管")

    global _TRACE
    log_path = Path(args.log_file) if args.log_file else Path(WS) / "log" / ("seek_ball_" + time.strftime("%Y%m%d_%H%M%S") + ".jsonl")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    _TRACE = log_path.open("w", encoding="utf-8")
    trace("settings", args=vars(args))
    log("日志", str(log_path))
    run = Runner(args)
    try:
        # ── [3/5] 位姿 + 场地 ───────────────────────────────────────────
        print("\n[3/5] 读位姿与场地")
        if not run.wait_pose(8.0):
            print(f"      {C_BAD}✗ 读不到 /localization/pose{C_OFF}")
            return 1
        sx, sy, syaw = run.pose
        import yaml
        fld = (yaml.safe_load(open("src/rb_mission/config/mission.yaml",
                                   encoding="utf-8")) or {}).get("field", {}) or {}
        L = float(fld.get("length_m", 14.0))
        W = float(fld.get("width_m", 7.5))
        # 交给 Runner：三级找球的"向场内移动"要用场地中心
        run.field_l, run.field_w = L, W
        log("位姿", f"出发点 ({sx:+.2f}, {sy:+.2f}, {math.degrees(syaw):+.1f}°)")
        log("场地", f"{L:.1f} × {W:.1f} m，绕场 inset={args.inset:.2f}m")

        run.wait_motion_ready()

        if args.dry:
            print(f"\n  {C_OK}--dry：前置条件都过了，没动车{C_OFF}")
            return 0

        run.motion_guard = True

        if not args.yes:
            print(f"\n  {C_WARN}接下来车会真的动。急停准备好（另开终端 bash tools/panic.sh）{C_OFF}")
            if input("  输入 y 开始，其它取消 > ").strip().lower() != "y":
                print("  已取消。")
                return 0

        if args.route == "fixed":
            return 0 if run.fixed_seek_and_scoop() else 1

        # ── [4/5] 绕场一圈 ──────────────────────────────────────────────
        if args.skip_perimeter:
            log("跳过", "不绕场，直接找球")
        else:
            print("\n[4/5] 沿场地外围走一圈（最后回到出发点）")
            k = args.inset
            wps = [(k, k), (L - k, k), (L - k, W - k), (k, W - k), (sx, sy)]
            for i, (tx, ty) in enumerate(wps, 1):
                log("绕场", f"[{i}/{len(wps)}] 目标 ({tx:.2f},{ty:.2f})")
                if not run.goto(tx, ty, timeout=90.0):
                    log("中止", "绕场未完成，停车退出", C_BAD)
                    return 1
            log("绕场完成", "已回到出发点", C_OK)

        # ── [5/5] 找球 → 精密对准（70cm）→ 转 180° → 后退 20cm ─────────────
        print("\n[5/5] 找球 → 摄像头正对球 → 旋转中心停在球前 "
              f"{args.align_dist:.2f}m" + (f" → 转 {args.turn_deg:.0f}° → 后退 {args.backup:.2f}m" if args.turn_back else " → 停车"))
        det = run.find_ball(timeout=args.search_timeout)
        if det is None:
            return 1

        # ① 闭环视觉伺服：摄像头正对球，且旋转中心距球 align_dist
        ok_align, last = run.servo_to_ball(
            target_d=args.align_dist,
            tol_b_deg=args.align_tol_deg,
            tol_d=args.align_dist_tol,
            steps=args.servo_steps,
            max_move=args.servo_move,
            max_turn_deg=args.servo_turn_deg)
        if not ok_align:
            log("对准失败", "视觉伺服没收敛 —— 不盲目转身（宁可不做也别做错）", C_WARN)
            return 1

        if not args.turn_back:
            run.stop()
            log("完成", f"已停车对准球，目标距离 {args.align_dist:.2f}m；转身需 --turn-back", C_OK)
            return 0
        # 转身前再次以停车后的独立帧复核，不能用刚才的旧测量。
        fresh = run.measure_ball_stable(n=3)
        if fresh is None or abs(math.degrees(fresh[0])) > args.align_tol_deg or abs(fresh[1]-args.align_dist) > args.align_dist_tol:
            log("中止", "转身前目标复核失败，保持停车", C_WARN)
            return 1

        # ② 原地转 turn_deg（默认 180°），用里程计 yaw 精确转，不靠视野反推
        if run.pose is None:
            log("失败", "位姿丢了，无法转身", C_BAD)
            return 1
        yaw0 = run.pose[2]
        target_yaw = wrap_pi(yaw0 + math.radians(args.turn_deg))
        log("转身", f"车头 {math.degrees(yaw0):+.1f}° → {math.degrees(target_yaw):+.1f}°"
                    f"（转 {args.turn_deg:.0f}°）")
        ok_turn = run.rotate_to(target_yaw, timeout=45.0)
        if not ok_turn:
            log("转身未到位", "角度没到，停车退出", C_WARN)
            return 1

        # ③ 转身后【用后摄】做精准定位 —— 这正是后摄存在的意义
        #    前摄此时背对球看不到，后摄正对着球（装得低、水平，近距离更清楚）
        run.spin(0.4)
        rear = run.measure_rear_stable(timeout=6.0)
        front_after = run.measure_ball_stable(timeout=3.0, n=3)
        if rear is not None:
            rb, rd, rn = rear
            wa = math.degrees(rear_world_angle(run.pose[2] if run.pose else 0.0, rb))
            log("后摄定位", f"方位 {math.degrees(rb):+.2f}°（世界 {wa:+.1f}°）、"
                            f"深度 {rd:.3f}m  [{rn} 帧中位数]", C_OK)
            bw = ball_world_from_rear(
                run.pose[0], run.pose[1], run.pose[2],
                args.rear_offset, rb, rd) if run.pose else None
            if bw:
                log("球位置", f"据后摄推算球在场地 ({bw[0]:.2f}, {bw[1]:.2f})")
                # 旋转中心到球的距离 = |球位 - 车心|
                cd = math.hypot(bw[0] - run.pose[0], bw[1] - run.pose[1])
                log("距离核对", f"旋转中心距球 {cd:.3f}m（对准时目标 {args.align_dist:.2f}m）")
        else:
            log("后摄无检测", "后摄没看到球 —— 检查话题 /perception/detections_rear、"
                              "内参是否标定、球是否在视野里", C_WARN)
        if front_after is not None:
            log("注意", f"转身前摄还能看到球（方位 {math.degrees(front_after[0]):+.1f}°）"
                        f" —— 球可能在侧面，核对一下", C_WARN)

        # ④ 朝球方向退 backup（现场要 20cm）
        #    ⚠️ 转完 180° 后车头【背对】球，"朝球方向"= 车头反方向
        before = run.pose
        ok_back = run.backup_along(-abs(args.backup), timeout=30.0) \
            if args.turn_deg >= 90 else run.backup_along(abs(args.backup), timeout=30.0)

        # ⑤ 退完再用后摄确认（闭环的最后一眼）
        run.spin(0.3)
        after_rear = run.measure_rear_stable(timeout=5.0, n=3)
        final_d = None
        if after_rear is not None:
            ab, ad, an = after_rear
            log("后摄复核", f"退完后：方位 {math.degrees(ab):+.2f}°、深度 {ad:.3f}m  [{an} 帧]",
                C_OK)
            if run.pose:
                bw2 = ball_world_from_rear(run.pose[0], run.pose[1], run.pose[2],
                                           args.rear_offset, ab, ad)
                if bw2:
                    final_d = math.hypot(bw2[0] - run.pose[0], bw2[1] - run.pose[1])
        if before and run.pose:
            moved = math.hypot(run.pose[0] - before[0], run.pose[1] - before[1])
            log("位移核对", f"后退实际走了 {moved:.3f}m（指令 {args.backup:.2f}m）")

        print()
        if ok_turn and ok_back:
            log("完成", f"找球 ✓  对准(方位容差±{args.align_tol_deg:.1f}°) ✓  "
                        f"旋转中心距球 {args.align_dist:.2f}m ✓  "
                        f"转 {args.turn_deg:.0f}° ✓  后退 {args.backup:.2f}m ✓", C_OK)
            if final_d:
                log("终点", f"后摄确认：旋转中心距球约 {final_d:.2f}m，背面朝球", C_OK)
        elif ok_turn:
            log("部分完成", "对准 ✓  转身 ✓  但后退没完成", C_WARN)
        else:
            log("部分完成", "对准 ✓  转身没到位", C_WARN)
        log("注意", "车已停。恢复任务：重启 boot.sh，或直接 set_mission")
        return 0 if ok_turn and ok_back else 1

    except KeyboardInterrupt:
        print(f"\n  {C_WARN}Ctrl-C —— 发零速停车{C_OFF}")
        run.stop()
        return 130
    except Exception as e:  # noqa: BLE001
        print(f"\n  {C_BAD}异常：{type(e).__name__}: {e}{C_OFF}")
        run.stop()
        return 1
    finally:
        if not args.dry:
            run.stop()
        run.close()
        if _TRACE is not None:
            _TRACE.close()
            _TRACE = None


if __name__ == "__main__":
    sys.exit(main())
