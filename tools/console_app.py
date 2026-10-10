#!/usr/bin/env python3
"""RoboCup 篮球机器人 · 综合控制台（后端）

一个页面搞定：起/停整套软件、点按钮起任务、看实时日志、看相机、看雷达俯视图、
看车辆状态（持球/区域/子系统 ok）、看硬件链路（CAN/雷达网卡/机构串口）。

为什么用 Flask 而不是 Qt
------------------------
本机没有 Qt（PyQt/PySide 都没装），但有 flask。而且网页版有两个实际好处：
  ① 相机画面和雷达俯视图直接用 MJPEG 推流，不用把图像搬进 GUI 控件；
  ② 手机/另一台电脑连上就能当遥控台 —— 比赛现场操作员不用挤在 NUC 前。

用法
----
    cd ~/robocup_basketball_ws
    rbws
    python3 tools/console_app.py                    # 默认 0.0.0.0:8090
    python3 tools/console_app.py --port 9000
    python3 tools/console_app.py --no-stack-control # 禁用"起/停整套"按钮

然后用浏览器打开  http://<本机IP>:8090  或  http://127.0.0.1:8090

安全
----
* 页面上的「启动整套」= 跑 tools/boot.sh（会清理残留进程再 launch）。
* 「急停」= 跑 tools/panic.sh。
* **任何按钮都不能替代手上的电源开关。**
"""

from __future__ import annotations

import argparse
import io
import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

WS = Path("/home/user/robocup_basketball_ws")

# ─────────────────────────────────────────────────────────────────────────────
# 画图（雷达俯视图）
# ─────────────────────────────────────────────────────────────────────────────
LIDAR_W, LIDAR_H = 900, 560          # 画布尺寸
LIDAR_RANGE_M = 16.0                  # 视野半径（米）—— 场地 14×7.5，够用


def pc2_xyz(msg):
    """PointCloud2 -> (N,3) float32。只取 x/y/z，按 fields 里的 offset 解析。"""
    names = {f.name: f.offset for f in msg.fields}
    if not {"x", "y", "z"} <= set(names):
        return None
    n = msg.width * msg.height
    if n == 0:
        return None
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    step = msg.point_step
    if step <= 0 or buf.size < n * step:
        return None
    buf = buf[: n * step].reshape(n, step)
    out = np.empty((n, 3), dtype=np.float32)
    for i, k in enumerate(("x", "y", "z")):
        off = names[k]
        out[:, i] = buf[:, off : off + 4].copy().view(np.float32).ravel()
    m = np.isfinite(out).all(axis=1)
    return out[m]


# ⚠️ cv2.putText 只支持 ASCII（Hershey 字体），中文会全变成 "?????"。
#    所以文字统一交给 PIL 画：先把所有图形用 cv2 画完，再一次性转 PIL 写文字。
CJK_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
)
_font_cache: dict = {}


def _cjk_font_path():
    for f in CJK_FONT_CANDIDATES:
        if Path(f).exists():
            return f
    return None


def _font(size: int):
    fp = _cjk_font_path()
    if fp is None:
        return None
    if size not in _font_cache:
        from PIL import ImageFont
        try:
            _font_cache[size] = ImageFont.truetype(fp, size)
        except Exception:  # noqa: BLE001
            return None
    return _font_cache[size]


def draw_texts(img_bgr, items):
    """items: [(text, (x,y), (b,g,r), size)]。一次性转 PIL 画完再转回。"""
    import cv2
    font0 = _font(13)
    if font0 is None:                      # 没有中文字体 → 退回 ASCII
        for text, xy, col, size in items:
            cv2.putText(img_bgr, text, xy, cv2.FONT_HERSHEY_SIMPLEX,
                        size / 26.0, col, 1, cv2.LINE_AA)
        return img_bgr
    from PIL import Image, ImageDraw
    pil = Image.fromarray(img_bgr[:, :, ::-1])     # BGR → RGB
    d = ImageDraw.Draw(pil)
    for text, xy, col, size in items:
        f = _font(size) or font0
        d.text(xy, text, font=f, fill=(col[2], col[1], col[0]))   # BGR → RGB
    return np.array(pil)[:, :, ::-1].copy()


def render_lidar(cloud_xyz, pose, dets, field, launcher_ok, loc_ok) -> bytes:
    """把点云画成俯视图（鸟瞰），叠加机器人位姿、球检测、场地边界。"""
    import cv2

    img = np.zeros((LIDAR_H, LIDAR_W, 3), dtype=np.uint8)
    img[:] = (24, 22, 20)

    # 世界坐标 → 画布：以机器人为中心，朝上为 +x（车头方向固定朝上）
    cx, cy, cyaw = (pose if pose else (0.0, 0.0, 0.0))
    scale = min(LIDAR_W, LIDAR_H) / (2.0 * LIDAR_RANGE_M)
    c, s = math.cos(-cyaw - math.pi / 2), math.sin(-cyaw - math.pi / 2)

    def to_px(wx, wy):
        dx, dy = wx - cx, wy - cy
        # 先按 -yaw 转到车体系，再旋转 90° 让车头朝上
        bx = c * dx - s * dy
        by = s * dx + c * dy
        return int(LIDAR_W / 2 + bx * scale), int(LIDAR_H / 2 - by * scale)

    # 场地边界（浅灰矩形）
    L = float(field.get("length_m", 14.0))
    W = float(field.get("width_m", 7.5))
    pts = np.array([to_px(x, y) for x, y in ((0, 0), (L, 0), (L, W), (0, W))], np.int32)
    cv2.polylines(img, [pts], True, (70, 70, 70), 1, cv2.LINE_AA)

    # 得分点参考（传球区/投篮点/篮筐）
    refs = [("传球区", field.get("pass_zone_target")), ("投篮点", field.get("shoot_zone_target")),
            ("篮筐", field.get("hoop")), ("出发点", field.get("home"))]
    texts = []
    for name, p in refs:
        if isinstance(p, dict) and "x" in p:
            px = to_px(p["x"], p["y"])
            cv2.drawMarker(img, px, (90, 160, 90), cv2.MARKER_DIAMOND, 8, 1)
            texts.append((name, (px[0] + 7, px[1] - 16), (120, 180, 120), 13))

    # 点云（按高度上色）
    if cloud_xyz is not None and len(cloud_xyz):
        p = cloud_xyz
        bx = c * (p[:, 0] - cx) - s * (p[:, 1] - cy)
        by = s * (p[:, 0] - cx) + c * (p[:, 1] - cy)
        px = (LIDAR_W / 2 + bx * scale).astype(np.int32)
        py = (LIDAR_H / 2 - by * scale).astype(np.int32)
        # 只画视野内的（±1.5 倍范围，画布外丢弃）
        m = ((px >= 0) & (px < LIDAR_W) & (py >= 0) & (py < LIDAR_H)
             & (np.abs(bx) < LIDAR_RANGE_M * 1.2) & (np.abs(by) < LIDAR_RANGE_M * 1.2))
        z = p[m, 2]
        zn = np.clip((z + 0.3) / 2.0, 0, 1)
        cols = np.stack([(60 + 195 * zn), (200 - 120 * zn), (255 - 200 * zn)], axis=1)
        img[py[m], px[m]] = cols.astype(np.uint8)

    # 机器人（中心红箭头，朝上）
    cv2.arrowedLine(img, (LIDAR_W // 2, LIDAR_H // 2),
                    (LIDAR_W // 2, LIDAR_H // 2 - 26), (60, 60, 255), 3,
                    cv2.LINE_AA, tipLength=0.35)
    cv2.circle(img, (LIDAR_W // 2, LIDAR_H // 2), 14, (60, 60, 255), 1, cv2.LINE_AA)

    # 视野半径标尺
    r = int(LIDAR_RANGE_M * scale)
    cv2.circle(img, (LIDAR_W // 2, LIDAR_H // 2), r, (48, 48, 48), 1, cv2.LINE_AA)
    texts.append((f"{LIDAR_RANGE_M:.0f} m", (LIDAR_W // 2 + 6, LIDAR_H // 2 - r + 2),
                  (110, 110, 110), 13))

    # 球检测（相机方位角 → 场地方向）
    for d in dets or []:
        dist = getattr(d, "distance_m", float("nan"))
        if not (dist == dist and dist > 0):
            continue
        bearing = getattr(d, "bearing_rad", 0.0)
        # bearing 右正左负；场地系 y 朝左 → 世界方位 = yaw - bearing
        ang = cyaw - bearing
        wx, wy = cx + dist * math.cos(ang), cy + dist * math.sin(ang)
        px, py = to_px(wx, wy)
        label = getattr(d, "label", "?")
        col = (0, 200, 255) if "basket" in label else (255, 180, 60)
        cv2.circle(img, (px, py), 9, col, 2, cv2.LINE_AA)
        cv2.line(img, (LIDAR_W // 2, LIDAR_H // 2), (px, py), col, 1, cv2.LINE_AA)
        texts.append((f"{label} {dist:.2f}m", (px + 12, py - 8), col, 13))

    # 左上角文字状态
    lines = [
        (f"位姿 ({cx:+.2f}, {cy:+.2f})   yaw {math.degrees(cyaw):+.0f}°", (215, 215, 215), 15),
        (f"定位 {'正常' if loc_ok else '不可用'}", (120, 220, 120) if loc_ok else (120, 120, 240), 15),
        (f"机构串口 {'正常' if launcher_ok else '未连接'}", (120, 220, 120) if launcher_ok else (120, 120, 240), 15),
        (f"点云点数 {0 if cloud_xyz is None else len(cloud_xyz)}", (150, 150, 150), 15),
    ]
    for i, (txt, col, size) in enumerate(lines):
        texts.append((txt, (12, 8 + i * 20), col, size))

    img = draw_texts(img, texts)
    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 78])
    return jpg.tobytes() if ok else b""


# ─────────────────────────────────────────────────────────────────────────────
# ROS 桥
# ─────────────────────────────────────────────────────────────────────────────
class Bridge:
    """持有 ROS 节点，缓存最新状态供 HTTP 线程读取。"""

    def __init__(self, cfg: dict):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import CompressedImage, Image, PointCloud2
        from std_msgs.msg import Bool, String as StringMsg
        from rcl_interfaces.msg import Log
        from rb_msgs.msg import DetectionArray, MissionStatus, RobotState
        from rb_msgs.srv import GotoPose, Launch, SetMission

        self.rclpy = rclpy
        self.cfg = cfg
        self.lock = threading.Lock()
        self.logs: deque = deque(maxlen=400)
        self.launch_logs: deque = deque(maxlen=400)

        rclpy.init()
        self.node = Node("rb_console")
        self._Twist = Twist

        self.state = {
            "mission": "-", "phase": "-", "detail": "", "elapsed": 0.0, "score": -1,
            "pose": None, "loc_ok": False,
            "has_ball": False, "ball_count": 0, "ball_type": "none",
            "in_pass_zone": False, "in_shoot_out": False, "estop": False,
            "launcher_ok": False, "perception_ok": False,
            "launcher_status": "", "odom_delay": None, "odom_x": None,
            "dets": [], "cloud": None, "cam": None, "cam_stamp": 0.0,
            "cam_fps": 0.0, "cloud_fps": 0.0,
        }
        self._cam_times: deque = deque(maxlen=30)
        self._cloud_times: deque = deque(maxlen=30)
        self._last_odom_wall = 0.0

        t = cfg.get("topics", {}) or {}
        self._sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                      history=HistoryPolicy.KEEP_LAST)

        def sub1(cls, topic, cb, qos=1):
            self.node.create_subscription(cls, topic, cb, qos)

        sub1(MissionStatus, t.get("status", "/mission/status"), self._on_status, 10)
        sub1(RobotState, t.get("state", "/mission/state"), self._on_rstate, 10)
        sub1(PoseWithCovarianceStamped, t.get("pose", "/localization/pose"), self._on_pose, 10)
        sub1(Bool, t.get("ok", "/localization/ok"), self._on_ok, 10)
        sub1(Odometry, t.get("odom", "/Odometry"), self._on_odom, self._sensor_qos)
        sub1(Bool, "/rb_launcher/ok", self._on_lok, 10)
        sub1(StringMsg, "/rb_launcher/status", self._on_lstatus, 10)
        sub1(DetectionArray, t.get("dets", "/perception/detections"), self._on_dets, 10)
        sub1(CompressedImage, t.get("image", "/camera/image_raw/compressed"),
             self._on_image, self._sensor_qos)
        # 原图兜底：离线仿真的 publish_test_image 发的是未压缩 /camera/image_raw，
        # 真机 camera_node 发 compressed。两个都订，谁先到用谁。
        sub1(Image, t.get("image_raw", "/camera/image_raw"), self._on_raw_image, self._sensor_qos)
        sub1(PointCloud2, t.get("cloud", "/cloud_registered"), self._on_cloud, self._sensor_qos)
        sub1(Log, "/rosout", self._on_log, 50)

        self.cli_mission = self.node.create_client(SetMission, t.get("set_mission", "/rb_mission/set_mission"))
        self.cli_goto = self.node.create_client(GotoPose, t.get("goto_service", "/rb_mission/goto_pose"))
        self.cli_launch = self.node.create_client(Launch, t.get("launch_service", "/rb_launcher/launch"))
        self.pub_cmd = self.node.create_publisher(Twist, t.get("cmd_vel", "/cmd_vel"), 10)
        self.pub_goal = self.node.create_publisher(PoseStamped, t.get("goal_pose", "/goal_pose"), 5)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    # -- 回调 ---------------------------------------------------------------
    def _on_status(self, m):
        with self.lock:
            self.state.update(mission=m.mission, phase=m.phase, detail=m.detail,
                              elapsed=float(m.elapsed_s), score=int(m.score_estimate))

    def _on_rstate(self, m):
        with self.lock:
            self.state.update(has_ball=bool(m.has_ball), ball_count=int(m.ball_count),
                              ball_type=m.ball_type, in_pass_zone=bool(m.in_pass_zone),
                              in_shoot_out=bool(m.in_shoot_zone_outside),
                              estop=bool(m.estop), launcher_ok=bool(m.launcher_ok),
                              perception_ok=bool(m.perception_ok))

    def _on_pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if math.hypot(q.x, q.y, q.z, q.w) > 1e-6:
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        else:
            yaw = p.z
        with self.lock:
            self.state["pose"] = (float(p.x), float(p.y), float(yaw))

    def _on_ok(self, m):
        with self.lock:
            self.state["loc_ok"] = bool(m.data)

    def _on_odom(self, m):
        now = self.node.get_clock().now().nanoseconds * 1e-9
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.state["odom_delay"] = now - st
            self.state["odom_x"] = float(m.pose.pose.position.x)

    def _on_lok(self, m):
        with self.lock:
            self.state["launcher_ok"] = bool(m.data)

    def _on_lstatus(self, m):
        with self.lock:
            self.state["launcher_status"] = m.data

    def _on_dets(self, m):
        with self.lock:
            self.state["dets"] = list(m.detections)

    def _on_image(self, m):
        now = time.time()
        self._cam_times.append(now)
        with self.lock:
            self.state["cam"] = bytes(m.data)
            self.state["cam_stamp"] = now
            if len(self._cam_times) > 1:
                dt = self._cam_times[-1] - self._cam_times[0]
                self.state["cam_fps"] = (len(self._cam_times) - 1) / dt if dt > 0.05 else 0.0

    def _on_raw_image(self, m):
        """未压缩图 → JPEG（只在还没收到 compressed 时用，避免覆盖高质量源）。"""
        try:
            import cv2
            h, w, enc = m.height, m.width, m.encoding
            buf = np.frombuffer(m.data, dtype=np.uint8)
            nch = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(enc)
            if nch is None or buf.size < h * w * nch:
                return
            arr = buf[: h * w * nch].reshape(h, w, nch)
            if enc == "rgb8":
                arr = arr[:, :, ::-1]
            ok, jpg = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if not ok:
                return
            now = time.time()
            self._cam_times.append(now)
            with self.lock:
                if self.state["cam"] is None or now - self.state["cam_stamp"] > 1.0:
                    self.state["cam"] = jpg.tobytes()
                    self.state["cam_stamp"] = now
                if len(self._cam_times) > 1:
                    dt = self._cam_times[-1] - self._cam_times[0]
                    self.state["cam_fps"] = (len(self._cam_times) - 1) / dt if dt > 0.05 else 0.0
        except Exception:  # noqa: BLE001
            pass

    def _on_cloud(self, m):
        self._cloud_times.append(time.time())
        xyz = pc2_xyz(m)
        fps = 0.0
        if len(self._cloud_times) > 1:
            dt = self._cloud_times[-1] - self._cloud_times[0]
            fps = (len(self._cloud_times) - 1) / dt if dt > 0.05 else 0.0
        with self.lock:
            self.state["cloud"] = xyz
            self.state["cloud_fps"] = fps

    def _on_log(self, m):
        lvl = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}.get(m.level, "?")
        with self.lock:
            self.logs.append((time.time(), lvl, m.name, m.msg))

    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:  # noqa: BLE001
            pass

    # -- 动作 ---------------------------------------------------------------
    def _call(self, cli, req, timeout=3.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return False, "服务不可用"
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        r = fut.result()
        if r is None:
            return False, "无响应"
        return bool(getattr(r, "accepted", getattr(r, "success", True))), \
            str(getattr(r, "message", ""))

    def set_mission(self, name: str):
        from rb_msgs.srv import SetMission
        req = SetMission.Request()
        req.mission = name
        return self._call(self.cli_mission, req)

    def goto(self, x: float, y: float, yaw: float = 0.0, align_yaw: bool = False):
        from rb_msgs.srv import GotoPose
        req = GotoPose.Request()
        req.x, req.y, req.yaw, req.align_yaw = float(x), float(y), float(yaw), bool(align_yaw)
        return self._call(self.cli_goto, req)

    def launcher(self, action: int, speed: int = 0, angle: int = 0):
        from rb_msgs.srv import Launch
        req = Launch.Request()
        req.action, req.speed, req.angle = int(action), int(speed), int(angle)
        return self._call(self.cli_launch, req)

    def jog(self, vx: float, vy: float, wz: float, seconds: float = 0.4):
        """点动：先发速度，再自动归零（不依赖前端松手）。"""
        def run():
            t0 = time.time()
            while time.time() - t0 < seconds:
                t = self._Twist()
                t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
                self.pub_cmd.publish(t)
                time.sleep(0.05)
            self.pub_cmd.publish(self._Twist())
        threading.Thread(target=run, daemon=True).start()
        return True, f"点动 {seconds}s"

    def snapshot(self):
        """返回可 JSON 化的快照。⚠️ logs 必须是 dict（api_state 还要往里追加
        boot.sh 的日志再统一按时间排序，混元组会 TypeError）。"""
        with self.lock:
            s = dict(self.state)
            s["logs"] = [{"t": tt, "lvl": lv, "src": src, "msg": ms}
                         for (tt, lv, src, ms) in list(self.logs)[-120:]]
            # ⚠️ cam 是原始 JPEG 字节、cloud 是 ndarray，都**不能**进 JSON
            s["has_cam"] = s.get("cam") is not None
            s["cam"] = None
            s["cloud"] = None if s["cloud"] is None else len(s["cloud"])
            s["dets"] = [{"label": d.label, "conf": float(d.confidence),
                          "bearing_deg": math.degrees(d.bearing_rad),
                          "dist": (None if d.distance_m != d.distance_m else float(d.distance_m))}
                         for d in s["dets"][:12]]
            return s


# ─────────────────────────────────────────────────────────────────────────────
# 整套软件管理（起/停 boot.sh）
# ─────────────────────────────────────────────────────────────────────────────
class StackManager:
    def __init__(self, enabled: bool, log_deque: deque):
        self.enabled = enabled
        self.log = log_deque
        self.proc: subprocess.Popen | None = None
        self.lock = threading.Lock()

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def _pump(self, proc):
        for raw in iter(proc.stdout.readline, b""):
            try:
                self.log.append((time.time(), "LAUNCH", "boot.sh",
                                 raw.decode("utf-8", "replace").rstrip()))
            except Exception:  # noqa: BLE001
                pass
        self.log.append((time.time(), "LAUNCH", "boot.sh", f"— 进程退出 code={proc.poll()} —"))

    def start(self):
        if not self.enabled:
            return False, "本进程启动时带了 --no-stack-control"
        with self.lock:
            if self.running:
                return False, "已经在跑"
            self.log.append((time.time(), "LAUNCH", "boot.sh", "=== 启动整套软件 ==="))
            self.proc = subprocess.Popen(
                ["bash", "tools/boot.sh"], cwd=str(WS),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True)
            threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
            return True, "已启动 boot.sh（看日志面板）"

    def stop(self):
        if not self.running:
            return False, "没在跑"
        with self.lock:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except Exception:  # noqa: BLE001
                try:
                    self.proc.send_signal(signal.SIGINT)
                except Exception:  # noqa: BLE001
                    pass
            self.log.append((time.time(), "LAUNCH", "boot.sh", "=== 已发送停止（SIGINT）==="))
        return True, "已发送 SIGINT（优雅退出、先发零速）"


# ─────────────────────────────────────────────────────────────────────────────
# 硬件链路探测（不依赖 ROS 的裸查）
# ─────────────────────────────────────────────────────────────────────────────
def probe_hardware():
    def sh(cmd):
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=3).stdout.strip()
        except Exception:  # noqa: BLE001
            return ""

    can_up = "UP" in sh(["ip", "-br", "link", "show", "can0"])
    lidar_carrier = sh(["cat", "/sys/class/net/enp114s0/carrier"]) == "1"
    lidar_ip = "enp114s0" in sh(["ip", "-br", "addr", "show", "enp114s0"])
    ping = subprocess.run(["ping", "-c", "1", "-W", "1", "192.168.1.138"],
                          capture_output=True).returncode == 0
    serials = sorted(p.name for p in Path("/dev").glob("R1_usb2ttl"))
    serials += sorted(p.name for p in Path("/dev").glob("usb2ttl"))
    serials += sorted(p.name for p in Path("/dev").glob("ttyUSB*"))
    serials += sorted(p.name for p in Path("/dev").glob("ttyACM*"))
    return {
        "can_up": can_up,
        "lidar_link": lidar_carrier and lidar_ip,
        "lidar_ping": ping,
        "camera": Path("/dev/video0").exists(),
        "serial_ports": sorted(set(serials)),
        "r1_symlink": Path("/dev/R1_usb2ttl").exists(),
    }


def panic():
    try:
        subprocess.Popen(["bash", "tools/panic.sh"], cwd=str(WS),
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True, "已触发 panic.sh"
    except Exception as e:  # noqa: BLE001
        return False, str(e)


# ─────────────────────────────────────────────────────────────────────────────
# Flask 应用
# ─────────────────────────────────────────────────────────────────────────────
def make_app(bridge: Bridge, stack: StackManager, field: dict):
    import cv2
    from flask import Flask, Response, jsonify, request, send_file

    app = Flask(__name__)

    @app.route("/")
    def index():
        return send_file(str(Path(__file__).with_name("console_ui.html")))

    @app.route("/api/state")
    def api_state():
        s = bridge.snapshot()
        hw = probe_hardware()
        s["hw"] = hw
        s["stack_running"] = stack.running
        s["stack_enabled"] = stack.enabled
        s["field"] = field
        s["now"] = time.time()
        s["logs"] += [{"t": tt, "lvl": lv, "src": src, "msg": ms}
                      for (tt, lv, src, ms) in list(stack.log)[-40:]]
        s["logs"] = sorted(s["logs"], key=lambda x: x["t"])[-140:]
        return jsonify(s)

    @app.route("/api/camera.mjpg")
    def api_camera():
        def gen():
            # ⚠️ 必须直接读 bridge.state["cam"]：snapshot() 为了 JSON 序列化
            # 会把 cam 置成 None，用 snapshot 拿画面会永远拿到空。
            last = None
            idle = 0
            while True:
                with bridge.lock:
                    img = bridge.state["cam"]
                if img is not None and img is not last:
                    last = img
                    idle = 0
                elif last is not None:
                    idle += 1
                if last is not None:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                           b"Content-Length: " + str(len(last)).encode() + b"\r\n\r\n"
                           + last + b"\r\n")
                # 长时间没有新帧就停下（避免无相机时空转刷屏）
                if last is None and idle > 200:
                    break
                time.sleep(0.05)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/api/lidar.mjpg")
    def api_lidar():
        def gen():
            while True:
                with bridge.lock:
                    xyz = None if bridge.state["cloud"] is None else bridge.state["cloud"].copy()
                    pose = bridge.state["pose"]
                    dets = list(bridge.state["dets"])
                    lok = bridge.state["launcher_ok"]
                    ok = bridge.state["loc_ok"]
                jpg = render_lidar(xyz, pose, dets, field, lok, ok)
                if jpg:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                           b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                           + jpg + b"\r\n")
                time.sleep(0.2)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/api/action", methods=["POST"])
    def api_action():
        d = request.get_json(force=True, silent=True) or {}
        name = (d.get("name") or "").strip()
        args = d.get("args") or {}
        try:
            if name in ("PASS", "SHOOT", "IDLE"):
                return jsonify(dict(zip(("ok", "msg"), bridge.set_mission(name))))
            if name == "PANIC":
                return jsonify(dict(zip(("ok", "msg"), panic())))
            if name == "LAUNCH":
                return jsonify(dict(zip(("ok", "msg"), bridge.launcher(
                    int(args.get("action", 0)), int(args.get("speed", 0)),
                    int(args.get("angle", 0))))))
            if name == "GOTO":
                return jsonify(dict(zip(("ok", "msg"), bridge.goto(
                    float(args.get("x", 0)), float(args.get("y", 0)),
                    float(args.get("yaw", 0)), bool(args.get("align", False))))))
            if name == "JOG":
                return jsonify(dict(zip(("ok", "msg"), bridge.jog(
                    float(args.get("vx", 0)), float(args.get("vy", 0)),
                    float(args.get("wz", 0)), float(args.get("seconds", 0.4))))))
            if name == "STACK_START":
                return jsonify(dict(zip(("ok", "msg"), stack.start())))
            if name == "STACK_STOP":
                return jsonify(dict(zip(("ok", "msg"), stack.stop())))
            return jsonify({"ok": False, "msg": f"未知动作 {name}"})
        except Exception as e:  # noqa: BLE001
            return jsonify({"ok": False, "msg": f"{type(e).__name__}: {e}"})

    return app


def main(argv=None):
    import yaml
    ap = argparse.ArgumentParser(description="篮球机器人综合控制台")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--no-stack-control", action="store_true",
                    help="禁用页面上的「启动/停止整套」按钮")
    args = ap.parse_args(argv)

    mission_cfg = yaml.safe_load(
        (WS / "src/rb_mission/config/mission.yaml").read_text(encoding="utf-8")) or {}
    field = mission_cfg.get("field", {}) or {}
    topics = mission_cfg.get("topics", {}) or {}

    stack = StackManager(not args.no_stack_control, deque(maxlen=400))
    bridge = Bridge({"topics": topics})
    app = make_app(bridge, stack, field)

    print(f"\n  🏀 综合控制台 http://{args.host}:{args.port}")
    print(f"     本机访问   http://127.0.0.1:{args.port}")
    print(f"     场地 {field.get('length_m')}×{field.get('width_m')} m"
          f"   起停控制 {'开' if not args.no_stack_control else '关'}")
    print("     Ctrl-C 退出\n")
    app.run(host=args.host, port=args.port, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    raise SystemExit(main())
