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


def field_to_screen(wx, wy, fx0, fy0, scale, margin=28.0, w=LIDAR_W, h=LIDAR_H,
                    mirror_x: bool = False):
    """场地固定视角：x 向右、y 向上；mirror_x=True 时 x 向左（水平镜像）。

    fx0/fy0 是场地左下角在场地系里的坐标（一般 0,0）。
    标量/ndarray 都适用。

    ⚠️ 为什么有 mirror：现场对着实物核对时，操作员习惯的"篮筐在左"和
    标准俯视图（x 向右、篮筐在右）正好相反。镜像只是**显示**选择，
    坐标本身不变 —— 页面两种都提供，按现场实物对不对得上来选。
    """
    px = margin + (np.asarray(wx) - fx0) * scale
    if mirror_x:
        px = w - px
    return px, h - margin - (np.asarray(wy) - fy0) * scale


def depth_to_slant(depth_m: float, bearing_rad: float, max_bearing_deg: float = 78.0):
    """单目【光轴深度】→ 斜距。

    感知给的 distance_m = fx·real_size/pixel_size，是**沿相机光轴的深度 Z**，
    不是直线距离（docs/13 明确"按光轴深度处理"）。水平偏移 = Z·tan(b)，
    所以斜距 r = √(Z² + (Z·tan b)²) = Z / cos(b)。

    ⚠️ 曾经直接拿 Z 当斜距 —— 球越偏离画面中心位置越错（偏 45° 少算 29%）。
    b 接近 ±90° 时 cos→0 斜距发散，超过 max_bearing_deg 判不可信，返回 nan。
    """
    if not (depth_m == depth_m) or depth_m <= 0:
        return float("nan")
    if not (bearing_rad == bearing_rad):
        return float("nan")
    if abs(math.degrees(bearing_rad)) > max_bearing_deg:
        return float("nan")
    c = math.cos(bearing_rad)
    if c < 1e-3:
        return float("nan")
    return depth_m / c


class BallSmoother:
    """按标签维护球的场地坐标历史，返回中位数 —— **仅供显示**。

    为什么需要：单目测距逐帧抖 + 机器人 yaw 微抖，标记会到处跳，看不出球在哪。
    取最近 window_s 内若干样本的中位数：对偶发跳变更稳，又不会太滞后。
    不把平滑结果下发给任务 —— 任务侧有自己的新鲜度/一致性检查，显示层不该干扰它。
    """

    def __init__(self, window_s: float = 1.2, samples: int = 9):
        self.window_s = window_s
        self.samples = samples
        self._hist: dict = {}
        self._lock = threading.Lock()

    def update(self, label: str, t: float, wx: float, wy: float, dist: float):
        """喂一个观测，返回平滑后的 (wx, wy, dist, 样本数)；NaN 输入返回 None。"""
        if not all(v == v for v in (wx, wy, dist)):
            return None
        with self._lock:
            h = self._hist.setdefault(label, [])
            h.append((t, wx, wy, dist))
            cutoff = t - self.window_s
            while h and h[0][0] < cutoff:
                h.pop(0)
            if len(h) > self.samples:
                del h[:-self.samples]
            n = len(h)
            xs = sorted(s[1] for s in h)
            ys = sorted(s[2] for s in h)
            ds = sorted(s[3] for s in h)
            return xs[n // 2], ys[n // 2], ds[n // 2], n

    def age(self, label: str, now: float):
        with self._lock:
            h = self._hist.get(label)
            return None if not h else now - h[-1][0]


_smoother = BallSmoother()


def world_to_screen(wx, wy, cx, cy, cyaw, scale, w=LIDAR_W, h=LIDAR_H):
    """场地系 (wx,wy) → 俯视图像素。**车头朝上，车的左边在屏幕左边。**

    对标量/ndarray 都适用（雷达点云用数组形式一次算完）。
    返回 (px, py)。
    """
    dx = np.asarray(wx) - cx
    dy = np.asarray(wy) - cy
    fwd = dx * math.cos(cyaw) + dy * math.sin(cyaw)      # 车体系前向分量
    left = -dx * math.sin(cyaw) + dy * math.cos(cyaw)    # 车体系左向分量
    return w / 2.0 - left * scale, h / 2.0 - fwd * scale


def render_lidar(cloud_xyz, pose, dets, field, launcher_ok, loc_ok,
                 view: str = "field", mirror_x: bool = False) -> bytes:
    """把点云画成俯视图（鸟瞰），叠加机器人位姿、球检测、场地边界。"""
    import cv2

    img = np.zeros((LIDAR_H, LIDAR_W, 3), dtype=np.uint8)
    img[:] = (24, 22, 20)
    texts = []          # ⚠️ 必须在任何 append 之前建（曾经放在后面 → UnboundLocalError）

    # 世界坐标 → 画布：以机器人为中心，朝上为 +x（车头方向固定朝上）
    cx, cy, cyaw = (pose if pose else (0.0, 0.0, 0.0))
    L = float(field.get("length_m", 14.0))
    W = float(field.get("width_m", 7.5))

    # ── 两套坐标变换 ──────────────────────────────────────────────
    if view == "robot":
        # 车头朝上：以车为中心，看"球在我哪一侧"方便
        scale = min(LIDAR_W, LIDAR_H) / (2.0 * LIDAR_RANGE_M)

        def to_px(wx, wy):
            px, py = world_to_screen(wx, wy, cx, cy, cyaw, scale)
            return int(px), int(py)
    else:
        # 场地固定（默认）：整个场地画全，x→右、y→上，和 RViz 俯视一致。
        # 现场对着实物核对时不容易左右误判。
        margin = 30.0
        scale = min((LIDAR_W - 2 * margin) / max(L, 1e-3),
                    (LIDAR_H - 2 * margin) / max(W, 1e-3)) * 0.94
        _mir = (mirror_x and view == "field")

        def to_px(wx, wy):
            px, py = field_to_screen(wx, wy, 0.0, 0.0, scale, margin, mirror_x=_mir)
            return int(px), int(py)

    # 场地边界（浅灰矩形）+ 中线
    pts = np.array([to_px(x, y) for x, y in ((0, 0), (L, 0), (L, W), (0, W))], np.int32)
    cv2.polylines(img, [pts], True, (90, 90, 90), 2, cv2.LINE_AA)
    if view != "robot":
        cv2.line(img, to_px(L / 2, 0), to_px(L / 2, W), (52, 52, 52), 1, cv2.LINE_AA)
        cv2.line(img, to_px(0, W / 2), to_px(L, W / 2), (52, 52, 52), 1, cv2.LINE_AA)

    # ── 场地各处区域（全部从配置读，现场量完改配置就变）──────────────
    def _poly(world_pts, color, thick=1, close=True, dash=False):
        """世界坐标点列 → 屏幕折线。超出场地的自动裁掉（只看场内）。"""
        ps = []
        for (px_, py_) in world_pts:
            sx, sy = to_px(px_, py_)
            ps.append((sx, sy))
        if len(ps) >= 2:
            cv2.polylines(img, [np.array(ps, np.int32)], close, color, thick, cv2.LINE_AA)

    def _arc(cxw, cyw, r, color, thick=1, a0=0.0, a1=360.0, steps=72):
        """圆弧（世界坐标），自动裁到场内。"""
        pts = []
        for i in range(steps + 1):
            a = math.radians(a0 + (a1 - a0) * i / steps)
            wx_, wy_ = cxw + r * math.cos(a), cyw + r * math.sin(a)
            if -0.05 <= wx_ <= L + 0.05 and -0.05 <= wy_ <= W + 0.05:
                pts.append((wx_, wy_))
        if len(pts) >= 2:
            _poly(pts, color, thick, close=False)

    # 三分线（以 three_point_center 为心）
    tp = field.get("three_point_center") or {}
    r3 = float(field.get("three_point_radius_m", 0.0) or 0.0)
    if isinstance(tp, dict) and "x" in tp and r3 > 0:
        _arc(float(tp["x"]), float(tp["y"]), r3, (60, 110, 150), 1)
        # 三分线在场内的两条直线段（足球场式的直线部分）
        _poly([(float(tp["x"]), float(tp["y"]) - r3), (L, float(tp["y"]) - r3)], (60, 110, 150))
        _poly([(float(tp["x"]), float(tp["y"]) + r3), (L, float(tp["y"]) + r3)], (60, 110, 150))
        tp_px = to_px(float(tp["x"]), float(tp["y"]))
        texts.append(("三分线", (tp_px[0] - 20, tp_px[1] + 16), (70, 120, 165), 12))

    # 投篮线（以篮筐为心）
    hoop = field.get("hoop") or {}
    rs = float(field.get("shoot_line_radius_m", 0.0) or 0.0)
    if isinstance(hoop, dict) and "x" in hoop and rs > 0:
        _arc(float(hoop["x"]), float(hoop["y"]), rs, (60, 130, 100), 1)

    # 传球区（多边形）
    pz = (field.get("pass_zone") or {}).get("polygon")
    if pz:
        try:
            poly = [(float(a), float(b)) for a, b in pz]
            _poly(poly + [poly[0]], (150, 120, 60), 1, close=False)
            # 半透明填充，便于一眼看出"我在不在区内"
            sp = np.array([to_px(a, b) for a, b in poly], np.int32)
            ov = img.copy()
            cv2.fillPoly(ov, [sp], (60, 48, 24))
            cv2.addWeighted(ov, 0.45, img, 0.55, 0, img)
            cxp = sum(a for a, _ in poly) / len(poly)
            cyp = sum(b for _, b in poly) / len(poly)
            q = to_px(cxp, cyp)
            texts.append(("传球区", (q[0] - 16, q[1] + 4), (190, 160, 90), 13))
        except (TypeError, ValueError):
            pass

    # 出发区（配置里只有 home 点；画一个 0.6m 的方块示意，实际尺寸待现场确认）
    hm = field.get("home") or {}
    if isinstance(hm, dict) and "x" in hm:
        hx, hy = float(hm["x"]), float(hm["y"])
        half = 0.30
        _poly([(hx - half, hy - half), (hx + half, hy - half),
               (hx + half, hy + half), (hx - half, hy + half), (hx - half, hy - half)],
              (90, 150, 90), 1, close=False)
        _arc(hx, hy, 0.22, (90, 150, 90), 1)
        q = to_px(hx, hy)
        texts.append(("出发区", (q[0] + 10, q[1] + 20), (110, 180, 110), 13))

    # 篮筐符号（篮板 + 篮圈）
    if isinstance(hoop, dict) and "x" in hoop:
        hx, hy = float(hoop["x"]), float(hoop["y"])
        _arc(hx, hy, 0.225, (0, 140, 255), 2)          # 篮圈 φ45cm
        # 篮板：在篮筐靠边线那一侧，画一条短线
        bx = min(L, hx + 0.30)
        _poly([(bx, hy - 0.90), (bx, hy + 0.90)], (0, 140, 255), 2, close=False)
        q = to_px(hx, hy)
        texts.append(("篮筐", (q[0] - 14, q[1] - 16), (0, 160, 255), 13))

    # 得分点参考（传球区/投篮点/篮筐）
    refs = [("投篮点", field.get("shoot_zone_target"))]
    for name, p in refs:
        if isinstance(p, dict) and "x" in p:
            px = to_px(p["x"], p["y"])
            cv2.drawMarker(img, px, (90, 160, 90), cv2.MARKER_DIAMOND, 8, 1)
            texts.append((name, (px[0] + 7, px[1] - 16), (120, 180, 120), 13))

    # 点云（按高度上色）
    if cloud_xyz is not None and len(cloud_xyz):
        p = cloud_xyz
        if view == "robot":
            px_f, py_f = world_to_screen(p[:, 0], p[:, 1], cx, cy, cyaw, scale)
            dx = p[:, 0] - cx
            dy = p[:, 1] - cy
            fwd = dx * math.cos(cyaw) + dy * math.sin(cyaw)
            left = -dx * math.sin(cyaw) + dy * math.cos(cyaw)
            keep_near = ((np.abs(fwd) < LIDAR_RANGE_M * 1.2) &
                         (np.abs(left) < LIDAR_RANGE_M * 1.2))
        else:
            px_f, py_f = field_to_screen(p[:, 0], p[:, 1], 0.0, 0.0, scale, 30.0,
                                         mirror_x=_mir)
            keep_near = np.ones(len(p), dtype=bool)     # 场地模式不按半径裁
        px = px_f.astype(np.int32)
        py = py_f.astype(np.int32)
        m = ((px >= 0) & (px < LIDAR_W) & (py >= 0) & (py < LIDAR_H) & keep_near)
        z = p[m, 2]
        zn = np.clip((z + 0.3) / 2.0, 0, 1)
        cols = np.stack([(60 + 195 * zn), (200 - 120 * zn), (255 - 200 * zn)], axis=1)
        img[py[m], px[m]] = cols.astype(np.uint8)

    # 机器人（红箭头）
    rx, ry = to_px(cx, cy)
    if view == "robot":
        tip = (rx, ry - 26)
    else:
        # 场地模式：箭头按 yaw 转（世界 +x 是屏幕右，+y 是屏幕上）
        tip = (int(rx + 30 * math.cos(cyaw)), int(ry - 30 * math.sin(cyaw)))
    cv2.arrowedLine(img, (rx, ry), tip, (60, 60, 255), 3, cv2.LINE_AA, tipLength=0.35)
    cv2.circle(img, (rx, ry), 13, (60, 60, 255), 1, cv2.LINE_AA)

    # 视野半径标尺（只在车头朝上模式有意义）
    if view == "robot":
        r = int(LIDAR_RANGE_M * scale)
        cv2.circle(img, (LIDAR_W // 2, LIDAR_H // 2), r, (48, 48, 48), 1, cv2.LINE_AA)
        texts.append((f"{LIDAR_RANGE_M:.0f} m", (LIDAR_W // 2 + 6, LIDAR_H // 2 - r + 2),
                      (110, 110, 110), 13))

    # 球检测（相机方位角 + 光轴深度 → 场地坐标）
    now = time.time()
    for d in dets or []:
        bearing = getattr(d, "bearing_rad", float("nan"))
        depth = getattr(d, "distance_m", float("nan"))
        # ⚠️ distance_m 是【光轴深度】不是斜距，必须先换算（曾经直接用 → 位置偏）
        r = depth_to_slant(depth, bearing)
        if not (r == r):
            continue
        # bearing 右正左负；场地系 y 朝左 → 世界方位 = yaw - bearing
        ang = cyaw - bearing
        wx, wy = cx + r * math.cos(ang), cy + r * math.sin(ang)
        label = getattr(d, "label", "?")
        # 显示层时间平滑（治"一直在跳动"）—— 只影响画面，不下发给任务
        sm = _smoother.update(label, now, wx, wy, r)
        if sm is None:
            continue
        wx, wy, r, _n = sm
        px, py = to_px(wx, wy)
        col = (0, 200, 255) if "basket" in label else (255, 180, 60)
        cv2.circle(img, (px, py), 9, col, 2, cv2.LINE_AA)
        cv2.line(img, (rx, ry), (px, py), col, 1, cv2.LINE_AA)
        texts.append((f"{label} {r:.2f}m", (px + 12, py - 8), col, 13))

    # 左上角文字状态
    lines = [
        (f"位姿 ({cx:+.2f}, {cy:+.2f})   yaw {math.degrees(cyaw):+.0f}°"
         + ("   [场地固定→右]" if view == "field" and not (mirror_x and view == "field")
            else ("   [场地固定→左]" if view == "field" else "   [车头朝上]")),
         (215, 215, 215), 15),
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
        # 阶段变迁时间线：方便一眼看出"跑到哪一步、卡在哪一步"
        self.timeline: deque = deque(maxlen=200)
        self._last_phase_key = None
        # 本轮各阶段累计耗时（阶段名 -> 秒）
        self.phase_seconds: dict = {}
        self._phase_enter: float | None = None
        self._phase_name: str | None = None
        self.action_count = 0          # 本轮的发射动作次数（LAUNCH 进入次数）

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
    # 一轮里按顺序会经历的阶段（用于页面上画"已完成/进行中/待执行"）
    ROUND_STAGES = ("START_DELAY", "SEEK_BALL", "ACQUIRE", "NAV_TO_ZONE",
                    "ALIGN", "LAUNCH", "RETURN_HOME", "DONE")

    def _on_status(self, m):
        """任务状态 + 阶段变迁记录。

        为什么要记时间线：比赛时最需要回答的是"跑到哪一步了 / 卡在哪一步"。
        光看当前 phase 不够，得看出**走过的顺序和各段耗时**。
        """
        now = time.time()
        key = (m.mission, m.phase)
        with self.lock:
            self.state.update(mission=m.mission, phase=m.phase, detail=m.detail,
                              elapsed=float(m.elapsed_s), score=int(m.score_estimate))
            if key == self._last_phase_key:
                # 同一阶段内 detail 是实时变的（"对准 hoop 误差 +2.1°"这种），
                # 要回写到时间线最后一条，否则页面只显示进入该阶段时的那句话。
                if self.timeline:
                    self.timeline[-1]["detail"] = m.detail or ""
                    self.timeline[-1]["elapsed"] = float(m.elapsed_s)
                return
            # 结算上一阶段耗时
            if self._phase_name is not None and self._phase_enter is not None:
                self.phase_seconds[self._phase_name] = (
                    self.phase_seconds.get(self._phase_name, 0.0) + (now - self._phase_enter))
            prev_phase = self._last_phase_key[1] if self._last_phase_key else None

            # 新任务的起点：从 IDLE 跳到 PASS/SHOOT 时清空旧时间线，
            # 否则页面上会残留上一轮的 IDLE 段（显示成一条没意义的"待命"）。
            prev_mission = self._last_phase_key[0] if self._last_phase_key else None
            if m.phase == "IDLE" or (prev_mission == "IDLE" and m.mission in ("PASS", "SHOOT")):
                self.timeline.clear()
                self.phase_seconds = {}
                self.action_count = 0

            self.timeline.append({
                "t": now, "mission": m.mission, "phase": m.phase,
                "detail": m.detail or "", "elapsed": float(m.elapsed_s), "dur": None,
            })
            if len(self.timeline) >= 2:
                self.timeline[-2]["dur"] = round(now - self.timeline[-2]["t"], 2)

            # 发射动作计数：进入 LAUNCH 且上一段不是 LAUNCH 才算一次
            if m.phase == "LAUNCH" and prev_phase != "LAUNCH":
                self.action_count += 1

            self._last_phase_key = key
            self._phase_enter = now
            self._phase_name = m.phase

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
        """返回可 JSON 化的快照。

        ⚠️ 三处必须处理的不可序列化对象：
          cam   = 原始 JPEG 字节；cloud = ndarray；dets = 消息对象
        """
        with self.lock:
            s = dict(self.state)
            s["logs"] = [{"t": tt, "lvl": lv, "src": src, "msg": ms}
                         for (tt, lv, src, ms) in list(self.logs)[-120:]]
            s["timeline"] = list(self.timeline)
            s["action_count"] = self.action_count
            s["phase_seconds"] = dict(self.phase_seconds)
            s["phase_elapsed"] = (round(time.time() - self._phase_enter, 1)
                                  if self._phase_name and self._phase_enter else 0.0)
            s["round_stages"] = list(self.ROUND_STAGES)
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
# 系统检查：就绪检查 / CAN 恢复（把脚本结果解析成结构化状态给页面）
# ─────────────────────────────────────────────────────────────────────────────
_ANSI = __import__("re").compile(r"\x1b\[[0-9;]*m")


def parse_check_ready(text: str) -> dict:
    """把 tools/check_ready.sh 的输出解析成 {ok, items[], verdict}。

    脚本输出形如（已去色）：
        ✓ /livox/lidar             10.3 Hz
        ✗ /Odometry.x              发散
        ✅ 全部就绪，可以开车 / 开地图了
        ⚠️ 还有 3 项没就绪
    """
    items = []
    verdict = ""
    for raw in _ANSI.sub("", text or "").splitlines():
        s = raw.strip()
        if s.startswith("✓ "):
            items.append({"ok": True, "text": s[2:].strip()})
        elif s.startswith("✗ "):
            items.append({"ok": False, "text": s[2:].strip()})
        elif s.startswith("✅") or s.startswith("⚠️"):
            verdict = s
    return {
        "items": items,
        "failed": [i["text"] for i in items if not i["ok"]],
        "verdict": verdict,
        "ok": bool(items) and all(i["ok"] for i in items) and "全部就绪" in verdict,
        "checked_at": time.time(),
    }


class SystemChecks:
    """跑 check_ready.sh / can_recover.sh，把结果缓存下来给页面。"""

    def __init__(self, log_deque: deque):
        self.log = log_deque
        self.lock = threading.Lock()
        self.state = {"running": None, "result": None, "can_result": None,
                      "flow": None, "flow_step": ""}

    def _run(self, name: str, script: str, timeout: float = 180.0) -> str:
        self.log.append((time.time(), "TOOL", name, f"=== 运行 {script} ==="))
        try:
            r = subprocess.run(["bash", script], cwd=str(WS),
                               capture_output=True, text=True, timeout=timeout)
            out = (r.stdout or "") + (r.stderr or "")
        except subprocess.TimeoutExpired:
            out = f"（超时 {timeout:.0f}s）"
        for ln in _ANSI.sub("", out).splitlines():
            self.log.append((time.time(), "TOOL", name, ln))
        return out

    def check_ready(self):
        with self.lock:
            if self.state["running"]:
                return False, "已有检查在跑"
            self.state["running"] = "check_ready"
        try:
            out = self._run("就绪检查", "tools/check_ready.sh")
            res = parse_check_ready(out)
            with self.lock:
                self.state["result"] = res
                self.state["running"] = None
            n = len(res["failed"])
            return True, ("✅ 全部就绪" if res["ok"] else f"⚠️ {n} 项没就绪")
        except Exception as e:  # noqa: BLE001
            with self.lock:
                self.state["running"] = None
            return False, f"{type(e).__name__}: {e}"

    def can_recover(self):
        with self.lock:
            if self.state["running"]:
                return False, "已有检查在跑"
            self.state["running"] = "can_recover"
        try:
            out = self._run("CAN 恢复", "tools/can_recover.sh", timeout=60.0)
            with self.lock:
                self.state["can_result"] = _ANSI.sub("", out).strip()[-400:]
                self.state["running"] = None
            return True, "CAN 恢复已执行（看日志）"
        except Exception as e:  # noqa: BLE001
            with self.lock:
                self.state["running"] = None
            return False, f"{type(e).__name__}: {e}"

    def startup_flow(self, stack: "StackManager"):
        """一键流程：boot.sh → 等 mission 就绪 → 自动就绪检查。"""
        with self.lock:
            if self.state["flow"] == "running":
                return False, "一键流程已在跑"
            self.state["flow"] = "running"
            self.state["flow_step"] = "启动整套"

        def worker():
            try:
                ok, msg = stack.start()
                self.log.append((time.time(), "TOOL", "一键流程", f"boot.sh: {msg}"))
                if not ok:
                    with self.lock:
                        self.state["flow"] = "failed"
                        self.state["flow_step"] = msg
                    return
                # 等关键就绪标志（最多 120s）
                deadline = time.time() + 120
                seen = {"chassis": False, "mission": False}
                while time.time() < deadline:
                    txt = "\n".join(m for (_, _, _, m) in list(stack.log)[-200:])
                    if "底盘已使能" in txt:
                        seen["chassis"] = True
                    if "mission 就绪" in txt:
                        seen["mission"] = True
                    if seen["chassis"] and seen["mission"]:
                        break
                    if not stack.running:
                        with self.lock:
                            self.state["flow"] = "failed"
                            self.state["flow_step"] = "boot.sh 退出了（看日志）"
                        return
                    time.sleep(1.5)
                else:
                    with self.lock:
                        self.state["flow"] = "failed"
                        self.state["flow_step"] = "等就绪标志超时（看日志）"
                    return

                self.log.append((time.time(), "TOOL", "一键流程",
                                 "底盘已使能 + mission 就绪 → 开始就绪检查"))
                with self.lock:
                    self.state["flow_step"] = "就绪检查"
                out = self._run("就绪检查", "tools/check_ready.sh")
                res = parse_check_ready(out)
                with self.lock:
                    self.state["result"] = res
                    self.state["flow"] = "done" if res["ok"] else "notready"
                    self.state["flow_step"] = ("可以开车了" if res["ok"]
                                               else f"{len(res['failed'])} 项没就绪")
            except Exception as e:  # noqa: BLE001
                with self.lock:
                    self.state["flow"] = "failed"
                    self.state["flow_step"] = f"{type(e).__name__}: {e}"

        threading.Thread(target=worker, daemon=True).start()
        return True, "一键流程已启动（boot.sh → 等就绪 → 就绪检查）"


# ─────────────────────────────────────────────────────────────────────────────
# 调试脚本运行器（寻球/绕场等整车测试）
# ─────────────────────────────────────────────────────────────────────────────
# 这些脚本会**接管 /cmd_vel**（用 --takeover 让 mission 节点退出），
# 所以页面上点之前必须确认：任务停了、急停手段就绪。
DEBUG_TOOLS = {
    "TEST_BALL": {
        "script": "tools/test_perimeter_ball.py",
        "args": ["--skip-perimeter", "--takeover", "--yes"],
        "label": "寻球 → 走到球前 50cm → 转身背对球",
    },
    "TEST_PERIMETER": {
        "script": "tools/test_perimeter_ball.py",
        "args": ["--takeover", "--yes"],
        "label": "绕场一圈 → 寻球 → 走到球前 → 转身背对球",
    },
    "TEST_CHASSIS_DRY": {
        "script": "tools/test_chassis.py",
        "args": [],
        "label": "底盘干跑自检（不下发 CAN，安全）",
    },
}


class ToolRunner:
    """跑一个调试脚本，把它的输出流进页面日志面板。同一时刻只允许一个。"""

    def __init__(self, log_deque: deque):
        self.log = log_deque
        self.proc: subprocess.Popen | None = None
        self.name: str | None = None
        self.lock = threading.Lock()

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def _pump(self, proc, name):
        for raw in iter(proc.stdout.readline, b""):
            try:
                self.log.append((time.time(), "TOOL", name,
                                 raw.decode("utf-8", "replace").rstrip()))
            except Exception:  # noqa: BLE001
                pass
        code = proc.poll()
        lvl = "TOOL" if code == 0 else "ERROR"
        self.log.append((time.time(), lvl, name, f"— {name} 结束，退出码 {code} —"))

    def start(self, key: str):
        spec = DEBUG_TOOLS.get(key)
        if not spec:
            return False, f"未知调试脚本 {key}"
        with self.lock:
            if self.running:
                return False, f"{self.name} 还在跑，先停掉它"
            if not (WS / spec["script"]).is_file():
                return False, f"找不到 {spec['script']}"
            # 用显式的 env，避免控制台是别的方式起的导致子进程缺 ROS 环境
            cmd = ["bash", "-c",
                   "source /opt/ros/humble/setup.bash && "
                   f"source {WS}/install/setup.bash && "
                   f"exec python3 {spec['script']} " + " ".join(spec["args"])]
            self.log.append((time.time(), "TOOL", key, f"=== 启动：{spec['label']} ==="))
            self.proc = subprocess.Popen(
                cmd, cwd=str(WS), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True)
            self.name = key
            threading.Thread(target=self._pump, args=(self.proc, key), daemon=True).start()
            return True, f"已启动（{spec['label']}），看日志面板"

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
            self.log.append((time.time(), "TOOL", self.name or "tool",
                             "=== 已发送停止（SIGINT，脚本会先发零速）==="))
        return True, "已发送停止"


# ─────────────────────────────────────────────────────────────────────────────
# 硬件链路探测（不依赖 ROS 的裸查）
# ─────────────────────────────────────────────────────────────────────────────
# 关键节点：正常各 1 个。>1 = 孤儿进程/重复启动 → 会互相抢话题，
# 症状是"数据抖动/阶段乱跳"（实测：两个 mission_node 会让 /mission/status 的
# phase 在 START_DELAY 和 SEEK_BALL 之间高频振荡）。
KEY_NODES = {
    "livox 驱动": "livox_ros_driver2_node",
    "FAST-LIO": "fastlio_mapping",
    "相机": "camera_node",
    "感知": "perception_node",
    "定位": "localization_node",
    "底盘": "rb_chassis_node",
    "机构": "rb_launcher_node",
    "任务": "mission_node",
}


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
    # 数关键节点（用 install 路径数，避免匹配到自己）
    counts = {}
    try:
        out = subprocess.run(["ps", "-eo", "cmd"], capture_output=True, text=True,
                             timeout=4).stdout
        for label, name in KEY_NODES.items():
            counts[label] = sum(1 for ln in out.splitlines()
                                if f"install/" in ln and ln.rstrip().endswith(name) or
                                   (f"/{name} " in ln and "install/" in ln))
    except Exception:  # noqa: BLE001
        pass
    dup = {k: v for k, v in counts.items() if v > 1}
    return {
        "node_counts": counts,
        "node_dup": dup,
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
def make_app(bridge: Bridge, stack: StackManager, tool: ToolRunner,
             checks: SystemChecks, field: dict):
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
        s["tool_running"] = tool.running
        s["tool_name"] = tool.name
        with checks.lock:
            s["readiness"] = checks.state["result"]
            s["checks_running"] = checks.state["running"]
            s["flow"] = checks.state["flow"]
            s["flow_step"] = checks.state["flow_step"]
            s["can_result"] = checks.state["can_result"]
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
        # ?view=robot 切换成"车头朝上"；默认场地固定（和 RViz 俯视一致）
        view = (request.args.get("view") or "field").lower()
        if view not in ("field", "robot"):
            view = "field"
        # ?mirror=1 → x 向左（篮筐在左，与现场实物核对习惯一致）
        mirror = (request.args.get("mirror") or "0") in ("1", "true", "yes")

        def gen():
            while True:
                with bridge.lock:
                    xyz = None if bridge.state["cloud"] is None else bridge.state["cloud"].copy()
                    pose = bridge.state["pose"]
                    dets = list(bridge.state["dets"])
                    lok = bridge.state["launcher_ok"]
                    ok = bridge.state["loc_ok"]
                jpg = render_lidar(xyz, pose, dets, field, lok, ok,
                                   view=view, mirror_x=mirror)
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
            if name == "TOOL_STOP":
                return jsonify(dict(zip(("ok", "msg"), tool.stop())))
            if name == "CHECK_READY":
                return jsonify(dict(zip(("ok", "msg"), checks.check_ready())))
            if name == "CAN_RECOVER":
                return jsonify(dict(zip(("ok", "msg"), checks.can_recover())))
            if name == "STARTUP_FLOW":
                return jsonify(dict(zip(("ok", "msg"), checks.startup_flow(stack))))
            if name in DEBUG_TOOLS:
                return jsonify(dict(zip(("ok", "msg"), tool.start(name))))
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

    logbuf: deque = deque(maxlen=400)
    stack = StackManager(not args.no_stack_control, logbuf)
    tool = ToolRunner(logbuf)
    checks = SystemChecks(logbuf)
    bridge = Bridge({"topics": topics})
    app = make_app(bridge, stack, tool, checks, field)

    print(f"\n  🏀 综合控制台 http://{args.host}:{args.port}")
    print(f"     本机访问   http://127.0.0.1:{args.port}")
    print(f"     场地 {field.get('length_m')}×{field.get('width_m')} m"
          f"   起停控制 {'开' if not args.no_stack_control else '关'}")
    print("     Ctrl-C 退出\n")
    app.run(host=args.host, port=args.port, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    raise SystemExit(main())
