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

try:                                  # 雷达找球（纯 numpy，导入很轻）
    from rb_perception.lidar_ball import (body_frame_cloud, find_ball_candidates,
                                          candidate_to_repr)
    _LIDAR_BALL_OK = True
except Exception:                     # noqa: BLE001
    _LIDAR_BALL_OK = False

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
                    mirror_x: bool = False, mirror_y: bool = False,
                    field_l: float | None = None, field_w: float | None = None):
    """场地固定视角：x 向右、y 向上；mirror_x=True 时 x 向左（水平镜像）。

    fx0/fy0 是场地左下角在场地系里的坐标（一般 0,0）。
    标量/ndarray 都适用。

    ⚠️ 为什么有 mirror：现场对着实物核对时，操作员习惯的"篮筐在左"和
    标准俯视图（x 向右、篮筐在右）正好相反。镜像只是**显示**选择，
    坐标本身不变 —— 页面两种都提供，按现场实物对不对得上来选。
    """
    # ⚠️ mirror_x/mirror_y 只是**显示**选择，坐标本身不变。
    #    官方场地图：出发区 A 在左上、篮筐在右中 → 我们的 home=(1,1) 在左下，
    #    要显示成官方那个样子就得开 mirror_y（上下翻）。
    px = (np.asarray(wx) - fx0) * scale
    py = (np.asarray(wy) - fy0) * scale
    if field_l and field_w:
        # 场地在画布中【真正居中】——只按单轴 margin 锚会造成另一个轴偏心
        # （实测：900x560 画布、14x7.5 场地，纵向偏下 38px）
        off_x = (w - float(field_l) * scale) / 2.0
        off_y = (h - float(field_w) * scale) / 2.0
        px = px + off_x
        # y 向上：世界 y 越大 → 屏幕 y 越小
        py = (h - off_y) - (np.asarray(wy) - fy0) * scale
    else:
        px = px + margin
        py = h - margin - py
    if mirror_x:
        px = w - px
    if mirror_y:
        py = h - py
    return px, py


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


def odom_cloud_to_field(cloud_xyz, tx: float, ty: float, theta: float):
    """把 camera_init 系的点云变换到 field 系。

    定位节点发 `field → camera_init` 的**刚性变换**（不是 EKF 后的位姿）：
      p_field = R(theta)·p_ci + (tx, ty)
    点云不做这步就直接画，会落在 camera_init 原点附近 —— 现场表现为
    "点云显示在 B、而箭头（field 位姿）在 A"，两者对不上。
    """
    c, s = math.cos(theta), math.sin(theta)
    out = np.empty_like(cloud_xyz)
    out[:, 0] = c * cloud_xyz[:, 0] - s * cloud_xyz[:, 1] + tx
    out[:, 1] = s * cloud_xyz[:, 0] + c * cloud_xyz[:, 1] + ty
    out[:, 2] = cloud_xyz[:, 2]
    return out


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
                 view: str = "field", mirror_x: bool = False,
                 mirror_y: bool = False, odom_tf=None, lb_cands=None) -> bytes:
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
        # ⚠️ 出发区 A/B 突出在场地左边线【之外】（x<0），
        #    所以绘图范围要含场外一圈，否则方块会被裁掉。
        _zs = (field.get("start_zones") or {})
        # 出发区可能从【任一】边线向外突出（现场是上下边线），
        # 所以四个方向都留出余量，否则突出的方块会被裁掉。
        _ext = float(field.get("start_zone_offset_m", 1.0) or 0.0) + 0.6
        X0, X1 = -_ext, L + _ext
        Y0, Y1 = -_ext, W + _ext
        _fl, _fw = X1 - X0, Y1 - Y0
        scale = min((LIDAR_W - 2 * margin) / max(_fl, 1e-3),
                    (LIDAR_H - 2 * margin) / max(_fw, 1e-3)) * 0.98
        _mir = (mirror_x and view == "field")
        _miry = (mirror_y and view == "field")

        def to_px(wx, wy):
            # 用"场外左下角"当原点，让整个可见区居中
            px, py = field_to_screen(wx - X0, wy - Y0, 0.0, 0.0, scale, margin,
                                     mirror_x=_mir, mirror_y=_miry,
                                     field_l=_fl, field_w=_fw)
            return int(px), int(py)

    # ── 雷达找球候选（与相机检测区分开：青色虚线圈 + 「雷达」标注）──
    for cd in (lb_cands or []):
        f = cd.get("field")
        if not f:
            continue
        qx, qy = to_px(float(f[0]), float(f[1]))
        r_px = max(6, int(0.12 * scale))
        cv2.circle(img, (qx, qy), r_px, (255, 220, 60), 2, cv2.LINE_AA)
        cv2.circle(img, (qx, qy), r_px + 4, (120, 110, 40), 1, cv2.LINE_AA)
        texts.append((f"雷达球 {cd.get('slant_m', 0):.1f}m", (qx + r_px + 3, qy - 4),
                      (255, 220, 60), 12))

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
            # 严格裁到场内：官方图上三分线也是被边线裁掉的，不该画到场外
            if 0.0 <= wx_ <= L and 0.0 <= wy_ <= W:
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

    # 中线置球区（官方图左侧那个大圆：R2500，圆心距左边线 2500mm → (2.5, 3.75)）
    fz = field.get("front_zone_circle") or {}
    if isinstance(fz, dict) and "x" in fz and float(fz.get("radius_m", 0) or 0) > 0:
        _arc(float(fz["x"]), float(fz["y"]), float(fz["radius_m"]), (110, 90, 140), 1)
        q = to_px(float(fz["x"]), float(fz["y"]))
        texts.append((str(fz.get("name", "中线置球区")), (q[0] - 30, q[1] + 4),
                      (140, 115, 175), 12))

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

    # 出发区：官方图里 A/B 是**突出在场地左边线之外**的方块。
    # 尺寸与中心都从配置读（现场量完改 start_zone_size_m / start_zones 即可）。
    sz = float(field.get("start_zone_size_m", 1.0) or 1.0)
    zones = field.get("start_zones") or {}
    home = field.get("home") or {}
    if not zones and isinstance(home, dict) and "x" in home:
        zones = {"A": home}                      # 兜底：只配了 home 就画一个
    for zname, zc in zones.items():
        if not (isinstance(zc, dict) and "x" in zc):
            continue
        zx, zy = float(zc["x"]), float(zc["y"])
        h2 = sz / 2.0
        _poly([(zx - h2, zy - h2), (zx + h2, zy - h2), (zx + h2, zy + h2),
               (zx - h2, zy + h2), (zx - h2, zy - h2)],
              (90, 150, 90), 2, close=False)
        q = to_px(zx, zy)
        is_home = (isinstance(home, dict) and abs(zx - float(home.get("x", 1e9))) < 1e-6
                   and abs(zy - float(home.get("y", 1e9))) < 1e-6)
        col = (140, 220, 140) if is_home else (80, 110, 80)
        texts.append((f"出发区{zname}" + ("（当前）" if is_home else ""),
                      (q[0] - 22, q[1] + 4), col, 13))

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
        # ⚠️ 点云在 FAST-LIO 的 camera_init 系里，必须先变换到 field 系再画。
        #    不做这步点云会落在 camera_init 原点附近（现场现象："点云在 B、箭头在 A"）。
        if odom_tf is not None:
            p = odom_cloud_to_field(cloud_xyz, *odom_tf)
        if view == "robot":
            px_f, py_f = world_to_screen(p[:, 0], p[:, 1], cx, cy, cyaw, scale)
            dx = p[:, 0] - cx
            dy = p[:, 1] - cy
            fwd = dx * math.cos(cyaw) + dy * math.sin(cyaw)
            left = -dx * math.sin(cyaw) + dy * math.cos(cyaw)
            keep_near = ((np.abs(fwd) < LIDAR_RANGE_M * 1.2) &
                         (np.abs(left) < LIDAR_RANGE_M * 1.2))
        else:
            px_f, py_f = field_to_screen(p[:, 0] - X0, p[:, 1] - Y0, 0.0, 0.0, scale, 30.0,
                                         mirror_x=_mir, mirror_y=_miry,
                                         field_l=_fl, field_w=_fw)
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
        # 车头朝上视角：箭头恒朝上
        tip = (rx, ry - 26)
    else:
        # ⚠️ 场地模式必须用 to_px 算车头前方的第二个世界点，
        #    这样 mirror_x 才会一起生效。手算 (rx+30cos, …) 在镜像视角下车头会反。
        tx, ty = to_px(cx + 0.55 * math.cos(cyaw), cy + 0.55 * math.sin(cyaw))
        tip = (int(tx), int(ty))
    cv2.arrowedLine(img, (rx, ry), tip, (60, 60, 255), 3, cv2.LINE_AA, tipLength=0.35)
    cv2.circle(img, (rx, ry), 13, (60, 60, 255), 1, cv2.LINE_AA)

    # 视野半径标尺（只在车头朝上模式有意义）
    if view == "robot":
        r = int(LIDAR_RANGE_M * scale)
        cv2.circle(img, (LIDAR_W // 2, LIDAR_H // 2), r, (48, 48, 48), 1, cv2.LINE_AA)
        texts.append((f"{LIDAR_RANGE_M:.0f} m", (LIDAR_W // 2 + 6, LIDAR_H // 2 - r + 2),
                      (110, 110, 110), 13))

    # 球检测（相机方位角 + 光轴深度 → 场地坐标）
    # ⚠️ 只画【球】：rack_ring / hoop / pillar 是静态场地物，已作为场地区域画出来，
    #    再当"检测到的球"画一遍既重复又误导（现场明确要求去掉 rack_ring）。
    now = time.time()
    for d in dets or []:
        label0 = getattr(d, "label", "")
        if not str(label0).startswith("ball"):
            continue
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
         + ("" if view == "robot"
            else f"   [x→{'左' if mirror_x else '右'} y→{'下' if mirror_y else '上'}]"),
         (215, 215, 215), 15),
        (f"定位 {'正常' if loc_ok else '不可用'}", (120, 220, 120) if loc_ok else (120, 120, 240), 15),
        (f"机构串口 {'正常' if launcher_ok else '未连接'}", (120, 220, 120) if launcher_ok else (120, 120, 240), 15),
        (f"点云点数 {0 if cloud_xyz is None else len(cloud_xyz)}"
         + ("" if view == "robot" else
            ("　✓已变换到场地系" if odom_tf is not None
             else "　⚠️未变换(缺 field←camera_init TF)")), (150, 150, 150), 15),
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
            "dets": [], "dets_stamp": 0.0,
            # 雷达找球（界面可调；默认关，保持与改动前一致）
            "odom": None,
            "lb_enabled": False, "lb_mode": "fuse", "lb_cands": [],
            "lb_stats": {}, "lb_ms": 0.0, "lb_cfg": {}, "cloud": None, "cam": None, "cam_stamp": 0.0,
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

        from rb_msgs.srv import SerialText
        self.cli_text = self.node.create_client(
            SerialText, t.get("launch_text", "/rb_launcher/text"))
        self.cli_mission = self.node.create_client(SetMission, t.get("set_mission", "/rb_mission/set_mission"))
        self.cli_goto = self.node.create_client(GotoPose, t.get("goto_service", "/rb_mission/goto_pose"))
        self.cli_launch = self.node.create_client(Launch, t.get("launch_service", "/rb_launcher/launch"))
        self.pub_cmd = self.node.create_publisher(Twist, t.get("cmd_vel", "/cmd_vel"), 10)
        self.pub_goal = self.node.create_publisher(PoseStamped, t.get("goal_pose", "/goal_pose"), 5)

        # tf2：点云要从 camera_init 变换到 field 才能和位姿对齐
        # （定位节点发 field→camera_init，是刚性变换，不是 EKF 位姿）
        self._tf_buffer = None
        self._tf_listener = None
        try:
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self.node)
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().warn(f"tf2 不可用，点云将不做坐标变换: {e}")
        self.odom_frame_name = str(
            ((cfg.get("localization") or {}).get("odom_frame")) or "camera_init")
        self.field_frame_name = str(
            ((cfg.get("localization") or {}).get("field_frame")) or "field")

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
        pp = m.pose.pose
        q = pp.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.state["odom_delay"] = now - st
            self.state["odom_x"] = float(pp.position.x)
            # ⚠️ 雷达找球要把 camera_init 系的点云按【里程计】转到车体系，
            #    所以必须存完整的 (x, y, yaw)，不能只存 x。
            self.state["odom"] = (float(pp.position.x), float(pp.position.y), float(yaw))

    def _on_lok(self, m):
        with self.lock:
            self.state["launcher_ok"] = bool(m.data)

    def _on_lstatus(self, m):
        with self.lock:
            self.state["launcher_status"] = m.data

    def _on_dets(self, m):
        with self.lock:
            self.state["dets"] = list(m.detections)
            self.state["dets_stamp"] = time.time()

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

    def launcher_text(self, command: str):
        """发一行文本指令（STOP / ZERO / MOVE 1 90 5）。这台机构的真实协议。"""
        from rb_msgs.srv import SerialText
        req = SerialText.Request()
        req.command = str(command)
        return self._call(self.cli_text, req)

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

    def load_lidar_ball_cfg(self) -> None:
        """从 perception.yaml 读 lidar_ball 默认参数（读不到就用模块默认值）。"""
        try:
            import yaml as _y
            f = (WS / "src/rb_perception/config/perception.yaml")
            cfg = ((_y.safe_load(f.read_text(encoding="utf-8")) or {})
                   .get("perception") or {}).get("lidar_ball") or {}
        except Exception:  # noqa: BLE001
            cfg = {}
        from rb_perception.lidar_ball import DEFAULT_CFG
        d = dict(DEFAULT_CFG)
        d.update({k: v for k, v in cfg.items() if v is not None})
        with self.lock:
            self.state["lb_cfg"] = d
            self.state["lb_enabled"] = bool(d.get("enabled", False))
            self.state["lb_mode"] = str(d.get("mode", "fuse"))

    def lidar_ball_candidates(self):
        """用车体系点云找地面球候选（界面调的参数优先）。返回 (cands, 耗时ms)。"""
        if not _LIDAR_BALL_OK:
            return [], 0.0
        with self.lock:
            cfg = dict(self.state["lb_cfg"])
            enabled = bool(self.state["lb_enabled"])
            cloud = None if self.state["cloud"] is None else self.state["cloud"]
            odom = self.state["odom"]
        if not enabled or cloud is None or odom is None:
            return [], 0.0
        t0 = time.time()
        try:
            body = body_frame_cloud(cloud, odom)
            cands = find_ball_candidates(body, cfg)
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().warn(f"雷达找球失败: {e}")
            return [], 0.0
        return cands, (time.time() - t0) * 1000.0

    def odom_to_field_tf(self):
        """返回 (tx, ty, theta)：camera_init 原点在 field 下的位置与相对朝向。

        点云乘这个变换才能和 /localization/pose 对上。查不到返回 None。
        """
        if self._tf_buffer is None:
            return None
        try:
            import rclpy.time
            tr = self._tf_buffer.lookup_transform(
                self.field_frame_name, self.odom_frame_name, rclpy.time.Time())
            q = tr.transform.rotation
            th = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (float(tr.transform.translation.x),
                    float(tr.transform.translation.y), float(th))
        except Exception:  # noqa: BLE001
            return None

    def body_to_field_xy(self, bx: float, by: float):
        """车体系 (前,左) → 场地系 (x,y)。

        用当前位姿：p_field = p_robot + R(yaw)·(bx, by)，+y 是左（ROS 标准）。
        """
        with self.lock:
            pose = self.state["pose"] or {}
        try:
            cx, cy = float(pose.get("x", 0.0)), float(pose.get("y", 0.0))
            yaw = float(pose.get("yaw", 0.0))
        except Exception:  # noqa: BLE001
            return [bx, by]
        c, s = math.cos(yaw), math.sin(yaw)
        return [cx + c * bx - s * by, cy + s * bx + c * by]

    def lidar_ball_vs_camera(self, cands=None) -> dict:
        """雷达候选与相机球检测的配对统计（多近算同一颗：match_distance_m）。

        cands 由调用方传入（场地系坐标的候选列表）—— 不要在这里读 state["lb_cands"]，
        那个字段正是调用方要写的，会拿到上一轮的值。
        """
        with self.lock:
            if cands is None:
                cands = list(self.state["lb_cands"]) if self.state.get("lb_cands") else []
            dets = list(self.state["dets"])
            pose = self.state["pose"] or {}
            match_d = float(self.state["lb_cfg"].get("match_distance_m", 0.7))
        if not cands:
            return {}
        try:
            yaw = float(pose.get("yaw", 0.0))
            cx0, cy0 = float(pose.get("x", 0.0)), float(pose.get("y", 0.0))
        except Exception:  # noqa: BLE001
            yaw, cx0, cy0 = 0.0, 0.0, 0.0
        balls = [d for d in dets if str(getattr(d, "label", "")).startswith("ball")]
        matched = 0
        for d in balls:
            b = float(getattr(d, "bearing_rad", float("nan")))
            dist = float(getattr(d, "distance_m", float("nan")))
            if not (math.isfinite(b) and math.isfinite(dist)):
                continue
            slant = dist / max(math.cos(b), 1e-3)
            bx, by = slant * math.cos(b), -slant * math.sin(b)   # 车体系
            c, s = math.cos(yaw), math.sin(yaw)
            fx, fy = cx0 + c * bx - s * by, cy0 + s * bx + c * by
            for cd in cands:
                f = cd.get("field") or [0.0, 0.0]
                if math.hypot(fx - f[0], fy - f[1]) < match_d:
                    matched += 1
                    break
        return {"camera_balls": len(balls), "lidar_cands": len(cands),
                "matched": matched, "lidar_only": len(cands) - matched,
                "camera_only": len(balls) - matched}

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
            # 检测过期就清空 —— 否则感知停了之后旧列表一直挂在页面上
            # （现场现象："雷达没开怎么还有 rack_ring"）
            if time.time() - float(s.get("dets_stamp") or 0.0) > 1.5:
                s["dets"] = []
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

    # 一键启动要等的关键进程（用进程名判断：比 ros2 node list 快得多；
    # pgrep -x 精确匹配进程名，不会匹配到我们自己 ✓）
    # 注意：这里可以写全名，_proc_running() 会按 comm 的 15 字符上限截断 ✓
    # ⚠️ 这里填【进程名】，不是 ROS 节点名 —— 两者不一样：
    #    后摄和后感知用的是【和前摄同一个可执行文件】camera_node / perception_node，
    #    只是 ROS 节点名不同（rb_camera_rear / rb_perception_rear）。
    #    曾经写成 camera_rear_node / perception_rear_node → pgrep 永远找不到
    #    → "一键启动"一直误报"缺少节点"，而后摄其实跑得好好的 ✗。
    FLOW_NODES = ("rb_chassis_node", "mission_node", "localization_node",
                  "camera_node", "perception_node")

    def startup_flow(self, stack: "StackManager"):
        """一键启动（极简版）：起 boot.sh → 等关键进程出现 → 完事。

        ⚠️ 原实现叠了三层，每层都脆：
          ① 翻日志文本找 "底盘已使能" / "mission 就绪" 两个字符串
             —— 日志一改措辞就永远等不到 ✗
          ② 超时 120 秒 —— 出问题时白等两分钟 ✗
          ③ 再调 tools/check_ready.sh 并解析它的输出 —— 多一个脚本+格式 ✗
        现在：一个判据（关键进程在不在）、一个超时（45s）、不调任何脚本。
        要看更细的检查点旁边的「✅ 就绪检查」——那是独立功能，不塞进一键启动。
        """
        with self.lock:
            if self.state["flow"] == "running":
                return False, "一键启动已在跑"
            self.state["flow"] = "running"
            self.state["flow_step"] = "启动整套"

        def worker():
            missing = []
            ok = False
            msg = ""
            try:
                ok, msg = stack.start()
                self.log.append((time.time(), "TOOL", "一键启动", f"boot.sh: {msg}"))
                if ok:
                    deadline = time.time() + 45
                    missing = list(self.FLOW_NODES)
                    while time.time() < deadline:
                        missing = [n for n in self.FLOW_NODES if not _proc_running(n)]
                        if not missing or not stack.running:
                            break
                        time.sleep(1.0)
                step = ("可以开车了" if (ok and not missing)
                        else ("boot.sh: " + msg if not ok
                              else "缺少节点: " + ", ".join(missing)))
            except Exception as e:  # noqa: BLE001
                step = f"{type(e).__name__}: {e}"
                ok = False
            with self.lock:
                self.state["flow"] = "done" if (ok and not missing) else "notready"
                self.state["flow_step"] = step

        threading.Thread(target=worker, daemon=True).start()
        return True, "一键启动已开始（boot.sh → 等关键进程）"


# ─────────────────────────────────────────────────────────────────────────────
# 底盘使能：CAN 检查 → 恢复 → 起底盘节点 → 等"已使能 4 个电机"
# ─────────────────────────────────────────────────────────────────────────────
PCAN_VIDPID = "0c72:000c"


def _proc_running(name: str) -> bool:
    """进程在不在。

    ⚠️ 两个坑都踩过，所以这里必须写得明确：
      ① 不能用 `pgrep -f`（匹配整条命令行）—— 它会匹配到【我们自己】，
         因为调用方的命令行里就写着这个名字。这个坑反复出现过 ✗。
      ② Linux 的进程名（/proc/<pid>/comm）【最多 15 个字符】，更长的会被截断：
           rb_launcher_node       → rb_launcher_nod
           localization_node      → localization_no
           livox_ros_driver2_node → livox_ros_drive
         拿全名去 `pgrep -x` 永远匹配不上，于是"一键启动"会一直报
         "缺少节点: localization_node" ✗（实测就是这个现象）。
         这里按 comm 的实际上限截断后再精确匹配。
    """
    try:
        return subprocess.run(["pgrep", "-x", name[:15]],
                              capture_output=True, timeout=3).returncode == 0
    except Exception:  # noqa: BLE001
        return False


def probe_can():
    """CAN 链路的裸查结果（不依赖 ROS）。"""
    def sh(cmd):
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=4).stdout
        except Exception:  # noqa: BLE001
            return ""

    usb = sh(["lsusb"])
    pcan = PCAN_VIDPID in usb
    link = sh(["ip", "-br", "link", "show", "can0"]).strip()
    up = "UP" in link
    drops = 0
    try:
        log = Path("/var/log/kern.log")
        if log.is_file():
            drops = log.read_text(errors="replace").count("USB disconnect")
    except Exception:  # noqa: BLE001
        pass
    return {"pcan_present": pcan, "can0_up": up, "can0_line": link,
            "usb_disconnects": drops, "vidpid": PCAN_VIDPID}


class MissionManager:
    """单独起/停 rb_mission。

    ⚠️ 为什么需要它：寻球/绕场脚本会 SIGTERM 掉 mission 节点来接管 /cmd_vel，
    跑完不会自己回来，之后所有 GOTO / 起任务都报"服务不可用"。
    """

    def __init__(self, log_deque):
        self.log = log_deque
        self.proc: subprocess.Popen | None = None
        self.lock = threading.Lock()

    @property
    def running(self) -> bool:
        # ⚠️ 不能只看 self.proc：boot.sh / ros2 launch 起的 mission 节点
        #    不归这里管，不看进程就会起【重复节点】（实测 /rb_mission 出现两个，
        #    两个节点抢 /cmd_vel，行为不可预测）。
        if self.proc is not None and self.proc.poll() is None:
            return True
        return self._external_running()

    @staticmethod
    def _external_running() -> bool:
        """按可执行名找 mission_node（不用 -f，避免匹配到自己的命令行）。"""
        try:
            out = subprocess.run(["ps", "-eo", "comm"], capture_output=True,
                                 text=True, timeout=4).stdout
            return any(l.strip() == "mission_node" for l in out.splitlines())
        except Exception:  # noqa: BLE001
            return False

    def _pump(self, proc):
        for raw in iter(proc.stdout.readline, b""):
            try:
                self.log.append((time.time(), "MISSION", "rb_mission",
                                 raw.decode("utf-8", "replace").rstrip()))
            except Exception:  # noqa: BLE001
                pass
        self.log.append((time.time(), "MISSION", "rb_mission",
                         f"— 任务节点退出 code={proc.poll()} —"))

    def start(self, wait_s: float = 6.0):
        with self.lock:
            if self.running:
                return True, "任务节点已经在跑"
        cfg = WS / "install/rb_mission/share/rb_mission/config/mission.yaml"
        if not cfg.is_file():
            cfg = WS / "src/rb_mission/config/mission.yaml"
        # ⚠️ 必须 -p config_file:=（节点自己读 yaml），不能 --params-file
        cmd = ["bash", "-c",
               "source /opt/ros/humble/setup.bash && "
               f"source {WS}/install/setup.bash && "
               "exec ros2 run rb_mission mission_node --ros-args "
               "-r __node:=rb_mission "
               f"-p config_file:={cfg}"]
        self.log.append((time.time(), "MISSION", "rb_mission", "=== 启动任务节点 ==="))
        try:
            self.proc = subprocess.Popen(cmd, cwd=str(WS), stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, start_new_session=True)
        except Exception as e:  # noqa: BLE001
            return False, f"启动失败: {e}"
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
        time.sleep(max(0.5, wait_s))          # 等 DDS 把服务挂上去
        if not self.running:
            return False, "任务节点起来就退了（看日志）"
        return True, "任务节点已启动（等 1~2 秒服务才可见）"

    def stop(self):
        with self.lock:
            if not self.running:
                return False, "任务节点没在跑"
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except Exception:  # noqa: BLE001
                pass
        return True, "已发送停止"

    def snapshot(self):
        ext = self._external_running()
        return {"running": self.running, "managed": self.proc is not None
                and self.proc.poll() is None, "external": ext}


class ChassisManager:
    """单独起/停 rb_chassis，并盯日志确认"电机已使能"。"""

    def __init__(self, log_deque: deque, checks: "SystemChecks"):
        self.log = log_deque
        self.checks = checks
        self.proc: subprocess.Popen | None = None
        self.lock = threading.Lock()
        self.can_line = ""          # 底盘日志里那句 "CAN 已启动"
        self.enabled_line = ""      # "底盘已使能 4 个电机"
        self.state = {"flow": "idle", "step": ""}

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def _pump(self, proc):
        for raw in iter(proc.stdout.readline, b""):
            try:
                line = raw.decode("utf-8", "replace").rstrip()
            except Exception:  # noqa: BLE001
                continue
            with self.lock:
                if "CAN 已启动" in line:
                    self.can_line = line.split("]", 1)[-1].strip()
                if "底盘已使能" in line:
                    self.enabled_line = line.split("]", 1)[-1].strip()
            self.log.append((time.time(), "CHASSIS", "rb_chassis", line))
        self.log.append((time.time(), "CHASSIS", "rb_chassis",
                         f"— 底盘节点退出 code={proc.poll()} —"))

    def start(self, wait_s: float = 25.0):
        """起底盘并等"已使能"。返回 (ok, msg)。"""
        with self.lock:
            if self.running:
                return bool(self.enabled_line), (
                    self.enabled_line or "底盘节点已在跑，但还没看到使能日志")
            self.can_line = ""
            self.enabled_line = ""
            self.state.update(flow="running", step="检查 CAN")

        # ① CAN 链路
        can = probe_can()
        if not can["pcan_present"]:
            n = can["usb_disconnects"]
            msg = (f"lsusb 里没有 PEAK/XCAN 适配器（{PCAN_VIDPID}）—— "
                   f"适配器没插好或掉线了（本次开机 USB 掉线 {n} 次）")
            self.log.append((time.time(), "ERROR", "CAN", msg))
            with self.lock:
                self.state.update(flow="failed", step="CAN 适配器不在")
            return False, msg
        if not can["can0_up"]:
            self.log.append((time.time(), "CHASSIS", "CAN",
                             "can0 没 UP → 跑 tools/can_recover.sh"))
            with self.lock:
                self.state["step"] = "恢复 can0"
            self.checks._run("CAN 恢复", "tools/can_recover.sh", timeout=60.0)
            can = probe_can()
            if not can["can0_up"]:
                with self.lock:
                    self.state.update(flow="failed", step="can0 起不来")
                return False, "can0 拉不起来（看日志里的 can_recover 输出）"

        # ② 起底盘
        with self.lock:
            self.state["step"] = "启动底盘节点"
        self.log.append((time.time(), "CHASSIS", "rb_chassis",
                         "=== 启动 rb_chassis（真下发 CAN）==="))
        cmd = ["bash", "-c",
               "source /opt/ros/humble/setup.bash && "
               f"source {WS}/install/setup.bash && "
               "exec ros2 launch rb_chassis chassis.launch.py"]
        self.proc = subprocess.Popen(cmd, cwd=str(WS), stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()

        # ③ 等使能日志
        deadline = time.time() + wait_s
        while time.time() < deadline:
            with self.lock:
                if self.enabled_line:
                    self.state.update(flow="done", step=self.enabled_line)
                    return True, self.enabled_line
                if not self.running:
                    self.state.update(flow="failed", step="底盘节点退出了")
                    return False, "底盘节点启动后就退出了（看日志）"
            time.sleep(0.4)
        with self.lock:
            self.state.update(flow="failed", step="等使能日志超时")
        return False, (f"{wait_s:.0f}s 内没看到「底盘已使能」—— "
                       "多半是 CAN 通了但电机板没响应（板子锁了？断电 10 秒重上电）")

    def stop(self):
        if not self.running:
            return False, "底盘节点没在跑"
        with self.lock:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except Exception:  # noqa: BLE001
                pass
            self.state.update(flow="idle", step="已停止")
        self.log.append((time.time(), "CHASSIS", "rb_chassis",
                         "=== 已发送停止（SIGINT，会先发零速）==="))
        return True, "已发送 SIGINT"

    def snapshot(self):
        with self.lock:
            return {"running": self.running, "flow": self.state["flow"],
                    "step": self.state["step"], "enabled_line": self.enabled_line,
                    "can_line": self.can_line, **probe_can()}


# ─────────────────────────────────────────────────────────────────────────────
# 调试脚本运行器（寻球/绕场等整车测试）
# ─────────────────────────────────────────────────────────────────────────────
# 这些脚本会**接管 /cmd_vel**（用 --takeover 让 mission 节点退出），
# 所以页面上点之前必须确认：任务停了、急停手段就绪。
DEBUG_TOOLS = {
    "TEST_BALL": {
        "script": "tools/test_perimeter_ball.py",
        "args": ["--skip-perimeter", "--takeover", "--yes"],
        "label": "寻球 → 走到球前 0.60m → 转身背对球",
    },
    "TEST_PERIMETER": {
        "script": "tools/test_perimeter_ball.py",
        "args": ["--takeover", "--yes"],
        "label": "绕场一圈 → 寻球 → 走到球前 → 转身背对球",
    },
    "TEST_GRAB_PLACE": {
        "script": "tools/test_grab_place.py",
        "args": [],
        "label": "抓球并放下（机构序列）",
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

    def start(self, key: str, extra_args=None):
        spec = DEBUG_TOOLS.get(key)
        if not spec:
            return False, f"未知调试脚本 {key}"
        with self.lock:
            if self.running:
                return False, f"{self.name} 还在跑，先停掉它"
            if not (WS / spec["script"]).is_file():
                return False, f"找不到 {spec['script']}"
            # 用显式的 env，避免控制台是别的方式起的导致子进程缺 ROS 环境。
            #
            # ⚠️ 必须【显式 export ROS_DOMAIN_ID】，不能只靠继承。
            #    实测四组对照（--skip-perimeter --dry，其余条件相同）：
            #      无 DOMAIN_ID + 无 lidar ws → 读不到 /localization/pose ✗
            #      有 DOMAIN_ID + 无 lidar ws → 全部通过 ✓
            #      无 DOMAIN_ID + 有 lidar ws → 读不到 /localization/pose ✗
            #      有 DOMAIN_ID + 有 lidar ws → 全部通过 ✓
            #    唯一决定因素就是 ROS_DOMAIN_ID。若控制台不是经 console.sh/rbws
            #    起（那两处才会 export 2），子进程就跑在 domain 0 上，现象是
            #    "节点都在、但一个话题都收不到"，极难排查 ✗。这里自己设死。
            #
            # 顺带 source 雷达工作空间：脚本若要用 livox 的 CustomMsg
            # （ros2 topic hz 之类）不加会认不出类型。当前用不到，但和
            # check_ready.sh 保持一致，省得以后踩。
            lidar_ws = Path.home() / "lidar_360"
            cmd = ["bash", "-c",
                   "source /opt/ros/humble/setup.bash && "
                   f"source {lidar_ws}/ws_livox/install/setup.bash 2>/dev/null; "
                   f"source {lidar_ws}/fast_prop_ws/install/setup.bash 2>/dev/null; "
                   f"source {WS}/install/setup.bash && "
                   "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-2} && "
                   f"exec python3 {spec['script']} " +
                   " ".join(list(spec["args"]) + list(extra_args or []))]
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
             checks: SystemChecks, chassis: ChassisManager,
             mission: MissionManager, field: dict):
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
        s["chassis"] = chassis.snapshot()
        s["mission_node"] = mission.snapshot()
        s["odom_tf_ok"] = bridge.odom_to_field_tf() is not None
        # 雷达找球：算一次，地图和面板共用；候选在【车体系】，
        # 这里顺手转成场地系供地图直接画
        # ⚠️ 这几个调用内部都会拿 bridge.lock —— 绝不能放在 `with bridge.lock` 里面，
        #    那是同一把非重入锁，会直接死锁（现场现象：/api/state 请求永远不返回）。
        cands, ms = bridge.lidar_ball_candidates()
        lb_list = [
            dict(c, **{k: v for k, v in candidate_to_repr(c).items()},
                 field=bridge.body_to_field_xy(c["x"], c["y"]))
            for c in cands]
        lb_stats = bridge.lidar_ball_vs_camera(lb_list)
        with bridge.lock:
            s["lb_cands"] = lb_list
            s["lb_ms"] = round(ms, 1)
            s["lb_enabled"] = bridge.state["lb_enabled"]
            s["lb_mode"] = bridge.state["lb_mode"]
            s["lb_cfg"] = dict(bridge.state["lb_cfg"])
            s["lb_stats"] = lb_stats
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
        mirror_y = (request.args.get("mirror_y") or "0") in ("1", "true", "yes")

        def gen():
            while True:
                with bridge.lock:
                    xyz = None if bridge.state["cloud"] is None else bridge.state["cloud"].copy()
                    pose = bridge.state["pose"]
                    dets = list(bridge.state["dets"])
                    lok = bridge.state["launcher_ok"]
                    ok = bridge.state["loc_ok"]
                odom_tf = bridge.odom_to_field_tf()
                _lc, _ = bridge.lidar_ball_candidates()
                lb_cands = [dict(c, **{k: v for k, v in candidate_to_repr(c).items()},
                                 field=bridge.body_to_field_xy(c["x"], c["y"]))
                            for c in _lc]
                jpg = render_lidar(xyz, pose, dets, field, lok, ok,
                                   view=view, mirror_x=mirror, mirror_y=mirror_y,
                                   odom_tf=odom_tf, lb_cands=lb_cands)
                if jpg:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                           b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                           + jpg + b"\r\n")
                # 0.1s = 10fps：原来 0.2s 时切换视角要等好几帧才跟手
                time.sleep(0.1)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/api/action", methods=["POST"])
    def api_action():
        d = request.get_json(force=True, silent=True) or {}
        name = (d.get("name") or "").strip()
        args = d.get("args") or {}
        try:
            if name in ("PASS", "SHOOT", "IDLE"):
                return jsonify(dict(zip(("ok", "msg"), bridge.set_mission(name))))
            # ⚠️ 起任务：后端一直有 set_mission 接口和客户端，但分发里没接分支 ——
            #    页面按「传球/投篮/停止任务」是【静默无反应】。上车前审计才发现。
            if name in ("PASS", "SHOOT", "IDLE"):
                ok, msg = bridge.set_mission(name)
                return jsonify({"ok": bool(ok), "msg": msg})
            if name == "PANIC":
                return jsonify(dict(zip(("ok", "msg"), panic())))
            if name == "LAUNCH_TEXT":
                # 同样兼容两种请求体（见下面 LAUNCH 的注释）
                src = args if args else (request.get_json(silent=True) or {})
                return jsonify(dict(zip(("ok", "msg"),
                                        bridge.launcher_text(
                                            str(src.get("command") or "").strip()))))
            if name == "LAUNCH_INIT":
                # 现场要的开机顺序：先 STOP，再 ZERO 初始化
                seq = [("STOP", 0.25), ("ZERO", 0.6)]
                out = []
                allok = True
                for cmd, wait in seq:
                    ok, msg = bridge.launcher_text(cmd)
                    out.append(f"{cmd}: {'✓' if ok else '✗'} {msg}")
                    allok = allok and ok
                    if not ok:
                        break
                    time.sleep(wait)
                return jsonify({"ok": allok, "msg": " / ".join(out)})
            if name == "LAUNCH_GRAB":
                ok, msg = bridge.launcher_text("MOVE 1 90 5")
                return jsonify({"ok": ok, "msg": f"抓取 MOVE 1 90 5 → {msg}"})
            if name == "LAUNCH_PLACE":
                ok, msg = bridge.launcher_text("MOVE 1 0 5")
                return jsonify({"ok": ok, "msg": f"放下 MOVE 1 0 5 → {msg}"})
            if name == "LAUNCH":
                # ⚠️ 兼容两种请求体：前端发 {"name":..., "args":{...}}，
                #    手工 curl 常直接发 {"name":..., "action":7}。
                #    之前只认 args，用手工请求测时 action 会静默取默认值 0（=SHOOT），
                #    看起来像"按什么键都发射"。
                src = args if args else (request.get_json(silent=True) or {})
                return jsonify(dict(zip(("ok", "msg"), bridge.launcher(
                    int(src.get("action", 0)), int(src.get("speed", 0)),
                    int(src.get("angle", 0))))))
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
            if name == "LIDAR_BALL":
                body = request.get_json(silent=True) or {}
                with bridge.lock:
                    if "enabled" in body:
                        bridge.state["lb_enabled"] = bool(body["enabled"])
                    if body.get("mode") in ("camera", "lidar", "fuse"):
                        bridge.state["lb_mode"] = body["mode"]
                        bridge.state["lb_cfg"]["mode"] = body["mode"]
                    # 数值参数白名单（别让界面写进任意键）
                    for k in ("z_min", "z_max", "ball_radius_m", "radius_tolerance_m",
                              "cluster_tolerance_m", "min_points", "max_range_m",
                              "match_distance_m", "floor_band_m"):
                        if k in body and body[k] is not None:
                            try:
                                bridge.state["lb_cfg"][k] = (
                                    int(body[k]) if k == "min_points" else float(body[k]))
                            except (TypeError, ValueError):
                                pass
                    if "remove_floor" in body:
                        bridge.state["lb_cfg"]["remove_floor"] = bool(body["remove_floor"])
                return jsonify({"ok": True, "msg": "已更新雷达找球参数",
                                "cfg": bridge.state["lb_cfg"]})
            if name == "LIDAR_BALL_RESET":
                bridge.load_lidar_ball_cfg()
                return jsonify({"ok": True, "msg": "已从 perception.yaml 重新读取默认值"})
            if name == "GRAB_PLACE":
                body = request.get_json(silent=True) or {}
                a = ["--preset", str(body.get("preset") or "grab_place")]
                if body.get("dry_run"):
                    a.append("--dry-run")
                    a.append("--no-countdown")
                for k, flag in (("speed", "--speed"), ("angle", "--angle"),
                                ("repeats", "--repeats"), ("interval", "--interval")):
                    v = body.get(k)
                    if v is not None and str(v) != "":
                        try:
                            a += [flag, str(int(v) if k in ("speed", "angle", "repeats")
                                          else float(v))]
                        except (TypeError, ValueError):
                            pass
                # 自定义步骤：前端传 JSON 字符串，原样透传（脚本会校验动作码）
                if body.get("steps"):
                    a += ["--steps", str(body["steps"])]
                return jsonify(dict(zip(("ok", "msg"),
                                        tool.start("TEST_GRAB_PLACE", a))))
            if name == "MISSION_RESTART":
                ok, msg = mission.stop() if mission.running else (True, "")
                if ok:
                    time.sleep(1.0)
                return jsonify(dict(zip(("ok", "msg"), mission.start())))
            if name == "MISSION_START":
                return jsonify(dict(zip(("ok", "msg"), mission.start())))
            if name == "MISSION_STOP":
                return jsonify(dict(zip(("ok", "msg"), mission.stop())))
            if name == "CHASSIS_ENABLE":
                return jsonify(dict(zip(("ok", "msg"), chassis.start())))
            if name == "CHASSIS_STOP":
                return jsonify(dict(zip(("ok", "msg"), chassis.stop())))
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
    chassis = ChassisManager(logbuf, checks)
    try:
        loc_cfg = yaml.safe_load(
            (WS / "src/rb_localization/config/localization.yaml").read_text(
                encoding="utf-8")) or {}
        loc_conv = loc_cfg.get("odom_frame_conversion") or {}
        loc_out = loc_cfg.get("output") or {}
    except Exception:  # noqa: BLE001
        loc_conv, loc_out = {}, {}
    bridge = Bridge({"topics": topics, "localization": {
        # ⚠️ 名字要和定位节点实际用的完全一致，否则 TF 查不到（静默不变换）
        "odom_frame": loc_conv.get("odom_frame") or "camera_init",
        "field_frame": loc_out.get("field_frame") or "field",
    }})
    mission = MissionManager(logbuf)
    bridge.load_lidar_ball_cfg()   # 必须在 Bridge 建好之后！
    app = make_app(bridge, stack, tool, checks, chassis, mission, field)

    print(f"\n  🏀 综合控制台 http://{args.host}:{args.port}")
    print(f"     本机访问   http://127.0.0.1:{args.port}")
    print(f"     场地 {field.get('length_m')}×{field.get('width_m')} m"
          f"   起停控制 {'开' if not args.no_stack_control else '关'}")
    print("     Ctrl-C 退出\n")
    app.run(host=args.host, port=args.port, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    raise SystemExit(main())
