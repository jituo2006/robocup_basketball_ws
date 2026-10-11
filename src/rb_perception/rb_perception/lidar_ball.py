"""雷达找球（粗）+ 相机精定位（细）—— 纯逻辑部分，可单测。

现场需求："能不能用雷达扫出地面的球的位置，先确定大概位置，再用摄像头精确定位"，
并要求**保留现在的模式，做个代码开关**。

三档 mode（配置 perception.lidar_ball.mode）：
  * "camera" —— 只用相机检测，**与改动前行为完全一致**（默认，保险）
  * "lidar"  —— 只用雷达地面球候选
  * "fuse"   —— 以相机为主（类别准、横向精度高），雷达补相机看不到的球

为什么值得做：
  * 雷达不受光照影响；相机在过曝/逆光/远距离会漏检
  * 雷达能"看到"画面外的球（相机 FOV 只有约 60°，雷达 360°）
  * 雷达给的是直接测距，相机是单目测距（靠球径先验，误差大）

⚠️ 坐标约定（很容易错，这里写清楚）：
  * 雷达点云在 FAST-LIO 的 camera_init 系里 → 先按 /Odometry 变换到【车体系】
    （+x 前、+y 左、+z 上）
  * Detection.bearing_rad 是**右正左负**（相机约定），而车体系 +y 是左
    ⇒ bearing = -atan2(y_body, x_body)
  * Detection.distance_m 语义被下游当成【光轴深度 Z】用（任务侧再乘 1/cos(b) 转斜距）
    ⇒ 这里也填 Z = slant * cos(b)，保持同一条链路自洽
"""
from __future__ import annotations

import math
from typing import Iterable

import numpy as np

DEFAULT_CFG = {
    "enabled": False,          # ⚠️ 默认关 = 完全保持现在的行为
    "cloud_topic": "/cloud_registered",
    "mode": "camera",          # camera | lidar | fuse
    "z_min": -1.35,            # 车体系 z 下限（球在地面，雷达装高约 1.2m）
    "z_max": -0.75,            # 车体系 z 上限；球心离地约 0.12m
    "ball_radius_m": 0.12,     # 篮球直径约 24cm → 半径 12cm（排球 21cm）
    "radius_tolerance_m": 0.07,
    "cluster_tolerance_m": 0.09,
    "min_points": 6,
    "max_range_m": 6.0,
    "match_distance_m": 0.7,   # 相机检测与雷达候选多远算同一颗球
    "remove_floor": True,      # 必须先剔地面
    "floor_band_m": 0.05,
}


def body_frame_cloud(xyz: np.ndarray, robot_xy_yaw) -> np.ndarray:
    """把 camera_init 系的点云变换到车体系（+x 前、+y 左）。

    robot_xy_yaw = (x, y, yaw) 是**机器人在 camera_init 系**里的位姿
    （来自 /Odometry）。绕 -yaw 旋转再平移取反。
    """
    if xyz is None or len(xyz) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    p = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    rx, ry, yaw = float(robot_xy_yaw[0]), float(robot_xy_yaw[1]), float(robot_xy_yaw[2])
    c, s = math.cos(yaw), math.sin(yaw)
    dx = p[:, 0] - rx
    dy = p[:, 1] - ry
    out = np.empty_like(p)
    out[:, 0] = c * dx + s * dy      # 前
    out[:, 1] = -s * dx + c * dy     # 左
    out[:, 2] = p[:, 2]
    return out


def remove_floor(points_body: np.ndarray, band_m: float = 0.05) -> np.ndarray:
    """剔除地面点。

    ⚠️ 这一步是【必须】的：不剔地面的话，地面上的小起伏会被聚成"球形"候选，
    实测会找出成堆假球（真球反而漏掉）。雷达装得平，地面在车体系里 z 近似常数，
    所以取 z 直方图主峰当地面，±band 全删掉。
    """
    if points_body is None or len(points_body) < 20:
        return np.zeros((0, 3), dtype=np.float64)
    p = np.asarray(points_body, dtype=np.float64).reshape(-1, 3)
    hist, edges = np.histogram(p[:, 2], bins=60)
    k = int(np.argmax(hist))
    floor_z = 0.5 * (edges[k] + edges[k + 1])
    keep = np.abs(p[:, 2] - floor_z) > max(band_m, 1e-3)
    return p[keep]


def _grid_clusters(pts: np.ndarray, tol: float, min_pts: int) -> list[np.ndarray]:
    """体素网格 + BFS 连通聚类（不依赖 scipy）。"""
    if len(pts) < min_pts:
        return []
    cell = max(tol, 1e-3)
    keys = np.floor(pts[:, :3] / cell).astype(np.int64)
    buckets: dict[tuple, list[int]] = {}
    for i, k in enumerate(map(tuple, keys)):
        buckets.setdefault(k, []).append(i)

    seen: set[tuple] = set()
    clusters: list[np.ndarray] = []
    for k in list(buckets.keys()):
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            cur = stack.pop()
            comp.extend(buckets.get(cur, ()))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        nb = (cur[0] + dx, cur[1] + dy, cur[2] + dz)
                        if nb in buckets and nb not in seen:
                            seen.add(nb)
                            stack.append(nb)
        if len(comp) >= min_pts:
            clusters.append(pts[np.asarray(comp, dtype=int)])
    return clusters


def cluster_is_ball(cluster: np.ndarray, ball_radius: float, tol: float) -> dict | None:
    """判断一簇地面点像不像球，返回候选 dict 或 None。

    判据（都很朴素，但足以"先确定大概位置"）：
      * 点数够
      * 水平跨度 ≈ 2r（球被扫到的是上半弧，跨度接近直径）
      * 高度跨度 ≈ r（可见的上半部分）
      * 水平形状接近圆（长宽比接近 1）
    """
    n = len(cluster)
    if n < 4:
        return None
    c = cluster[:, :3].mean(axis=0)
    d = cluster[:, :3] - c
    # 紧致度：离均值的最大距离。球是有界球面片，这个量直接是球半径量级；
    # 而地面碎块/长条会被拉得很大 —— 比 bbox 跨度稳得多（bbox 对单个离群点敏感）。
    max_r = float(np.linalg.norm(d, axis=1).max())
    span_x = float(d[:, 0].max() - d[:, 0].min())
    span_y = float(d[:, 1].max() - d[:, 1].min())
    span_z = float(d[:, 2].max() - d[:, 2].min())
    if not (ball_radius - tol) <= max_r <= (ball_radius + tol):
        return None
    # 高度跨度不该超过球半径（扫到的只是上半球）
    if span_z > 2.0 * ball_radius + tol:
        return None
    return {
        "x": float(c[0]), "y": float(c[1]), "z": float(c[2]),
        "n": int(n), "max_r": max_r,
        "span": (span_x, span_y, span_z),
    }


def find_ball_candidates(points_body: np.ndarray, cfg: dict | None = None) -> list[dict]:
    """从车体系点云里找地面的球形候选（粗定位）。"""
    c = dict(DEFAULT_CFG)
    c.update(cfg or {})
    if points_body is None or len(points_body) == 0:
        return []
    p = np.asarray(points_body, dtype=np.float64).reshape(-1, 3)
    # ① 只看地面上那一层
    m = (p[:, 2] >= float(c["z_min"])) & (p[:, 2] <= float(c["z_max"]))
    p = p[m]
    if len(p) == 0:
        return []
    # ①' 剔除地面（不做这步会找出成堆假球，真球反而漏掉）
    if c.get("remove_floor", True):
        p = remove_floor(p, float(c.get("floor_band_m", 0.05)))
        if len(p) == 0:
            return []
    # ② 只看前向有效距离内（太远点太稀，聚不出球）
    rng = np.hypot(p[:, 0], p[:, 1])
    p = p[(rng > 0.4) & (rng <= float(c["max_range_m"]))]
    if len(p) == 0:
        return []
    # ③ 聚类 + 形状判别
    out = []
    for cl in _grid_clusters(p, float(c["cluster_tolerance_m"]), int(c["min_points"])):
        cand = cluster_is_ball(cl, float(c["ball_radius_m"]), float(c["radius_tolerance_m"]))
        if cand is not None:
            out.append(cand)
    out.sort(key=lambda d: math.hypot(d["x"], d["y"]))
    return out


def candidate_to_repr(cand: dict) -> dict:
    """把车体系候选转成 Detection 的语义字段（bearing / 光轴深度）。"""
    x, y = float(cand["x"]), float(cand["y"])
    slant = math.hypot(x, y)
    b = -math.atan2(y, x)               # 右正左负，与相机约定一致
    depth = slant * math.cos(b)         # 下游把 distance_m 当光轴深度用
    return {"bearing_rad": b, "distance_m": depth, "slant_m": slant}


def _xy_from_bearing(det) -> tuple[float, float]:
    """用 Detection 的 bearing/深度还原车体系 (x, y)，用于和雷达候选配对。"""
    b = float(getattr(det, "bearing_rad", 0.0) or 0.0)
    if not math.isfinite(b):
        return (math.nan, math.nan)
    d = float(getattr(det, "distance_m", math.nan))
    if not math.isfinite(d):
        return (math.nan, math.nan)
    slant = d / max(math.cos(b), 1e-3)
    # bearing 右正左负 ⇒ 车体系 y（左正）= -slant*sin(b)
    return (slant * math.cos(b), -slant * math.sin(b))


def fuse(camera_dets: Iterable, lidar_cands: Iterable[dict], cfg: dict | None = None):
    """按 mode 融合。

    返回 (dets, stats)。dets 里既可能是原有 Detection 对象，也可能是
    由雷达候选生成的 `LidarDetection`（鸭子类型：有 label/confidence/
    bearing_rad/distance_m/px/py 等，够任务侧直接用）。
    """
    c = dict(DEFAULT_CFG)
    c.update(cfg or {})
    mode = str(c.get("mode", "camera")).lower()
    cam = list(camera_dets or [])
    lid = list(lidar_cands or [])
    stats = {"camera": len(cam), "lidar": len(lid), "matched": 0, "lidar_only": 0,
             "mode": mode}

    if mode == "camera" or not c.get("enabled", False):
        return cam, stats          # ← 与改动前完全一致
    if mode == "lidar":
        stats["lidar_only"] = len(lid)
        return [LidarDetection(x) for x in lid], stats

    # fuse：相机为主，雷达补漏
    match_d = float(c.get("match_distance_m", 0.7))
    used = set()
    for d in cam:
        cx, cy = _xy_from_bearing(d)
        if not (math.isfinite(cx) and math.isfinite(cy)):
            continue
        best, bestd = None, match_d
        for i, x in enumerate(lid):
            if i in used:
                continue
            dd = math.hypot(cx - x["x"], cy - x["y"])
            if dd < bestd:
                best, bestd = i, dd
        if best is not None:
            used.add(best)
            stats["matched"] += 1
    extra = [LidarDetection(x) for i, x in enumerate(lid) if i not in used]
    stats["lidar_only"] = len(extra)
    return cam + extra, stats


class LidarDetection:
    """雷达候选伪装成 Detection（只实现下游真正用到的字段）。

    ⚠️ 没有 px/py/bbox —— 雷达没有图像坐标。任务侧只按 label + bearing +
    distance 用，所以够；但任何按图像坐标用的地方必须先判 `source`。
    """

    source = "lidar"

    def __init__(self, cand: dict, label: str = "ball_unknown"):
        r = candidate_to_repr(cand)
        self.label = label
        self.confidence = 0.5           # 雷达没有"置信度"，给个中值
        self.bearing_rad = r["bearing_rad"]
        self.distance_m = r["distance_m"]
        self.diameter_m = float("nan")
        self.px = float("nan")
        self.py = float("nan")
        self.bbox_px_w = 0.0
        self.bbox_px_h = 0.0
        self.x_min = self.y_min = self.x_max = self.y_max = float("nan")
        self.cx = self.cy = self.width = self.height = float("nan")
        # ⚠️ 还要有 bbox：感知节点的 to_msg() 会读 d.bbox 拼消息，
        #    缺了会 AttributeError（和"不可迭代"是同一类错：接口没对齐）。
        self.bbox = (float("nan"), float("nan"), float("nan"), float("nan"))
        self._cand = cand
