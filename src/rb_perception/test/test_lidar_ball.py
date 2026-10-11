"""雷达找球（粗）+ 相机精定位（细）的逻辑测试。

现场需求："能不能用雷达扫出地面的球的位置，先确定大概位置，再用摄像头精确定位"，
并要求保留现有模式、做代码开关。
"""
import math

import numpy as np
import pytest

from rb_perception.lidar_ball import (LidarDetection, body_frame_cloud,
                                      candidate_to_repr, cluster_is_ball, fuse,
                                      find_ball_candidates, remove_floor)


def _scene(balls, floor_z=-1.20, ball_r=0.12, seed=1, dense_floor=True):
    """密集地面 + 若干球。球心离地 = 半径（球放在地上）。"""
    rng = np.random.default_rng(seed)
    if dense_floor:
        gx, gy = np.meshgrid(np.arange(0.5, 6, 0.04), np.arange(-4, 4, 0.04))
        floor = np.column_stack([gx.ravel(), gy.ravel(),
                                 np.full(gx.size, floor_z) + rng.normal(0, 0.008, gx.size)])
    else:
        floor = np.zeros((0, 3))
    pts = [floor]
    n = 300
    th = rng.uniform(0, math.pi / 2, n)
    ph = rng.uniform(0, 2 * math.pi, n)
    for (bx, by) in balls:
        pts.append(np.column_stack([
            bx + ball_r * np.cos(ph) * np.sin(th),
            by + ball_r * np.sin(ph) * np.sin(th),
            floor_z + ball_r + ball_r * np.cos(th)]))
    return np.vstack(pts)


# ── 坐标变换 ──────────────────────────────────────────────────────────────
def test_body_frame_identity():
    pts = np.array([[1.0, 2.0, 0.0], [3.0, -1.0, 0.5]])
    out = body_frame_cloud(pts, (0.0, 0.0, 0.0))
    assert np.allclose(out, pts)


def test_body_frame_rotates_and_translates():
    """车在 (1,1) 朝 +x 时，它正前方 2m 的点应变到车体系 (2,0)。"""
    pts = np.array([[3.0, 1.0, 0.0]])
    out = body_frame_cloud(pts, (1.0, 1.0, 0.0))
    assert out[0][0] == pytest.approx(2.0)
    assert out[0][1] == pytest.approx(0.0)


def test_body_frame_yaw_90_maps_ahead_to_left():
    """车 yaw=90°（朝 +y）时，其"正前方"在 camera_init 里是 (0,1) 方向。"""
    pts = np.array([[1.0, 3.0, 0.0]])      # 车在(1,1)，前方 2m 沿 +y
    out = body_frame_cloud(pts, (1.0, 1.0, math.pi / 2))
    assert out[0][0] == pytest.approx(2.0, abs=1e-9)
    assert out[0][1] == pytest.approx(0.0, abs=1e-9)


# ── 地面剔除（最关键的一步）─────────────────────────────────────────────
def test_remove_floor_kills_the_ground_plane():
    p = _scene([], dense_floor=True)
    kept = remove_floor(p)
    assert len(kept) == 0, f"纯地面应被全部剔除，实际剩 {len(kept)}"


def test_dense_floor_produces_no_false_balls():
    """2.7 万个地面点里只有 1 颗球 → 必须恰好找到 1 个，不能有假球。

    两道防线：remove_floor 先按 z 直方图主峰剔掉地面（主要是**提速**：
    27600 点 → 几百点），cluster_is_ball 的紧致度判据再兜底。
    实测两道都在时 0 假球；只靠紧致度也不出假球，但聚类会慢很多。
    """
    p = _scene([(2.0, -0.5)])
    for cfg in ({"enabled": True, "remove_floor": True},
                {"enabled": True, "remove_floor": False}):
        got = find_ball_candidates(p, cfg)
        assert len(got) == 1, \
            f"remove_floor={cfg['remove_floor']} 时应恰好 1 个球，实际 {len(got)}"


def test_remove_floor_cuts_point_count_for_speed():
    """剔地面必须大幅减少点数（聚类是 O(n) 体素 BFS，点数决定耗时）。"""
    p = _scene([(2.0, -0.5)])
    assert len(p) > 20000
    assert len(remove_floor(p)) < len(p) * 0.05, "剔除后剩下的应不到 5%"


# ── 找球精度 ─────────────────────────────────────────────────────────────
def test_finds_single_ball_accurately():
    p = _scene([(2.0, -0.5)])
    c = find_ball_candidates(p, {"enabled": True})
    assert len(c) == 1
    assert math.hypot(c[0]["x"] - 2.0, c[0]["y"] + 0.5) < 0.05, "位置误差应 < 5cm"


def test_finds_two_balls():
    p = _scene([(2.0, -0.5), (3.5, 1.8)])
    c = find_ball_candidates(p, {"enabled": True})
    assert len(c) == 2
    got = sorted((round(x["x"], 1), round(x["y"], 1)) for x in c)
    assert got == [(2.0, -0.5), (3.5, 1.8)]


def test_ball_outside_height_window_is_ignored():
    """悬空的球（不在 z 窗口内）不该被地面找球找到。"""
    p = _scene([(2.0, -0.5)])
    c = find_ball_candidates(p, {"enabled": True, "z_min": -0.5, "z_max": 0.5})
    assert c == []


# ── bearing 约定 ─────────────────────────────────────────────────────────
def test_bearing_is_right_positive():
    """球在车右前方 → bearing 必须为【负】？不：右正是正。"""
    left = candidate_to_repr({"x": 2.0, "y": 1.0})     # 左前（y>0 是左）
    right = candidate_to_repr({"x": 2.0, "y": -1.0})   # 右前
    assert left["bearing_rad"] < 0, "左侧目标 bearing 应为负（相机约定右正左负）"
    assert right["bearing_rad"] > 0, "右侧目标 bearing 应为正"
    assert left["slant_m"] == pytest.approx(math.hypot(2.0, 1.0))


def test_distance_is_optical_axis_depth():
    """distance_m 要填【光轴深度】= 斜距*cos(b)，与相机链路语义一致。"""
    r = candidate_to_repr({"x": 2.0, "y": 1.0})
    assert r["distance_m"] == pytest.approx(r["slant_m"] * math.cos(r["bearing_rad"]))


# ── 开关行为（用户明确要求"保留现在的模式"）────────────────────────────
class _FakeDet:
    source = "camera"

    def __init__(self, b, d):
        self.bearing_rad, self.distance_m, self.label = b, d, "ball_basketball"


def _cam_det_at(x, y):
    b = -math.atan2(y, x)
    return _FakeDet(b, math.hypot(x, y) * math.cos(b))


def test_disabled_is_byte_identical_to_old_behaviour():
    """enabled=false 时只返回相机检测 —— 与改动前完全一致。"""
    cam = [_cam_det_at(2.0, -0.5)]
    cands = find_ball_candidates(_scene([(2.0, -0.5)]), {"enabled": True})
    out, st = fuse(cam, cands, {"enabled": False, "mode": "fuse"})
    assert out == cam and st["mode"] == "fuse"


def test_mode_camera_ignores_lidar():
    cam = [_cam_det_at(2.0, -0.5)]
    cands = find_ball_candidates(_scene([(2.0, -0.5)]), {"enabled": True})
    out, _ = fuse(cam, cands, {"enabled": True, "mode": "camera"})
    assert out == cam


def test_mode_lidar_uses_only_lidar():
    cands = find_ball_candidates(_scene([(2.0, -0.5)]), {"enabled": True})
    out, _ = fuse([_cam_det_at(2.0, -0.5)], cands, {"enabled": True, "mode": "lidar"})
    assert out and all(getattr(d, "source", "") == "lidar" for d in out)


def test_mode_fuse_prefers_camera_and_adds_lidar_only():
    """相机和雷达都看到同一颗球时不该重复；雷达独有的球才补进来。"""
    scene = _scene([(2.0, -0.5), (3.5, 1.8)])
    cands = find_ball_candidates(scene, {"enabled": True})
    # 相机只看到第一颗
    cam = [_cam_det_at(2.0, -0.5)]
    out, st = fuse(cam, cands, {"enabled": True, "mode": "fuse"})
    assert len(out) == 2, f"应融合成 2 个（1 相机 + 1 雷达补），实际 {len(out)}"
    assert st["matched"] == 1 and st["lidar_only"] == 1
    assert sum(1 for d in out if getattr(d, "source", "camera") == "camera") == 1


def test_lidar_detection_has_no_image_coords():
    """雷达检测没有图像坐标，必须显式标出来（避免下游误用 px/py）。"""
    d = LidarDetection({"x": 2.0, "y": -0.5, "z": -1.08, "n": 300})
    assert d.source == "lidar"
    assert math.isnan(d.px) and math.isnan(d.py)
    assert math.isfinite(d.bearing_rad) and math.isfinite(d.distance_m)


def test_shipped_config_path_is_correct():
    """配置里 lidar_ball 必须挂在 perception: 下 —— 节点就是按这个路径读的。

    ⚠️ 曾经把它写到了 yaml 顶层，节点 self.cfg["perception"]["lidar_ball"]
    永远读不到 → 功能静默失效（开关打开也没用）。这种错只能靠路径断言拦。
    """
    import pathlib
    import yaml
    cfg_path = (pathlib.Path(__file__).resolve().parents[1]
                / "config" / "perception.yaml")
    c = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    assert "perception" in c, "配置必须有 perception 段"
    assert "lidar_ball" in c["perception"], \
        "lidar_ball 必须在 perception: 下（节点按 cfg['perception']['lidar_ball'] 读）"
    lb = c["perception"]["lidar_ball"]
    # 当前需求明确启用雷达测距；未经标定必须拒绝权威测距。
    assert lb["enabled"] is True
    assert lb["mode"] == "precision"
    assert lb["extrinsics_confirmed"] is False
    assert lb["cloud_topic"] == "/cloud_registered_body"
