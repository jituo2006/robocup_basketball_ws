"""单元测试：tools/console_app.py 的纯函数部分。

只测不依赖 ROS/cv2 运行时的逻辑 —— 主要盯两处容易错的地方：
  ① PointCloud2 → (N,3) 的解析（字段偏移算错会得到垃圾坐标）
  ② 检测方位角 → 场地坐标的符号（bearing 是「右正左负」，场地系是「y 朝左」，
     写反了俯视图上的球会镜像到另一侧）
"""

from __future__ import annotations

import importlib.util
import math
import sys
import types

import pytest

from rb_tests.ws import WS


def _load():
    path = WS / "tools" / "console_app.py"
    if not path.is_file():
        pytest.skip(f"{path} 不存在")
    spec = importlib.util.spec_from_file_location("_tool_console_app", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# pc2_xyz：PointCloud2 解析
# ---------------------------------------------------------------------------
class _Field:
    def __init__(self, name, offset, datatype=7, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


def _cloud(points, point_step=16, extra_pad=True):
    """构造一个最小 PointCloud2 替身：x,y,z 各 4 字节 + 可选 4 字节 padding。"""
    import struct
    m = types.SimpleNamespace()
    m.fields = [_Field("x", 0), _Field("y", 4), _Field("z", 8)]
    m.point_step = point_step
    m.width, m.height = len(points), 1
    buf = b""
    for (x, y, z) in points:
        buf += struct.pack("<fff", x, y, z)
        if extra_pad:
            buf += b"\x00" * (point_step - 12)
    m.data = buf
    return m


def test_pc2_xyz_reads_xyz_in_order():
    mod = _load()
    out = mod.pc2_xyz(_cloud([(1.0, 2.0, 3.0), (-4.0, 5.0, 6.0)]))
    assert out is not None and out.shape == (2, 3)
    assert out[0].tolist() == pytest.approx([1.0, 2.0, 3.0])
    assert out[1].tolist() == pytest.approx([-4.0, 5.0, 6.0])


def test_pc2_xyz_handles_padding():
    """带 4 字节 padding（point_step=16）也要读对。"""
    mod = _load()
    out = mod.pc2_xyz(_cloud([(7.0, 8.0, 9.0)], point_step=16))
    assert out is not None
    assert out[0].tolist() == pytest.approx([7.0, 8.0, 9.0])


def test_pc2_xyz_drops_non_finite():
    mod = _load()
    out = mod.pc2_xyz(_cloud([(1.0, 2.0, 3.0), (float("nan"), 0.0, 0.0)]))
    assert out is not None and len(out) == 1


def test_pc2_xyz_returns_none_without_xyz():
    mod = _load()
    m = _cloud([(1.0, 2.0, 3.0)])
    m.fields = [_Field("intensity", 0)]
    assert mod.pc2_xyz(m) is None


def test_pc2_xyz_returns_none_on_empty():
    mod = _load()
    m = _cloud([(1.0, 2.0, 3.0)])
    m.width = m.height = 0
    assert mod.pc2_xyz(m) is None


# ---------------------------------------------------------------------------
# 检测方位角 → 场地坐标（符号是本文件最容易错的地方）
# ---------------------------------------------------------------------------
def _det(label, bearing_rad, distance_m):
    return types.SimpleNamespace(label=label, bearing_rad=bearing_rad,
                                 distance_m=distance_m, confidence=1.0)


def _dets_to_field(mod, pose, dets):
    """复刻 render_lidar 里球坐标的算法，独立验一遍符号。

    约定：bearing_rad 右正左负；场地系 y 朝左、逆时针为正
          → 世界方位角 = yaw − bearing
    """
    cx, cy, cyaw = pose
    out = []
    for d in dets:
        ang = cyaw - d.bearing_rad
        out.append((cx + d.distance_m * math.cos(ang),
                    cy + d.distance_m * math.sin(ang)))
    return out


def test_detection_dead_ahead_is_plus_x():
    mod = _load()
    (x, y), = _dets_to_field(mod, (0.0, 0.0, 0.0), [_det("ball_basketball", 0.0, 2.0)])
    assert (x, y) == pytest.approx((2.0, 0.0))


def test_detection_on_the_right_goes_minus_y():
    """⚠️ 关键符号：相机 bearing 右正，但场地系 y 朝左 → 右侧的球 y 应为负。"""
    mod = _load()
    (x, y), = _dets_to_field(mod, (0.0, 0.0, 0.0),
                             [_det("ball_basketball", math.pi / 2, 2.0)])
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(-2.0), f"右侧的球 y 应为 -2，实际 {y}"


def test_detection_on_the_left_goes_plus_y():
    mod = _load()
    (x, y), = _dets_to_field(mod, (0.0, 0.0, 0.0),
                             [_det("ball_basketball", -math.pi / 2, 2.0)])
    assert y == pytest.approx(2.0)


def test_detection_respects_heading():
    """车头朝 +y（90°），正前方的球应落在 (0, 2)。"""
    mod = _load()
    (x, y), = _dets_to_field(mod, (0.0, 0.0, math.pi / 2),
                             [_det("ball_basketball", 0.0, 2.0)])
    assert x == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# parse_check_ready：把 check_ready.sh 的输出解析成结构化状态
# （页面上「就绪状态」面板靠它，解析错会把故障显示成全绿 —— 很危险）
# ---------------------------------------------------------------------------
_SAMPLE_OK = """
  🤖 就绪检查 23:07:37

    ── 硬件 ──
    \x1b[32m✓\x1b[0m CAN (can0)               UP
    ── 数据链 ──
    \x1b[32m✓\x1b[0m /livox/lidar             10.3 Hz
    \x1b[32m✓\x1b[0m /Odometry 延迟         27 ms（新鲜）

    \x1b[32m✅ 全部就绪，可以开车 / 开地图了\x1b[0m
"""

_SAMPLE_BAD = """
  🤖 就绪检查 23:07:37

    ── 硬件 ──
    \x1b[31m✗\x1b[0m CAN (can0)               没 UP（bash tools/can_recover.sh）
    \x1b[32m✓\x1b[0m /livox/lidar             10.3 Hz

    \x1b[31m⚠️ 还有 1 项没就绪\x1b[0m
      CAN 没 UP → bash tools/can_recover.sh
"""


def test_parse_check_ready_all_green():
    mod = _load()
    r = mod.parse_check_ready(_SAMPLE_OK)
    assert r["ok"] is True
    assert r["failed"] == []
    assert len(r["items"]) == 3
    assert "全部就绪" in r["verdict"]
    # 颜色码必须被剥掉，否则页面上会显示乱码
    assert "\x1b" not in " ".join(i["text"] for i in r["items"])


def test_parse_check_ready_with_failure():
    mod = _load()
    r = mod.parse_check_ready(_SAMPLE_BAD)
    assert r["ok"] is False, "有 ✗ 时绝不能报 ok"
    assert len(r["failed"]) == 1
    assert "CAN" in r["failed"][0]
    assert "没就绪" in r["verdict"]


def test_parse_check_ready_empty_is_not_ok():
    """空输出（脚本没跑起来/没装）不能误判成全绿。"""
    mod = _load()
    for bad in ("", "随便什么内容", "  \n  \n"):
        r = mod.parse_check_ready(bad)
        assert r["ok"] is False, f"输入 {bad!r} 不该判成 ok"
        assert r["items"] == []


# ---------------------------------------------------------------------------
# world_to_screen：雷达俯视图的坐标变换
# ⚠️ 这里曾经前后反了（现场实测："地图显示前后反了"）—— 车正前方的点被画到了
#    图像下方。原因是多转了一个 −90° 且符号用错。几何断言必须锁住。
# ---------------------------------------------------------------------------
W, H, SCALE = 900, 560, 20.0


def _px(wx, wy, cx=0.0, cy=0.0, yaw=0.0):
    mod = _load()
    return mod.world_to_screen(wx, wy, cx, cy, yaw, SCALE)


def test_point_ahead_is_above_center():
    """车头方向上的点必须画在【上方】（车头朝上）。"""
    px, py = _px(5.0, 0.0, yaw=0.0)
    assert abs(px - W / 2) < 1e-6, f"正前方不该有横向偏移，实际 px={px}"
    assert py < H / 2, f"正前方应在图像上方（py<{H/2}），实际 py={py}"


def test_point_behind_is_below_center():
    px, py = _px(-5.0, 0.0, yaw=0.0)
    assert py > H / 2, f"正后方应在图像下方，实际 py={py}"


def test_left_of_robot_is_left_on_screen():
    """车的左边（车体系 +y）应画在屏幕左边。"""
    px, py = _px(0.0, 5.0, yaw=0.0)
    assert px < W / 2, f"车的左边应在屏幕左半，实际 px={px}"
    assert abs(py - H / 2) < 1e-6


def test_right_of_robot_is_right_on_screen():
    px, py = _px(0.0, -5.0, yaw=0.0)
    assert px > W / 2, f"车的右边应在屏幕右半，实际 px={px}"


def test_heading_rotates_the_view():
    """车头朝 +y（yaw=90°）时，场地 +y 方向上的点应在图像上方。"""
    px, py = _px(0.0, 5.0, yaw=math.pi / 2)
    assert abs(px - W / 2) < 1e-6, f"朝 +y 时正前方不该有横向偏移，实际 {px}"
    assert py < H / 2, "场地 +y 应是车头方向 → 图像上方"


def test_left_point_with_rotated_heading():
    """车头朝 +y 时，车的左边是场地 −x 方向 → 应画在屏幕左边。"""
    px, py = _px(-5.0, 5.0, yaw=math.pi / 2)
    assert px < W / 2, f"车的左边应在屏幕左半，实际 px={px}"


def test_works_with_numpy_arrays():
    """点云走的是数组路径，结果必须和标量路径一致。"""
    import numpy as np
    mod = _load()
    xs = np.array([1.0, 2.0, 3.0])
    ys = np.array([0.5, 1.5, -2.5])
    px, py = mod.world_to_screen(xs, ys, 0.0, 0.0, 0.3, SCALE)
    for i in range(3):
        sx, sy = mod.world_to_screen(xs[i], ys[i], 0.0, 0.0, 0.3, SCALE)
        assert abs(px[i] - sx) < 1e-9 and abs(py[i] - sy) < 1e-9


# ---------------------------------------------------------------------------
# depth_to_slant：单目【光轴深度】→ 斜距
# ⚠️ distance_m 是沿光轴的深度 Z，不是斜距。直接当斜距用会让球越偏离画面中心
#    位置越错（偏 45° 少算 29%）—— 现场实测"雷达显示球的位置不对"就是这个。
# ---------------------------------------------------------------------------
def test_depth_to_slant_center_is_identity():
    mod = _load()
    assert mod.depth_to_slant(2.0, 0.0) == pytest.approx(2.0)


def test_depth_to_slant_off_axis_grows():
    mod = _load()
    assert mod.depth_to_slant(2.0, math.radians(30)) == pytest.approx(2.0 / math.cos(math.radians(30)))
    assert mod.depth_to_slant(2.0, math.radians(45)) == pytest.approx(2.0 * math.sqrt(2))


def test_depth_to_slant_rejects_near_horizon():
    """b 接近 ±90° 时斜距发散 → 必须判不可信，而不是给个巨大的数。"""
    mod = _load()
    assert math.isnan(mod.depth_to_slant(2.0, math.radians(89)))
    assert math.isnan(mod.depth_to_slant(2.0, math.radians(-85)))


def test_depth_to_slant_rejects_bad_input():
    mod = _load()
    for d, b in ((float("nan"), 0.0), (0.0, 0.0), (-1.0, 0.0), (2.0, float("nan"))):
        assert math.isnan(mod.depth_to_slant(d, b)), f"({d},{b}) 应判不可信"


# ---------------------------------------------------------------------------
# BallSmoother：显示层的滑动中位数（治"球一直在跳动"）
# ---------------------------------------------------------------------------
def test_smoother_resists_single_outlier():
    mod = _load()
    s = mod.BallSmoother()
    out = None
    for v in (1.0, 1.1, 0.9, 5.0):          # 最后一个是大跳变
        out = s.update("ball_basketball", 100.0, v, 0.0, v)
    assert out is not None
    assert 0.9 <= out[0] <= 1.1, f"中位数不该被 5.0 带跑，实际 {out[0]}"
    assert out[3] == 4


def test_smoother_drops_old_samples():
    mod = _load()
    s = mod.BallSmoother(window_s=1.0)
    s.update("x", 0.0, 9.0, 9.0, 9.0)       # 会被窗口丢弃
    out = s.update("x", 2.0, 1.0, 1.0, 1.0)
    assert out[0] == pytest.approx(1.0), "超出时间窗的旧样本必须丢掉"


def test_smoother_caps_sample_count():
    mod = _load()
    s = mod.BallSmoother(window_s=1e9, samples=3)
    for i in range(10):
        out = s.update("y", float(i), float(i), 0.0, 1.0)
    assert out[3] == 3, f"样本数应被截到 3，实际 {out[3]}"


def test_smoother_ignores_nan():
    mod = _load()
    s = mod.BallSmoother()
    assert s.update("z", 1.0, float("nan"), 0.0, 1.0) is None
