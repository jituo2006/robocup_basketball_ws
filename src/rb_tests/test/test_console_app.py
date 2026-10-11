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


# ---------------------------------------------------------------------------
# field_to_screen：场地固定视角（x 右、y 上，与 RViz 俯视一致）
# ⚠️ 现场核心诉求：车在出发点 (1,1) 必须显示在场地的【左下】，不是右下。
#    之前只有"车头朝上"一种视角，整个场地跟着车头转，车看起来在框的右下角，
#    对着实物核对时左右会误判。
# ---------------------------------------------------------------------------
def test_field_view_origin_is_bottom_left():
    mod = _load()
    px, py = mod.field_to_screen(0.0, 0.0, 0.0, 0.0, 10.0, 30.0)
    assert px < W / 2 and py > H / 2, f"场地原点应在左下，实际 ({px},{py})"


def test_field_view_x_grows_right():
    mod = _load()
    x1, _ = mod.field_to_screen(2.0, 0.0, 0.0, 0.0, 10.0, 30.0)
    x2, _ = mod.field_to_screen(12.0, 0.0, 0.0, 0.0, 10.0, 30.0)
    assert x2 > x1, "场地 x 增大应向右"


def test_field_view_y_grows_up():
    mod = _load()
    _, y1 = mod.field_to_screen(0.0, 1.0, 0.0, 0.0, 10.0, 30.0)
    _, y2 = mod.field_to_screen(0.0, 6.0, 0.0, 0.0, 10.0, 30.0)
    assert y2 < y1, "场地 y 增大应向上（图像 y 变小）"


def test_field_view_start_pose_is_left_of_field_center():
    """出发点 (1,1) 在 14×7.5 场地里必须落在左半边 —— 这就是用户要的。"""
    mod = _load()
    px, py = mod.field_to_screen(1.0, 1.0, 0.0, 0.0, 50.0, 30.0)
    assert px < W / 2, f"出发点应在屏幕左半边，实际 px={px}"
    # 篮筐在场地远端 → 右半边
    hx, _ = mod.field_to_screen(12.425, 3.75, 0.0, 0.0, 50.0, 30.0)
    assert hx > W / 2, f"篮筐应在屏幕右半边，实际 px={hx}"


def test_field_view_mirror_flips_x_only():
    """mirror_x=True 时 x 左右翻转，y 不变。"""
    mod = _load()
    x1, y1 = mod.field_to_screen(2.0, 3.0, 0.0, 0.0, 40.0, 30.0, mirror_x=False)
    x2, y2 = mod.field_to_screen(2.0, 3.0, 0.0, 0.0, 40.0, 30.0, mirror_x=True)
    assert abs(y1 - y2) < 1e-9, "y 不该被镜像影响"
    assert x1 != x2
    # 镜像后整体应在画布内且左右对称
    assert abs((x1 + x2) - W) < 1e-6, f"镜像应关于画布中线对称，{x1}+{x2}≠{W}"


def test_field_view_mirror_puts_hoop_on_the_left():
    """现场要求：篮筐(x 大)必须显示在【左边】—— 这是"场地→左"视角的核心。"""
    mod = _load()
    L, scale, margin = 14.0, 40.0, 30.0
    x_hoop, _ = mod.field_to_screen(12.425, 3.75, 0.0, 0.0, scale, margin, mirror_x=True)
    x_home, _ = mod.field_to_screen(1.0, 1.0, 0.0, 0.0, scale, margin, mirror_x=True)
    assert x_hoop < W / 2, f"镜像后篮筐应在左半边，实际 px={x_hoop}"
    assert x_home > W / 2, f"镜像后出发点应在右半边，实际 px={x_home}"
    assert x_hoop < x_home, "篮筐必须在出发点的左边"


def test_mirrored_view_heading_follows_the_map():
    """镜像视角下，车头前方必须落在车的【左边】—— 和篮筐在同一侧。

    ⚠️ 曾经的 BUG：mirror_x 只作用在 to_px 上，车头箭头却是手算的
    (rx + 30·cos yaw, …)，结果"地图翻了、箭头还朝右"。
    现场原话："现在雷达方向朝右，把它变成朝左就对了，地图不用动"。
    """
    mod = _load()
    cx, cy, yaw = 1.0, 1.0, 0.0          # 车在出发区，车头朝 +x（朝篮筐）
    scale, margin = 56.4, 30.0

    rx, ry = mod.field_to_screen(cx, cy, 0, 0, scale, margin, mirror_x=True)
    tx, ty = mod.field_to_screen(cx + 0.55 * math.cos(yaw), cy + 0.55 * math.sin(yaw),
                                 0, 0, scale, margin, mirror_x=True)
    assert tx < rx, f"镜像视角下车头应朝屏幕左，实际 车 x={rx} 车头 x={tx}"

    hx, _ = mod.field_to_screen(12.425, 3.75, 0, 0, scale, margin, mirror_x=True)
    assert hx < W / 2, "篮筐也应在左边 —— 车头和篮筐必须同侧，否则指反了"
    assert abs(ty - ry) < 1e-6, "yaw=0 时箭头不该有纵向偏移"


def test_unmirrored_view_heading_still_points_right():
    """不镜像时保持标准方向（x 向右）—— 两种视角都要自洽。"""
    mod = _load()
    scale, margin = 56.4, 30.0
    rx, _ = mod.field_to_screen(1.0, 1.0, 0, 0, scale, margin, mirror_x=False)
    tx, _ = mod.field_to_screen(1.55, 1.0, 0, 0, scale, margin, mirror_x=False)
    assert tx > rx, "不镜像时车头应朝屏幕右"


def test_official_layout_matches_diagram():
    """与官方场地图一致：出发区 A 在左上、篮筐在右中。

    官方图标了 14×7.5m、篮筐距右边线 1575mm、投篮边界线 R2000、三分线 R3750、
    出发区在左侧上下两角（A 上 / B 下）。我们的 home=(1,1) 在左下，
    所以要"x→右 + y 翻"才能对上官方那张图。
    """
    mod = _load()
    scale, margin, L, Wd = 56.4, 30.0, 14.0, 7.5
    # 官方图视角：x→右、y 翻；传场地长宽才走"画布居中"模式
    kw = dict(mirror_x=False, mirror_y=True, field_l=L, field_w=Wd)
    hx, hy = mod.field_to_screen(1.0, 1.0, 0, 0, scale, margin, **kw)
    ox, oy = mod.field_to_screen(12.425, 3.75, 0, 0, scale, margin, **kw)
    assert hx < W / 2 and hy < H / 2, f"出发区应在左上，实际 ({hx:.0f},{hy:.0f})"
    assert ox > W / 2, f"篮筐应在右半边，实际 px={ox:.0f}"
    assert abs(oy - H / 2) < 2, f"篮筐应在纵向中间（y=3.75），实际 py={oy:.0f}"


def test_mirror_y_flips_y_only():
    mod = _load()
    _, y1 = mod.field_to_screen(1.0, 2.0, 0, 0, 50.0, 30.0, mirror_y=False)
    _, y2 = mod.field_to_screen(1.0, 2.0, 0, 0, 50.0, 30.0, mirror_y=True)
    x1, _ = mod.field_to_screen(1.0, 2.0, 0, 0, 50.0, 30.0, mirror_y=False)
    x2, _ = mod.field_to_screen(1.0, 2.0, 0, 0, 50.0, 30.0, mirror_y=True)
    assert x1 == x2, "mirror_y 不该影响 x"
    assert abs((y1 + y2) - H) < 1e-6, f"y 应关于画布中线对称，{y1}+{y2}≠{H}"
