"""单元测试：tools/test_perimeter_ball.py 的方位角/转身几何。

为什么单测这几个纯函数：**方位角符号是整个文件最容易错、又最难从现象看出问题的地方**。
`Detection.bearing_rad` 是相机约定（右正左负），而场地系是 y 朝左、逆时针为正
—— 两者相反。写反了车就会朝镜像方向走，"背对球"会变成"面对球"。

运行：colcon test --packages-select rb_tests
"""

from __future__ import annotations

from pathlib import Path

_WS = Path(__file__).resolve().parents[3]

import importlib.util
import math
import sys

import pytest

from rb_tests.ws import WS


def _load():
    path = WS / "tools" / "test_perimeter_ball.py"
    if not path.is_file():
        pytest.skip(f"{path} 不存在")
    spec = importlib.util.spec_from_file_location("_tool_perimeter_ball", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# bearing_to_world：相机约定 → 场地系
# ---------------------------------------------------------------------------
def test_ball_dead_ahead_is_at_robot_heading():
    """车头朝 0°，球在正前方（bearing=0）→ 球在场地系方位 0（+x 方向）。"""
    m = _load()
    assert abs(m.bearing_to_world(0.0, 0.0)) < 1e-9


def test_ball_on_the_right_maps_to_negative_y():
    """⚠️ 关键符号：相机 bearing 右正，而场地系 y 朝左 → 右边的球应是负方位角。"""
    m = _load()
    # 车头朝 0°，球在右前方 45°（bearing=+π/4）
    got = m.bearing_to_world(0.0, math.pi / 4)
    assert abs(got - (-math.pi / 4)) < 1e-9, f"应为 -45°，实际 {math.degrees(got):.1f}°"
    # 负方位角 → 场地系 y 为负（右侧）✓
    assert math.sin(got) < 0


def test_ball_on_the_left_maps_to_positive_y():
    m = _load()
    got = m.bearing_to_world(0.0, -math.pi / 4)      # 左边
    assert got > 0 and math.sin(got) > 0


def test_bearing_is_relative_to_heading():
    """车头转到 90° 后，正前方的球应落在场地系 90°。"""
    m = _load()
    assert abs(m.bearing_to_world(math.pi / 2, 0.0) - math.pi / 2) < 1e-9


def test_camera_yaw_offset_shifts_result():
    m = _load()
    got = m.bearing_to_world(0.0, 0.0, cam_yaw_offset=math.radians(10))
    assert abs(got - math.radians(10)) < 1e-9


# ---------------------------------------------------------------------------
# yaw_to_put_ball_behind：转身让背面对着球
# ---------------------------------------------------------------------------
def test_ball_ahead_turn_180():
    """球在正前方 → 车头要转到 180°（正后方才是球的方向）。"""
    m = _load()
    got = m.yaw_to_put_ball_behind(0.0, 0.0)
    assert abs(abs(got) - math.pi) < 1e-9, f"应为 ±180°，实际 {math.degrees(got):.1f}°"


def test_back_faces_ball_geometrically():
    """几何验证：转身后，球的方向与车头方向夹角应为 180°。"""
    m = _load()
    for yaw in (0.0, 0.7, -1.3, 2.9):
        for bearing in (0.0, 0.4, -0.6, 1.1):
            target = m.yaw_to_put_ball_behind(yaw, bearing)
            ball_world = m.bearing_to_world(yaw, bearing)
            diff = m.wrap_pi(ball_world - target)          # 球相对车头的夹角
            assert abs(abs(diff) - math.pi) < 1e-9, (
                f"yaw={yaw} bearing={bearing}: 球在车头 {math.degrees(diff):.1f}° 处，"
                f"应恰好 180°（背面）")


# ---------------------------------------------------------------------------
# ball_xy：反算球在场地系的坐标
# ---------------------------------------------------------------------------
def test_ball_xy_dead_ahead():
    m = _load()
    x, y = m.ball_xy(1.0, 2.0, 0.0, 0.0, 3.0)
    assert abs(x - 4.0) < 1e-9 and abs(y - 2.0) < 1e-9


def test_ball_xy_on_the_right_is_minus_y():
    """右前方 45°、光轴深度 2m → 斜距 2/cos45° ≈ 2.828，落在右前方。

    ⚠️ 这里以前按"distance 就是斜距"写（用 90° 直接给 y=-2），是错的：
    Detection.distance_m 是【光轴深度 Z】，90° 时斜距发散、不可信。
    """
    m = _load()
    b = math.radians(45)
    x, y = m.ball_xy(0.0, 0.0, 0.0, b, 2.0)
    r = 2.0 / math.cos(b)
    assert x == pytest.approx(r * math.cos(b))
    assert y == pytest.approx(-r * math.sin(b))
    assert y < 0, "右前方 → 场地系 y 应为负"


def test_ball_xy_uses_slant_not_optical_depth():
    """关键回归：偏轴时不能用光轴深度当斜距（否则球位置偏近）。"""
    m = _load()
    b = math.radians(45)
    x, y = m.ball_xy(0.0, 0.0, 0.0, b, 2.0)
    r_depth_only = math.hypot(x, y)
    assert r_depth_only > 2.0 + 0.5, (
        f"斜距应明显大于光轴深度 2.0（45° 时约 2.83），实际 {r_depth_only}")


def test_ball_xy_rejects_near_horizon_bearing():
    """方位角接近 ±90° 时斜距不可信，应报错而不是给个巨大的坐标。"""
    m = _load()
    with pytest.raises(ValueError):
        m.ball_xy(0.0, 0.0, 0.0, math.radians(89), 2.0)


def test_ball_xy_dead_ahead_is_unchanged_by_slant():
    """正前方时 cos=1，斜距 = 光轴深度 —— 结果应与旧实现一致。"""
    m = _load()
    x, y = m.ball_xy(1.0, 2.0, 0.0, 0.0, 3.0)
    assert abs(x - 4.0) < 1e-9 and abs(y - 2.0) < 1e-9


def test_ball_xy_respects_heading():
    """车头朝 +y（90°），球在正前方 2m → 应落在 (0, 2)。"""
    m = _load()
    x, y = m.ball_xy(0.0, 0.0, math.pi / 2, 0.0, 2.0)
    assert abs(x) < 1e-9 and abs(y - 2.0) < 1e-9


# ---------------------------------------------------------------------------
# 现场规格：找球 → 摄像头正对球 → 旋转中心停在球前 70cm → 转 180° → 后退 20cm
# ---------------------------------------------------------------------------
def _load():
    import importlib.util
    from pathlib import Path
    ws = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        "_tpb", ws / "tools" / "test_perimeter_ball.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_median_ball_rejects_outliers():
    """单帧离群值（把干扰物当球）不能把估计带偏 —— 用中位数。"""
    mod = _load()
    good = [(0.10, 0.70)] * 5
    bad = [(1.20, 4.50)]          # 一帧离谱的
    r = mod.median_ball(good + bad)
    assert r is not None
    b, d, n = r
    assert abs(b - 0.10) < 1e-9, f"方位被离群值带偏: {b}"
    assert abs(d - 0.70) < 1e-9, f"距离被离群值带偏: {d}"
    assert n == 6


def test_median_ball_needs_at_least_two():
    mod = _load()
    assert mod.median_ball([]) is None
    assert mod.median_ball([(0.1, 0.7)]) is None


def test_median_ball_drops_nan_and_tiny_distance():
    """NaN 和过小的距离必须被丢掉，剩下的样本才参与中位数。"""
    mod = _load()
    r = mod.median_ball([(float("nan"), 0.7), (0.1, float("nan")),
                         (0.1, 0.0),                       # 距离 0 非法
                         (0.10, 0.70), (0.12, 0.72), (0.11, 0.71)])
    assert r is not None
    assert r[2] == 3, f"应剩 3 个合法样本，实际 {r[2]}"
    assert abs(r[0] - 0.11) < 1e-9, f"中位数应为 0.11，实际 {r[0]}"
    assert abs(r[1] - 0.71) < 1e-9


def test_approach_axis_is_yaw_minus_bearing():
    """球在场地系的方位 = yaw − bearing（bearing 右正左负）。"""
    import math
    mod = _load()
    assert abs(mod.approach_axis(0.0, 0.0)) < 1e-9
    # 球在正左（车体系 y>0）⇒ bearing 负，方位应为 +90°
    assert abs(mod.approach_axis(0.0, -math.pi / 2) - math.pi / 2) < 1e-9
    # 球在正右 ⇒ bearing 正，方位应为 -90°
    assert abs(mod.approach_axis(0.0, math.pi / 2) + math.pi / 2) < 1e-9


def test_servo_approach_math_moves_toward_ball():
    """沿 approach_axis 走就是朝球走 —— 正对着球时（bearing≈0）应沿车头前进。"""
    import math
    mod = _load()
    yaw = 0.5
    ang = mod.approach_axis(yaw, 0.0)
    assert abs(ang - yaw) < 1e-9, "正对球时前进方向应等于车头朝向"
    # 从 (0,0) 沿该方向走 0.5m，终点应朝球
    tx, ty = 0.5 * math.cos(ang), 0.5 * math.sin(ang)
    assert abs(math.hypot(tx, ty) - 0.5) < 1e-9


def test_shipped_defaults_match_the_site_spec():
    """出厂默认必须就是现场要的：70cm / 180° / 20cm。"""
    src = (_WS / "tools" / "test_perimeter_ball.py").read_text(encoding="utf-8")
    assert '"--align-dist", type=float, default=0.70' in src, "对准距离默认应为 70cm"
    assert '"--turn-deg", type=float, default=180.0' in src, "转身默认应为 180°"
    assert '"--backup", type=float, default=0.20' in src, "后退默认应为 20cm"


def test_align_tolerance_is_tight_enough_to_face_the_ball():
    """对准容差要够紧才叫"正对球"；太松等于没对准。"""
    src = (_WS / "tools" / "test_perimeter_ball.py").read_text(encoding="utf-8")
    i = src.find('"--align-tol-deg"')
    assert i > 0
    seg = src[i:i + 160]
    import re
    m = re.search(r"default=([\d.]+)", seg)
    assert m, "读不到 align-tol-deg 默认值"
    assert float(m.group(1)) <= 3.0, f"对准容差 {m.group(1)}° 太松，谈不上正对球"
