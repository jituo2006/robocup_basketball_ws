"""单元测试：tools/test_perimeter_ball.py 的方位角/转身几何。

为什么单测这几个纯函数：**方位角符号是整个文件最容易错、又最难从现象看出问题的地方**。
`Detection.bearing_rad` 是相机约定（右正左负），而场地系是 y 朝左、逆时针为正
—— 两者相反。写反了车就会朝镜像方向走，"背对球"会变成"面对球"。

运行：colcon test --packages-select rb_tests
"""

from __future__ import annotations

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
