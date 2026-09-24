"""rb_mission 几何工具的单元测试（纯数学，无需 ROS 运行时）。

对应规则依据：§2.5 的传球区/投篮边界判定、状态机导航用的 P 控制器。
运行：colcon test --packages-select rb_mission   或   pytest src/rb_mission/test
"""

from __future__ import annotations

import math

import pytest
from rb_mission.geometry import (
    clamp,
    face_bearing_command,
    goto_command,
    point_in_polygon,
    wrap_pi,
)

SQUARE = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]
# 一个凹多边形（L 形），用来确认射线法不是只会处理凸多边形
L_SHAPE = [[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [2.0, 2.0], [2.0, 4.0], [0.0, 4.0]]


# -- point_in_polygon -------------------------------------------------------
@pytest.mark.parametrize("x,y,expected", [
    (2.0, 2.0, True),     # 正中
    (0.1, 0.1, True),     # 贴近角
    (3.9, 3.9, True),
    (-0.1, 2.0, False),   # 左外
    (4.1, 2.0, False),    # 右外
    (2.0, -0.1, False),   # 下外
    (2.0, 4.1, False),    # 上外
])
def test_point_in_polygon_square(x, y, expected):
    assert point_in_polygon(x, y, SQUARE) is expected


def test_point_in_polygon_concave():
    """凹多边形：L 形的缺口处必须判为外部，否则传球区判定会多算面积。"""
    assert point_in_polygon(1.0, 1.0, L_SHAPE) is True    # 竖条部分
    assert point_in_polygon(3.0, 1.0, L_SHAPE) is True    # 横条部分
    assert point_in_polygon(3.0, 3.0, L_SHAPE) is False   # 缺口里


def test_point_in_polygon_degenerate():
    """点数不足 3 无法构成多边形，必须安全返回 False 而不是抛异常。"""
    assert point_in_polygon(0.0, 0.0, []) is False
    assert point_in_polygon(0.0, 0.0, [[0.0, 0.0], [1.0, 1.0]]) is False


# -- wrap_pi / clamp --------------------------------------------------------
@pytest.mark.parametrize("a,expected", [
    (0.0, 0.0),
    (math.pi / 2, math.pi / 2),
    (math.pi, math.pi),
    (math.pi + 0.1, -math.pi + 0.1),
    (3 * math.pi, math.pi),
    (-3 * math.pi, -math.pi),      # 注意：-π 与 +π 等价，实现取 -π
    (-math.pi - 0.1, math.pi - 0.1),
])
def test_wrap_pi(a, expected):
    assert wrap_pi(a) == pytest.approx(expected, abs=1e-9)


def test_wrap_pi_always_in_range():
    for k in range(-10, 11):
        v = wrap_pi(k * 1.3)
        assert -math.pi - 1e-9 <= v <= math.pi + 1e-9


@pytest.mark.parametrize("v,lo,hi,expected", [
    (5.0, 0.0, 1.0, 1.0),
    (-5.0, 0.0, 1.0, 0.0),
    (0.5, 0.0, 1.0, 0.5),
    (0.0, 0.0, 1.0, 0.0),
    (1.0, 0.0, 1.0, 1.0),
])
def test_clamp(v, lo, hi, expected):
    assert clamp(v, lo, hi) == expected


# -- goto_command -----------------------------------------------------------
def test_goto_forward_is_body_x():
    """车头朝 +x（yaw=0）时朝目标走，速度应落在车体 x 方向。"""
    vx, vy, wz, dist = goto_command(0.0, 0.0, 0.0, 5.0, 0.0,
                                    kp_lin=1.5, kp_yaw=2.0, max_lin=0.4, max_ang=0.8)
    assert dist == pytest.approx(5.0)
    assert vx == pytest.approx(0.4)      # 被 max_lin 限幅
    assert vy == pytest.approx(0.0, abs=1e-9)
    assert wz == pytest.approx(0.0, abs=1e-9)


def test_goto_rotates_into_body_frame():
    """车头朝 +y（yaw=90°）时要往世界 +x 走。

    世界 +x 相对"车头朝 +y"是**正右方**，而车体系里右方是 **-y**，
    所以期望 vy≈-0.4（不是 +0.4）。这条专门锁住车体系符号约定，
    写反了会导致横移方向相反。
    """
    vx, vy, _, _ = goto_command(0.0, 0.0, math.pi / 2, 5.0, 0.0,
                                kp_lin=1.5, kp_yaw=2.0, max_lin=0.4, max_ang=0.8)
    assert vx == pytest.approx(0.0, abs=1e-6)
    assert vy == pytest.approx(-0.4, abs=1e-6)


def test_goto_respects_max_lin_for_diagonal():
    """对角线目标也必须限幅到 max_lin（不能因为两个分量各自限幅而超速）。"""
    vx, vy, _, _ = goto_command(0.0, 0.0, 0.0, 10.0, 10.0,
                                kp_lin=1.5, kp_yaw=2.0, max_lin=0.4, max_ang=0.8)
    assert math.hypot(vx, vy) == pytest.approx(0.4, rel=1e-6)


def test_goto_does_not_oversteer_when_close():
    """近目标时线性速度应随距离衰减（避免冲过目标来回摆）。"""
    _, _, _, d_far = goto_command(0.0, 0.0, 0.0, 5.0, 0.0, 1.5, 2.0, 0.4, 0.8)
    vx_near, _, _, d_near = goto_command(4.9, 0.0, 0.0, 5.0, 0.0, 1.5, 2.0, 0.4, 0.8)
    assert d_near < d_far
    assert 0.0 < vx_near < 0.4          # 未被限幅，按距离给


def test_goto_yaw_limited_by_max_ang():
    """目标在正后方（相差 180°）时转向量必须被 max_ang 夹住。

    180° 时转向哪边都合理，实现选了 +（逆时针），这里只锁"被限幅"这个约束。
    """
    _, _, wz, _ = goto_command(0.0, 0.0, 0.0, -5.0, 0.0, 1.5, 2.0, 0.4, 0.8)
    assert abs(wz) == pytest.approx(0.8)


def test_goto_face_travel_false_keeps_heading():
    """face_travel=False（例如横移）时不应产生自转。"""
    _, _, wz, _ = goto_command(0.0, 0.0, 0.0, 0.0, 5.0,
                               1.5, 2.0, 0.4, 0.8, face_travel=False)
    assert wz == pytest.approx(0.0)


def test_goto_yaw_sign_is_negative_when_target_on_right():
    """目标在右侧（y<0，车体系右为负）时应对应负的角速度。"""
    _, _, wz, _ = goto_command(0.0, 0.0, 0.0, 5.0, -5.0, 1.5, 2.0, 0.4, 0.8)
    assert wz < 0


# -- face_bearing_command ---------------------------------------------------
def test_face_bearing_zero_error():
    wz, err = face_bearing_command(0.0, 0.0, kp_yaw=2.0, max_ang=0.8)
    assert wz == pytest.approx(0.0)
    assert err == pytest.approx(0.0)


def test_face_bearing_clamped():
    wz, err = face_bearing_command(0.0, math.pi, kp_yaw=2.0, max_ang=0.8)
    assert abs(wz) <= 0.8
    assert abs(err) <= math.pi


def test_face_bearing_wraps_across_pi():
    """目标方位角写成 358° 时，真实误差只有 2°，不能当成 358° 猛转。

    注意第二个参数是**相对车体**的方位角（右正左负），不是世界 yaw。
    """
    _, err = face_bearing_command(0.0, math.radians(358.0), 2.0, 0.8)
    assert abs(err) == pytest.approx(math.radians(2.0), abs=1e-6)
    # 负方向同理
    _, err2 = face_bearing_command(0.0, math.radians(-358.0), 2.0, 0.8)
    assert abs(err2) == pytest.approx(math.radians(2.0), abs=1e-6)
