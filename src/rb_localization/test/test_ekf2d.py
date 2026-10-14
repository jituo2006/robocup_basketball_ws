"""rb_localization 的 2D EKF 单元测试（纯数值，无需 ROS 运行时）。

覆盖：运动学预测、绝对位姿更新、协方差收敛。
运行：colcon test --packages-select rb_localization
"""

from __future__ import annotations

import math

import pytest
from rb_localization.ekf2d import (
    Ekf2D,
    EkfConfig,
    sensor_to_base_xy,
    world_vel_to_body,
    wrap_pi,
)


@pytest.fixture()
def ekf() -> Ekf2D:
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    return e


# -- 初始位姿 ---------------------------------------------------------------
def test_set_pose_roundtrip():
    e = Ekf2D()
    e.set_pose(1.5, -2.5, 0.7)
    assert e.pose == pytest.approx((1.5, -2.5, 0.7))
    assert e.initialized is True


def test_set_pose_wraps_yaw():
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 4 * math.pi + 0.5)
    assert e.pose[2] == pytest.approx(0.5)


def test_initial_std_matches_config():
    cfg = EkfConfig(init_std_xy=0.5, init_std_yaw=0.3)
    e = Ekf2D(cfg)
    e.set_pose(0.0, 0.0, 0.0)
    assert e.std[0] == pytest.approx(0.5)
    assert e.std[2] == pytest.approx(0.3)


# -- 预测 -------------------------------------------------------------------
def test_predict_straight_line():
    """yaw=0 时 vx=0.5 走 1s，应正好走 0.5m 且 yaw 不变。"""
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    for _ in range(100):
        e.predict(0.5, 0.0, 0.0, 0.01)
    x, y, yaw = e.pose
    assert x == pytest.approx(0.5, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)
    assert yaw == pytest.approx(0.0, abs=1e-6)


def test_predict_rotation_only():
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    for _ in range(100):
        e.predict(0.0, 0.0, math.pi / 2, 0.01)   # 90°/s 转 1s
    assert e.pose[2] == pytest.approx(math.pi / 2, abs=1e-6)


def test_predict_uses_body_frame():
    """车头朝 +y(yaw=90°) 时往前开(vx>0)，应沿世界 +y 移动而不是 +x。

    这条锁住"速度是车体系"这个约定；写成世界系会让整车运动方向错 90°。
    """
    e = Ekf2D()
    e.set_pose(0.0, 0.0, math.pi / 2)
    for _ in range(100):
        e.predict(0.5, 0.0, 0.0, 0.01)
    x, y, _ = e.pose
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.5, abs=1e-6)


def test_predict_lateral_body_frame():
    """yaw=0 时 vy>0（车体左移）应让世界 y 增大。"""
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    for _ in range(100):
        e.predict(0.0, 0.3, 0.0, 0.01)
    assert e.pose[1] == pytest.approx(0.3, abs=1e-6)


def test_predict_ignores_nonpositive_dt():
    e = Ekf2D()
    e.set_pose(1.0, 2.0, 0.3)
    before = e.pose
    e.predict(1.0, 1.0, 1.0, 0.0)
    e.predict(1.0, 1.0, 1.0, -0.5)
    assert e.pose == pytest.approx(before)


def test_predict_grows_covariance():
    """只有预测没有观测时不确定度必须变大，否则滤波器会过度自信。"""
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    s0 = e.std
    for _ in range(50):
        e.predict(0.3, 0.0, 0.1, 0.02)
    s1 = e.std
    assert s1[0] > s0[0]
    assert s1[2] > s0[2]


def test_predict_auto_initializes():
    """未初始化就被要求预测时，应自动落到原点而不是抛异常。"""
    e = Ekf2D()
    e.predict(0.1, 0.0, 0.0, 0.1)
    assert e.initialized is True


# -- 绝对位姿更新 -----------------------------------------------------------
def test_update_pose_pulls_toward_measurement():
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    e.update_pose(1.0, 0.5, 0.2)
    x, y, yaw = e.pose
    assert 0.0 < x < 1.0          # 没有被直接覆盖，而是按卡尔曼增益折中
    assert 0.0 < y < 0.5
    assert 0.0 < yaw < 0.2


def test_update_pose_reduces_std():
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    before = e.std
    e.update_pose(0.1, 0.1, 0.05)
    assert e.std[0] < before[0]


def test_update_pose_converges_when_repeated():
    """反复喂同一个观测，估计应收敛到该值（滤波器不发散）。

    注意是**渐近**收敛：协方差变小后卡尔曼增益也变小，不会一步到位。
    """
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    for _ in range(200):
        e.update_pose(3.0, -1.0, 0.4)
    assert e.pose == pytest.approx((3.0, -1.0, 0.4), abs=2e-2)
    assert e.std[0] < 0.05      # 不确定度确实收小了


def test_update_pose_wraps_yaw_across_pi():
    """在 ±π 附近更新时不能把 yaw 拉成 2π 之外的数。"""
    e = Ekf2D()
    e.set_pose(0.0, 0.0, math.radians(179.0))
    for _ in range(50):
        e.update_pose(0.0, 0.0, math.radians(-179.0))
    assert -math.pi <= e.pose[2] <= math.pi


def test_update_pose_initializes_if_needed():
    e = Ekf2D()
    e.update_pose(2.0, 3.0, 0.1)
    assert e.pose == pytest.approx((2.0, 3.0, 0.1))
    assert e.initialized is True


def test_wrap_pi_is_stable_for_many_angles():
    from rb_localization.ekf2d import wrap_pi
    for k in range(-20, 21):
        v = wrap_pi(k * 0.9)
        assert -math.pi - 1e-9 <= v <= math.pi + 1e-9


# -- 雷达安装偏移：必须按【车体 yaw】旋转，不是雷达 yaw -----------------------
#
# 本车雷达装了 sensor_to_base_yaw = −85°（几乎转了 90°）。若用雷达 yaw 去转
# 安装偏移，"雷达在中心正前方 30cm" 会被算成"侧方 30cm"，
# 症状是 RViz 里位姿箭头跑到车体的左上角。


def test_sensor_to_base_uses_body_yaw_not_sensor_yaw():
    """⭐ 回归测试：偏移按车体 yaw 旋转。

    雷达在车体中心正前方 30cm（车体系）⇒ s2b = (−0.30, 0)。
    车体 yaw = −85° 时，"车体后方 0.30m" 在里程计系的方向是 −85°+180° = 95°。
    """
    bx, by, byaw = sensor_to_base_xy(0.0, 0.0, 0.0,
                                     -0.30, 0.0, math.radians(-85.0))
    assert byaw == pytest.approx(math.radians(-85.0), abs=1e-9)
    assert bx == pytest.approx(0.30 * math.cos(math.radians(95.0)), abs=1e-9)
    assert by == pytest.approx(0.30 * math.sin(math.radians(95.0)), abs=1e-9)


def test_sensor_to_base_preserves_distance():
    """偏移长度与朝向无关（任意雷达 yaw 下，base 都应距 sensor 0.30m）。"""
    for deg in (-85.0, 0.0, 37.0, 180.0):
        bx, by, _ = sensor_to_base_xy(1.0, 2.0, math.radians(20.0),
                                      -0.30, 0.0, math.radians(deg))
        assert math.hypot(bx - 1.0, by - 2.0) == pytest.approx(0.30, abs=1e-9)


def test_sensor_to_base_zero_offset_is_identity():
    """偏移全 0 时只应加上雷达 yaw。"""
    bx, by, byaw = sensor_to_base_xy(3.0, 4.0, math.radians(10.0),
                                     0.0, 0.0, math.radians(-85.0))
    assert (bx, by) == pytest.approx((3.0, 4.0), abs=1e-12)
    assert byaw == pytest.approx(math.radians(-75.0), abs=1e-9)


def test_sensor_to_base_matches_expected_buggy_vs_fixed():
    """把"写错"和"写对"的差别钉住：两者相差约 90°。

    写错（用雷达 yaw=0 旋转）→ base 落在 sensor 的 −x 方向。
    写对（用车体 yaw=−85° 旋转）→ base 落在 +y 方向附近。
    这条断言让"改回用雷达 yaw"这种回退立刻被测出来。
    """
    buggy = sensor_to_base_xy(0.0, 0.0, 0.0, -0.30, 0.0, 0.0)          # s2b_yaw=0
    fixed = sensor_to_base_xy(0.0, 0.0, 0.0, -0.30, 0.0, math.radians(-85.0))
    assert buggy[0] == pytest.approx(-0.30, abs=1e-9)
    assert abs(fixed[1]) > 0.29, "修好后应主要落在 y 方向（差 90°）"


# -- 里程计 twist 的坐标系：world → body -------------------------------------
#
# FAST-LIO 的 /Odometry.twist 是 **world(camera_init) 系**速度
# （common_lib.h: "the estimated velocity at the end lidar point (world frame)"），
# 而 Ekf2D.predict() 要的是**车体系**（内部会再按 yaw 旋转一次）。
# 直接喂 = 多转一次 yaw，预测方向全错 → 运动时定位发飘。


def test_world_vel_to_body_rotates_back():
    """⭐ 回归测试：车体朝 +90° 时，世界系 (0,1) 应转成车体系 (1,0)（正前方）。"""
    vx, vy = world_vel_to_body(0.0, 1.0, math.pi / 2)
    assert (vx, vy) == pytest.approx((1.0, 0.0), abs=1e-9)


def test_world_vel_to_body_identity_at_zero_yaw():
    """车体朝 0° 时不应改变速度。"""
    assert world_vel_to_body(1.0, 2.0, 0.0) == pytest.approx((1.0, 2.0), abs=1e-12)


def test_world_vel_to_body_preserves_speed():
    """纯旋转，不改变速度大小。"""
    for yaw in (0.0, 0.7, -1.9, math.pi, math.radians(-85.0)):
        vx, vy = world_vel_to_body(0.3, -0.4, yaw)
        assert math.hypot(vx, vy) == pytest.approx(0.5, abs=1e-9)


def test_world_vel_to_body_is_inverse_of_body_to_world():
    """与 Ekf2D.predict 内部的 R(yaw) 必须恰好互为逆变换（否则就是重复旋转）。"""
    yaw = math.radians(37.0)
    vx_w, vy_w = 0.4, -0.2
    vx_b, vy_b = world_vel_to_body(vx_w, vy_w, yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    # predict 内部做的旋转：world = R(yaw) · body
    assert vx_b * c - vy_b * s == pytest.approx(vx_w, abs=1e-9)
    assert vx_b * s + vy_b * c == pytest.approx(vy_w, abs=1e-9)


def test_pose_delta_retains_independent_pose_correction():
    e=Ekf2D();e.set_pose(10.,20.,math.pi/2)
    # Odom moves along its +x; corrected heading rotates that move to +y.
    e.predict_pose_delta((0.,0.,0.),(1.,0.,.2),.1)
    assert e.pose == pytest.approx((10.,21.,math.pi/2+.2))
