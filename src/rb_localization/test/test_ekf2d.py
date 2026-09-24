"""rb_localization 的 2D EKF 单元测试（纯数值，无需 ROS 运行时）。

覆盖：运动学预测、绝对位姿更新、定位柱方位角(bearing-only)更新、协方差收敛。
运行：colcon test --packages-select rb_localization
"""

from __future__ import annotations

import math

import pytest
from rb_localization.ekf2d import Ekf2D, EkfConfig


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


# -- 定位柱方位角更新 -------------------------------------------------------
def test_update_bearing_requires_init():
    e = Ekf2D()
    assert e.update_bearing((5.0, 0.0), 0.0) is False


@pytest.mark.parametrize("landmark", [(0.1, 0.0), (20.0, 0.0)])
def test_update_bearing_rejects_bad_distance(ekf, landmark):
    """太近(<0.3m)或太远(>8m)的地标方位角不可靠，必须拒绝而不是硬用。"""
    assert ekf.update_bearing(landmark, 0.0) is False


def test_update_bearing_accepts_valid_and_reduces_std(ekf):
    before = ekf.std
    ok = ekf.update_bearing((5.0, 0.0), 0.2)
    assert ok is True
    assert ekf.std[2] < before[2]


def _bearing_world_from_truth(e, landmark, true_pose):
    """按 localization_node 的方式，把"真实相机读数"换算成观测的世界方位角。

    调用方实际是 bearing_world = ekf_yaw + 相机读数，这里如实复现，
    这样测试才覆盖真实调用链（而不是喂一个与状态无关的理想值）。
    """
    from rb_localization.ekf2d import wrap_pi
    tx, ty, tyaw = true_pose
    cam = wrap_pi(math.atan2(landmark[1] - ty, landmark[0] - tx) - tyaw)
    return wrap_pi(e.pose[2] + cam)


def test_update_bearing_corrects_yaw_error():
    """纯朝向偏差必须被修回来。

    ⚠️ 这是**回归测试**：H 矩阵符号写反时，这条会失败——更新会把 yaw
    推向背离真值的方向（实测估计发散到几个弧度外）。符号问题的其余表现
    还很难一眼看出，所以必须由测试锁住。
    """
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    landmark = (5.0, 0.0)
    true_pose = (0.0, 0.0, 0.3)          # 真实朝向 0.3，估计以为 0
    for _ in range(60):
        e.update_bearing(landmark, _bearing_world_from_truth(e, landmark, true_pose))
    assert e.pose[2] == pytest.approx(0.3, abs=0.05)


def test_update_bearing_does_not_diverge():
    """（保留同名入口，指向有界性检查，见 test_update_bearing_stays_bounded。）"""
    test_update_bearing_stays_bounded()


def test_update_bearing_reduces_yaw_variance(ekf):
    before = ekf.std[2]
    ekf.update_bearing((5.0, 0.0), 0.0)
    assert ekf.std[2] < before


def test_update_bearing_stays_bounded():
    """位置+朝向都偏时，反复更新后估计必须**有界**（不能发散）。

    这是 H 符号写反时最直接的后果：实测估计会跑到 (10.2, 4.7) 并继续飞。
    注意这里只断言"有界"，不断言"误差单调下降"——单个静止地标对方位角观测
    是**不可观测**的（3 个状态、1 个标量观测），误差在某个方向上变差是正常的，
    真正的位置约束来自底盘运动 + 绝对位姿，视觉柱子只是补充。
    """
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    landmark = (5.0, 0.0)
    true_pose = (0.4, -0.6, 0.2)
    peak = 0.0
    for _ in range(200):
        e.update_bearing(landmark, _bearing_world_from_truth(e, landmark, true_pose))
        peak = max(peak, max(abs(v) for v in e.pose))
    assert peak < 5.0, f"估计发散：峰值 {peak:.2f}"
    assert e.std[0] < 5.0


def test_update_bearing_reduces_residual(ekf):
    """观测与几何不一致时，反复更新后残差必须变小（而不是越修越大）。"""
    from rb_localization.ekf2d import wrap_pi

    lm = (5.0, 0.0)
    meas = 0.2

    def residual() -> float:
        dx, dy = lm[0] - ekf.pose[0], lm[1] - ekf.pose[1]
        return abs(wrap_pi(meas - math.atan2(dy, dx)))

    r0 = residual()
    for _ in range(50):
        ekf.update_bearing(lm, meas)
    assert residual() < r0


def test_update_bearing_does_not_move_when_consistent():
    """观测与几何完全一致时（残差为 0），状态不应被推动。"""
    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    lm = (5.0, 0.0)
    geometric = math.atan2(lm[1] - 0.0, lm[0] - 0.0)   # = 0
    before = e.pose
    e.update_bearing(lm, geometric)
    assert e.pose == pytest.approx(before, abs=1e-9)


def test_wrap_pi_is_stable_for_many_angles():
    from rb_localization.ekf2d import wrap_pi
    for k in range(-20, 21):
        v = wrap_pi(k * 0.9)
        assert -math.pi - 1e-9 <= v <= math.pi + 1e-9
