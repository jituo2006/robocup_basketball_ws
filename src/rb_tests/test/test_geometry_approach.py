"""单元测试：goto_command 的"接近限速"（消除冲过目标点）。

背景：反馈链路有 ~300ms 延迟（实测 /Odometry 消息时延中位 304ms）。
P 控制器离目标 0.27m 时仍是满速，超调 ≈ 延迟 × 接近速度 = 0.3 × 0.40 ≈ 12cm。
approach_radius / approach_speed 把接近段速度压下来，超调降到 ~3.6cm。

运行：colcon test --packages-select rb_tests
"""

from __future__ import annotations

import math

from rb_mission.geometry import goto_command


def _speed(vx: float, vy: float) -> float:
    return math.hypot(vx, vy)


def test_without_approach_limiting_keeps_full_speed_when_close():
    """不传 approach_* 时，行为与原来一致（P 控制器，近处仍是 kp*dist 或 max_lin）。"""
    # 距目标 0.20m，kp=1.5 → 期望 0.30 m/s（< max_lin 0.40，不触发限幅）
    vx, vy, _wz, dist = goto_command(
        0.0, 0.0, 0.0, 0.20, 0.0,
        kp_lin=1.5, kp_yaw=2.0, max_lin=0.40, max_ang=0.80,
        face_travel=False,
    )
    assert abs(dist - 0.20) < 1e-9
    assert abs(_speed(vx, vy) - 0.30) < 1e-6


def test_approach_limiting_caps_speed_inside_radius():
    """进入 approach_radius 后，速度被压到 approach_speed。"""
    # 距目标 0.30m：不限速应 0.45 m/s（被 max_lin 截到 0.40）；
    # 限速后应为 0.12。
    vx_unlimited, vy_unlimited, _, _ = goto_command(
        0.0, 0.0, 0.0, 0.30, 0.0,
        kp_lin=1.5, kp_yaw=2.0, max_lin=0.40, max_ang=0.80,
        face_travel=False,
    )
    vx_limited, vy_limited, _, _ = goto_command(
        0.0, 0.0, 0.0, 0.30, 0.0,
        kp_lin=1.5, kp_yaw=2.0, max_lin=0.40, max_ang=0.80,
        face_travel=False, approach_radius=0.45, approach_speed=0.12,
    )
    assert abs(_speed(vx_unlimited, vy_unlimited) - 0.40) < 1e-6, "不限速时应在 0.30m 处满速"
    assert abs(_speed(vx_limited, vy_limited) - 0.12) < 1e-6, "限速后应为 approach_speed"


def test_approach_limiting_not_active_outside_radius():
    """approach_radius 之外，速度不受影响。"""
    vx, vy, _, _ = goto_command(
        0.0, 0.0, 0.0, 1.00, 0.0,
        kp_lin=1.5, kp_yaw=2.0, max_lin=0.40, max_ang=0.80,
        face_travel=False, approach_radius=0.45, approach_speed=0.12,
    )
    # 1.00m 在半径外 → 仍按 max_lin 截到 0.40
    assert abs(_speed(vx, vy) - 0.40) < 1e-6


def test_approach_limiting_never_increases_speed():
    """approach_speed 比 max_lin 大时不应反而提速（取 min）。"""
    vx, vy, _, _ = goto_command(
        0.0, 0.0, 0.0, 0.50, 0.0,
        kp_lin=0.10, kp_yaw=2.0, max_lin=0.40, max_ang=0.80,
        face_travel=False, approach_radius=0.60, approach_speed=5.0,
    )
    # kp=0.1，dist=0.5 → 期望 0.05（很小），不应被 approach_speed=5 放大
    assert abs(_speed(vx, vy) - 0.05) < 1e-6


def test_approach_limiting_keeps_direction():
    """限速只改速度大小，不改方向（全向底盘平移方向必须保持）。"""
    vx, vy, _, _ = goto_command(
        0.0, 0.0, 0.0, 0.30, 0.40,   # dist = 0.5，在半径外
        kp_lin=1.5, kp_yaw=2.0, max_lin=0.40, max_ang=0.80,
        face_travel=False,
    )
    vx_l, vy_l, _, _ = goto_command(
        0.0, 0.0, 0.0, 0.30, 0.40,
        kp_lin=1.5, kp_yaw=2.0, max_lin=0.40, max_ang=0.80,
        face_travel=False, approach_radius=0.60, approach_speed=0.10,
    )
    ang = math.atan2(vy, vx)
    ang_l = math.atan2(vy_l, vx_l)
    assert abs(ang - ang_l) < 1e-9, "方向必须保持"
    assert abs(_speed(vx_l, vy_l) - 0.10) < 1e-6
