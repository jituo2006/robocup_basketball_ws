"""单元测试：yaw_from_pose_msg —— 锁住"把高度当成朝向"的真 BUG。

背景（实测踩到的真 BUG）：
  FAST-LIO 的 `pose.position.z` 是**高度**（实测 -0.007 m），不是 yaw。
  原判据 `hypot(q.x,q.y,q.z) < 1e-6` 在机器人朝向≈0° 时成立（此时四元数
  x,y,z 恰好是 0），于是把**高度当成了朝向**。高度一漂，朝向就跟着漂，
  车体系速度方向全错 → 实测表现为"过冲 + 往回走"。

  正确判据：四元数**模长**≈1 就是有效旋转。只有四元数全零（自家老消息
  不带姿态）才退回 position.z。

运行：colcon test --packages-select rb_tests
"""

from __future__ import annotations

import math
import types

from rb_mission.geometry import yaw_from_pose_msg, yaw_from_stamped_pose


def _msg(x: float, y: float, z: float, yaw: float):
    """造一个带位置和四元数的位姿消息（yaw 用标准 z 轴四元数）。"""
    m = types.SimpleNamespace()
    m.pose = types.SimpleNamespace()
    m.pose.pose = types.SimpleNamespace()
    m.pose.pose.position = types.SimpleNamespace(x=x, y=y, z=z)
    q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
    m.pose.pose.orientation = q
    return m


def test_yaw_zero_does_not_use_height():
    """朝向 0° 时四元数 x,y,z 都是 0 —— 绝不能把高度(z)当成 yaw。"""
    m = _msg(1.0, 2.0, z=-0.007, yaw=0.0)   # z = 高度 -7mm
    assert abs(yaw_from_pose_msg(m)) < 1e-9, "朝向 0° 必须返回 0，不能返回高度 -0.007"


def test_yaw_zero_with_large_height():
    """高度大时更不能拿它当 yaw（原来的 bug 会返回 0.5 rad ≈ 28.6°）。"""
    m = _msg(0.0, 0.0, z=0.5, yaw=0.0)
    got = yaw_from_pose_msg(m)
    assert abs(got) < 1e-9, f"应返回 0，实际 {got}（把 0.5m 高度当成了朝向）"


def test_yaw_recovers_actual_orientation():
    for yaw in (0.0, 0.3, -1.2, math.pi / 2, math.pi - 0.01, -math.pi + 0.01):
        m = _msg(0.0, 0.0, z=0.123, yaw=yaw)
        got = yaw_from_pose_msg(m)
        assert abs(got - yaw) < 1e-6, f"yaw={yaw} 应还原，实际 {got}"


def test_falls_back_to_z_when_quaternion_is_zero():
    """四元数全零（自家老消息不带姿态）时才退回 position.z。"""
    m = _msg(0.0, 0.0, z=0.75, yaw=0.0)
    m.pose.pose.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)
    assert abs(yaw_from_pose_msg(m) - 0.75) < 1e-9


# ── PoseStamped 版本（RViz "2D Goal Pose"）─────────────────────────────────

def test_stamped_pose_yaw_zero_ignores_height():
    """RViz 点击给的 position.z 常是 0，但若有人填了高度，绝不能被当朝向。"""
    from rb_mission.geometry import yaw_from_stamped_pose

    m = types.SimpleNamespace()
    m.pose = types.SimpleNamespace()
    m.pose.position = types.SimpleNamespace(x=3.0, y=-1.0, z=0.42)   # 高度 42cm
    m.pose.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)  # 朝向 0°
    got = yaw_from_stamped_pose(m)
    assert abs(got) < 1e-9, f"应返回 0，实际 {got}（把 0.42m 高度当成了朝向）"


def test_stamped_pose_recovers_orientation():
    from rb_mission.geometry import yaw_from_stamped_pose

    for yaw in (0.0, 0.7, -2.0, math.pi / 2):
        m = types.SimpleNamespace()
        m.pose = types.SimpleNamespace()
        m.pose.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        m.pose.orientation = types.SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        assert abs(yaw_from_stamped_pose(m) - yaw) < 1e-6
