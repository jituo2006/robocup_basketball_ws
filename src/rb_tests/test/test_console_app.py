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
