"""感知回调链路的接口对齐测试。

⚠️ 起因（真事故）：给 Detection 加 source 字段时，把代码写成了
       out = DetectionArray(); out.detections = [...]
       for d in out:            # ← DetectionArray 不可迭代！
   节点一收到图像就 TypeError 崩掉。现象是"检测数永远是 0、寻球跑不了"，
   但**单测全绿** —— 因为单测从不跑回调链路。这里补上这类接口对齐检查。
"""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

_WS = Path(__file__).resolve().parents[3]
_NODE = _WS / "src/rb_perception/rb_perception/perception_node.py"


def _node_src() -> str:
    return _NODE.read_text(encoding="utf-8")


def test_no_iteration_over_the_message_object():
    """绝不允许 `for d in out:` 这种写法（out 是 DetectionArray 消息）。

    只做静态检查：抓 `for ... in out:` 且 out 在该函数里被赋值为 DetectionArray。
    """
    src = _node_src()
    assert "DetectionArray()" in src
    bad = []
    for i, line in enumerate(src.splitlines(), 1):
        s = line.strip()
        if s.startswith("for ") and " in out:" in s:
            bad.append((i, s))
    assert not bad, f"这些地方在遍历消息对象（会 TypeError）: {bad}"


def test_fuse_is_called_with_detection_objects_not_message():
    """fuse_lidar_balls 要传 Detection 对象列表，不能传消息。"""
    src = _node_src()
    i = src.find("fuse_lidar_balls(")
    assert i > 0
    seg = src[i:i + 200]
    assert "found" in seg or "out.detections" in seg, \
        f"融合的参数看起来还是消息对象: {seg[:120]}"


def test_to_msg_sets_source_and_reads_bbox():
    """to_msg 必须设 source，并且读 d.bbox —— LidarDetection 也得有 bbox。"""
    src = _node_src()
    i = src.find("def to_msg(")
    assert i > 0
    body = src[i:i + 1200]
    assert "m.source" in body, "to_msg 没设 source（界面就分不清看到/扫到）"
    assert "d.bbox" in body, "to_msg 读 d.bbox —— 被传对象必须有这个属性"


def test_lidar_detection_has_every_field_to_msg_needs():
    """把 to_msg 需要的属性逐个对一遍 —— 接口不齐就是下一个 TypeError。"""
    import sys
    sys.path.insert(0, str(_WS / "build" / "rb_perception"))
    from rb_perception.lidar_ball import LidarDetection
    d = LidarDetection({"x": 2.0, "y": -0.5, "z": -1.08, "n": 300})
    for attr in ("label", "confidence", "bbox", "px", "py", "bbox_px_w", "bbox_px_h",
                 "bearing_rad", "distance_m", "diameter_m", "source"):
        assert hasattr(d, attr), f"LidarDetection 缺 {attr}（to_msg 会炸）"
    # bbox 必须是 4 元组，且能被 float() 转（NaN 可以）
    assert len(d.bbox) == 4
    for v in d.bbox:
        float(v)


def test_fusion_returns_list_of_objects():
    import sys
    sys.path.insert(0, str(_WS / "build" / "rb_perception"))
    from rb_perception.lidar_ball import fuse
    cam = []
    cands = [{"x": 2.0, "y": -0.5, "z": -1.08, "n": 300}]
    out, st = fuse(cam, cands, {"enabled": True, "mode": "fuse"})
    assert isinstance(out, list), "融合必须返回列表，不是消息对象"
    assert len(out) == 1 and getattr(out[0], "source", "") == "lidar"
