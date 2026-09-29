"""OnnxDetector 的单元测试（不需要真实模型文件）。

为什么重点测这几块：ONNX 检测器最容易错、又最难从现象看出问题的地方是
  ① **输出布局判定**：YOLOv8/v11 是 (1, 4+nc, N)，v5/v7 是 (1, N, 5+nc)。
     判错就会把坐标当成类别分数，输出一堆垃圾框，而且**不报错**。
  ② **letterbox 反变换**：缩放+填充后要把框映射回原图，算错框就整体偏移/缩放。
  ③ **NMS**：按类抑制、阈值边界。
  ④ **类别映射**：模型类别名 → 工程内部标签（name_map > class_map > 原名）。

这些都纯函数可测，所以不依赖 onnxruntime 与模型文件。
运行：colcon test --packages-select rb_perception
"""

from __future__ import annotations

import numpy as np
import pytest
from rb_perception.detectors import CameraModel, OnnxDetector


def _det(**cfg) -> OnnxDetector:
    """建一个"没加载模型"的检测器（用不存在的路径），再按需覆盖属性。

    这样既能真实走一遍 __init__ 的参数解析，又不需要模型文件。
    """
    cfg.setdefault("model_path", "/nonexistent/__no_model__.onnx")
    d = OnnxDetector("yolo", cfg)
    assert not d.available          # 确认没加载到模型
    if "input_size" in cfg:
        d.input_size = cfg["input_size"]
    return d


# ---------------------------------------------------------------------------
# 输出布局判定
# ---------------------------------------------------------------------------
def test_decode_v8_layout_channels_first():
    """YOLOv8/v11: (1, 4+nc, N)，无 objectness。"""
    d = _det(input_size=640, conf_threshold=0.5)
    d.model_names = {0: "basketball", 1: "volleyball"}      # nc=2 → 4+2=6
    # 1 个候选框：cx,cy,w,h + 两个类别分数
    pred = np.array([[[320.0], [240.0], [40.0], [40.0], [0.9], [0.05]]], np.float32)
    boxes, scores, cids = d._decode(pred)
    assert len(boxes) == 1
    assert cids[0] == 0
    assert scores[0] == pytest.approx(0.9, abs=1e-5)
    assert boxes[0] == pytest.approx([300, 220, 340, 260], abs=1e-3)


def test_decode_v5_layout_with_objectness():
    """YOLOv5/v7: (1, N, 5+nc)，第 5 列是 objectness，最终分数 = obj × cls。"""
    d = _det(conf_threshold=0.1)
    d.model_names = {0: "a", 1: "b"}                        # nc=2 → 5+2=7
    pred = np.array([[[320, 240, 40, 40, 0.8, 0.5, 0.1]]], np.float32)
    boxes, scores, cids = d._decode(pred)
    assert len(boxes) == 1
    assert cids[0] == 0
    assert scores[0] == pytest.approx(0.8 * 0.5, abs=1e-5)   # objectness × 类别分


def test_decode_handles_transposed_v8_output():
    """有些导出是 (1, N, 4+nc)，也必须能正确识别（按"属性维较小"判定）。"""
    d = _det(conf_threshold=0.5)
    d.model_names = {0: "x"}
    pred = np.array([[[320, 240, 40, 40, 0.9]]], np.float32)   # (1,1,5)
    boxes, scores, cids = d._decode(pred)
    assert len(boxes) == 1 and cids[0] == 0


def test_decode_thresholds_low_scores():
    d = _det(conf_threshold=0.5)
    d.model_names = {0: "x"}
    pred = np.array([[[320.0], [240.0], [40.0], [40.0], [0.3]]], np.float32)
    boxes, scores, _ = d._decode(pred)
    assert len(boxes) == 0


def test_decode_empty_when_all_below_threshold():
    d = _det(conf_threshold=0.99)
    d.model_names = {0: "x"}
    pred = np.zeros((1, 5, 10), np.float32)
    boxes, scores, cids = d._decode(pred)
    assert boxes.size == 0 and scores.size == 0


# ---------------------------------------------------------------------------
# NMS
# ---------------------------------------------------------------------------
def test_nms_suppresses_overlapping_keeps_distinct():
    boxes = np.array([[0, 0, 100, 100], [5, 5, 105, 105], [500, 500, 600, 600]], np.float32)
    scores = np.array([0.9, 0.8, 0.7], np.float32)
    keep = OnnxDetector._nms(boxes, scores, 0.5)
    assert 0 in keep                 # 最高分保留
    assert 1 not in keep             # 与 0 高度重叠 → 抑制
    assert 2 in keep                 # 远处独立框 → 保留


def test_nms_keeps_all_when_no_overlap():
    boxes = np.array([[0, 0, 10, 10], [100, 100, 110, 110]], np.float32)
    scores = np.array([0.5, 0.4], np.float32)
    assert sorted(OnnxDetector._nms(boxes, scores, 0.5)) == [0, 1]


def test_nms_empty():
    assert OnnxDetector._nms(np.empty((0, 4), np.float32), np.empty(0, np.float32), 0.5) == []


# ---------------------------------------------------------------------------
# letterbox
# ---------------------------------------------------------------------------
def test_letterbox_geometry_and_inverse():
    """16:9 的图放进 640 方框：宽度铺满、上下留灰边，反变换必须能还原。"""
    d = _det(input_size=640)
    frame = np.zeros((720, 1280, 3), np.uint8)
    tensor, s, pad_x, pad_y = d._letterbox(frame)
    assert tensor.shape == (1, 3, 640, 640)
    assert s == pytest.approx(640 / 1280)
    assert pad_x == 0
    assert pad_y == (640 - int(round(720 * s))) // 2

    # 正反变换：原图上的点 → letterbox → 反算回来应一致
    px, py = 900.0, 500.0
    lx, ly = px * s + pad_x, py * s + pad_y
    assert (lx - pad_x) / s == pytest.approx(px)
    assert (ly - pad_y) / s == pytest.approx(py)


def test_letterbox_normalizes_to_0_1():
    d = _det(input_size=64)
    frame = np.full((32, 32, 3), 255, np.uint8)
    tensor, _, _, _ = d._letterbox(frame)
    assert tensor.max() == pytest.approx(1.0, abs=1e-6)
    assert tensor.min() >= 0.0


# ---------------------------------------------------------------------------
# 类别映射
# ---------------------------------------------------------------------------
def test_map_label_priority_name_over_id():
    d = _det(class_map={0: "ball_basketball"},
             name_map={"volleyball": "ball_volleyball"})
    assert d._map_label(0, "volleyball") == "ball_volleyball"   # name_map 优先
    assert d._map_label(0, "") == "ball_basketball"             # 无名时用 class_map


def test_map_label_falls_back_to_model_name_then_synthetic():
    d = _det()
    assert d._map_label(3, "basketball") == "basketball"
    assert d._map_label(7, "") == "yolo_7"


# ---------------------------------------------------------------------------
# 端到端：伪造一次完整 detect（用假 session，走完整预处理/后处理链）
# ---------------------------------------------------------------------------
class _FakeSession:
    def __init__(self, out):
        self._out = out

    def run(self, _names, _feed):
        return [self._out]


def test_detect_end_to_end_geometry():
    """用假 session 跑完整 detect，验证框/置信度/类别都正确落到 Detection。

    这里也顺便锁住"letterbox 反变换"与"归一化 bbox"是否一致 —— 这两处算错
    在实车上表现为"框画在球旁边"，很难定位。
    """
    d = _det(input_size=640, conf_threshold=0.4, real_diameter_m=0.24)
    d.model_names = {0: "basketball", 1: "volleyball"}
    d.class_map = {0: "ball_basketball", 1: "ball_volleyball"}
    # 让模型"看到"一个位于 letterbox 中心、80x80 的框，类别 1
    d.session = _FakeSession(np.array(
        [[[320.0], [320.0], [80.0], [80.0], [0.02], [0.95]]], np.float32))

    frame = np.zeros((720, 1280, 3), np.uint8)
    cam = CameraModel.from_config({}, 1280, 720)
    dets = d.detect(frame, cam)

    assert len(dets) == 1
    det = dets[0]
    assert det.label == "ball_volleyball"
    assert det.confidence == pytest.approx(0.95, abs=1e-4)

    s = 640 / 1280
    pad_y = (640 - int(round(720 * s))) // 2
    # letterbox 中心 (320,320) → 原图中心附近
    exp_px = (320 - 0) / s
    exp_py = (320 - pad_y) / s
    assert det.px == pytest.approx(exp_px, abs=1.0)
    assert det.py == pytest.approx(exp_py, abs=1.0)
    # bbox 归一化：内部 Detection 用 bbox 元组，msg 里才拆成 x_min/x_max
    x0, y0, x1, y1 = det.bbox
    assert 0.0 <= x0 < x1 <= 1.0
    assert 0.0 <= y0 < y1 <= 1.0
    # 方位角符号：框在画面中心 → 接近 0
    assert abs(det.bearing_rad) < 0.05
    # 单目测距：给了真实直径就应该有数
    assert det.distance_m > 0


def test_every_n_frames_reuses_cached_result():
    """跳帧时**必须复用上次结果**，不能返回空。

    返回空会让下游以为"球突然不见了"（SEEK_BALL 会开始原地搜索），
    表现为机器人走走停停 —— 比掉帧更糟。
    """
    d = _det(input_size=640, conf_threshold=0.4, every_n_frames=2,
             class_map={0: "ball_basketball"})
    d.model_names = {0: "basketball"}
    d.session = _FakeSession(np.array(
        [[[320.0], [320.0], [80.0], [80.0], [0.9]]], np.float32))
    frame = np.zeros((720, 1280, 3), np.uint8)
    cam = CameraModel.from_config({}, 1280, 720)

    first = d.detect(frame, cam)      # 第 1 帧：非推理帧（1%2!=0）→ 还没有缓存
    assert first == []                # 首次跳帧确实没有历史结果可说
    second = d.detect(frame, cam)     # 第 2 帧：推理
    assert len(second) == 1
    third = d.detect(frame, cam)      # 第 3 帧：跳帧 → 复用
    assert len(third) == 1, "跳帧返回了空，会让状态机以为球丢了"
    assert third[0].label == second[0].label


def test_every_n_frames_one_runs_every_frame():
    d = _det(input_size=640, conf_threshold=0.4, every_n_frames=1,
             class_map={0: "ball_basketball"})
    d.model_names = {0: "b"}
    d.session = _FakeSession(np.array(
        [[[320.0], [320.0], [80.0], [80.0], [0.9]]], np.float32))
    frame = np.zeros((720, 1280, 3), np.uint8)
    cam = CameraModel.from_config({}, 1280, 720)
    assert len(d.detect(frame, cam)) == 1      # 第 1 帧就要出结果


def test_only_mapped_drops_unmapped_classes():
    """⭐ 只保留映射过的类别。

    COCO 预训练有 80 类，不筛的话 person/chair/cat 会全混进 /perception/detections，
    而且会被套上 real_diameter_m（按篮球尺寸）算出荒谬距离（实测 person 报 0.28m）。
    """
    d = _det(input_size=640, conf_threshold=0.3, only_mapped=True,
             name_map={"sports ball": "ball_unknown"}, real_diameter_m=0.24)
    d.model_names = {0: "sports ball", 1: "person"}
    # 两个候选分属不同类别：候选0 → class0(球，映射过)，候选1 → class1(人，没映射)
    out = np.array([[[320.0, 480.0], [200.0, 200.0], [80.0, 80.0], [80.0, 80.0],
                     [0.9, 0.1], [0.1, 0.9]]], np.float32)
    d.session = _FakeSession(out)
    dets = d.detect(np.zeros((720, 1280, 3), np.uint8),
                    CameraModel.from_config({}, 1280, 720))
    assert len(dets) == 1, f"应只剩 1 个（球），实际 {[x.label for x in dets]}"
    assert dets[0].label == "ball_unknown"


def test_only_mapped_false_keeps_all():
    d = _det(input_size=640, conf_threshold=0.3, only_mapped=False,
             name_map={"sports ball": "ball_unknown"})
    d.model_names = {0: "sports ball", 1: "person"}
    out = np.array([[[320.0, 480.0], [200.0, 200.0], [80.0, 80.0], [80.0, 80.0],
                     [0.9, 0.1], [0.1, 0.9]]], np.float32)
    d.session = _FakeSession(out)
    dets = d.detect(np.zeros((720, 1280, 3), np.uint8),
                    CameraModel.from_config({}, 1280, 720))
    assert len(dets) == 2
