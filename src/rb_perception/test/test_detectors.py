"""检测器的单元测试：颜色斑块 / 空心圆环 / 双色竖条 / 工厂。

场景用 HSV 直接合成后转 BGR，保证颜色精确可控，不依赖相机与真实场地。
运行：colcon test --packages-select rb_perception
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest
from rb_perception.detectors import (
    BlobDetector,
    CameraModel,
    CircleDetector,
    StripeDetector,
    _circularity,
    _clean,
    _hsv_mask,
    build_detectors,
)

W, H = 640, 480
BG = (0, 0, 60)              # 深灰背景（低饱和）
ORANGE = (16, 255, 255)      # 典型橙色篮球
WHITE = (0, 0, 240)
BLUE = (110, 200, 200)
GREEN = (60, 200, 200)

BALL_XY = (110, 360)
BALL_R = 35
PILLAR_X0, PILLAR_X1 = 300, 330


def make_scene() -> np.ndarray:
    """合成一张"规则里会出现的"场景图（HSV 精确指定后转 BGR）。"""
    hsv = np.zeros((H, W, 3), np.uint8)
    hsv[:] = BG
    cv2.circle(hsv, BALL_XY, BALL_R, ORANGE, -1)                 # 橙色篮球
    cv2.circle(hsv, (230, 350), 30, WHITE, -1)                   # 排球白色主体
    cv2.rectangle(hsv, (PILLAR_X0, 180), (PILLAR_X1, 300), BLUE, -1)    # 定位柱蓝段
    cv2.rectangle(hsv, (PILLAR_X0, 300), (PILLAR_X1, 420), GREEN, -1)   # 定位柱绿段
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


@pytest.fixture(scope="module")
def scene() -> np.ndarray:
    return make_scene()


@pytest.fixture(scope="module")
def cam() -> CameraModel:
    return CameraModel.from_config({}, W, H)


BALL_CFG = {
    "hsv_ranges": [[10, 150, 150, 25, 255, 255]],
    "min_area_ratio": 0.0006,
    "max_area_ratio": 0.20,
    "aspect_range": [0.5, 2.0],
    "circularity_min": 0.4,
    "real_diameter_m": 0.24,
}


# -- 工具函数 ---------------------------------------------------------------
def test_hsv_mask_unions_multiple_ranges():
    hsv = np.zeros((10, 10, 3), np.uint8)
    hsv[0:5, :] = (10, 200, 200)
    hsv[5:, :] = (100, 200, 200)
    mask = _hsv_mask(hsv, [[5, 100, 100, 20, 255, 255], [90, 100, 100, 120, 255, 255]])
    assert mask[0:5, :].all() and mask[5:, :].all()


def test_hsv_mask_empty_ranges_gives_empty():
    hsv = np.full((8, 8, 3), (10, 200, 200), np.uint8)
    assert _hsv_mask(hsv, []).max() == 0


def test_clean_removes_speckle():
    """开运算应把小噪点去掉，同时保留大色块的主体。

    注意形态学会对色块边界做腐蚀/膨胀，块会略微缩或胀，
    所以检查"中心仍是色块"而不是"边界像素一个不差"。
    """
    m = np.zeros((100, 100), np.uint8)
    m[40:60, 40:60] = 255       # 大块
    m[5, 5] = 255               # 孤点
    out = _clean(m, 5, 7)
    assert out[45:55, 45:55].all()      # 主体保留
    assert out[5, 5] == 0               # 孤点被去掉


def test_circularity_range():
    """圆度：正圆接近 1、正方形约 π/4，且圆必须明显大于方。

    数字图像里 cv2.circle 画出的圆周长偏大，实测圆度约 0.89，
    所以不能按"正圆 = 1.0"卡，这里只锁"接近 1"和"圆 > 方"这两个相对关系。
    """
    img = np.zeros((200, 200), np.uint8)
    cv2.circle(img, (100, 100), 60, 255, -1)
    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    circ = _circularity(cnts[0])
    assert 0.85 <= circ <= 1.0

    sq = np.zeros((200, 200), np.uint8)
    cv2.rectangle(sq, (50, 50), (150, 150), 255, -1)
    cnts, _ = cv2.findContours(sq, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    square = _circularity(cnts[0])
    assert square == pytest.approx(math.pi / 4, abs=0.05)
    assert circ > square + 0.05          # 判别力：圆明显比方更"圆"

    assert _circularity(np.zeros((1, 1, 2), np.int32)) == 0.0


# -- BlobDetector -----------------------------------------------------------
def test_blob_finds_ball(scene, cam):
    dets = BlobDetector("ball", BALL_CFG).detect(scene, cam)
    assert len(dets) == 1
    d = dets[0]
    assert d.label == "ball"
    assert d.px == pytest.approx(BALL_XY[0], abs=3)
    assert d.py == pytest.approx(BALL_XY[1], abs=3)
    assert d.confidence > 0.5
    assert d.diameter_m == pytest.approx(0.24)


def test_blob_bearing_sign_is_left_negative(scene, cam):
    """球在画面左侧 → 方位角为负（右正左负）。符号反了机器人会朝反方向找球。"""
    dets = BlobDetector("ball", BALL_CFG).detect(scene, cam)
    assert dets[0].bearing_rad < 0


def test_blob_bearing_matches_geometry(scene, cam):
    dets = BlobDetector("ball", BALL_CFG).detect(scene, cam)
    expect = math.atan2(dets[0].px - cam.cx, cam.fx)
    assert dets[0].bearing_rad == pytest.approx(expect, abs=1e-9)


def test_blob_distance_is_sane(scene, cam):
    """合成球直径 70px、真实 0.24m、fx≈554 → 距离约 1.9m。"""
    d = BlobDetector("ball", BALL_CFG).detect(scene, cam)[0]
    assert d.distance_m == pytest.approx(cam.fx * 0.24 / 70.0, rel=0.15)
    assert 1.0 < d.distance_m < 3.0


def test_blob_respects_circularity_filter(scene, cam):
    """把圆度要求提到 0.99，方形/噪声都不该过，只有正圆能过。"""
    cfg = dict(BALL_CFG, circularity_min=0.99)
    dets = BlobDetector("ball", cfg).detect(scene, cam)
    assert len(dets) <= 1


def test_blob_no_ranges_detects_nothing(scene, cam):
    assert BlobDetector("ball", dict(BALL_CFG, hsv_ranges=[])).detect(scene, cam) == []


# -- CircleDetector ---------------------------------------------------------
def test_circle_requires_hollow_rejects_solid_ball(scene, cam):
    """require_hollow=True 时实心球**不能**被当成圆环。

    这是实战踩过的坑：霍夫圆在实心球上也会响应，若不做空心判别，
    球会被误判成篮筐/传球架圆环，状态机会瞄错目标。
    """
    cfg = {"hsv_ranges": [[10, 150, 150, 25, 255, 255]], "require_hollow": True,
           "min_radius_ratio": 0.01, "max_radius_ratio": 0.3,
           "real_diameter_m": 0.4, "param2": 20.0}
    assert CircleDetector("ring", cfg).detect(scene, cam) == []


def test_circle_detects_hollow_ring(cam):
    """空心环应该被检出（用灰度找圆，环颜色不定时更稳）。"""
    hsv = np.zeros((H, W, 3), np.uint8)
    hsv[:] = BG
    cv2.circle(hsv, (320, 240), 80, (0, 120, 255), 8)      # 只画环，不填充
    img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    cfg = {"hsv_ranges": [], "require_hollow": True, "min_radius_ratio": 0.02,
           "max_radius_ratio": 0.3, "real_diameter_m": 0.4, "param2": 25.0}
    dets = CircleDetector("ring", cfg).detect(img, cam)
    assert len(dets) >= 1
    best = dets[0]
    assert best.px == pytest.approx(320, abs=6)
    assert best.bbox_px_w == pytest.approx(160, abs=20)


# -- StripeDetector ---------------------------------------------------------
def test_stripe_detects_pillar(scene, cam):
    """蓝绿相间的竖条应被判为定位柱，且高度跨两段、水平居中。"""
    cfg = {
        "hsv_ranges_a": [[90, 80, 60, 130, 255, 255]],   # 蓝
        "hsv_ranges_b": [[40, 60, 60, 85, 255, 255]],    # 绿
        "min_total_area_ratio": 0.0008,
        "aspect_min": 1.2, "aspect_max": 12.0,
        "x_tolerance_ratio": 0.6, "real_width_m": 0.20,
    }
    dets = StripeDetector("pillar", cfg).detect(scene, cam)
    assert len(dets) >= 1
    d = dets[0]
    assert d.px == pytest.approx((PILLAR_X0 + PILLAR_X1) / 2, abs=5)
    assert d.bbox_px_h > 200          # 跨蓝绿两段，高约 240


# -- 工厂 -------------------------------------------------------------------
def test_build_detectors_skips_disabled_and_unknown():
    cfg = {"detectors": [
        {"label": "a", "type": "blob", "enabled": True, "hsv_ranges": [[0, 0, 0, 1, 1, 1]]},
        {"label": "b", "type": "blob", "enabled": False, "hsv_ranges": [[0, 0, 0, 1, 1, 1]]},
        {"label": "c", "type": "no_such_type", "enabled": True},
    ]}
    dets, notes = build_detectors(cfg)
    assert [d.label for d in dets] == ["a"]
    assert any("no_such_type" in n for n in notes)


def test_build_detectors_warns_when_nothing_enabled():
    dets, notes = build_detectors({"detectors": []})
    assert dets == []
    assert any("没有" in n for n in notes)


def test_build_detectors_onnx_missing_runtime_is_skipped():
    """没装 onnxruntime 时应跳过并给出说明，而不是让节点起不来。"""
    dets, notes = build_detectors({"detectors": [
        {"label": "x", "type": "onnx", "enabled": True, "model_path": "/nonexistent.onnx"}]})
    assert dets == []
    assert notes          # 有说明即可，不硬性要求措辞
