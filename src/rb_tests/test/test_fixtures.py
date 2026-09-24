"""夹具自身的集成测试：合成图 + 检测链路。

这是**集成测试**（会导入检测器、跑真实图像处理），与各包 test/ 下的
纯单元测试区分开。运行：colcon test --packages-select rb_tests
"""

from __future__ import annotations

import pytest

from rb_tests.fakes.make_test_image import make_image


@pytest.fixture(scope="module")
def img():
    return make_image()


def test_synthetic_image_has_expected_size(img):
    assert img.shape == (480, 640, 3)


def test_synthetic_image_is_not_blank(img):
    """合成图必须有实际内容，否则"检不到"这种失败无法区分是算法问题还是图是空的。"""
    assert img.std() > 20.0


def test_synthetic_image_detects_all_rule_targets(img):
    """规则里会出现的 5 类目标都要能在合成图上被检出。

    ⚠️ 注意用的是**冻结的标准色配置** perception_test.yaml，不是生产配置。
    生产配置会被 tools/calibrate_color.py 改成真球颜色，届时合成图可能检不到，
    那是正常的（真球可能不是橙色）；这里只检验"视觉管道通不通"。
    """
    import yaml
    from rb_perception.detectors import CameraModel, build_detectors
    from rb_tests.ws import WS

    cfg_path = WS / "src" / "rb_perception" / "config" / "perception_test.yaml"
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    detectors, _ = build_detectors(cfg.get("perception", {}))
    cam = CameraModel.from_config(cfg.get("camera", {}) or {}, img.shape[1], img.shape[0])

    found: set[str] = set()
    for det in detectors:
        for r in det.detect(img, cam):
            found.add(r.label)

    expected = {"ball_basketball", "ball_volleyball", "hoop", "rack_ring", "pillar"}
    assert expected <= found, f"漏检: {sorted(expected - found)}；实际检出 {sorted(found)}"


def test_ball_bearing_sign_and_distance_are_physical(img):
    """左侧的球方位角必须为负，且测距为正的合理值（符号/公式错了直接毁掉抓球）。"""
    from rb_perception.detectors import BlobDetector, CameraModel

    cfg = {"hsv_ranges": [[5, 110, 70, 22, 255, 255]], "min_area_ratio": 0.0006,
           "max_area_ratio": 0.2, "aspect_range": [0.6, 1.7],
           "circularity_min": 0.55, "real_diameter_m": 0.24}
    cam = CameraModel.from_config({}, img.shape[1], img.shape[0])
    dets = BlobDetector("ball_basketball", cfg).detect(img, cam)
    assert dets, "橙色球没检出"
    d = dets[0]
    assert d.bearing_rad < 0          # 球在画面左侧
    assert 0.5 < d.distance_m < 5.0
