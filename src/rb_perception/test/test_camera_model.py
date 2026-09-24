"""CameraModel 的单元测试：内参回退、方位角、去畸变、单目测距。

对应实车影响：这些数值直接决定「球在哪个方向、多远」，进而影响抓球判定与避障半径。
运行：colcon test --packages-select rb_perception
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest
from rb_perception.detectors import CameraModel

FX = FY = 1000.0
CX, CY = 640.0, 360.0
W, H = 1280, 720
# 典型消费级镜头畸变
DIST = [-0.25, 0.06, 0.0012, -0.0008, 0.0]


def K() -> np.ndarray:
    return np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)


def project_with_distortion(bearing_deg: float, elev_deg: float) -> tuple[float, float]:
    """把"真实方向"投影成带畸变的像素，作为已知真值。"""
    x = math.tan(math.radians(bearing_deg))
    y = math.tan(math.radians(elev_deg))
    pts, _ = cv2.projectPoints(np.array([[[x, y, 1.0]]], np.float64),
                               np.zeros(3), np.zeros(3), K(), np.array(DIST))
    return float(pts[0, 0, 0]), float(pts[0, 0, 1])


# -- 内参回退 ---------------------------------------------------------------
def test_from_config_uses_given_intrinsics():
    cam = CameraModel.from_config({"fx": 900.0, "fy": 910.0, "cx": 300.0, "cy": 200.0}, 640, 480)
    assert (cam.fx, cam.fy, cam.cx, cam.cy) == (900.0, 910.0, 300.0, 200.0)
    assert cam.has_distortion is False


def test_from_config_falls_back_to_assumed_fov():
    """未标定时按假定 FOV 反推 fx，并把主点放在图像中心（不能崩、不能给 0）。"""
    cam = CameraModel.from_config({"assumed_hfov_deg": 60.0}, W, H)
    assert cam.fx == pytest.approx((W / 2) / math.tan(math.radians(30.0)))
    assert cam.fy == pytest.approx(cam.fx)
    assert (cam.cx, cam.cy) == (W / 2, H / 2)


def test_from_config_reads_distortion():
    cam = CameraModel.from_config({"distortion": DIST}, W, H)
    assert cam.distortion == pytest.approx(tuple(DIST))
    assert cam.has_distortion is True


def test_has_distortion_false_for_zeros():
    cam = CameraModel.from_config({"distortion": [0.0] * 5}, W, H)
    assert cam.has_distortion is False


# -- 方位角 -----------------------------------------------------------------
def test_bearing_zero_on_axis():
    cam = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY)
    assert cam.bearing_rad(CX, CY) == pytest.approx(0.0, abs=1e-9)


def test_bearing_sign_follows_image_x():
    """右为正：像素 x > cx 时方位角为正。符号反了会让机器人朝反方向转。"""
    cam = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY)
    assert cam.bearing_rad(CX + 100, CY) > 0
    assert cam.bearing_rad(CX - 100, CY) < 0


def test_bearing_matches_pinhole_without_distortion():
    cam = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY)
    assert cam.bearing_rad(CX + 500, CY) == pytest.approx(math.atan2(500.0, FX))


# -- 去畸变 -----------------------------------------------------------------
@pytest.mark.parametrize("bearing_deg,elev_deg", [
    (0, 0), (5, 0), (15, 5), (25, 10), (35, 15),
])
def test_undistort_recovers_true_bearing(bearing_deg, elev_deg):
    """带畸变的像素经去畸变后，方位角应还原到真实值（误差 < 0.5 mrad）。"""
    cam = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY, width=W, height=H, distortion=tuple(DIST))
    px, py = project_with_distortion(bearing_deg, elev_deg)
    assert cam.bearing_rad(px, py) == pytest.approx(math.radians(bearing_deg), abs=5e-4)


def test_distortion_correction_actually_matters():
    """不去畸变的误差必须显著大于去畸变后的误差（否则这段代码是白加的）。"""
    cam_d = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY, width=W, height=H, distortion=tuple(DIST))
    cam_n = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY, width=W, height=H)
    px, py = project_with_distortion(35, 15)
    true_b = math.radians(35.0)
    err_no = abs(cam_n.bearing_rad(px, py) - true_b)
    err_yes = abs(cam_d.bearing_rad(px, py) - true_b)
    assert err_no > 0.03          # 边缘处 >1.7°
    assert err_yes < 1e-3
    assert err_no > 20 * err_yes


def test_undistort_is_identity_near_principal_point():
    """光轴附近畸变位移极小，去畸变不应改变坐标（防止实现写反方向）。"""
    cam = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY, distortion=tuple(DIST))
    ux, uy = cam.undistort_px(CX, CY)
    assert (ux, uy) == pytest.approx((CX, CY), abs=1e-6)


def test_undistort_passthrough_without_distortion():
    cam = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY)
    assert cam.undistort_px(123.0, 456.0) == (123.0, 456.0)


# -- 尺寸 / 测距 -------------------------------------------------------------
def test_measure_size_identity_near_center():
    cam = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY, distortion=tuple(DIST))
    assert cam.measure_size((CX - 55, CY), (CX + 55, CY)) == pytest.approx(110.0, abs=0.5)


def test_measure_size_positive_and_corrects_at_edge():
    """边缘处去畸变后的尺寸应大于原始像素尺寸（桶形畸变把边缘压小了）。"""
    cam = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY, distortion=tuple(DIST))
    raw = 80.0
    fixed = cam.measure_size((1000 - 40, 200), (1000 + 40, 200))
    assert fixed > raw


def test_distance_scales_inversely_with_pixel_size():
    """距离与像素尺寸成反比：球看起来小一半 → 距离翻倍。"""
    cam = CameraModel(fx=FX, fy=FY, cx=CX, cy=CY)
    d1 = cam.distance_m(200.0, 0.24)
    d2 = cam.distance_m(100.0, 0.24)
    assert d2 == pytest.approx(2 * d1)


def test_distance_formula_value():
    cam = CameraModel(fx=1000.0, fy=1000.0, cx=0.0, cy=0.0)
    # d = fx * real / px = 1000 * 0.24 / 240 = 1.0
    assert cam.distance_m(240.0, 0.24) == pytest.approx(1.0)


@pytest.mark.parametrize("px_size,real", [(0.0, 0.24), (-5.0, 0.24), (100.0, 0.0), (100.0, -0.1)])
def test_distance_returns_nan_for_invalid_input(px_size, real):
    """无效输入必须返回 NaN（表示"未知"），不能返回 0 或负数被下游当成真距离。"""
    cam = CameraModel(fx=1000.0, fy=1000.0, cx=0.0, cy=0.0)
    assert math.isnan(cam.distance_m(px_size, real))


def test_distance_nan_when_uncalibrated():
    cam = CameraModel(fx=0.0, fy=0.0, cx=0.0, cy=0.0)
    assert math.isnan(cam.distance_m(100.0, 0.24))


def test_bearing_nan_when_uncalibrated():
    cam = CameraModel(fx=0.0, fy=0.0, cx=0.0, cy=0.0)
    assert math.isnan(cam.bearing_rad(100.0, 100.0))
