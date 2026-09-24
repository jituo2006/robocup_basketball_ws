"""RoboCup 篮球机器人 · 视觉检测器集合。

设计原则
--------
1. **不写死颜色阈值**。2026 规则要求识别"各队自带用球的颜色/图案差异"，
   所以所有颜色范围都来自配置，并提供在线标定流程（见 calibrate.py）。
2. **检测器可插拔**。每种目标一个检测器，配置里决定启用哪些。
3. **不依赖 onnxruntime**。本机没装 onnxruntime，所以基线是"颜色+形状"，
   但保留 OnnxDetector 接口，装上依赖后可直接启用。
4. **全部可在无相机情况下自测**：见 tools/make_test_image.py。

坐标约定
--------
- bbox 全部归一化到 0..1（左上角为原点）
- bearing_rad：目标相对**相机光轴**的水平方位角，右正左负（b = atan((px-cx)/fx)）
- distance_m：单目测距 d = fx * real_size / pixel_size，未标定时为 NaN
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------


@dataclass
class CameraModel:
    """针孔相机模型。未标定时 fx/fy 由图像宽度和假定 FOV 估算。

    畸变
    ----
    带 `distortion`（k1,k2,p1,p2,k3，来自 calibrate_camera.py）时，计算方位角/
    尺寸前会先把像素坐标**去畸变**。为什么必须做：畸变让目标在图像上的位置
    相对理想针孔模型系统性偏移，越靠边缘越明显（典型镜头在 720p 边缘可达
    20~30px ≈ 1°~2°，在 3m 处就是 5~10cm）。抓球有闭环能修，但**瞄准投/传
    是按方位角对准的，系统偏差不会自己消失**，所以要修。
    """

    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    width: int = 0
    height: int = 0
    distortion: tuple[float, ...] = ()

    @staticmethod
    def from_config(cfg: dict[str, Any], width: int, height: int) -> "CameraModel":
        fx = float(cfg.get("fx", 0.0) or 0.0)
        fy = float(cfg.get("fy", 0.0) or 0.0)
        cx = float(cfg.get("cx", 0.0) or 0.0)
        cy = float(cfg.get("cy", 0.0) or 0.0)

        if fx <= 0.0:
            # 用假定水平 FOV 反推（默认 60°，常见 USB 相机量级）
            hfov_deg = float(cfg.get("assumed_hfov_deg", 60.0))
            fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        if fy <= 0.0:
            fy = fx
        if cx <= 0.0:
            cx = width / 2.0
        if cy <= 0.0:
            cy = height / 2.0
        dist = tuple(float(v) for v in (cfg.get("distortion", []) or []))
        return CameraModel(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height,
                           distortion=dist)

    @property
    def has_distortion(self) -> bool:
        return len(self.distortion) >= 4 and any(abs(v) > 1e-9 for v in self.distortion[:5])

    def _K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], dtype=np.float64)

    def undistort_px(self, px: float, py: float) -> tuple[float, float]:
        """把**畸变图像**上的像素坐标换算到理想针孔模型下的像素坐标。

        用 cv2.undistortPoints 并传 P=K，输出仍在同一像素尺度，
        于是可以直接喂给 bearing_rad / measure_size。
        """
        if not self.has_distortion:
            return float(px), float(py)
        pts = np.array([[[float(px), float(py)]]], dtype=np.float64)
        d = np.array(list(self.distortion[:5]) + [0.0] * max(0, 5 - len(self.distortion)),
                     dtype=np.float64)
        out = cv2.undistortPoints(pts, self._K(), d, P=self._K())
        return float(out[0, 0, 0]), float(out[0, 0, 1])

    def bearing_rad(self, px: float, py: float | None = None) -> float:
        """像素 x → 水平方位角（右正左负）。

        py 参与去畸变（径向畸变同时依赖 x 与 y），所以**应尽量传**；
        不传则退化为"不做去畸变"的老行为（仅为兼容）。
        """
        if self.fx <= 0:
            return float("nan")
        if py is not None:
            px, _ = self.undistort_px(px, py)
        return math.atan2(px - self.cx, self.fx)

    def measure_size(self, p0: tuple[float, float], p1: tuple[float, float]) -> float:
        """两点在**去畸变后**图像上的距离（像素）。用于测距的像素尺寸。"""
        u0 = self.undistort_px(*p0)
        u1 = self.undistort_px(*p1)
        return math.hypot(u1[0] - u0[0], u1[1] - u0[1])

    def distance_m(self, pixel_size: float, real_size_m: float) -> float:
        """单目测距：已知目标物理尺寸时估算距离。pixel_size 应是去畸变后的尺寸。"""
        if pixel_size <= 1e-6 or real_size_m <= 0 or self.fx <= 0:
            return float("nan")
        return self.fx * real_size_m / pixel_size


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]  # 归一化
    px: float = 0.0
    py: float = 0.0
    bbox_px_w: float = 0.0
    bbox_px_h: float = 0.0
    bearing_rad: float = float("nan")
    distance_m: float = float("nan")
    diameter_m: float = float("nan")


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------


def _hsv_mask(frame_hsv: np.ndarray, ranges: Sequence[Sequence[int]]) -> np.ndarray:
    """把多段 HSV 范围合成一张掩码（H 用 OpenCV 的 0..179）。"""
    mask = np.zeros(frame_hsv.shape[:2], dtype=np.uint8)
    for item in ranges:
        if len(item) != 6:
            continue
        lo = np.array(item[:3], dtype=np.uint8)
        hi = np.array(item[3:], dtype=np.uint8)
        mask |= cv2.inRange(frame_hsv, lo, hi)
    return mask


def _clean(mask: np.ndarray, open_ksize: int = 5, close_ksize: int = 7) -> np.ndarray:
    """开运算去噪 + 闭运算补洞。"""
    if open_ksize >= 3:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_ksize, open_ksize)))
    if close_ksize >= 3:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_ksize, close_ksize)))
    return mask


def _circularity(contour: np.ndarray) -> float:
    area = cv2.contourArea(contour)
    peri = cv2.arcLength(contour, True)
    if peri <= 1e-6:
        return 0.0
    return float(4.0 * math.pi * area / (peri * peri))


# ---------------------------------------------------------------------------
# 检测器基类
# ---------------------------------------------------------------------------


class BaseDetector:
    label: str = "unknown"

    def detect(self, frame_bgr: np.ndarray, cam: CameraModel) -> list[Detection]:
        raise NotImplementedError


class BlobDetector(BaseDetector):
    """颜色斑块 + 轮廓形状过滤。适合：球（圆形）、定位柱的单个色段。"""

    def __init__(self, label: str, cfg: dict[str, Any]) -> None:
        self.label = label
        self.hsv_ranges = cfg.get("hsv_ranges", [])
        self.min_area_ratio = float(cfg.get("min_area_ratio", 0.0005))
        self.max_area_ratio = float(cfg.get("max_area_ratio", 0.5))
        self.aspect_range = cfg.get("aspect_range", [0.5, 2.0])
        self.circularity_min = float(cfg.get("circularity_min", 0.0))
        self.real_diameter_m = float(cfg.get("real_diameter_m", 0.0) or 0.0)
        self.use_bbox_width_for_distance = bool(cfg.get("use_bbox_width_for_distance", True))

    def detect(self, frame_bgr: np.ndarray, cam: CameraModel) -> list[Detection]:
        if not self.hsv_ranges:
            return []

        h, w = frame_bgr.shape[:2]
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = _clean(_hsv_mask(hsv, self.hsv_ranges))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        total = float(h * w)
        out: list[Detection] = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            ratio = area / total
            if ratio < self.min_area_ratio or ratio > self.max_area_ratio:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            if bh <= 0:
                continue
            aspect = bw / float(bh)
            if not (self.aspect_range[0] <= aspect <= self.aspect_range[1]):
                continue

            circ = _circularity(cnt)
            if circ < self.circularity_min:
                continue

            # 置信度：面积越大、越接近正圆越可信（截断到 0..1）
            conf = float(min(1.0, 0.35 + 0.65 * min(circ, 1.0)))

            px = x + bw / 2.0
            py = y + bh / 2.0
            # 去畸变后再算：方位角用中心点，像素尺寸用去畸变后的宽/高
            uw = cam.measure_size((x, py), (x + bw, py))
            size_px = uw if self.use_bbox_width_for_distance \
                else max(uw, cam.measure_size((px, y), (px, y + bh)))

            out.append(
                Detection(
                    label=self.label,
                    confidence=conf,
                    bbox=(x / w, y / h, (x + bw) / w, (y + bh) / h),
                    px=px,
                    py=py,
                    bbox_px_w=float(bw),
                    bbox_px_h=float(bh),
                    bearing_rad=cam.bearing_rad(px, py),
                    distance_m=cam.distance_m(float(size_px), self.real_diameter_m),
                    diameter_m=self.real_diameter_m,
                )
            )

        out.sort(key=lambda d: d.confidence, reverse=True)
        return out


class CircleDetector(BaseDetector):
    """圆环检测（霍夫圆）。适合：迷你篮筐、传球架顶部圆环。"""

    def __init__(self, label: str, cfg: dict[str, Any]) -> None:
        self.label = label
        self.hsv_ranges = cfg.get("hsv_ranges", [])
        self.use_edges = bool(cfg.get("use_edges", True))
        self.dp = float(cfg.get("dp", 1.2))
        self.min_dist_ratio = float(cfg.get("min_dist_ratio", 0.05))
        self.param1 = float(cfg.get("param1", 120.0))
        self.param2 = float(cfg.get("param2", 30.0))
        self.min_radius_ratio = float(cfg.get("min_radius_ratio", 0.01))
        self.max_radius_ratio = float(cfg.get("max_radius_ratio", 0.5))
        self.real_diameter_m = float(cfg.get("real_diameter_m", 0.0) or 0.0)
        # 只接受"空心环"。这是区分【篮筐/传球架圆环】与【实心球】的关键条件：
        # 霍夫圆在实心球上也会响，若不做这个判别，球会被误判成环（实测出现过）。
        self.require_hollow = bool(cfg.get("require_hollow", True))
        self.hollow_max_ratio = float(cfg.get("hollow_max_ratio", 0.75))

    def _is_hollow(self, work: np.ndarray, cx: int, cy: int, r: int) -> bool:
        """圆心区域的亮度应显著低于圆环本身，否则认为是实心目标。"""
        h, w = work.shape[:2]
        rr = max(1, int(r * 0.5))
        x0, x1 = max(0, cx - rr), min(w, cx + rr)
        y0, y1 = max(0, cy - rr), min(h, cy + rr)
        if x1 - x0 < 2 or y1 - y0 < 2:
            return True
        inner = work[y0:y1, x0:x1]
        inner_mean = float(inner.mean())

        # 圆环上的平均亮度：取半径 r 附近的一圈像素
        ring_vals: list[int] = []
        for ang in np.linspace(0.0, 2.0 * math.pi, 72, endpoint=False):
            px = int(round(cx + r * math.cos(ang)))
            py = int(round(cy + r * math.sin(ang)))
            if 0 <= px < w and 0 <= py < h:
                ring_vals.append(int(work[py, px]))
        if not ring_vals:
            return True
        ring_mean = float(np.mean(ring_vals))
        if ring_mean <= 1.0:
            return True
        return (inner_mean / ring_mean) <= self.hollow_max_ratio

    def detect(self, frame_bgr: np.ndarray, cam: CameraModel) -> list[Detection]:
        h, w = frame_bgr.shape[:2]
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        if self.hsv_ranges:
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            mask = _clean(_hsv_mask(hsv, self.hsv_ranges))
            work = cv2.bitwise_and(gray, gray, mask=mask)
        else:
            work = gray

        circles = cv2.HoughCircles(
            work,
            cv2.HOUGH_GRADIENT,
            dp=self.dp,
            minDist=max(10.0, self.min_dist_ratio * min(h, w)),
            param1=self.param1,
            param2=self.param2,
            minRadius=int(self.min_radius_ratio * min(h, w)),
            maxRadius=int(self.max_radius_ratio * min(h, w)),
        )

        out: list[Detection] = []
        if circles is None:
            return out

        for cx, cy, r in np.round(circles[0]).astype(int):
            if r <= 0:
                continue
            if self.require_hollow and not self._is_hollow(work, int(cx), int(cy), int(r)):
                continue
            x0, y0 = max(0, cx - r), max(0, cy - r)
            x1, y1 = min(w, cx + r), min(h, cy + r)
            conf = float(min(1.0, 0.5 + 0.5 * min(1.0, r / (0.25 * min(h, w)))))
            out.append(
                Detection(
                    label=self.label,
                    confidence=conf,
                    bbox=(x0 / w, y0 / h, x1 / w, y1 / h),
                    px=float(cx),
                    py=float(cy),
                    bbox_px_w=float(2 * r),
                    bbox_px_h=float(2 * r),
                    bearing_rad=cam.bearing_rad(float(cx), float(cy)),
                    distance_m=cam.distance_m(
                        cam.measure_size((cx - r, cy), (cx + r, cy)), self.real_diameter_m),
                    diameter_m=self.real_diameter_m,
                )
            )

        out.sort(key=lambda d: d.confidence, reverse=True)
        return out


class StripeDetector(BaseDetector):
    """双色竖条检测。适合：规则提供的定位柱（直径 20cm、高 1m、蓝绿相间）。

    做法：分别求两种颜色的连通块，若两者的 bounding box 在竖直方向重叠、
    水平方向接近、且整体高宽比符合"竖柱"，则判定为定位柱。
    """

    def __init__(self, label: str, cfg: dict[str, Any]) -> None:
        self.label = label
        self.range_a = cfg.get("hsv_ranges_a", [])
        self.range_b = cfg.get("hsv_ranges_b", [])
        self.min_total_area_ratio = float(cfg.get("min_total_area_ratio", 0.0008))
        self.aspect_min = float(cfg.get("aspect_min", 1.2))   # 高/宽
        self.aspect_max = float(cfg.get("aspect_max", 12.0))
        self.x_tolerance_ratio = float(cfg.get("x_tolerance_ratio", 0.6))
        self.real_width_m = float(cfg.get("real_width_m", 0.20))

    @staticmethod
    def _components(mask: np.ndarray, min_area: float) -> list[tuple[int, int, int, int]]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for cnt in contours:
            if cv2.contourArea(cnt) < min_area:
                continue
            boxes.append(cv2.boundingRect(cnt))
        return boxes

    def detect(self, frame_bgr: np.ndarray, cam: CameraModel) -> list[Detection]:
        if not self.range_a or not self.range_b:
            return []

        h, w = frame_bgr.shape[:2]
        total = float(h * w)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask_a = _clean(_hsv_mask(hsv, self.range_a), 3, 5)
        mask_b = _clean(_hsv_mask(hsv, self.range_b), 3, 5)

        boxes_a = self._components(mask_a, self.min_total_area_ratio * total * 0.3)
        boxes_b = self._components(mask_b, self.min_total_area_ratio * total * 0.3)

        out: list[Detection] = []
        for ax, ay, aw, ah in boxes_a:
            for bx, by, bw, bh in boxes_b:
                # 两个色段必须水平方向重合
                tol = self.x_tolerance_ratio * max(aw, bw)
                if abs((ax + aw / 2) - (bx + bw / 2)) > tol:
                    continue
                # 竖直方向必须相邻/重叠
                if min(ay + ah, by + bh) - max(ay, by) < -0.5 * max(ah, bh):
                    continue

                x0, y0 = min(ax, bx), min(ay, by)
                x1, y1 = max(ax + aw, bx + bw), max(ay + ah, by + bh)
                cw, ch = x1 - x0, y1 - y0
                if cw <= 0 or ch <= 0:
                    continue
                aspect = ch / float(cw)
                if not (self.aspect_min <= aspect <= self.aspect_max):
                    continue
                if (cw * ch) / total < self.min_total_area_ratio:
                    continue

                px, py = x0 + cw / 2.0, y0 + ch / 2.0
                out.append(
                    Detection(
                        label=self.label,
                        confidence=0.85,
                        bbox=(x0 / w, y0 / h, x1 / w, y1 / h),
                        px=px,
                        py=py,
                        bbox_px_w=float(cw),
                        bbox_px_h=float(ch),
                        bearing_rad=cam.bearing_rad(px, py),
                        distance_m=cam.distance_m(cam.measure_size((x0, py), (x0 + cw, py)),
                                                  self.real_width_m),
                        diameter_m=self.real_width_m,
                    )
                )

        out.sort(key=lambda d: d.confidence, reverse=True)
        return out


class OnnxDetector(BaseDetector):
    """可选：ONNX 目标检测（需自行安装 onnxruntime）。

    本机未安装 onnxruntime，所以默认不启用。接口留好，
    装上依赖后把配置里的 `type: onnx` 打开即可，与其它检测器并存。
    """

    def __init__(self, label: str, cfg: dict[str, Any]) -> None:
        self.label = label
        self.cfg = cfg
        self.session = None
        self.error = ""
        try:  # pragma: no cover - 依赖可选
            import onnxruntime  # type: ignore

            self.session = onnxruntime.InferenceSession(
                cfg["model_path"], providers=["CPUExecutionProvider"]
            )
        except Exception as exc:  # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"

    @property
    def available(self) -> bool:
        return self.session is not None

    def detect(self, frame_bgr: np.ndarray, cam: CameraModel) -> list[Detection]:  # pragma: no cover
        if self.session is None:
            return []
        # 具体预处理/后处理按模型而定，这里不臆造实现。
        raise NotImplementedError(
            "OnnxDetector 需要按具体模型补 preprocess/postprocess 后再启用"
        )


# ---------------------------------------------------------------------------
# 工厂
# ---------------------------------------------------------------------------

_REGISTRY = {
    "blob": BlobDetector,
    "circle": CircleDetector,
    "stripe": StripeDetector,
    "onnx": OnnxDetector,
}


def build_detectors(cfg: dict[str, Any]) -> tuple[list[BaseDetector], list[str]]:
    """按配置构建检测器。返回 (检测器列表, 警告/错误说明)。"""
    detectors: list[BaseDetector] = []
    notes: list[str] = []

    for item in cfg.get("detectors", []) or []:
        if not item.get("enabled", True):
            continue
        kind = str(item.get("type", "")).lower()
        label = str(item.get("label", kind or "unknown"))
        cls = _REGISTRY.get(kind)
        if cls is None:
            notes.append(f"未知检测器类型 '{kind}'（label={label}），已跳过")
            continue

        det = cls(label, item)
        if isinstance(det, OnnxDetector) and not det.available:
            notes.append(f"onnx 检测器 '{label}' 不可用（{det.error}），已跳过")
            continue
        detectors.append(det)

    if not detectors:
        notes.append("没有任何检测器被启用，perception 将只发布空结果")
    return detectors, notes
