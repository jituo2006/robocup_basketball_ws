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
import re
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


# 框内颜色分类的默认规则：[h_lo, h_hi, s_lo, v_lo]（OpenCV H 0..179）
# 用于"YOLO 找到球 → 看框内主色 → 判断篮球还是排球"。
# 为什么需要：全图 HSV 阈值会把墙面/地板误判成球（实测排球阈值命中 71% 画面），
# 而**框内**是干净的 ROI，判断"偏橙还是偏蓝/黄"要可靠得多。
_DEFAULT_BALL_COLOR_RULES: dict[str, list[list[int]]] = {
    "ball_basketball": [[0, 25, 90, 60], [170, 179, 90, 60]],   # 橙/红
    "ball_volleyball": [[90, 140, 60, 50], [18, 40, 90, 80]],   # 蓝块 / 黄块
}


class OnnxDetector(BaseDetector):
    """ONNX 目标检测（YOLO 系列，两种常见输出布局自适应）。

    支持的模型
    ----------
    * **YOLOv8 / YOLOv11**（ultralytics 导出）：输出 `(1, 4+nc, N)`，无 objectness
    * **YOLOv5 / YOLOv7**：输出 `(1, N, 5+nc)`，第 5 列是 objectness
    布局是**自动判定**的（属性维是较小的那一维），不需要手工指定。

    类别映射（关键）
    ----------------
    模型自己的类别名未必等于工程内部标签，所以要给映射。两种写法：
      class_map: {0: ball_basketball, 1: ball_volleyball}      # 按 id
      name_map:  {basketball: ball_basketball, volleyball: ...} # 按模型类别名
    ultralytics 导出的 ONNX 通常在 metadata 里带 `names`，本类会自动读取，
    此时优先用 name_map 匹配名字；都没有就退回 class_map / 直接用模型名。

    配置项（perception.yaml 里那一项）
    ----------------------------------
      model_path        必填
      class_map/name_map 见上
      conf_threshold    默认 0.35
      iou_threshold     默认 0.45（NMS）
      input_size        默认自动读模型输入；读不到用 640
      real_diameter_m   单目测距用的真实直径
      class_real_size   可选 {工程label: 米}，不同类别尺寸不同时用它覆盖
      max_detections    默认 20
    """

    def __init__(self, label: str, cfg: dict[str, Any]) -> None:
        self.label = label
        self.cfg = cfg
        self.session = None
        self.error = ""
        self.model_names: dict[int, str] = {}
        self.input_name = ""
        self.input_size = int(cfg.get("input_size", 0) or 0)
        self.conf_thr = float(cfg.get("conf_threshold", 0.35))
        self.iou_thr = float(cfg.get("iou_threshold", 0.45))
        self.max_det = int(cfg.get("max_detections", 20))
        self.real_size = float(cfg.get("real_diameter_m", 0.0) or 0.0)
        self.class_real_size = cfg.get("class_real_size", {}) or {}
        self.class_map = {int(k): str(v) for k, v in (cfg.get("class_map", {}) or {}).items()}
        self.name_map = {str(k): str(v) for k, v in (cfg.get("name_map", {}) or {}).items()}
        # 跳帧：YOLO 是 CPU 上的大头（416 输入约 13ms/帧）。球在相邻帧之间位置变化很小，
        # 没必要每帧都推理 —— 设 2 表示"每 2 帧推理一次，另一帧复用上次结果"。
        self.every_n = max(1, int(cfg.get("every_n_frames", 1) or 1))
        self._frame_i = 0
        self._cached: list[Detection] = []
        # 只保留"映射过的"类别（默认开）。COCO 预训练有 80 类，不筛的话
        # person/chair/cat 会全混进 /perception/detections，而且被套上
        # real_diameter_m（按篮球尺寸）算出荒谬距离 —— 实测 person 报 0.28m。
        self.only_mapped = bool(cfg.get("only_mapped", True))
        # 框内颜色分类：YOLO 只负责"找到球"，类别由框内主色决定。
        # 这样既躲开全图 HSV 的背景误检，又能区分篮球/排球（COCO 只有一类做不到）。
        self.color_classify = bool(cfg.get("color_classify", False))
        rules_cfg = cfg.get("color_rules", None)
        self.color_rules = ({k: [list(map(int, r)) for r in v]
                             for k, v in rules_cfg.items()} if rules_cfg
                            else dict(_DEFAULT_BALL_COLOR_RULES))
        self.color_min_fraction = float(cfg.get("color_min_fraction", 0.15))

        try:  # pragma: no cover - 依赖可选
            import onnxruntime  # type: ignore

            self.session = onnxruntime.InferenceSession(
                cfg["model_path"], providers=["CPUExecutionProvider"]
            )
            inp = self.session.get_inputs()[0]
            self.input_name = inp.name
            # 输入形状形如 [1, 3, 640, 640]，动态导出则是 [1, 3, -1, -1]。
            # ⚠️ 只取 >32 的静态维：否则动态模型里 max([1,3]) = 3（通道数），
            #    会被当成输入尺寸，letterbox 到 3x3 直接错得离谱。
            dims = [d for d in inp.shape if isinstance(d, int) and d > 32]
            if self.input_size <= 0:
                self.input_size = max(dims) if dims else 640
            self.model_names = self._read_names()
        except Exception as exc:  # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"

    # -- 元数据 ------------------------------------------------------------
    def _read_names(self) -> dict[int, str]:
        """从 ONNX metadata 里读 ultralytics 写的 `names`（形如 "{0: 'ball', ...}"）。"""
        if self.session is None:
            return {}
        try:
            meta = self.session.get_modelmeta().custom_metadata_map or {}
        except Exception:  # noqa: BLE001
            return {}
        raw = meta.get("names") or meta.get("classes") or ""
        if not raw:
            return {}
        names: dict[int, str] = {}
        # 优先按 Python dict 字面量解析；失败则退回"逐项抓 key: 'value'"
        try:
            import ast

            parsed = ast.literal_eval(raw)
            if isinstance(parsed, dict):
                return {int(k): str(v) for k, v in parsed.items()}
            if isinstance(parsed, (list, tuple)):
                return {i: str(v) for i, v in enumerate(parsed)}
        except Exception:  # noqa: BLE001
            pass
        for m in re.finditer(r"(\d+)\s*:\s*['\"]([^'\"]+)['\"]", raw):
            names[int(m.group(1))] = m.group(2)
        return names

    @property
    def available(self) -> bool:
        return self.session is not None

    # -- 预处理 ------------------------------------------------------------
    def _letterbox(self, frame: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        """等比缩放 + 灰边填充到正方形。返回 (张量, 缩放比, 左padding, 上padding)。"""
        h, w = frame.shape[:2]
        s = min(self.input_size / w, self.input_size / h)
        nw, nh = max(1, int(round(w * s))), max(1, int(round(h * s)))
        resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        px, py = (self.input_size - nw) // 2, (self.input_size - nh) // 2
        canvas[py:py + nh, px:px + nw] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))[None, ...]      # NCHW
        return np.ascontiguousarray(tensor), s, px, py

    # -- 后处理 ------------------------------------------------------------
    @staticmethod
    def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thr: float) -> list[int]:
        """纯 numpy NMS（不引入 torch/torchvision，车上依赖越少越好）。"""
        if boxes.size == 0:
            return []
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        order = scores.argsort()[::-1]
        keep: list[int] = []
        while order.size > 0:
            i = int(order[0])
            keep.append(i)
            if order.size == 1:
                break
            rest = order[1:]
            xx1 = np.maximum(x1[i], x1[rest])
            yy1 = np.maximum(y1[i], y1[rest])
            xx2 = np.minimum(x2[i], x2[rest])
            yy2 = np.minimum(y2[i], y2[rest])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            iou = inter / np.maximum(areas[i] + areas[rest] - inter, 1e-9)
            order = rest[iou <= iou_thr]
        return keep

    def _classify_by_color(self, frame_bgr, box, default_label: str) -> str:
        """看框内（内接椭圆）主色，判断是哪个球。判不出来就返回 default_label。

        取内接椭圆而不是整个矩形：球是圆的，矩形四角是背景。
        """
        x1, y1, x2, y2 = (int(max(0, box[0])), int(max(0, box[1])),
                          int(box[2]), int(box[3]))
        if x2 - x1 < 6 or y2 - y1 < 6:
            return default_label
        crop = frame_bgr[y1:y2, x1:x2]
        h, w = crop.shape[:2]
        mask = np.zeros((h, w), np.uint8)
        cv2.ellipse(mask, (w // 2, h // 2), (max(1, int(w / 2 * 0.85)),
                                             max(1, int(h / 2 * 0.85))), 0, 0, 360, 255, -1)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        total = int((mask > 0).sum())
        if total <= 0:
            return default_label

        best_label, best_frac = default_label, 0.0
        for label, ranges in self.color_rules.items():
            hit = np.zeros((h, w), np.uint8)
            for r in ranges:
                if len(r) != 4:
                    continue
                hit |= cv2.inRange(hsv, np.array([r[0], r[2], r[3]], np.uint8),
                                   np.array([r[1], 255, 255], np.uint8))
            frac = float((hit & mask).sum()) / total
            if frac > best_frac:
                best_label, best_frac = label, frac
        return best_label if best_frac >= self.color_min_fraction else default_label

    def _is_mapped(self, cid: int, cname: str) -> bool:
        """该类别是否被显式映射到工程标签（only_mapped 时用它决定保留或丢弃）。"""
        return (bool(cname) and cname in self.name_map) or (cid in self.class_map)

    def _map_label(self, cid: int, cname: str) -> str:
        if cname and cname in self.name_map:
            return self.name_map[cname]
        if cid in self.class_map:
            return self.class_map[cid]
        if cname:
            return cname
        return f"{self.label}_{cid}"

    def _decode(self, out: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """把模型原始输出解成 (boxes_xyxy_letterbox坐标系, scores, class_ids)。"""
        pred = np.asarray(out)
        if pred.ndim == 3:
            pred = pred[0]
        if pred.ndim != 2:                       # 保底：压成二维
            pred = pred.reshape(pred.shape[0], -1)
        nc = len(self.model_names) or len(self.class_map)

        # 确定"属性维"。**优先用 nc 精确判定**，而不是"取较小维"：
        # 候选框数 N 未必远大于属性数（导出成小 N、或单张图上只留少数候选时），
        # 靠大小猜会把坐标当类别分数，输出一堆垃圾框而且不报错。
        def _attr_axis(a: np.ndarray) -> int:
            if nc:
                if a.shape[0] in (nc + 4, nc + 5):
                    return 0
                if a.shape[1] in (nc + 4, nc + 5):
                    return 1
            return 0 if a.shape[0] < a.shape[1] else 1

        if _attr_axis(pred) == 0:
            pred = pred.T
        n_attr = pred.shape[1]
        # v5/v7 比 v8 多一个 objectness 列
        has_obj = bool(nc and n_attr == nc + 5)
        if not nc:                               # 没有元数据时按 v8 处理（ultralytics 主流）
            has_obj = False

        if has_obj:
            obj = pred[:, 4]
            cls_scores = pred[:, 5:]
            cid = cls_scores.argmax(axis=1)
            score = obj * cls_scores[np.arange(len(cid)), cid]
        else:
            cls_scores = pred[:, 4:]
            cid = cls_scores.argmax(axis=1)
            score = cls_scores[np.arange(len(cid)), cid]

        keep = score >= self.conf_thr
        if not keep.any():
            return np.empty((0, 4), np.float32), np.empty(0, np.float32), np.empty(0, int)

        xywh = pred[keep, :4]
        cx, cy, bw, bh = xywh[:, 0], xywh[:, 1], xywh[:, 2], xywh[:, 3]
        boxes = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1)
        return (boxes.astype(np.float32), score[keep].astype(np.float32),
                cid[keep].astype(int))

    def detect(self, frame_bgr: np.ndarray, cam: CameraModel) -> list[Detection]:
        if self.session is None:
            return []
        # 跳帧：非推理帧直接复用上次结果（球在相邻帧间位移很小，够用）
        self._frame_i += 1
        if self.every_n > 1 and (self._frame_i % self.every_n) != 0:
            return self._cached

        h, w = frame_bgr.shape[:2]
        tensor, s, pad_x, pad_y = self._letterbox(frame_bgr)
        try:
            out = self.session.run(None, {self.input_name: tensor})[0]
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"ONNX 推理失败: {exc}") from exc

        boxes, scores, cids = self._decode(out)
        if boxes.size == 0:
            self._cached = []
            return []

        # 每类分别做 NMS（不同类别框重叠不应互相抑制）
        keep_all: list[int] = []
        for c in np.unique(cids):
            idx = np.where(cids == c)[0]
            keep_all.extend(idx[self._nms(boxes[idx], scores[idx], self.iou_thr)])
        keep_all.sort(key=lambda i: -scores[i])
        keep_all = keep_all[: self.max_det]

        results: list[Detection] = []
        for i in keep_all:
            x1, y1, x2, y2 = boxes[i]
            # letterbox 坐标 → 原图坐标
            x1 = (x1 - pad_x) / s
            x2 = (x2 - pad_x) / s
            y1 = (y1 - pad_y) / s
            y2 = (y2 - pad_y) / s
            x1, x2 = float(np.clip(x1, 0, w - 1)), float(np.clip(x2, 0, w - 1))
            y1, y2 = float(np.clip(y1, 0, h - 1)), float(np.clip(y2, 0, h - 1))
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue

            cid = int(cids[i])
            cname = self.model_names.get(cid, "")
            if self.only_mapped and not self._is_mapped(cid, cname):
                continue          # 丢弃没映射过的类别（如 person/chair）
            label = self._map_label(cid, cname)
            # 框内颜色分类：把"球候选"细分成篮球/排球
            if self.color_classify:
                label = self._classify_by_color(frame_bgr, (x1, y1, x2, y2), label)
                if self.only_mapped and label in self.color_rules:
                    pass          # 分类结果本身就是要的工程标签，直接放行
            real = float(self.class_real_size.get(label, self.real_size) or 0.0)
            px, py = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            # 去畸变后再算方位角与像素尺寸（与颜色检测器一致）
            uw = cam.measure_size((x1, py), (x2, py))
            uh = cam.measure_size((px, y1), (px, y2))
            size_px = max(uw, uh)

            results.append(Detection(
                label=label,
                confidence=float(scores[i]),
                bbox=(x1 / w, y1 / h, x2 / w, y2 / h),
                px=px, py=py,
                bbox_px_w=float(x2 - x1), bbox_px_h=float(y2 - y1),
                bearing_rad=cam.bearing_rad(px, py),
                distance_m=cam.distance_m(size_px, real),
                diameter_m=real,
            ))
        self._cached = results
        return results


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
