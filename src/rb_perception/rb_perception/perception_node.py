#!/usr/bin/env python3
"""RoboCup 篮球机器人 · 视觉感知节点。

职责：把相机图像变成结构化检测结果（rb_msgs/DetectionArray），供定位与决策使用。

为什么用配置文件而不是 ROS 参数
--------------------------------
检测器配置是"列表套字典"（每个检测器一组 HSV 范围），而 ROS 2 参数类型系统
不支持嵌套字典/字典数组。所以复杂配置通过 `config_file` 参数指向 YAML，
简单开关（话题名、是否发调试图）仍走 ROS 参数，便于命令行覆盖。

离线自测
--------
不需要相机也能验证：
  python3 tools/make_test_image.py --out /tmp/test.png
  ros2 run rb_perception perception_node --ros-args \
      -p config_file:=<...>/perception.yaml -p image_topic:=/test/image
  # 另一个终端反复发布这张图，即可看到检测结果
"""

from __future__ import annotations

import os
import time
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

from rb_msgs.msg import Detection, DetectionArray

from .detectors import BaseDetector, CameraModel, build_detectors

# 不同类别用不同颜色画框，方便肉眼调参
_DEBUG_COLORS = {
    "ball_basketball": (0, 140, 255),
    "ball_volleyball": (255, 200, 0),
    "ball_unknown": (0, 255, 255),
    "hoop": (0, 0, 255),
    "rack_ring": (255, 0, 255),
    "pass_rack": (255, 0, 128),
    "pillar": (0, 255, 0),
    "obstacle": (128, 128, 128),
}


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("rb_perception")

        default_cfg = os.path.join(
            get_package_share_directory("rb_perception"), "config", "perception.yaml"
        )
        self.config_file = self.declare_parameter("config_file", default_cfg).value
        self.image_topic = self.declare_parameter("image_topic", "/camera/image_raw").value
        # 压缩图订阅。实测：原始 640x480 (921KB) 消息投递只有 ~7fps（即使 best_effort
        # + localhost_only），而 JPEG 压缩后 (~40KB) 能跑满 30fps。所以机器人上默认用它。
        self.use_compressed = bool(self.declare_parameter("use_compressed", True).value)
        self.compressed_topic = self.declare_parameter(
            "compressed_topic", "/camera/image_raw/compressed").value
        self.output_topic = self.declare_parameter("output_topic", "/perception/detections").value
        self.debug_image_topic = self.declare_parameter("debug_image_topic", "/perception/debug_image").value
        self.publish_debug = bool(self.declare_parameter("publish_debug_image", True).value)
        self.max_per_label = int(self.declare_parameter("max_detections_per_label", 5).value)

        self.cfg: dict[str, Any] = {}
        self.load_config()

        self.detectors: list[BaseDetector] = []
        self.detector_notes: list[str] = []
        self.rebuild_detectors()

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_summary = ""
        # 自报性能：现场的"视觉跟不跟得上"只能靠节点自己数，外部订阅测量会被
        # Python 反序列化拖慢而失真。这两个计数也用于写 /perception/ok。
        self._stats_t = time.time()
        self._stats_n = 0
        self._proc_ms_sum = 0.0
        self.last_proc_ms = 0.0
        self.last_fps = 0.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        if self.use_compressed:
            self.sub = self.create_subscription(
                CompressedImage, self.compressed_topic, self.on_compressed, qos)
            self.source_desc = f"{self.compressed_topic} (JPEG 压缩)"
        else:
            self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
            self.source_desc = f"{self.image_topic} (原始图像)"
        self.pub = self.create_publisher(DetectionArray, self.output_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 2)
        self.stats_timer = self.create_timer(5.0, self.log_stats)

        self.get_logger().info(
            f"perception 就绪: {self.source_desc} -> {self.output_topic}, "
            f"检测器 {len(self.detectors)} 个, config={self.config_file}"
        )
        for note in self.detector_notes:
            self.get_logger().warn(note)

    # -- 配置 --------------------------------------------------------------
    def load_config(self) -> None:
        path = self.config_file
        if not path or not os.path.isfile(path):
            self.get_logger().error(f"配置文件不存在: {path}，将使用空配置（不检测任何目标）")
            self.cfg = {}
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                self.cfg = yaml.safe_load(fh) or {}
            self.get_logger().info(f"已加载配置 {path}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"读取配置失败: {exc}")
            self.cfg = {}

    def rebuild_detectors(self) -> None:
        self.detectors, self.detector_notes = build_detectors(self.cfg.get("perception", {}))

    # -- 主回调 ------------------------------------------------------------
    def on_image(self, msg: Image) -> None:
        _t0 = time.perf_counter()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"图像转换失败: {exc}")
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"图像转换异常: {exc}")
            return
        self.process_frame(frame, msg.header.stamp, msg.header.frame_id, _t0)

    def on_compressed(self, msg: CompressedImage) -> None:
        """JPEG 压缩图回调：解码后走同一条处理链。"""
        _t0 = time.perf_counter()
        try:
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"JPEG 解码失败: {exc}", throttle_duration_sec=5.0)
            return
        if frame is None:
            self.get_logger().warn("JPEG 解码返回空图", throttle_duration_sec=5.0)
            return
        self.process_frame(frame, msg.header.stamp, msg.header.frame_id, _t0)

    def process_frame(self, frame, stamp, frame_id: str, _t0: float) -> None:
        h, w = frame.shape[:2]
        cam = CameraModel.from_config(self.cfg.get("camera", {}) or {}, w, h)

        found: list[Detection] = []
        per_label: dict[str, int] = {}
        for det in self.detectors:
            try:
                results = det.detect(frame, cam)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"检测器 {det.label} 异常: {exc}")
                continue
            for r in results:
                if per_label.get(r.label, 0) >= self.max_per_label:
                    break
                per_label[r.label] = per_label.get(r.label, 0) + 1
                found.append(r)

        out = DetectionArray()
        out.header = Header()
        out.header.stamp = stamp
        out.header.frame_id = frame_id or "camera"
        out.frame_id_seq = self.frame_count
        out.fx = float(cam.fx)
        out.fy = float(cam.fy)
        out.cx_cam = float(cam.cx)
        out.cy_cam = float(cam.cy)
        out.img_width = int(w)
        out.img_height = int(h)
        out.detections = [self.to_msg(d, stamp) for d in found]
        self.pub.publish(out)

        # 限频打印，避免日志洪泛
        summary = " ".join(f"{k}:{v}" for k, v in sorted(per_label.items())) or "无目标"
        if summary != self.last_summary or self.frame_count % 100 == 0:
            self.get_logger().info(f"检测结果 {summary}")
            self.last_summary = summary

        if self.publish_debug:
            self.publish_debug_image(frame, found, cam)

        self.frame_count += 1
        self._stats_n += 1
        self._proc_ms_sum += (time.perf_counter() - _t0) * 1000.0

    def log_stats(self) -> None:
        """每 5s 报一次"视觉到底跑了多少帧、每帧多久"。

        这个数字很重要：外部用 Python 订阅去测帧率会失真（大图像反序列化本身就慢），
        只有节点自己数才是真的。若 fps 明显低于相机帧率，说明感知跟不上。
        """
        now = time.time()
        dt = now - self._stats_t
        if dt <= 1e-6 or self._stats_n == 0:
            self._stats_t, self._stats_n, self._proc_ms_sum = now, 0, 0.0
            return
        self.last_fps = self._stats_n / dt
        self.last_proc_ms = self._proc_ms_sum / self._stats_n
        self._stats_t, self._stats_n, self._proc_ms_sum = now, 0, 0.0
        self.get_logger().info(
            f"视觉 {self.last_fps:.1f} fps，每帧 {self.last_proc_ms:.1f} ms"
            f"（累计 {self.frame_count} 帧）")

    def to_msg(self, d, stamp) -> Detection:
        m = Detection()
        m.stamp = stamp
        m.label = d.label
        m.confidence = float(d.confidence)
        m.x_min, m.y_min, m.x_max, m.y_max = (float(v) for v in d.bbox)
        m.cx = float((d.bbox[0] + d.bbox[2]) / 2.0)
        m.cy = float((d.bbox[1] + d.bbox[3]) / 2.0)
        m.width = float(d.bbox[2] - d.bbox[0])
        m.height = float(d.bbox[3] - d.bbox[1])
        m.px = float(d.px)
        m.py = float(d.py)
        m.bbox_px_w = float(d.bbox_px_w)
        m.bbox_px_h = float(d.bbox_px_h)
        m.bearing_rad = float(d.bearing_rad)
        m.distance_m = float(d.distance_m)
        m.diameter_m = float(d.diameter_m)
        return m

    def publish_debug_image(self, frame: np.ndarray, dets: list[Detection], cam: CameraModel) -> None:
        vis = frame.copy()
        h, w = vis.shape[:2]
        for d in dets:
            color = _DEBUG_COLORS.get(d.label, (255, 255, 255))
            x0, y0 = int(d.bbox[0] * w), int(d.bbox[1] * h)
            x1, y1 = int(d.bbox[2] * w), int(d.bbox[3] * h)
            cv2.rectangle(vis, (x0, y0), (x1, y1), color, 2)
            text = f"{d.label} {d.confidence:.2f}"
            if d.distance_m == d.distance_m:  # not NaN
                text += f" {d.distance_m:.2f}m"
            cv2.putText(vis, text, (x0, max(14, y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        cv2.putText(vis, f"frames={self.frame_count}", (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding="bgr8"))
        except Exception:  # noqa: BLE001
            pass


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = PerceptionNode()
    _install_sigint_handler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown(node)


def _install_sigint_handler() -> None:
    """把 SIGINT 变成 KeyboardInterrupt，**不要**让 rclpy 的默认处理器先去
    shutdown context。

    原因（本机实测过的两个坑）：
      * rclpy 默认收到 Ctrl+C 会立刻关闭 context，于是 finally 里再想
        `publish_zero()` 就会抛 RCLError: publisher's context is invalid
        —— 对机器人来说，"退出时没能发出零速"是不可接受的。
      * 同理会报 destroy_service 失败。
    自己接管后，context 在 finally 里仍然有效，可以安全地先停车再销毁。
    """
    import signal as _signal

    def _handler(signum, frame):  # noqa: ARG001
        raise KeyboardInterrupt

    try:
        _signal.signal(_signal.SIGINT, _handler)
    except (ValueError, OSError):
        # 非主线程不允许注册信号处理器，忽略即可
        pass


def _shutdown(node) -> None:
    """统一的安全退出顺序：先停车 → 再销毁 → 最后 shutdown。"""
    try:
        if hasattr(node, "publish_zero"):
            node.publish_zero()
    except Exception:  # noqa: BLE001
        pass
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass


if __name__ == "__main__":
    main()
