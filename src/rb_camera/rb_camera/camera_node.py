#!/usr/bin/env python3
"""RoboCup 篮球机器人 · UVC 相机驱动节点。

做什么
------
把一台 UVC（免驱）USB 相机变成 ROS 2 图像话题：
    /camera/image_raw     sensor_msgs/Image  (bgr8)
    /camera/camera_info   sensor_msgs/CameraInfo（内参，供 RViz/调试用）
    /rb_camera/ok         std_msgs/Bool（相机是否在出图，供 preflight 判断）

为什么自己写而不是用 usb_cam
----------------------------
本机没装 usb_cam / v4l2_camera，而且队里约定"所有代码都在本工作空间里"。
自己写还能顺手解决 UVC 相机的两个老大难：

1. **MJPG 协商**：这台相机 YUYV 只能到 720p@9fps，必须选 MJPG 才能 720p@30。
   FOURCC 要**在设置分辨率之前**下发，顺序错了就拿不到目标帧率。
2. **延迟堆积**：如果"读到才发"，一旦处理慢，驱动缓冲就会积压，画面越来越延迟。
   所以用**独立采集线程**持续抓帧（只保留最新一帧），发布定时器只取最新帧，
   永远是"当下"的画面。`BUFFERSIZE=1` 进一步把驱动缓冲压到最小。

掉线自愈
--------
读失败累计 N 次就 release 重开；相机拔了又插也能自己恢复，不会让节点崩掉。

用法
----
    ros2 run rb_camera camera_node --ros-args -p config_file:=<...>/camera.yaml
    # 常用参数可用命令行覆盖：
    ros2 run rb_camera camera_node --ros-args -p device:=/dev/video0 -p width:=640 -p height:=480
"""

from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import threading
import time
from typing import Any

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool

# V4L2 控件名在不同 UVC 驱动上不一致，这里给出别名表，按顺序取第一个存在的。
# 本机实测这台 HD USB Camera 的实际控件名：
#   auto_exposure (menu 0..3, 3=Aperture Priority 自动, 1=Manual 手动)
#   exposure_time_absolute (1..5000, 自动模式下 flags=inactive **读不到实时值**)
#   white_balance_automatic (bool 1=自动)
#   white_balance_temperature (2800..6500, 自动模式下也是 inactive)
#   gain / brightness / contrast / saturation / sharpness / power_line_frequency
_V4L2_ALIASES: dict[str, list[str]] = {
    "auto_exposure": ["auto_exposure", "exposure_auto", "exposure_automatic"],
    "exposure": ["exposure_time_absolute", "exposure_absolute"],
    "white_balance_automatic": ["white_balance_automatic", "white_balance_temperature_auto"],
    "white_balance_temperature": ["white_balance_temperature"],
    "gain": ["gain"],
    "brightness": ["brightness"],
    "contrast": ["contrast"],
    "saturation": ["saturation"],
    "sharpness": ["sharpness"],
    "power_line_frequency": ["power_line_frequency"],
}
# auto_exposure 菜单里 1 是 Manual；不同驱动可能写成 bool/toggle，都按"关掉自动"处理
_AUTO_EXPOSURE_MANUAL = 1


def parse_v4l2_ctrls(text: str) -> dict[str, int]:
    """解析 `v4l2-ctl --list-ctrls` 的输出，返回 {控件名: 当前值}。

    抽成模块级纯函数是为了能在**没有相机**的情况下单测——解析写错会让
    "锁定曝光/白平衡"悄无声息地失败（值读不出来就走默认分支）。

    v4l2-ctl 的真实输出形如（前面的缩进和 flags 不固定）：
        brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
        auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
        exposure_time_absolute 0x009a0902 (int) : min=1 max=5000 ... value=156 flags=inactive
    """
    vals: dict[str, int] = {}
    for line in text.splitlines():
        m = re.match(r"\s*([a-z_0-9]+)\s+0x[0-9a-fA-F]+\s+\(\w+\)\s*:(.*)", line)
        if not m:
            continue
        vm = re.search(r"value=(-?\d+)", m.group(2))
        if vm:
            vals[m.group(1)] = int(vm.group(1))
    return vals


def resolve_ctrl_name(friendly: str, available) -> str | None:
    """把配置里的友好名解析成相机实际支持的控件名（不同驱动命名不一致）。"""
    for cand in _V4L2_ALIASES.get(friendly, []):
        if cand in available:
            return cand
    return None


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("rb_camera")

        default_cfg = os.path.join(get_package_share_directory("rb_camera"), "config", "camera.yaml")
        self.config_file = self.declare_parameter("config_file", default_cfg).value
        self.cfg: dict[str, Any] = {}
        self.load_config()

        cam = self.cfg.get("camera", {}) or {}
        out = self.cfg.get("output", {}) or {}
        intr = self.cfg.get("intrinsics", {}) or {}

        # 常用项允许 ROS 参数覆盖（方便现场临时换设备/分辨率，不改文件）
        self.device = str(self.declare_parameter("device", str(cam.get("device", "/dev/video0"))).value)
        self.width = int(self.declare_parameter("width", int(cam.get("width", 1280))).value)
        self.height = int(self.declare_parameter("height", int(cam.get("height", 720))).value)
        self.fps = float(self.declare_parameter("fps", float(cam.get("fps", 30.0))).value)
        self.fourcc = str(self.declare_parameter("fourcc", str(cam.get("fourcc", "MJPG"))).value)
        self.buffersize = int(cam.get("buffersize", 1))
        self.flip = int(cam.get("flip", 0))
        self.reconnect_interval_s = float(cam.get("reconnect_interval_s", 2.0))
        self.max_read_failures = int(cam.get("max_read_failures", 15))
        self.publish_rate_hz = float(cam.get("publish_rate_hz", 0.0))  # 0 = 不限速
        self.assumed_hfov_deg = float(intr.get("assumed_hfov_deg", 60.0))
        # QoS 可靠性。默认 reliable 的理由：
        #   ① reliable 发布者**兼容** best_effort 订阅者（rb_perception 就是 best_effort）；
        #   ② 反过来 best_effort 发布者会让默认 QoS 的 rqt_image_view / RViz 收不到图；
        #   ③ 同机通信无丢包，depth=1 时 reliable 不会堆积延迟（旧帧被覆盖）。
        self.qos_reliability = str(self.declare_parameter(
            "qos_reliability", str(cam.get("qos_reliability", "reliable"))).value).lower()
        self.qos_depth = int(self.declare_parameter(
            "qos_depth", int(cam.get("qos_depth", 1))).value)

        self.image_topic = str(self.declare_parameter(
            "image_topic", str(out.get("image_topic", "/camera/image_raw"))).value)
        self.camera_info_topic = str(self.declare_parameter(
            "camera_info_topic", str(out.get("camera_info_topic", "/camera/camera_info"))).value)
        self.frame_id = str(out.get("frame_id", "camera_link"))

        # 压缩图像（JPEG）。为什么默认开：
        #   本机实测——原始 640x480 (921KB) 图像，即使 best_effort、localhost-only，
        #   订阅端也只有 ~7fps；换成 320x240 (230KB) 就恢复 30fps。
        #   即"大图像消息的投递吞吐"约 7MB/s 是硬瓶颈。
        #   JPEG 把 640x480 压到 ~40KB，30fps 才 1.2MB/s，轻松跑满帧率。
        self.publish_compressed = bool(self.declare_parameter(
            "publish_compressed", bool(cam.get("publish_compressed", True))).value)
        self.compressed_topic = str(self.declare_parameter(
            "compressed_topic", str(out.get("compressed_topic", "/camera/image_raw/compressed"))).value)
        self.jpeg_quality = int(self.declare_parameter(
            "jpeg_quality", int(cam.get("jpeg_quality", 80))).value)

        self.fx = float(intr.get("fx", 0.0) or 0.0)
        self.fy = float(intr.get("fy", 0.0) or 0.0)
        self.cx = float(intr.get("cx", 0.0) or 0.0)
        self.cy = float(intr.get("cy", 0.0) or 0.0)
        self.distortion = [float(v) for v in (intr.get("distortion", [0.0] * 5) or [0.0] * 5)]

        # 相机固定项：锁定自动曝光/白平衡，让颜色标定不随光照漂移。
        # 现场用 tools/tune_camera.py 调好值后写回 camera.yaml。
        self.controls_cfg: dict[str, Any] = self.cfg.get("controls", {}) or {}
        self.controls_applied = False
        self.controls_actual: dict[str, int] = {}

        self.bridge = CvBridge()

        # 图像 QoS（见 __init__ 里 qos_reliability 的说明）
        reliability = (QoSReliabilityPolicy.BEST_EFFORT
                       if self.qos_reliability.startswith("best")
                       else QoSReliabilityPolicy.RELIABLE)
        img_qos = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=max(1, self.qos_depth),
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(Image, self.image_topic, img_qos)
        self.info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 5)
        self.ok_pub = self.create_publisher(Bool, "/rb_camera/ok", 1)
        self.compressed_pub = (
            self.create_publisher(CompressedImage, self.compressed_topic, img_qos)
            if self.publish_compressed else None)

        # 采集线程共享状态
        self._lock = threading.Lock()
        self._latest: Any = None
        self._latest_stamp: float = 0.0
        self._seq = 0
        self._running = True
        self._ok = False
        self._cap: cv2.VideoCapture | None = None
        self._read_failures = 0
        self._frames_read = 0
        self._frames_published = 0
        self._last_status_t = time.time()
        self._last_read = 0
        self._last_pub = 0
        self._actual_w = 0
        self._actual_h = 0
        self._actual_fourcc = ""

        self._thread = threading.Thread(target=self._capture_loop, name="rb_camera_capture", daemon=True)
        self._thread.start()

        rate = self.publish_rate_hz if self.publish_rate_hz > 0 else max(self.fps, 1.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_latest)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"camera 就绪: {self.device} {self.width}x{self.height}@{self.fps:.0f} {self.fourcc} "
            f"-> {self.image_topic}"
            + (f" (发布限速 {self.publish_rate_hz:.1f}Hz)" if self.publish_rate_hz > 0 else "")
        )
        if self.compressed_pub is not None:
            self.get_logger().info(
                f"压缩图已启用: {self.compressed_topic} (JPEG q={self.jpeg_quality}) "
                f"—— 建议 rb_perception 用 -p use_compressed:=true 订阅它")
        self.get_logger().info(
            f"内参 fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}"
            + ("  (未标定，CameraInfo 用假定 HFOV 估算；测距请先标定)" if self.fx <= 0 else "")
        )

    # -- 配置 --------------------------------------------------------------
    def load_config(self) -> None:
        path = self.config_file
        if not path or not os.path.isfile(path):
            self.get_logger().error(f"配置文件不存在: {path}，使用内置默认值")
            self.cfg = {}
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                self.cfg = yaml.safe_load(fh) or {}
            self.get_logger().info(f"已加载配置 {path}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"读取配置失败: {exc}")
            self.cfg = {}

    # -- 相机固定项：锁定自动曝光 / 白平衡 ---------------------------------
    def _v4l2_ctrls(self) -> dict[str, int]:
        """读 `v4l2-ctl --list-ctrls`，返回 {控件名: 当前值}（兼作"控件是否存在"的探测）。"""
        try:
            out = subprocess.run(["v4l2-ctl", "-d", self.device, "--list-ctrls"],
                                 capture_output=True, text=True, timeout=5)
        except Exception:  # noqa: BLE001
            return {}
        return parse_v4l2_ctrls(out.stdout)

    def _find_ctrl(self, friendly: str, available: dict[str, int]) -> str | None:
        return resolve_ctrl_name(friendly, available)

    def _set_ctrl(self, name: str, value: int) -> bool:
        try:
            r = subprocess.run(
                ["v4l2-ctl", "-d", self.device, f"--set-ctrl={name}={int(value)}"],
                capture_output=True, text=True, timeout=5)
        except Exception:  # noqa: BLE001
            return False
        return r.returncode == 0

    def _apply_controls(self) -> None:
        """锁定自动曝光 / 白平衡 / 增益，让图像稳定 —— 颜色标定一次就能长期有效。

        ⚠️ 本机实测这台相机：自动模式下 `exposure_time_absolute` 和
        `white_balance_temperature` 都带 `flags=inactive`，**读不到相机当前自动算出的值**。
        所以曝光和色温必须由配置指定（现场用 tools/tune_camera.py 调好写回 camera.yaml），
        不能"读出来再锁"。

        设置完会**读回确认**并打日志 —— 必须确认"锁"真的生效，而不是以为生效。
        """
        cfg = self.controls_cfg
        if not cfg.get("enabled", True):
            return
        if shutil.which("v4l2-ctl") is None:
            self.get_logger().warn("没装 v4l2-ctl，无法锁定曝光/白平衡（sudo apt install v4l-utils）")
            return
        avail = self._v4l2_ctrls()
        if not avail:
            self.get_logger().warn(f"读不到 {self.device} 的 V4L2 控件，跳过曝光/白平衡锁定")
            return

        done: list[str] = []
        failed: list[str] = []

        # 抗闪频：国内 50Hz
        plf = self._find_ctrl("power_line_frequency", avail)
        want_plf = cfg.get("power_line_frequency", None)
        if plf and want_plf is not None:
            (done if self._set_ctrl(plf, int(want_plf)) else failed).append(f"{plf}={want_plf}")

        # 曝光：先关自动，再写固定值
        if cfg.get("lock_exposure", True):
            auto = self._find_ctrl("auto_exposure", avail)
            ex = self._find_ctrl("exposure", avail)
            want = int(cfg.get("exposure", 0) or 0)
            if auto:
                (done if self._set_ctrl(auto, _AUTO_EXPOSURE_MANUAL) else failed).append(
                    "自动曝光→手动")
            if ex and want > 0:
                (done if self._set_ctrl(ex, want) else failed).append(f"{ex}={want}")
            elif ex:
                self.get_logger().warn(
                    "controls.exposure 为 0 → 曝光值保持相机当前值。"
                    "建议用 tools/tune_camera.py 定一个固定值填进 camera.yaml")

        # 白平衡：同样先关自动再写色温
        if cfg.get("lock_white_balance", True):
            awb = self._find_ctrl("white_balance_automatic", avail)
            wbt = self._find_ctrl("white_balance_temperature", avail)
            want = int(cfg.get("white_balance_temperature", 0) or 0)
            if awb:
                (done if self._set_ctrl(awb, 0) else failed).append("自动白平衡→关")
            if wbt and want > 0:
                (done if self._set_ctrl(wbt, want) else failed).append(f"{wbt}={want}")

        # 其余可选固定项（配置里给了才动）
        for friendly in ("gain", "brightness", "contrast", "saturation", "sharpness"):
            want = cfg.get(friendly, None)
            if want is None:
                continue
            name = self._find_ctrl(friendly, avail)
            if name and self._set_ctrl(name, int(want)):
                done.append(f"{name}={want}")

        # 读回确认
        after = self._v4l2_ctrls()
        self.controls_actual = after
        self.controls_applied = True
        if done:
            self.get_logger().info("相机固定项已应用: " + ", ".join(done))
        if failed:
            self.get_logger().warn("以下固定项设置失败: " + ", ".join(failed))

        checks: list[str] = []
        a = self._find_ctrl("auto_exposure", after)
        if a:
            checks.append("曝光已锁" if after.get(a) == _AUTO_EXPOSURE_MANUAL
                          else f"⚠️曝光仍自动(auto_exposure={after.get(a)})")
        w = self._find_ctrl("white_balance_automatic", after)
        if w:
            checks.append("白平衡已锁" if after.get(w) == 0 else "⚠️白平衡仍自动")
        e = self._find_ctrl("exposure", after)
        if e:
            checks.append(f"曝光值={after.get(e)}")
        t = self._find_ctrl("white_balance_temperature", after)
        if t:
            checks.append(f"色温={after.get(t)}")
        if checks:
            self.get_logger().info("读回确认: " + ", ".join(checks))

    # -- 采集线程 ----------------------------------------------------------
    def _open_capture(self) -> bool:
        try:
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"打开 {self.device} 异常: {exc}")
            return False
        if not cap.isOpened():
            self.get_logger().warn(f"打不开相机 {self.device}（插好了吗？被别的进程占用？）")
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass
            return False

        # ⚠️ FOURCC 必须在设置分辨率之前下发，否则 V4L2 会按 YUYV 协商，
        #    高分辨率下帧率直接掉到 6~9fps。
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc[:4].ljust(4)))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        cap.set(cv2.CAP_PROP_FPS, float(self.fps))
        if self.buffersize > 0:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, float(self.buffersize))

        # 锁定自动曝光/白平衡。放在"打开设备之后"：
        # 本机实测这样能生效（v4l2-ctl 可与 OpenCV 同时访问同一设备），
        # 且不会被 stream 启动重置。函数内部会读回确认。
        self._apply_controls()

        with self._lock:
            self._cap = cap
        self._read_failures = 0
        self._actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        self._actual_fourcc = "".join(chr((fcc >> 8 * i) & 0xFF) for i in range(4))
        if (self._actual_w, self._actual_h) != (self.width, self.height):
            self.get_logger().warn(
                f"实际分辨率 {self._actual_w}x{self._actual_h} 与请求 {self.width}x{self.height} 不同"
                f"（相机不支持该组合，已用最接近的）")
        self.get_logger().info(
            f"相机已打开: {self.device} {self._actual_w}x{self._actual_h} {self._actual_fourcc}")
        return True

    def _release_capture(self) -> None:
        with self._lock:
            cap, self._cap = self._cap, None
        if cap is not None:
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass

    def _capture_loop(self) -> None:
        """持续抓帧，只保留最新一帧。任何异常都不允许把节点带崩。"""
        while self._running:
            with self._lock:
                cap = self._cap
            if cap is None:
                if not self._open_capture():
                    self._ok = False
                    self._sleep(self.reconnect_interval_s)
                    continue
                # ⚠️ 必须重新读 self._cap：局部 cap 还是上面的 None，
                #    直接 cap.read() 会抛 'NoneType' object has no attribute 'read'。
                with self._lock:
                    cap = self._cap
                if cap is None:
                    self._sleep(self.reconnect_interval_s)
                    continue

            try:
                ok, frame = cap.read()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"读帧异常: {exc}")
                ok, frame = False, None

            if not ok or frame is None:
                self._read_failures += 1
                if self._read_failures >= self.max_read_failures:
                    self.get_logger().warn(
                        f"连续 {self._read_failures} 次读帧失败，重开相机 {self.device}")
                    self._ok = False
                    self._release_capture()
                    self._sleep(self.reconnect_interval_s)
                else:
                    self._sleep(0.01)
                continue

            self._read_failures = 0
            if self.flip == 1:
                frame = cv2.flip(frame, 1)
            elif self.flip == 2:
                frame = cv2.flip(frame, 0)
            elif self.flip == -1:
                frame = cv2.flip(frame, -1)

            now = time.time()
            with self._lock:
                self._latest = frame
                self._latest_stamp = now
                self._seq += 1
            self._frames_read += 1
            self._ok = True

    def _sleep(self, sec: float) -> None:
        """可被 stop() 打断的睡眠。"""
        end = time.time() + max(0.0, sec)
        while self._running and time.time() < end:
            time.sleep(min(0.05, max(0.0, end - time.time())))

    # -- 发布 --------------------------------------------------------------
    def _intrinsics(self) -> tuple[float, float, float, float]:
        """返回 (fx, fy, cx, cy)。未标定时用假定 HFOV 估一个，别让下游拿到全 0。"""
        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        w = self._actual_w or self.width
        h = self._actual_h or self.height
        if fx <= 0.0:
            fx = (w / 2.0) / math.tan(math.radians(self.assumed_hfov_deg) / 2.0)
        if fy <= 0.0:
            fy = fx
        if cx <= 0.0:
            cx = w / 2.0
        if cy <= 0.0:
            cy = h / 2.0
        return fx, fy, cx, cy

    def publish_latest(self) -> None:
        with self._lock:
            frame = None if self._latest is None else self._latest
            seq = self._seq
        if frame is None:
            return
        stamp = self.get_clock().now().to_msg()
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"图像转换失败: {exc}")
            return
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)
        self.info_pub.publish(self._make_info(stamp))
        if self.compressed_pub is not None:
            self._publish_compressed(frame, stamp)
        self._frames_published += 1

    def _publish_compressed(self, frame, stamp) -> None:
        """发 JPEG 压缩图。大图走原始消息会被吞吐瓶颈卡到 ~7fps，压缩后能跑满帧率。"""
        try:
            ok, buf = cv2.imencode(".jpg", frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"JPEG 编码失败: {exc}", throttle_duration_sec=5.0)
            return
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.compressed_pub.publish(msg)

    def _make_info(self, stamp) -> CameraInfo:
        fx, fy, cx, cy = self._intrinsics()
        w = self._actual_w or self.width
        h = self._actual_h or self.height
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = int(w)
        info.height = int(h)
        info.distortion_model = "plumb_bob"
        info.d = list(self.distortion) if len(self.distortion) >= 5 else [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish_status(self) -> None:
        msg = Bool()
        msg.data = bool(self._ok)
        self.ok_pub.publish(msg)
        if not self._ok:
            self.get_logger().warn("相机未出图", throttle_duration_sec=5.0)
            return
        # 用"两次状态之间的增量"算帧率，而不是固定时间窗 —— 固定窗口在启动初期会严重低估。
        now = time.time()
        dt = now - self._last_status_t
        if dt <= 1e-6:
            return
        read_fps = (self._frames_read - self._last_read) / dt
        pub_fps = (self._frames_published - self._last_pub) / dt
        self._last_status_t, self._last_read, self._last_pub = now, self._frames_read, self._frames_published
        self.get_logger().info(
            f"相机 {self._actual_w}x{self._actual_h} {self._actual_fourcc} "
            f"采集 {read_fps:.1f} fps / 发布 {pub_fps:.1f} fps"
            f"（累计读 {self._frames_read} 发 {self._frames_published}）",
            throttle_duration_sec=5.0)

    # -- 退出 --------------------------------------------------------------
    def stop(self) -> None:
        self._running = False
        try:
            self._thread.join(timeout=2.0)
        except Exception:  # noqa: BLE001
            pass
        self._release_capture()


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = CameraNode()
    _install_sigint_handler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown(node)


def _install_sigint_handler() -> None:
    """把 SIGINT 变成 KeyboardInterrupt，别让 rclpy 先关 context。"""
    import signal as _signal

    def _handler(signum, frame):  # noqa: ARG001
        raise KeyboardInterrupt

    try:
        _signal.signal(_signal.SIGINT, _handler)
    except (ValueError, OSError):
        pass


def _shutdown(node) -> None:
    try:
        node.stop()
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
