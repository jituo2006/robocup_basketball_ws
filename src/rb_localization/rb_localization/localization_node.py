#!/usr/bin/env python3
"""RoboCup 篮球机器人 · 定位节点。

数据来源与职责
--------------
  predict : /odom（底盘里程计或全场定位板的 twist 部分）
  update  : ① 绝对位姿（雷达 FAST-LIO / 全场定位板 的 pose 部分）
            ② 视觉看到的【定位柱】方位角（bearing-only，规则提供的视觉基准）

输出：
  /localization/pose   geometry_msgs/PoseWithCovarianceStamped（场地坐标系）
  /localization/ok     std_msgs/Bool（定位是否可信）
  TF: field -> base_link

⚠️ 合规提醒
-----------
RoboCup 规则 §2.4 的场地器材清单里**没有"全场定位系统"**。
若你依赖队里自架的定位板，务必先向技术委员会确认是否允许。
本节点的设计使得：**断掉绝对位姿输入后，仅靠里程计 + 视觉定位柱仍可运行**
（只是精度下降），这样即使定位板被判违规也有退路。
"""

from __future__ import annotations

import math
import os
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from rb_msgs.msg import DetectionArray

from .ekf2d import Ekf2D, EkfConfig, wrap_pi


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class LocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("rb_localization")

        default_cfg = os.path.join(
            get_package_share_directory("rb_localization"), "config", "localization.yaml"
        )
        self.config_file = self.declare_parameter("config_file", default_cfg).value
        self.cfg: dict[str, Any] = {}
        self.load_config()

        ekf_cfg = EkfConfig()
        for key, value in (self.cfg.get("ekf", {}) or {}).items():
            if hasattr(ekf_cfg, key):
                setattr(ekf_cfg, key, float(value))
        self.ekf = Ekf2D(ekf_cfg)

        src = self.cfg.get("sources", {}) or {}
        # 输入话题：YAML 给默认值，但允许命令行/launch 覆盖。
        # 这样同一份配置既能跑雷达(/Odometry)，也能跑定位板/假底盘(/odom)，不用改文件。
        self.odom_topic = str(self.declare_parameter(
            "odom_topic", str(src.get("odom_topic", "/odom"))).value)
        self.abs_pose_topic = str(self.declare_parameter(
            "abs_pose_topic", str(src.get("abs_pose_topic", ""))).value)
        self.detections_topic = str(self.declare_parameter(
            "detections_topic", str(src.get("detections_topic", "/perception/detections"))).value)
        self.use_odom_pose_as_abs = bool(src.get("use_odom_pose_as_abs", True))
        self.camera_yaw_offset = float(src.get("camera_yaw_offset_rad", 0.0))

        # ---- 雷达里程计 → 场地坐标系 的标定 ----
        #
        # FAST-LIO 发布的是**它自己初始化时刻**建立的坐标系（camera_init）下的位姿，
        # 与场地坐标系既不同原点也不同朝向。必须先做一次刚体变换才能拿来用。
        #
        # 变换参数只有两个来源：
        #   ① start_pose：开机时车被摆在场地里的已知位置（人工摆放 + 卷尺量）
        #   ② 首帧里程计：开机瞬间 FAST-LIO 报的位姿，作为它的原点
        # 旋转量 θ 由 "start_pose.yaw - 首帧yaw" 自动得出，
        # **不需要**手填 FOV/轴偏 —— 这也是 ROBOCON 老代码要填 -90° 的原因，
        # 但那样填容易错，这里直接由开机摆放姿态定出来。
        lc = self.cfg.get("odom_frame_conversion", {}) or {}
        self.odom_conv_enabled = bool(lc.get("enabled", False))
        self.s2b_x = float(lc.get("sensor_to_base_x", 0.0))
        self.s2b_y = float(lc.get("sensor_to_base_y", 0.0))
        self.s2b_yaw = float(lc.get("sensor_to_base_yaw_deg", 0.0)) * math.pi / 180.0
        sp = lc.get("start_pose", {}) or {}
        self.start_pose = (float(sp.get("x", 0.0)), float(sp.get("y", 0.0)),
                           float(sp.get("yaw", 0.0)))
        self.odom_zero_on_first = bool(lc.get("zero_on_first", True))
        self.odom_origin: tuple[float, float, float] | None = None
        self.theta = 0.0          # odom → field 的旋转量，首帧确定
        self.conv_ready = False

        out = self.cfg.get("output", {}) or {}
        self.pose_topic = str(out.get("pose_topic", "/localization/pose"))
        self.ok_topic = str(out.get("ok_topic", "/localization/ok"))
        self.publish_tf = bool(out.get("publish_tf", True))
        self.field_frame = str(out.get("field_frame", "field"))
        self.base_frame = str(out.get("base_frame", "base_link"))
        self.ok_std_threshold = float(out.get("ok_std_threshold_m", 0.60))
        # 发布频率。原来"每 50 条 odom 发一次"，在 50Hz odom 下只有 1Hz，
        # 下游状态机与 mux 都会觉得定位很卡。改成独立定时器。
        self.publish_rate_hz = float(out.get("publish_rate_hz", 20.0))

        start = self.cfg.get("initial_pose", {}) or {}
        self.ekf.set_pose(float(start.get("x", 0.0)), float(start.get("y", 0.0)),
                          float(start.get("yaw", 0.0)))

        self.landmarks: list[tuple[float, float]] = []
        for item in self.cfg.get("landmarks", []) or []:
            if "x" in item and "y" in item:
                self.landmarks.append((float(item["x"]), float(item["y"])))

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        if self.abs_pose_topic:
            self.create_subscription(Odometry, self.abs_pose_topic, self.on_abs_pose, 10)
        self.create_subscription(DetectionArray, self.detections_topic, self.on_detections, 5)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.ok_pub = self.create_publisher(Bool, self.ok_topic, 1)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.publish_timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0),
                                               lambda: self.publish_state(None))
        self.last_odom_time: Time | None = None
        self.odom_count = 0
        self.abs_count = 0
        self.pillar_updates = 0

        self.get_logger().info(
            f"定位就绪: odom={self.odom_topic} abs={self.abs_pose_topic or '(未启用)'} "
            f"det={self.detections_topic} 地标数={len(self.landmarks)}"
        )
        if not self.landmarks:
            self.get_logger().warn(
                "未配置任何定位柱地标位置 → 视觉方位角更新不会生效。"
                "请在 config/localization.yaml 的 landmarks 里填入本轮定位柱的实际坐标。"
            )
        if self.odom_conv_enabled:
            self.get_logger().info(
                f"定位方案: 雷达里程计({self.odom_topic}) + 视觉定位柱"
                f"{' + 独立绝对位姿(' + self.abs_pose_topic + ')' if self.abs_pose_topic else ''}")
            self.get_logger().info(
                f"  摆车位=({self.start_pose[0]:.2f}, {self.start_pose[1]:.2f}, "
                f"{math.degrees(self.start_pose[2]):.1f}°)  雷达安装偏移="
                f"({self.s2b_x:+.3f}, {self.s2b_y:+.3f}, {math.degrees(self.s2b_yaw):+.1f}°)")
        elif not self.abs_pose_topic:
            self.get_logger().warn(
                "未启用绝对位姿输入 → 只能靠里程计 + 定位柱，会漂移，仅适合短距离任务。"
            )

    def load_config(self) -> None:
        if self.config_file and os.path.isfile(self.config_file):
            try:
                with open(self.config_file, "r", encoding="utf-8") as fh:
                    self.cfg = yaml.safe_load(fh) or {}
                self.get_logger().info(f"已加载配置 {self.config_file}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"读取配置失败: {exc}")
                self.cfg = {}
        else:
            self.get_logger().error(f"配置文件不存在: {self.config_file}")
            self.cfg = {}

    # -- 雷达里程计 → 场地坐标系 的刚体变换 ---------------------------------
    def odom_to_field(self, ox: float, oy: float, oyaw: float):
        """把 (雷达/里程计坐标系) 的位姿换算到场地坐标系。

        步骤：
          ① 雷达 → 车体：加上安装偏移（位置要按当前 yaw 旋转）
          ② 车体 → 场地：整体刚体变换
             旋转量 θ = start_pose.yaw - 首帧yaw（首帧时自动定出）
             平移量 t = start_pose.xy - R(θ)·首帧xy
        """
        # ① 雷达安装偏移 → 车体
        c0, s0 = math.cos(oyaw), math.sin(oyaw)
        bx = ox + (self.s2b_x * c0 - self.s2b_y * s0)
        by = oy + (self.s2b_x * s0 + self.s2b_y * c0)
        byaw = wrap_pi(oyaw + self.s2b_yaw)

        if not self.odom_conv_enabled:
            return bx, by, byaw

        # 首帧：确定 odom 原点与旋转量
        if self.odom_origin is None:
            if not self.odom_zero_on_first:
                self.odom_origin = (0.0, 0.0, 0.0)
            else:
                self.odom_origin = (bx, by, byaw)
            self.theta = wrap_pi(self.start_pose[2] - self.odom_origin[2])
            self.conv_ready = True
            self.get_logger().info(
                f"odom→field 变换已确定: 首帧=({self.odom_origin[0]:.3f}, "
                f"{self.odom_origin[1]:.3f}, {math.degrees(self.odom_origin[2]):.1f}°), "
                f"旋转={math.degrees(self.theta):+.1f}°, "
                f"摆车位=({self.start_pose[0]:.2f}, {self.start_pose[1]:.2f}, "
                f"{math.degrees(self.start_pose[2]):.1f}°)")

        ox0, oy0, oyaw0 = self.odom_origin
        dx, dy = bx - ox0, by - oy0
        c, s = math.cos(self.theta), math.sin(self.theta)
        # ② 旋转到位场坐标，再平移到摆车位置
        fx = self.start_pose[0] + (c * dx - s * dy)
        fy = self.start_pose[1] + (s * dx + c * dy)
        fyaw = wrap_pi(byaw + self.theta)
        return fx, fy, fyaw

    # -- 里程计：既做预测，也可能做绝对观测 ---------------------------------
    def on_odom(self, msg: Odometry) -> None:
        now = self.get_clock().now()
        dt = 0.0
        if self.last_odom_time is not None:
            dt = (now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.0

        tw = msg.twist.twist
        if dt > 0.0:
            self.ekf.predict(tw.linear.x, tw.linear.y, tw.angular.z, dt)

        # 若这路里程计本身带绝对位姿（雷达 FAST-LIO / 全场定位板），做一次观测更新
        if self.use_odom_pose_as_abs:
            pos = msg.pose.pose.position
            # yaw 的取法：优先看四元数；本队老代码把 yaw 塞在 position.z，两种都兼容
            if abs(math.hypot(msg.pose.pose.orientation.x,
                              msg.pose.pose.orientation.y,
                              msg.pose.pose.orientation.z)) < 1e-6:
                yaw = pos.z
            else:
                yaw = self._yaw_from_quat(msg.pose.pose.orientation)
            fx, fy, fyaw = self.odom_to_field(pos.x, pos.y, yaw)
            self.ekf.update_pose(fx, fy, fyaw)
            self.abs_count += 1

        self.odom_count += 1

    def _yaw_from_quat(self, q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_abs_pose(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        yaw = pos.z if abs(pos.z) > 1e-9 else self._yaw_from_quat(msg.pose.pose.orientation)
        self.ekf.update_pose(pos.x, pos.y, yaw)

    # -- 视觉定位柱：方位角更新 --------------------------------------------
    def on_detections(self, msg: DetectionArray) -> None:
        if not self.landmarks:
            return
        used = 0
        for det in msg.detections:
            if det.label != "pillar":
                continue
            if det.bearing_rad != det.bearing_rad:  # NaN
                continue
            if det.distance_m == det.distance_m and det.distance_m > 0.0:
                # 有距离估计时：直接按"距离+方位"反算地标位置做数据关联
                pass
            # 观测到的世界方位角 = 当前 yaw + 相机安装偏置 + 目标 bearing
            bearing_world = wrap_pi(self.ekf.pose[2] + self.camera_yaw_offset + det.bearing_rad)

            # 数据关联：选预测方位角最接近的那个地标
            best = None
            best_err = float("inf")
            for lm in self.landmarks:
                dx, dy = lm[0] - self.ekf.pose[0], lm[1] - self.ekf.pose[1]
                d = math.hypot(dx, dy)
                if d < self.ekf.cfg.bearing_min_dist or d > self.ekf.cfg.bearing_max_dist:
                    continue
                err = abs(wrap_pi(bearing_world - math.atan2(dy, dx)))
                if err < best_err:
                    best_err, best = err, lm
            if best is None or best_err > 0.35:  # 关联门限 ~20°
                continue
            if self.ekf.update_bearing(best, bearing_world):
                used += 1
        if used:
            self.pillar_updates += used

    # -- 输出 --------------------------------------------------------------
    def publish_state(self, stamp) -> None:
        x, y, yaw = self.ekf.pose
        sx, sy, syaw = self.ekf.std

        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        out.header.frame_id = self.field_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = yaw  # 沿用本队约定：yaw 也塞一份在 z
        qx, qy, qz, qw = yaw_to_quat(yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance[0] = sx * sx
        out.pose.covariance[7] = sy * sy
        out.pose.covariance[35] = syaw * syaw
        self.pose_pub.publish(out)

        ok = Bool()
        ok.data = bool(math.hypot(sx, sy) <= self.ok_std_threshold)
        self.ok_pub.publish(ok)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = out.header.stamp
            tf.header.frame_id = self.field_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = LocalizationNode()
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
