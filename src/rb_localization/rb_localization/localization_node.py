#!/usr/bin/env python3
"""FAST-LIO 位姿增量预测与 2D EKF 位姿更新；输出场地位姿和可信状态。"""

from __future__ import annotations

import math
import os
import time
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


from .ekf2d import (
    Ekf2D,
    EkfConfig,
    sensor_to_base_xy,
    world_vel_to_body,
    wrap_pi,
)
# 输入新鲜度守卫（队友加）：真实位置输入过期就发 ok=false，防止拿陈旧位姿导航
from .freshness import SourceFreshness


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
        self.source_timeout_s = float(self.declare_parameter(
            "source_timeout_s", float(src.get("source_timeout_s", 1.0))).value)
        if not math.isfinite(self.source_timeout_s) or self.source_timeout_s <= 0:
            raise ValueError("source_timeout_s 必须为有限正数")
        self.source_freshness = SourceFreshness()
        self.last_source_stamp = None
        self.require_lidar_session = bool(self.declare_parameter('require_lidar_session', False).value)
        self.lidar_session_ready = False
        self.lidar_session_generation = None
        self.lidar_session_blocked = None
        self.lidar_session_received = 0.
        self.lidar_session_cutoff = 0.
        if self.require_lidar_session:
            self.create_subscription(String, '/lidar/session', self.on_lidar_session, 10)
        self.use_odom_pose_as_abs = bool(src.get("use_odom_pose_as_abs", True))

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
        # FAST-LIO 的点云/地图所在的坐标系名。它发 TF `camera_init → body`，
        # 而这里要补上 `field → camera_init`，否则 RViz 无法把点云画到 field 下
        # （症状：地图一片黑，明明 /cloud_registered 有数据）。
        self.odom_frame_name = str(lc.get("odom_frame", "camera_init"))
        self.relative_mode = bool(self.declare_parameter("relative_mode", False).value)
        if self.relative_mode:
            self.odom_conv_enabled = True
            self.odom_zero_on_first = True
            self.start_pose = (0.0, 0.0, 0.0)
        self.odom_origin: tuple[float, float, float] | None = None
        self.configured_start_pose = self.start_pose
        self.experiment_cutoff = 0.0
        self.theta = 0.0          # odom → field 的旋转量，首帧确定
        self.conv_ready = False

        out = self.cfg.get("output", {}) or {}
        self.pose_topic = str(out.get("pose_topic", "/localization/pose"))
        self.ok_topic = str(out.get("ok_topic", "/localization/ok"))
        self.publish_tf = bool(out.get("publish_tf", True))
        self.field_frame = str(out.get("field_frame", "field"))
        if self.relative_mode:
            self.field_frame = "lidar_local"
        self.base_frame = str(out.get("base_frame", "base_link"))
        self.ok_std_threshold = float(out.get("ok_std_threshold_m", 0.60))
        # 发布频率。原来"每 50 条 odom 发一次"，在 50Hz odom 下只有 1Hz，
        # 下游状态机与 mux 都会觉得定位很卡。改成独立定时器。
        self.publish_rate_hz = float(out.get("publish_rate_hz", 20.0))

        start = self.cfg.get("initial_pose", {}) or {}
        if self.relative_mode:
            self.ekf.set_pose(0.0, 0.0, 0.0)
        else:
            self.ekf.set_pose(float(start.get("x", 0.0)), float(start.get("y", 0.0)),
                              float(start.get("yaw", 0.0)))

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        if self.abs_pose_topic:
            self.create_subscription(Odometry, self.abs_pose_topic, self.on_abs_pose, 10)
        self.create_service(Trigger, '/localization/reset_start', self.reset_experiment_start)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.ok_pub = self.create_publisher(Bool, self.ok_topic, 1)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.publish_timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0),
                                               lambda: self.publish_state(None))
        self.last_odom_field_pose = None
        self.last_odom_time: Time | None = None
        self.odom_count = 0
        self.abs_count = 0
        self.add_on_set_parameters_callback(self.on_tuning_parameters)

        if self.odom_conv_enabled:
            self.get_logger().info(
                f"定位方案: 雷达里程计({self.odom_topic})"
                f"{' + 独立绝对位姿(' + self.abs_pose_topic + ')' if self.abs_pose_topic else ''}")
            self.get_logger().info(
                f"  摆车位=({self.start_pose[0]:.2f}, {self.start_pose[1]:.2f}, "
                f"{math.degrees(self.start_pose[2]):.1f}°)  雷达安装偏移="
                f"({self.s2b_x:+.3f}, {self.s2b_y:+.3f}, {math.degrees(self.s2b_yaw):+.1f}°)")
        elif not self.abs_pose_topic:
            self.get_logger().warn(
                "未启用绝对位姿输入 → 只能靠里程计，会漂移，仅适合短距离任务。"
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

    def on_tuning_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name != "source_timeout_s":
                return SetParametersResult(successful=False, reason="此参数修改需要重启节点")
            value = parameter.value
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                return SetParametersResult(successful=False, reason="source_timeout_s 必须为有限正数")
        return SetParametersResult(successful=True)

    def _refresh_tuning(self):
        self.source_timeout_s = float(self.get_parameter("source_timeout_s").value)

    def reset_experiment_start(self, request, response):
        # The operator places the robot back at A facing the basket before a run.
        if getattr(self, 'relative_mode', False) or not self.odom_conv_enabled or not self.use_odom_pose_as_abs:
            response.success = False
            response.message = '当前定位模式不支持场地起点重置'
            return response
        self.experiment_cutoff = self.get_clock().now().nanoseconds * 1e-9
        self.odom_count = self.abs_count = 0
        self.start_pose = self.configured_start_pose
        self.ekf.set_pose(*self.start_pose)
        self._reset_lidar_session()
        self.theta = 0.0
        self.publish_state(None)  # invalidate old health/pose stamps immediately
        response.success = True
        response.message = '已清除旧定位基准，等待新雷达帧重新锚定出发区A'
        self.get_logger().info(response.message)
        return response

    def _reset_lidar_session(self):
        # 保留最后接受的位置作为新会话锚点。断电期间人工搬车须重新摆到起点启动。
        if self.odom_count and all(math.isfinite(v) for v in self.ekf.pose):
            self.start_pose = tuple(self.ekf.pose)
            sx, sy, syaw = self.ekf.std
            self.ekf.set_pose(*self.start_pose, std_xy=max(sx, sy), std_yaw=syaw)
        self.odom_origin = None
        self.conv_ready = False
        self.odom_zero_on_first = True
        self.last_odom_time = None
        self.last_odom_field_pose = None
        self.last_source_stamp = None
        self.source_freshness = SourceFreshness()

    def on_lidar_session(self, msg):
        import json
        try:
            state = json.loads(msg.data)
            generation, ready, cutoff = state['generation'], state['ready'], float(state['cutoff'])
            if (type(generation) is not int or type(ready) is not bool
                    or not math.isfinite(cutoff) or ready and cutoff <= 0):
                return
        except (ValueError, TypeError, KeyError):
            return
        if generation != self.lidar_session_generation:
            self._reset_lidar_session()
            self.lidar_session_generation = generation
            self.lidar_session_blocked = None
            self.lidar_session_ready = False
        self.lidar_session_ready = ready and generation != self.lidar_session_blocked
        self.lidar_session_cutoff = cutoff
        self.lidar_session_received = time.monotonic()

    def _lidar_session_live(self):
        if not getattr(self, 'require_lidar_session', False):
            return True
        live = self.lidar_session_ready and time.monotonic()-self.lidar_session_received <= 1.
        return live

    def _accept_source(self, msg, source):
        if not self._lidar_session_live():
            return False
        self._refresh_tuning()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        tw = msg.twist.twist
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  tw.linear.x, tw.linear.y, tw.angular.z)
        if not all(math.isfinite(v) for v in values):
            return False
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= getattr(self, 'experiment_cutoff', 0.0):
            return False
        if getattr(self, 'require_lidar_session', False) and stamp < self.lidar_session_cutoff:
            return False  # 重启前的排队消息不能进入新会话。
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.source_freshness.accept(source, stamp, now, self.source_timeout_s):
            return False
        self.last_source_stamp = msg.header.stamp
        return True

    def _message_yaw(self, msg):
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm <= 1e-6:
            return msg.pose.pose.position.z  # 仅零四元数兼容旧 yaw-in-z 数据
        return self._yaw_from_quat(q)

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
        #
        # ⚠️ 偏移按【车体 yaw】旋转，**不是**雷达 yaw —— 细节与踩坑记录见
        #    ekf2d.sensor_to_base_xy（那里有单测锁住）。
        #    本车：雷达在中心正前方 30cm ⇒ s2b = (−0.30, 0.00)。
        bx, by, byaw = sensor_to_base_xy(
            ox, oy, oyaw, self.s2b_x, self.s2b_y, self.s2b_yaw)

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
        if not self._accept_source(msg, "odom"):
            return
        now = Time.from_msg(msg.header.stamp)
        dt = 0.0
        if self.last_odom_time is not None:
            dt = (now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = now
        pos = msg.pose.pose.position
        yaw = self._message_yaw(msg)

        if self.use_odom_pose_as_abs:
            current = self.odom_to_field(pos.x, pos.y, yaw)
            if (getattr(self, 'require_lidar_session', False) and self.last_odom_field_pose is not None
                    and math.dist(current[:2], self.last_odom_field_pose[:2]) > .25+max(0.,dt)):
                # 本节点也检查跳变，避免管理器与定位订阅回调的先后顺序影响结果。
                self._reset_lidar_session()
                self.lidar_session_ready = False
                self.lidar_session_blocked = self.lidar_session_generation
                return
            # FAST-LIO publish_odometry() fills pose only. Its /Odometry twist
            # remains zero; velocities on the separate IMU-propagated topic
            # cannot be assumed to be present here. Predict from pose changes
            # before the absolute update, rather than low-pass filtering motion.
            if self.last_odom_field_pose is not None and dt > 0.0:
                self.ekf.predict_pose_delta(self.last_odom_field_pose, current, dt)
            self.last_odom_field_pose = current
            self.ekf.update_pose(*current)
            self.abs_count += 1
        elif 0.0 < dt <= self.source_timeout_s:
            # Velocity-only sources retain the existing world-frame convention.
            tw = msg.twist.twist
            byaw = wrap_pi(yaw + self.s2b_yaw)
            vx_b, vy_b = world_vel_to_body(tw.linear.x, tw.linear.y, byaw)
            self.ekf.predict(vx_b, vy_b, tw.angular.z, dt)

        self.odom_count += 1

    def _yaw_from_quat(self, q) -> float:
        norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        return math.atan2(2.0 * (q.w*q.z + q.x*q.y) / norm2,
                          1.0 - 2.0 * (q.y*q.y + q.z*q.z) / norm2)

    def on_abs_pose(self, msg: Odometry) -> None:
        if not self._accept_source(msg, "abs"):
            return
        pos = msg.pose.pose.position
        # 同一坑：不能用 |pos.z|>0 判断（pos.z 是高度）。统一走 _message_yaw。
        self.ekf.update_pose(pos.x, pos.y, self._message_yaw(msg))

    # -- 输出 --------------------------------------------------------------
    def publish_state(self, stamp) -> None:
        session_live = self._lidar_session_live()
        self._refresh_tuning()
        x, y, yaw = self.ekf.pose
        sx, sy, syaw = self.ekf.std

        out = PoseWithCovarianceStamped()
        # 保留最近一次真实位置输入的时间，定时器不能给旧位姿续期。
        if self.last_source_stamp is not None and session_live:
            out.header.stamp = self.last_source_stamp
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
        source_ok = session_live and self.source_freshness.valid(
            self.get_clock().now().nanoseconds * 1e-9, self.source_timeout_s)
        ok.data = bool(not self.relative_mode and source_ok
                       and math.hypot(sx, sy) <= self.ok_std_threshold)
        self.ok_pub.publish(ok)

        if self.tf_broadcaster is not None and source_ok:
            tf = TransformStamped()
            tf.header.stamp = out.header.stamp
            tf.header.frame_id = self.field_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

            # ── 额外发 field → camera_init ────────────────────────────────
            # 为什么需要：FAST-LIO 的点云/地图（/cloud_registered、/Laser_map）
            # 都在 **camera_init** 坐标系里，它自己只发 `camera_init → body`；
            # 而我们发的位姿在 **field** 里。两棵树不连通 → RViz 的 Fixed Frame
            # 是 field 时**什么都画不出来，地图一片黑**（数据其实是好的）。
            #
            # 这个变换是**固定**的，由 odom→field 的对齐关系定出来：
            #   odom_to_field:  field = start_pose + R(theta)·(odom − origin)
            #   ⇒ 反向： camera_init 原点在 field 下 = start_pose − R(theta)·origin
            #            camera_init 相对 field 的朝向 = theta
            if self.odom_conv_enabled and self.conv_ready and self.odom_origin is not None:
                ox0, oy0, _ = self.odom_origin
                c, s = math.cos(self.theta), math.sin(self.theta)
                tf_ci = TransformStamped()
                tf_ci.header.stamp = out.header.stamp
                tf_ci.header.frame_id = self.field_frame
                tf_ci.child_frame_id = self.odom_frame_name
                tf_ci.transform.translation.x = self.start_pose[0] - (c * ox0 - s * oy0)
                tf_ci.transform.translation.y = self.start_pose[1] - (s * ox0 + c * oy0)
                _, _, qz_ci, qw_ci = yaw_to_quat(self.theta)
                tf_ci.transform.rotation.z = qz_ci
                tf_ci.transform.rotation.w = qw_ci
                self.tf_broadcaster.sendTransform(tf_ci)


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
