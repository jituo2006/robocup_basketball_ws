#!/usr/bin/env python3
"""RoboCup 篮球机器人 · 自主任务状态机。

它取代了 ROBOCON 版本里"人拿手柄操作"的那一层。
规则依据（2026 规则文本，已提取在 ~/篮球规则/2026规则.txt）：
  §2.2 核心技术点：自主定位 / 视觉识别 / 避障 / 弹射机构 / 动态环境适应
  §2.3-5 比赛过程中禁止无线通信或遥控 → 本节点是唯一的 /cmd_vel 来源
  §2.3-6 急停按钮 + 自主拍停       → estop 话题可随时锁死
  §2.3-7 启动延迟 5~15 秒，期间不得有任何动作
  §2.5   两个环节的**目标球与干扰球角色互换**：
            传球环节：目标=篮球，干扰=排球
            投篮环节：目标=排球，干扰=篮球
  §2.5   得分与位置强相关：
            传球：在传球区内 10 分/次，区外 5 分/次
            投篮：三分线内进 10 分，三分线外进 30 分；界线外投 10 分/线内 5 分
  §2.5   违规：单次持球 >2 颗、车轮越界、恶意冲撞

设计取舍
--------
* 面向 2 个回合 × 2 次动作的流程，用"阶段机 + 重试计数"而不是大而全的行为树。
* 所有时间/速度/容差都在 mission.yaml 里，方便现场调。
* 持球状态优先取 /mission/has_ball 输入；没有传感器时退化为"按时间假定已吸取"。
"""

from __future__ import annotations

import math
import os
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Header
from std_msgs.msg import String as StringMsg

from rb_msgs.msg import DetectionArray, MissionStatus, RobotState
from rb_msgs.srv import GotoPose, Launch, SetMission

from .geometry import clamp, face_bearing_command, goto_command, point_in_polygon, wrap_pi

# 阶段常量
P_IDLE = "IDLE"
P_START_DELAY = "START_DELAY"
P_SEEK_BALL = "SEEK_BALL"
P_ACQUIRE = "ACQUIRE"
P_NAV_ZONE = "NAV_TO_ZONE"
P_ALIGN = "ALIGN"
P_LAUNCH = "LAUNCH"
P_RETURN = "RETURN_HOME"
P_DONE = "DONE"
P_ESTOP = "ESTOP"
P_FAULT = "FAULT"
P_GOTO = "GOTO"

LAUNCH_SHOOT = 0
LAUNCH_FAST_SHOOT = 1


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("rb_mission")

        default_cfg = os.path.join(get_package_share_directory("rb_mission"), "config", "mission.yaml")
        self.config_file = self.declare_parameter("config_file", default_cfg).value
        self.cfg: dict[str, Any] = {}
        self.load_config()

        lim = self.cfg.get("limits", {}) or {}
        self.max_lin = float(lim.get("max_linear", 0.4))
        self.max_ang = float(lim.get("max_angular", 0.8))
        self.kp_lin = float(lim.get("kp_linear", 1.5))
        self.kp_yaw = float(lim.get("kp_yaw", 2.0))
        self.pos_tol = float(lim.get("position_tolerance", 0.10))
        self.yaw_tol = float(lim.get("yaw_tolerance", 0.08))

        tm = self.cfg.get("timeouts", {}) or {}
        self.start_delay_s = float(tm.get("start_delay_s", 8.0))
        self.state_timeout_s = float(tm.get("state_timeout_s", 25.0))
        # 导航类阶段（找球/去动作区/回位）要跨越整个场地，距离可能接近 10m，
        # 用同一个 25s 超时必然误触发 —— 实测就踩到过（10m ÷ 0.4m/s ≈ 25s）。
        # 所以给导航阶段一个单独、更宽松的超时。
        self.nav_timeout_s = float(tm.get("nav_timeout_s", 90.0))
        self.mission_timeout_s = float(tm.get("mission_timeout_s", 300.0))
        self.acquire_s = float(tm.get("acquire_seconds", 2.5))
        self.launch_hold_s = float(tm.get("launch_hold_s", 1.5))

        rules = self.cfg.get("rules", {}) or {}
        self.actions_per_mission = int(rules.get("actions_per_mission", 2))
        self.max_ball_count = int(rules.get("max_ball_count", 2))
        self.require_zone = bool(rules.get("require_zone_compliance", True))

        self.mission_labels = self.cfg.get("mission_labels", {}) or {}
        self.field = self.cfg.get("field", {}) or {}

        # 视觉势场避障参数
        av = self.cfg.get("avoidance", {}) or {}
        self.avoid_enabled = bool(av.get("enabled", True))
        self.avoid_range = float(av.get("range_m", 1.2))
        self.avoid_gain = float(av.get("repulsion_gain", 0.8))
        self.avoid_max = float(av.get("max_repel_vel", 0.30))
        self.avoid_min_conf = float(av.get("min_confidence", 0.30))
        self.avoid_extra = list(av.get("extra_labels", []) or [])

        # 运行时状态
        self.phase = P_IDLE
        self.mission = "IDLE"
        self.actions_done = 0
        self.phase_enter_time = self.get_clock().now()
        self.mission_start_time = self.get_clock().now()
        self.estop = False
        self.has_ball = False
        self.ball_count = 0
        self.launch_ok = False
        self.loc_ok = False
        self.last_pose = (0.0, 0.0, 0.0)
        self.has_pose = False
        self.detections: list[Any] = []
        self.target_zone_xy = (0.0, 0.0)
        self.goto_goal: tuple[float, float, float, bool] | None = None
        self.detail = ""
        self.launch_called = False
        self.last_ball_dist = float("inf")
        self.last_ball_time = None

        self.cmd_pub = self.create_publisher(Twist, self.cfg.get("topics", {}).get("cmd_vel", "/cmd_vel"), 10)
        self.status_pub = self.create_publisher(MissionStatus, "/mission/status", 10)
        self.state_pub = self.create_publisher(RobotState, "/mission/state", 10)

        topics = self.cfg.get("topics", {}) or {}
        self.create_subscription(PoseWithCovarianceStamped, topics.get("pose", "/localization/pose"), self.on_pose, 10)
        self.create_subscription(Bool, topics.get("loc_ok", "/localization/ok"), self.on_loc_ok, 5)
        self.create_subscription(DetectionArray, topics.get("detections", "/perception/detections"), self.on_detections, 5)
        self.create_subscription(Bool, topics.get("launcher_ok", "/rb_launcher/ok"), self.on_launcher_ok, 5)
        self.create_subscription(Bool, topics.get("estop", "/mission/estop"), self.on_estop, 5)
        self.create_subscription(Bool, topics.get("has_ball", "/mission/has_ball"), self.on_has_ball, 5)

        self.srv = self.create_service(SetMission, "~/set_mission", self.on_set_mission)
        self.goto_srv = self.create_service(GotoPose, "~/goto_pose", self.on_goto_pose)
        self.launch_client = self.create_client(Launch, topics.get("launch_service", "/rb_launcher/launch"))

        period = 1.0 / float(self.cfg.get("control", {}).get("rate_hz", 20.0))
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"mission 就绪: 场地 {self.field.get('length_m', '?')}x{self.field.get('width_m', '?')} m, "
            f"每次任务 {self.actions_per_mission} 次动作, 启动延迟 {self.start_delay_s}s"
        )
        self.get_logger().info("用 /rb_mission/set_mission 切换任务：PASS / SHOOT / IDLE")

    # -- 配置 --------------------------------------------------------------
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

    # -- 输入回调 ----------------------------------------------------------
    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        yaw = p.z
        q = msg.pose.pose.orientation
        if abs(math.hypot(q.x, q.y, q.z)) < 1e-6:
            yaw = p.z
        else:
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.last_pose = (p.x, p.y, yaw)
        self.has_pose = True

    def on_loc_ok(self, msg: Bool) -> None:
        self.loc_ok = bool(msg.data)

    def on_launcher_ok(self, msg: Bool) -> None:
        self.launch_ok = bool(msg.data)

    def on_estop(self, msg: Bool) -> None:
        if msg.data and not self.estop:
            self.get_logger().warn("收到急停/自主拍停 → 立即零速并锁死")
        self.estop = bool(msg.data)
        if self.estop:
            self.publish_zero()
            self.set_phase(P_ESTOP, "急停已触发")

    def on_has_ball(self, msg: Bool) -> None:
        self.has_ball = bool(msg.data)
        self.ball_count = 1 if self.has_ball else 0

    def on_detections(self, msg: DetectionArray) -> None:
        self.detections = list(msg.detections)

    def on_set_mission(self, req: SetMission.Request, res: SetMission.Response) -> SetMission.Response:
        name = (req.mission or "").strip().upper()
        if name in ("PASS", "SHOOT"):
            self.start_mission(name)
            res.accepted, res.message = True, f"已启动 {name} 任务"
        elif name in ("IDLE", "CANCEL", "STOP"):
            self.start_mission("IDLE")
            res.accepted, res.message = True, "已停止"
        else:
            res.accepted, res.message = False, f"未知任务 '{req.mission}'（可用 PASS/SHOOT/IDLE）"
        return res

    def on_goto_pose(self, req: GotoPose.Request, res: GotoPose.Response) -> GotoPose.Response:
        """电脑端/标定用的"走到场地某点"。仅在 IDLE 下可用，避免和自主任务抢 /cmd_vel。"""
        if self.estop:
            res.accepted, res.message = False, "急停中，拒绝 GOTO"
            return res
        if self.mission != "IDLE":
            res.accepted, res.message = False, (
                f"当前任务 {self.mission} 非 IDLE，请先 ros2 service call /rb_mission/set_mission ... mission:=IDLE")
            return res
        x, y = float(req.x), float(req.y)
        if not math.isfinite(x) or not math.isfinite(y):
            res.accepted, res.message = False, "目标坐标无效"
            return res
        # 场内软限位（宽松，只挡明显越界）
        fl, fw = float(self.field.get("length_m", 14.0)), float(self.field.get("width_m", 7.5))
        if not (-2.0 <= x <= fl + 2.0 and -2.0 <= y <= fw + 2.0):
            res.accepted, res.message = False, f"目标 ({x:.2f},{y:.2f}) 超出场地范围"
            return res
        self.goto_goal = (x, y, float(req.yaw), bool(req.align_yaw))
        self.publish_zero()
        self.set_phase(P_GOTO, f"GOTO ({x:.2f}, {y:.2f})" + (f" yaw={req.yaw:.2f}" if req.align_yaw else ""))
        res.accepted, res.message = True, "已开始 GOTO"
        return res

    # -- 状态机 ------------------------------------------------------------
    def set_phase(self, phase: str, detail: str = "") -> None:
        if phase != self.phase:
            self.get_logger().info(f"[{self.mission}] {self.phase} -> {phase}  {detail}")
        self.phase = phase
        self.phase_enter_time = self.get_clock().now()
        self.detail = detail
        if phase == P_LAUNCH:
            self.launch_called = False
        self.last_ball_dist = float("inf")
        self.last_ball_time = None

    def start_mission(self, mission: str) -> None:
        self.mission = mission
        self.goto_goal = None
        self.actions_done = 0
        self.mission_start_time = self.get_clock().now()
        if mission == "IDLE":
            self.publish_zero()
            self.set_phase(P_IDLE, "待命")
        else:
            self.has_ball = False
            self.ball_count = 0
            self.publish_zero()
            self.set_phase(P_START_DELAY, f"{mission}: 规则要求的 {self.start_delay_s}s 启动延迟")

    def elapsed(self) -> float:
        return (self.get_clock().now() - self.phase_enter_time).nanoseconds * 1e-9

    def mission_elapsed(self) -> float:
        return (self.get_clock().now() - self.mission_start_time).nanoseconds * 1e-9

    def target_label(self, kind: str) -> str:
        """kind ∈ {ball, interference, aim}；按当前环节取对应类别标签。"""
        entry = self.mission_labels.get(self.mission, {}) or {}
        return str(entry.get(kind, ""))

    def tick(self) -> None:
        if self.estop:
            self.publish_zero()
            self.publish_outputs()
            return

        if self.mission != "IDLE" and self.mission_elapsed() > self.mission_timeout_s:
            self.set_phase(P_FAULT, f"任务总超时 {self.mission_timeout_s}s")
            self.publish_zero()
            self.publish_outputs()
            return

        handler = getattr(self, f"_phase_{self.phase.lower()}", None)
        if handler is None:
            self.publish_zero()
        else:
            # 非等待类阶段加通用超时保护（导航类用更宽松的超时）
            limit = self.nav_timeout_s if self.phase in (P_SEEK_BALL, P_NAV_ZONE, P_RETURN, P_GOTO) \
                else self.state_timeout_s
            if self.phase not in (P_IDLE, P_DONE, P_ESTOP, P_FAULT) and self.elapsed() > limit:
                self.get_logger().warn(f"阶段 {self.phase} 超时 {limit:.0f}s，重试")
                self.retry_or_fault()
            else:
                handler()
        self.publish_outputs()

    def retry_or_fault(self) -> None:
        if self.phase == P_GOTO:
            self.goto_goal = None
            self.publish_zero()
            self.set_phase(P_IDLE, "GOTO 超时，已取消")
        elif self.actions_done >= self.actions_per_mission:
            self.set_phase(P_RETURN, "动作已做完，回位")
        elif self.mission == "IDLE":
            self.set_phase(P_IDLE)
        else:
            self.set_phase(P_SEEK_BALL, "重试找球")

    # -- 各阶段 ------------------------------------------------------------
    def _phase_idle(self) -> None:
        self.publish_zero()

    def _phase_start_delay(self) -> None:
        # §2.3-7：延迟期间不得有任何动作 → 不发速度（但不发速度会让底盘超时归零，正好）
        if self.elapsed() >= self.start_delay_s:
            self.set_phase(P_SEEK_BALL, "延迟结束，开始找球")

    def _phase_seek_ball(self) -> None:
        if not self.has_pose:
            self.publish_zero()
            self.detail = "等待定位…"
            return
        label = self.target_label("ball")
        det = self._best_detection(label)
        if det is None:
            # 关键的真实场景问题：车头贴近球时，球会钻到车底**移出相机视野**。
            # 如果"刚刚还看得很近"，那多半不是丢了，而是已经贴上去了。
            lost_close = False
            if self.last_ball_time is not None:
                since = (self.get_clock().now() - self.last_ball_time).nanoseconds * 1e-9
                if self.last_ball_dist < self.cfg.get("limits", {}).get("close_lost_distance_m", 0.9) \
                        and since > 0.5:
                    lost_close = True
            if lost_close:
                self.set_phase(P_ACQUIRE, "球在近距离消失（应已贴上），进入吸取")
                return
            # 真的没看到：原地慢转搜索
            self.publish_cmd(0.0, 0.0, self.max_ang * 0.5)
            self.detail = f"搜索 {label}"
            return
        if det.distance_m == det.distance_m:
            self.last_ball_dist = det.distance_m
            self.last_ball_time = self.get_clock().now()
        if det.distance_m == det.distance_m and det.distance_m < self.field.get("grab_distance_m", 0.45):
            self.set_phase(P_ACQUIRE, "接近到位，开始吸取")
            return
        # 朝球前进，同时对准。
        # 速度按"距离"给一个减速曲线（原来固定 max_lin*0.5 太慢，实测整段要 20s+）：
        #   远 → 接近全速；近 → 线性减速，避免冲过去。
        wz, err = face_bearing_command(0.0, det.bearing_rad, self.kp_yaw, self.max_ang)
        speed = self.max_lin * self.cfg.get("limits", {}).get("seek_speed_ratio", 0.8)
        if det.distance_m == det.distance_m:  # 有距离估计
            speed = min(speed, max(0.08, self.kp_lin * max(0.0, det.distance_m - 0.25)))
        self.publish_motion(speed, 0.0, wz)
        self.detail = f"接近 {label} bearing={det.bearing_rad:+.2f} v={speed:.2f}"

    def _phase_acquire(self) -> None:
        # 优先用真实传感器；没有就按时间假定完成（并在日志里说明）
        if self.has_ball:
            self.set_phase(P_NAV_ZONE, "已持球，前往动作区")
            return
        if self.elapsed() < self.acquire_s:
            self.publish_motion(self.max_lin * 0.3, 0.0, 0.0)
            self.detail = "吸取中（按时间假定，接传感器可改用 /mission/has_ball）"
            return
        self.has_ball = True
        self.ball_count = 1
        self.get_logger().warn("未收到 /mission/has_ball，按时间假定已持球（调通后应接真实传感器）")
        self.set_phase(P_NAV_ZONE, "假定已持球，前往动作区")

    def _phase_nav_to_zone(self) -> None:
        if not self.has_pose:
            self.publish_zero()
            self.detail = "等待定位…"
            return
        tgt = self._zone_target()
        x, y, yaw = self.last_pose
        vx, vy, wz, dist = goto_command(x, y, yaw, tgt[0], tgt[1],
                                        self.kp_lin, self.kp_yaw, self.max_lin, self.max_ang)
        if dist < self.pos_tol:
            self.publish_zero()
            self.set_phase(P_ALIGN, "已到动作区，开始对准")
            return
        self.publish_motion(vx, vy, wz)
        self.detail = f"前往动作区 剩余 {dist:.2f}m"

    def _phase_align(self) -> None:
        aim_label = self.target_label("aim")
        det = self._best_detection(aim_label)
        if det is None:
            self.publish_cmd(0.0, 0.0, self.max_ang * 0.4)
            self.detail = f"寻找 {aim_label}"
            return
        wz, err = face_bearing_command(0.0, det.bearing_rad, self.kp_yaw, self.max_ang)
        self.publish_cmd(0.0, 0.0, wz)
        self.detail = f"对准 {aim_label} 误差 {math.degrees(err):+.1f}°"
        if abs(err) < self.yaw_tol:
            self.set_phase(P_LAUNCH, "对准完成，执行动作")

    def _phase_launch(self) -> None:
        if not self._zone_ok():
            self.get_logger().warn("当前不在合规位置，退回动作区（否则本次动作只算低分）")
            self.set_phase(P_NAV_ZONE, "位置不合规")
            return

        if not self.launch_called:      # 每次进入 LAUNCH 只发一次，避免连发
            self.launch_called = True
            self._call_launcher()

        if self.elapsed() >= self.launch_hold_s:
            self.actions_done += 1
            self.has_ball = False
            self.ball_count = 0
            if self.actions_done >= self.actions_per_mission:
                self.set_phase(P_RETURN, f"已完成 {self.actions_done} 次动作，回位")
            else:
                self.set_phase(P_SEEK_BALL, f"第 {self.actions_done + 1} 次：继续找球")
            return
        self.detail = f"执行动作中（第 {self.actions_done + 1}/{self.actions_per_mission} 次）"

    def _phase_return_home(self) -> None:
        home = self.field.get("home", {}) or {}
        hx, hy = float(home.get("x", 0.0)), float(home.get("y", 0.0))
        if not self.has_pose:
            self.publish_zero()
            return
        x, y, yaw = self.last_pose
        vx, vy, wz, dist = goto_command(x, y, yaw, hx, hy, self.kp_lin, self.kp_yaw, self.max_lin, self.max_ang)
        if dist < self.pos_tol:
            self.publish_zero()
            self.set_phase(P_DONE, "已回到出发区")
            return
        self.publish_motion(vx, vy, wz)
        self.detail = f"回位 剩余 {dist:.2f}m"

    def _phase_goto(self) -> None:
        if self.goto_goal is None:
            self.publish_zero()
            self.set_phase(P_IDLE)
            return
        if not self.has_pose:
            self.publish_zero()
            self.detail = "GOTO 等待定位…"
            return
        gx, gy, gyaw, align = self.goto_goal
        x, y, yaw = self.last_pose
        vx, vy, wz, dist = goto_command(x, y, yaw, gx, gy,
                                        self.kp_lin, self.kp_yaw, self.max_lin, self.max_ang)
        if dist >= self.pos_tol:
            self.publish_motion(vx, vy, wz)
            self.detail = f"GOTO ({gx:.2f},{gy:.2f}) 剩余 {dist:.2f}m"
            return
        # 到位后：可选地原地转向
        if align:
            err = wrap_pi(gyaw - yaw)
            if abs(err) > self.yaw_tol:
                self.publish_cmd(0.0, 0.0, clamp(self.kp_yaw * err, -self.max_ang, self.max_ang))
                self.detail = f"GOTO 到位，对准 yaw 误差 {math.degrees(err):+.1f}°"
                return
        self.publish_zero()
        self.goto_goal = None
        self.set_phase(P_IDLE, "GOTO 完成")

    def _phase_done(self) -> None:
        self.publish_zero()

    def _phase_estop(self) -> None:
        self.publish_zero()

    def _phase_fault(self) -> None:
        self.publish_zero()

    # -- 工具 --------------------------------------------------------------
    def _best_detection(self, label: str):
        if not label:
            return None
        best = None
        for d in self.detections:
            if d.label != label:
                continue
            if best is None or d.confidence > best.confidence:
                best = d
        return best

    def _zone_target(self) -> tuple[float, float]:
        if self.mission == "PASS":
            z = self.field.get("pass_zone_target", {}) or {}
        else:
            z = self.field.get("shoot_zone_target", {}) or {}
        return float(z.get("x", 0.0)), float(z.get("y", 0.0))

    def _zone_ok(self) -> bool:
        """§2.5 的位置合规判定。"""
        if not self.require_zone or not self.has_pose:
            return True
        x, y, _ = self.last_pose
        hoop = self.field.get("hoop", {}) or {}
        hx, hy = float(hoop.get("x", 0.0)), float(hoop.get("y", 0.0))
        dist_hoop = math.hypot(x - hx, y - hy)

        if self.mission == "PASS":
            poly = (self.field.get("pass_zone", {}) or {}).get("polygon", [])
            return point_in_polygon(x, y, poly)
        if self.mission == "SHOOT":
            # 投篮必须在投篮边界线外
            return dist_hoop >= float(self.field.get("shoot_line_radius_m", 3.0))
        return True

    def is_three_point(self) -> bool:
        if not self.has_pose:
            return False
        hoop = self.field.get("hoop", {}) or {}
        hx, hy = float(hoop.get("x", 0.0)), float(hoop.get("y", 0.0))
        x, y, _ = self.last_pose
        return math.hypot(x - hx, y - hy) >= float(self.field.get("three_point_radius_m", 6.0))

    def _call_launcher(self) -> None:
        action = LAUNCH_SHOOT if self.mission == "SHOOT" else LAUNCH_SHOOT
        if self.mission == "SHOOT" and self.cfg.get("launcher", {}).get("use_fast_shoot"):
            action = LAUNCH_FAST_SHOOT

        if not self.launch_client.service_is_ready():
            self.get_logger().warn("发射服务未就绪，本次动作跳过（请检查 rb_launcher）")
            return
        req = Launch.Request()
        req.action = int(action)
        req.speed = int((self.cfg.get("launcher", {}) or {}).get("speed", 2800))
        req.angle = int((self.cfg.get("launcher", {}) or {}).get("angle", 64))
        self.launch_client.call_async(req)
        self.get_logger().info(f"已调用发射: action={action} speed={req.speed} angle={req.angle}")

    def _obstacle_labels(self) -> set[str]:
        """当前应该回避的类别集合。

        自主任务（PASS/SHOOT）：回避"干扰球 + obstacle + ball_unknown"，
        目标球不回避（否则永远贴不上去）。
        手动 GOTO/IDLE：没有"目标球"概念，回避所有球，避免撞上任何球。
        """
        labels: set[str] = {"obstacle"}
        labels.update(self.avoid_extra)
        if self.mission in ("PASS", "SHOOT"):
            inter = self.target_label("interference")
            if inter:
                labels.add(inter)
        else:
            labels.update({"ball_basketball", "ball_volleyball", "ball_unknown"})
        return labels

    def _avoidance_adjust(self, vx: float, vy: float) -> tuple[float, float]:
        """视觉势场避障：把期望车体系速度 (vx, vy) 叠加上附近障碍物的排斥速度。

        障碍物在车体系的方向由 bearing_rad（相对相机光轴，右正左负）给出，
        距离由单目测距给出（未标定时 NaN，安全跳过）。
        排斥速度方向 = 从障碍物指向机器人（即 -[cos b, sin b]）。
        """
        if not self.avoid_enabled:
            return vx, vy
        labels = self._obstacle_labels()
        fx, fy = float(vx), float(vy)
        for d in self.detections:
            if d.label not in labels:
                continue
            if d.confidence < self.avoid_min_conf:
                continue
            dist = d.distance_m
            if dist != dist or dist <= 0.0:  # NaN 或无效
                continue
            if dist >= self.avoid_range:
                continue
            b = d.bearing_rad
            if b != b:  # NaN（相机未标定）
                continue
            # 势场：越近排斥越强，作用半径 avoid_range
            mag = self.avoid_gain * (1.0 / max(dist, 0.15) - 1.0 / self.avoid_range)
            mag = clamp(mag, 0.0, self.avoid_max)
            fx -= mag * math.cos(b)
            fy -= mag * math.sin(b)
        # 限幅，避免排斥叠加后超过 max_lin
        speed = math.hypot(fx, fy)
        if speed > self.max_lin:
            fx *= self.max_lin / speed
            fy *= self.max_lin / speed
        return fx, fy

    def publish_motion(self, vx: float, vy: float, wz: float) -> None:
        """带避障的发布：所有"有前进分量"的阶段都应走这里，而不是 publish_cmd。"""
        vx, vy = self._avoidance_adjust(vx, vy)
        self.publish_cmd(vx, vy, wz)

    def publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = clamp(vx, -self.max_lin, self.max_lin)
        msg.linear.y = clamp(vy, -self.max_lin, self.max_lin)
        msg.angular.z = clamp(wz, -self.max_ang, self.max_ang)
        self.cmd_pub.publish(msg)

    def publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_outputs(self) -> None:
        status = MissionStatus()
        status.header = Header()
        status.header.stamp = self.get_clock().now().to_msg()
        status.mission = self.mission
        status.phase = self.phase
        status.detail = self.detail
        status.elapsed_s = float(self.mission_elapsed())
        status.score_estimate = self._score_estimate()
        self.status_pub.publish(status)

        state = RobotState()
        state.header = status.header
        state.phase = self.phase
        state.has_ball = bool(self.has_ball)
        state.ball_count = int(self.ball_count)
        state.ball_type = self.target_label("ball") if self.has_ball else "none"
        state.estop = bool(self.estop)
        state.localization_ok = bool(self.loc_ok and self.has_pose)
        state.launcher_ok = bool(self.launch_ok)
        state.perception_ok = bool(self.detections)
        if self.has_pose:
            x, y, _ = self.last_pose
            hoop = self.field.get("hoop", {}) or {}
            hx, hy = float(hoop.get("x", 0.0)), float(hoop.get("y", 0.0))
            d = math.hypot(x - hx, y - hy)
            state.in_pass_zone = point_in_polygon(x, y, (self.field.get("pass_zone", {}) or {}).get("polygon", []))
            state.in_shoot_zone_outside = d >= float(self.field.get("shoot_line_radius_m", 3.0))
        self.state_pub.publish(state)

    def _score_estimate(self) -> int:
        """粗略估分，用于现场判断策略是否值得（非官方的精确计分）。"""
        if self.mission == "SHOOT":
            per = 30 if self.is_three_point() else 10
            return per * self.actions_done
        if self.mission == "PASS":
            return 10 * self.actions_done
        return 0


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = MissionNode()
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
