#!/usr/bin/env python3
"""巡线找球 —— 纯视觉、分步可见的找球调试程序。

设计目标（现场要求）
--------------------
"从出发点逆时针转 90° → 直行 1 m → 后摄对着场地中线 → 沿出发点→篮筐
 的方向横着走过去找球 → 让球正对车头 → 用视觉标定测距 → 前进到
 球离车中心 70 cm。"

特点（为什么另写一个）
----------------------
* **只用视觉**：完全不依赖雷达点云球面拟合，因此不会碰到
  precision 那套 `stale_or_unsynchronised` 的坑 ✗
* **单目测距自己做**：D = fy * 球直径 / 框高（针孔模型），
  用的是相机真实内参（所以叫"视觉标定"）✓
* **一步一打印**：每一步都把位姿/朝向/距离打出来，
  界面日志面板里实时可见，出问题能立刻定位 ✓
* 复用任务侧的 `goto_command`（和比赛同一套运动学）✓

安全
----
* 先检查节点；`--takeover` 会 SIGTERM 掉 mission（它也发 /cmd_vel）✓
* 全程有超时；任何一步失败都先停车再退出 ✓
* 结束时连发 5 次零速 ✓

用法
----
    python3 tools/line_ball_search.py --dry              # 只检查，不动车
    python3 tools/line_ball_search.py --takeover --yes   # 真跑
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS / "src" / "rb_mission"))

# ── 现场常量（可在命令行覆盖）───────────────────────────────────────
COURT_L, COURT_W = 14.0, 7.5
HOOP = (12.425, 3.75)          # 篮筐位置（mission.yaml 的 field.hoop）
HOME = (1.50, 8.00)            # 出发点（实测 /localization/pose）
BALL_DIA_M = 0.24              # 篮球直径（7 号球，规则给的）
CAM_FX, CAM_FY = 794.16, 793.9  # 相机内参（perception.yaml，已标定）
CAM_CX, CAM_CY = 577.4, 378.38

C_OK, C_BAD, C_DIM, C_HI = "\033[32m", "\033[31m", "\033[2m", "\033[36m"
C_END = "\033[0m"


def log(tag: str, msg: str, color: str = "") -> None:
    print(f"  {C_DIM}{time.strftime('%H:%M:%S')}{C_END} "
          f"{color}{tag:<10}{C_END} {msg}", flush=True)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Runner:
    """管 ROS 连接、位姿/检测缓存，以及基本动作。"""

    def __init__(self, args) -> None:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rb_msgs.msg import DetectionArray
        from rb_mission.geometry import goto_command

        self.args = args
        self.rclpy = rclpy
        self._Twist = Twist
        self._goto_command = goto_command
        rclpy.init()
        self.node = Node("line_ball_search")
        self.pose = None          # (x, y, yaw)
        self.dets: list = []
        self.last_det = 0.0
        self.node.create_subscription(
            PoseWithCovarianceStamped, "/localization/pose", self._on_pose, 10)
        self.node.create_subscription(
            DetectionArray, "/perception/detections", self._on_dets, 10)
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

    # -- 回调 ---------------------------------------------------------------
    def _on_pose(self, m) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if math.hypot(q.x, q.y, q.z, q.w) > 1e-6:
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
        else:
            yaw = p.z
        self.pose = (float(p.x), float(p.y), float(yaw))

    def _on_dets(self, m) -> None:
        self.dets = list(m.detections)
        self.last_det = time.time()

    # -- 基础 ---------------------------------------------------------------
    def spin(self, sec: float) -> None:
        t0 = time.time()
        while time.time() - t0 < sec:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def stop(self) -> None:
        for _ in range(5):
            self.cmd_pub.publish(self._Twist())
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_pose(self, timeout: float = 8.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and self.pose is None:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.pose is not None

    def rotate_to(self, target_yaw: float, timeout: float) -> bool:
        """原地转到目标朝向。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin(0.05)
            if self.pose is None:
                self.stop()
                return False
            err = wrap_pi(target_yaw - self.pose[2])
            if abs(err) < self.args.yaw_tol:
                self.stop()
                log("转向到位", f"yaw 误差 {math.degrees(err):+.1f}°", C_OK)
                return True
            t = self._Twist()
            t.angular.z = max(-self.args.max_ang,
                              min(self.args.max_ang, self.args.kp_yaw * err))
            self.cmd_pub.publish(t)
        self.stop()
        log("转向超时", f"差 {math.degrees(err):+.1f}°", C_BAD)
        return False

    def drive_body(self, vx: float, sec: float) -> None:
        """按【车体系】速度走一段时间（不纠偏，用于搜索时的直行）。"""
        t0 = time.time()
        while time.time() - t0 < sec:
            t = self._Twist()
            t.linear.x = vx
            self.cmd_pub.publish(t)
            self.spin(0.05)
        self.stop()

    def forward_m(self, dist_m: float, timeout: float, speed: float) -> bool:
        """朝着【车头方向】直走 dist_m（用位姿积分判断走了多远）。"""
        if self.pose is None:
            return False
        x0, y0, yaw = self.pose
        cx, cy = math.cos(yaw), math.sin(yaw)
        t0 = time.time()
        last = 0.0
        while time.time() - t0 < timeout:
            self.spin(0.05)
            if self.pose is None:
                self.stop()
                return False
            dx, dy = self.pose[0] - x0, self.pose[1] - y0
            done = dx * cx + dy * cy          # 沿出发朝向的投影
            if done >= dist_m:
                self.stop()
                log("直行到位", f"走了 {done:.2f} m（目标 {dist_m:.2f}）", C_OK)
                return True
            t = self._Twist()
            t.linear.x = speed
            self.cmd_pub.publish(t)
            if time.time() - last > 1.0:
                last = time.time()
                log("直行", f"已走 {done:.2f} / {dist_m:.2f} m")
        self.stop()
        log("直行超时", f"只走了 {done:.2f} m", C_BAD)
        return False

    # -- 视觉 ---------------------------------------------------------------
    def balls(self) -> list:
        """当前有效的球检测（检测要在 1s 内，否则算过期 ✗）。"""
        if time.time() - self.last_det > 1.0:
            return []
        return [d for d in self.dets if str(d.label) == self.args.ball_label]

    @staticmethod
    def vision_distance(det) -> float:
        """单目测距：D = fy * 真实直径 / 框高(像素)。

        针孔模型：物体在图像里的高度 h_px = fy * H / D → D = fy * H / h_px ✓
        优先用消息里已经算好的 distance_m（感知算过就不用重算），
        拿不到（NaN）时才用上面的公式现算 ✓ —— 这就是"视觉标定"。
        """
        d = float(getattr(det, "distance_m", float("nan")))
        if d == d and d > 0.01:
            return d
        h_px = float(getattr(det, "bbox_px_h", 0.0) or 0.0)
        if h_px <= 1.0:
            x0, y0, x1, y1 = (float(v) for v in det.bbox)
            h_px = max(1.0, (y1 - y0) * 720.0)
        return CAM_FY * BALL_DIA_M / h_px

    def best_ball(self):
        c = self.balls()
        if not c:
            return None
        return max(c, key=lambda d: float(d.confidence))

    def face_ball(self, timeout: float = 20.0) -> object | None:
        """原地转，让球正对车头（bearing → 0）。返回对准后的检测。"""
        t0 = time.time()
        det = None
        while time.time() - t0 < timeout:
            self.spin(0.05)
            det = self.best_ball()
            if det is None:
                continue
            b = float(det.bearing_rad)
            if abs(b) < self.args.bearing_tol:
                self.stop()
                log("对准", f"{det.label} conf={det.confidence:.2f} "
                            f"bearing={math.degrees(b):+.1f}° "
                            f"dist={self.vision_distance(det):.2f}m", C_OK)
                return det
            t = self._Twist()
            # ⚠️ bearing 是【右正左负】，ROS 的角速度是【逆时针为正】✓
            #    所以往右偏要发负角速度：wz = -kp * bearing
            t.angular.z = max(-self.args.max_ang,
                              min(self.args.max_ang, -self.args.kp_aim * b))
            self.cmd_pub.publish(t)
        self.stop()
        log("对准失败", f"{timeout:.0f}s 内没对准", C_BAD)
        return None

    def approach_vision(self, target_cam_m: float, timeout: float) -> bool:
        """边前进边用视觉距离闭环，直到【相机到球】= target_cam_m。"""
        t0 = time.time()
        last = 0.0
        while time.time() - t0 < timeout:
            self.spin(0.05)
            det = self.best_ball()
            if det is None:
                self.stop()
                log("接近中断", "球跟丢了", C_BAD)
                return False
            dist = self.vision_distance(det)
            b = float(det.bearing_rad)
            if dist <= target_cam_m:
                self.stop()
                log("到位", f"相机到球 {dist:.2f} m（目标 {target_cam_m:.2f}）", C_OK)
                return True
            err = dist - target_cam_m
            v = max(-0.10, min(self.args.approach_speed, self.args.kp_dist * err))
            t = self._Twist()
            t.linear.x = v
            # 边走边微调朝向，别让球跑出画面 ✓
            t.angular.z = max(-0.4, min(0.4, -self.args.kp_aim * b))
            self.cmd_pub.publish(t)
            if time.time() - last > 1.0:
                last = time.time()
                log("接近", f"相机到球 {dist:.2f} m（目标 {target_cam_m:.2f}）"
                            f"  bearing={math.degrees(b):+.1f}°  v={v:.2f}")
        self.stop()
        log("接近超时", f"{timeout:.0f}s", C_BAD)
        return False


# ── 前置检查 ─────────────────────────────────────────────────────────
KEY_PROCS = (("底盘", "rb_chassis_node"), ("定位", "localization_node"),
             ("感知", "perception_node"), ("FAST-LIO", "fastlio_mapping"))


def check_nodes() -> bool:
    import subprocess
    try:
        out = subprocess.run(["ps", "-eo", "comm"], capture_output=True,
                             text=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        out = ""
    comms = {ln.strip() for ln in out.splitlines()}
    ok = True
    for label, name in KEY_PROCS:
        hit = any(c.startswith(name[:15]) for c in comms)
        print(f"        {C_OK if hit else C_BAD}{'✓' if hit else '✗'}{C_END} {label}")
        ok &= hit
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="巡线找球：逆时针转90° → 直行1m → 横向找球 → 对准 → 视觉测距接近")
    ap.add_argument("--dry", action="store_true", help="只查前置条件，不动车")
    ap.add_argument("--takeover", action="store_true", help="自动停掉 mission（它也发 /cmd_vel）")
    ap.add_argument("--yes", action="store_true", help="不问确认")
    ap.add_argument("--ball-label", default="ball_basketball", help="目标球标签")
    # ① ② 起步动作
    ap.add_argument("--turn-deg", type=float, default=90.0,
                    help="起步原地转角，正=逆时针（默认 +90）")
    ap.add_argument("--forward-m", type=float, default=1.0, help="转完直行距离")
    # ④ 横向搜索
    ap.add_argument("--search-sec", type=float, default=12.0, help="横向搜索最长时间")
    ap.add_argument("--search-speed", type=float, default=0.15, help="搜索时的前进速度")
    # ⑤⑥ 对准与接近
    ap.add_argument("--stop-center-m", type=float, default=0.70,
                    help="停位：球离【车中心】的距离")
    ap.add_argument("--cam-offset-m", type=float, default=0.27,
                    help="相机相对车中心的前向偏移（相机在车头）")
    ap.add_argument("--approach-speed", type=float, default=0.12)
    # 控制器参数
    ap.add_argument("--speed", type=float, default=0.25, help="平移速度上限")
    ap.add_argument("--kp-yaw", type=float, default=2.0)
    ap.add_argument("--kp-aim", type=float, default=1.5, help="对准比例增益")
    ap.add_argument("--kp-dist", type=float, default=1.2, help="接近比例增益")
    ap.add_argument("--max-ang", type=float, default=0.8)
    ap.add_argument("--yaw-tol", type=float, default=0.05, help="朝向容差 rad")
    ap.add_argument("--bearing-tol", type=float, default=0.04, help="对准容差 rad")
    args = ap.parse_args(argv)

    target_cam = args.stop_center_m - args.cam_offset_m

    print("🤖 巡线找球：逆时针转 → 直行 → 横向找球 → 对准 → 视觉测距接近")
    print(f"   {C_DIM}球径 {BALL_DIA_M} m · 内参 fy={CAM_FY} · "
          f"停位 球离车中心 {args.stop_center_m:.2f} m "
          f"(相机到球 {target_cam:.2f} m){C_END}")
    print()
    print("[1/6] 检查节点")
    if not check_nodes():
        print(f"  {C_BAD}节点不全，先点「⚡ 一键启动流程」{C_END}")
        return 1

    print("\n[2/6] 接管 /cmd_vel")
    if args.takeover:
        import subprocess
        subprocess.run(["pkill", "-f", "[m]ission_node"], capture_output=True)
        time.sleep(1.5)
        print(f"        {C_OK}✓{C_END} 已停 mission")
    else:
        print(f"        {C_DIM}（没加 --takeover；mission 在跑会抢话题）{C_END}")

    run = Runner(args)
    try:
        if not run.wait_pose():
            log("失败", "读不到位姿 /localization/pose", C_BAD)
            return 1

        x0, y0, yaw0 = run.pose
        log("起点", f"({x0:+.2f}, {y0:+.2f}) yaw={math.degrees(yaw0):+.1f}°", C_HI)
        # 出发点 → 篮筐 的方向（用来描述"横向"是哪边）
        to_hoop = math.atan2(HOOP[1] - HOME[1], HOOP[0] - HOME[0])
        log("参考", f"出发点→篮筐 方位 {math.degrees(to_hoop):+.1f}°，"
                    f"场地中线 y={COURT_W / 2:.2f}", C_DIM)

        if args.dry:
            print(f"\n  {C_OK}--dry：前置条件都过了，没动车{C_END}")
            return 0
        if not args.yes:
            print(f"\n  {C_BAD}没加 --yes，不动车{C_END}")
            return 0

        # ── ① 逆时针转 90° ────────────────────────────────────────────
        print(f"\n[3/6] 原地逆时针转 {args.turn_deg:.0f}°")
        tgt = wrap_pi(yaw0 + math.radians(args.turn_deg))
        if not run.rotate_to(tgt, timeout=15.0):
            return 1

        # ── ② 直行 1 m ────────────────────────────────────────────────
        print(f"\n[4/6] 朝车头方向直行 {args.forward_m:.2f} m")
        if not run.forward_m(args.forward_m, timeout=20.0, speed=args.speed):
            return 1
        if run.pose:
            x1, y1, yaw1 = run.pose
            log("现在", f"({x1:+.2f}, {y1:+.2f}) yaw={math.degrees(yaw1):+.1f}°", C_HI)
            # 后摄朝向 = 车头反方向
            rear = wrap_pi(yaw1 + math.pi)
            log("后摄朝向", f"{math.degrees(rear):+.1f}°"
                            f"（场地中线在 y={COURT_W / 2:.2f}）", C_HI)

        # ── ③ 横向搜索（沿车头方向走，即 A→篮筐 的横向）──────────────
        print(f"\n[5/6] 横向搜索 {args.ball_label}（最多 {args.search_sec:.0f}s）")
        t0 = time.time()
        det = None
        while time.time() - t0 < args.search_sec:
            run.spin(0.05)
            det = run.best_ball()
            if det is not None:
                run.stop()
                log("找到球", f"{det.label} conf={det.confidence:.2f} "
                              f"bearing={math.degrees(float(det.bearing_rad)):+.1f}° "
                              f"视觉距离={run.vision_distance(det):.2f}m", C_OK)
                break
            t = run._Twist()
            t.linear.x = args.search_speed
            run.cmd_pub.publish(t)
        else:
            run.stop()
            log("搜索失败", f"{args.search_sec:.0f}s 内没看到 {args.ball_label}", C_BAD)
            return 1
        run.stop()

        # ── ④ 让球正对车头 ────────────────────────────────────────────
        print("\n[6/6] 对准球 → 视觉测距接近")
        det = run.face_ball(timeout=20.0)
        if det is None:
            return 1

        # ── ⑤⑥ 视觉闭环接近到 70cm（球到车中心）──────────────────────
        if not run.approach_vision(target_cam, timeout=60.0):
            return 1

        if run.pose:
            xf, yf, yawf = run.pose
            d = run.vision_distance(run.best_ball() or det)
            log("完成", f"({xf:+.2f}, {yf:+.2f}) yaw={math.degrees(yawf):+.1f}° "
                        f"· 相机到球 {d:.2f} m · 距车中心约 "
                        f"{d + args.cam_offset_m:.2f} m", C_OK)
        print(f"\n  {C_OK}✅ 巡线找球完成：球离车中心约 {args.stop_center_m:.2f} m{C_END}")
        return 0
    except KeyboardInterrupt:
        run.stop()
        log("中断", "用户 Ctrl-C，已停车", C_BAD)
        return 130
    finally:
        try:
            run.stop()
            run.node.destroy_node()
            run.rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    raise SystemExit(main())
