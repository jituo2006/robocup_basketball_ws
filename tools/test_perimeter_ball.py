#!/usr/bin/env python3
"""整车测试程序：沿场地外围走一圈 → 回出发点 → 找篮球 → 走到球前 → 转身让背面对着球。

设计要点
--------
* **独立脚本，不改状态机**：直接发 /cmd_vel。但 rb_mission 在 IDLE 时也会 20Hz
  发零速，会和本脚本抢话题 —— 所以要先让 mission 闭嘴。用 `--takeover` 自动
  SIGTERM 掉它（优雅退出），也可手动 `pkill -f '[m]ission_node'`。
  不动底盘/定位/感知，它们照常工作。
* **复用经过测试的运动学**：到点用 `rb_mission.geometry.goto_command`
  （任务侧在用的同一份，坐标系/单位约定一致）。
* **安全兜底**：每步都有超时；超时/异常/Ctrl-C 一律发零速再退出。

用法
----
    rbws
    python3 tools/test_perimeter_ball.py --dry              # 只查前置条件，不动车
    python3 tools/test_perimeter_ball.py --takeover --yes   # 完整跑
    python3 tools/test_perimeter_ball.py --skip-perimeter   # 跳过绕场，直接找球

    # 调参：
    python3 tools/test_perimeter_ball.py --takeover --speed 0.20 --inset 0.8
    python3 tools/test_perimeter_ball.py --takeover --ball-label ball_basketball
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time

WS = "/home/user/robocup_basketball_ws"
C_OK, C_BAD, C_WARN, C_DIM, C_OFF = (
    "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m")


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def bearing_to_world(yaw: float, bearing_rad: float, cam_yaw_offset: float = 0.0) -> float:
    """相机方位角 → 场地系方位角。

    ⚠️ 符号是本文件最容易错的地方：`Detection.bearing_rad` 是相机约定
    **右正左负**（顺时针为正），而场地系是 **y 朝左、逆时针为正**，两者相反。
    所以是 `yaw - bearing`，不是 `yaw + bearing`。
    （与 rb_localization.ekf2d.camera_bearing_to_world 同一约定，那边有单测锁住。）
    """
    return wrap_pi(yaw - bearing_rad + cam_yaw_offset)


def yaw_to_put_ball_behind(yaw: float, bearing_rad: float,
                          cam_yaw_offset: float = 0.0) -> float:
    """算出"让【背面】对着球"时车头该朝哪。

    球在场地系方位是 B；车头朝 B+π 时，车头背对球 → 背面正对球。
    """
    return wrap_pi(bearing_to_world(yaw, bearing_rad, cam_yaw_offset) + math.pi)


def depth_to_slant(depth_m: float, bearing_rad: float,
                   max_bearing_deg: float = 78.0) -> float:
    """单目【光轴深度】→ 斜距；不可信时返回 nan。

    ⚠️ Detection.distance_m 是**沿相机光轴的深度 Z**（感知 = fx·real_size/pixel_size，
    docs/13 明确"按光轴深度处理"），不是直线斜距。水平偏移 = Z·tan(b)，
    所以 r = √(Z²+(Z·tan b)²) = Z/cos(b)。直接拿 Z 当斜距会让球越偏离画面中心
    位置越错（偏 45° 少算 29%）。b 接近 ±90° 时斜距发散，判不可信。
    """
    if not (depth_m == depth_m) or depth_m <= 0:
        return float("nan")
    if not (bearing_rad == bearing_rad):
        return float("nan")
    if abs(math.degrees(bearing_rad)) > max_bearing_deg:
        return float("nan")
    c = math.cos(bearing_rad)
    return depth_m / c if c > 1e-3 else float("nan")


def ball_xy(x: float, y: float, yaw: float, bearing_rad: float, distance: float,
            cam_yaw_offset: float = 0.0) -> tuple[float, float]:
    """用「自车位置 + 方位角 + 光轴深度」反算球在场地系下的坐标。

    `distance` 是光轴深度，先换算成斜距（见 depth_to_slant）；不可信时抛 ValueError。
    """
    slant = depth_to_slant(distance, bearing_rad)
    if not (slant == slant):
        raise ValueError(
            f"方位角 {math.degrees(bearing_rad):.0f}° 或深度 {distance} 不可信，无法定位")
    ang = bearing_to_world(yaw, bearing_rad, cam_yaw_offset)
    return x + slant * math.cos(ang), y + slant * math.sin(ang)


def log(tag: str, msg: str, color: str = "") -> None:
    print(f"  {C_DIM}{time.strftime('%H:%M:%S')}{C_OFF} "
          f"{color}{tag:10}{C_OFF} {msg}", flush=True)


class Runner:
    """管 ROS 连接、位姿/检测缓存、以及三个基本动作（到点 / 转向 / 找球）。"""

    def __init__(self, args) -> None:
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from rb_msgs.msg import DetectionArray

        self.args = args
        self.rclpy = rclpy
        self._Twist = Twist
        rclpy.init()
        self.node = rclpy.create_node("perimeter_ball_test")
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.pose: tuple[float, float, float] | None = None
        self.dets: list = []
        self.last_det_time = 0.0

        self.node.create_subscription(PoseWithCovarianceStamped, "/localization/pose",
                                      self._on_pose, 10)
        self.node.create_subscription(DetectionArray, "/perception/detections",
                                      self._on_dets, 10)

        # 延迟导入任务侧的运动学（与任务用同一份，避免公式两套）
        sys.path.insert(0, f"{WS}/build/rb_mission")
        from rb_mission.geometry import goto_command
        self._goto_command = goto_command

    # -- 回调 ---------------------------------------------------------------
    def _on_pose(self, m) -> None:
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        if math.hypot(q.x, q.y, q.z, q.w) > 1e-6:
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        else:
            yaw = p.z
        self.pose = (p.x, p.y, yaw)

    def _on_dets(self, m) -> None:
        self.dets = list(m.detections)
        self.last_det_time = time.time()

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

    # -- 基本动作 -----------------------------------------------------------
    def goto(self, tx: float, ty: float, timeout: float) -> bool:
        """平移到 (tx, ty)。用任务侧同一套 goto_command。"""
        t0 = time.time()
        last_log = 0.0
        while time.time() - t0 < timeout:
            self.spin(0.05)
            if self.pose is None:
                self.stop()
                return False
            x, y, yaw = self.pose
            vx, vy, _wz, dist = self._goto_command(
                x, y, yaw, tx, ty,
                self.args.kp, self.args.kp_yaw, self.args.speed, self.args.max_ang,
                face_travel=False,
                approach_radius=self.args.approach_radius,
                approach_speed=self.args.approach_speed)
            if dist < self.args.tol:
                self.stop()
                log("到点", f"({tx:.2f},{ty:.2f}) 剩余 {dist*100:.1f}cm", C_OK)
                return True
            t = self._Twist()
            t.linear.x, t.linear.y = vx, vy
            self.cmd_pub.publish(t)
            if time.time() - last_log > 1.0:
                last_log = time.time()
                log("移动", f"→({tx:.2f},{ty:.2f}) 剩余 {dist:.2f}m  "
                            f"v={math.hypot(vx, vy):.2f}")
        self.stop()
        log("超时", f"到 ({tx:.2f},{ty:.2f}) 未完成", C_BAD)
        return False

    def rotate_to(self, target_yaw: float, timeout: float) -> bool:
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
        log("超时", "转向未完成", C_BAD)
        return False

    # -- 视觉 ---------------------------------------------------------------
    def best_ball(self):
        """当前最好的目标球检测（有有效距离的优先，再按置信度）。"""
        if time.time() - self.last_det_time > 1.0:
            return None
        cands = [d for d in self.dets if d.label == self.args.ball_label]
        if not cands:
            return None
        valid = [d for d in cands
                 if d.distance_m == d.distance_m and d.distance_m > 0]
        return max(valid or cands, key=lambda d: d.confidence)

    def ball_bearing_world(self, det) -> float:
        """球在场地系里的方位角。

        ⚠️ bearing_rad 是"右正左负"（相机约定），场地系是 y 朝左、逆时针为正，
        两者相反 → 用 yaw - bearing（与 localization.camera_bearing_to_world 一致）。
        """
        return bearing_to_world(self.pose[2], det.bearing_rad, self.args.cam_yaw_offset)

    def ball_field_xy(self, det) -> tuple[float, float] | None:
        if self.pose is None or det.distance_m != det.distance_m or det.distance_m <= 0:
            return None
        return ball_xy(self.pose[0], self.pose[1], self.pose[2],
                       det.bearing_rad, det.distance_m, self.args.cam_yaw_offset)

    def find_ball(self, timeout: float):
        """原地慢转搜索。找到返回检测，超时返回 None。"""
        log("找球", f"原地搜索 {self.args.ball_label} …")
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin(0.05)
            det = self.best_ball()
            if det is not None:
                self.stop()
                log("找到球", f"{det.label} conf={det.confidence:.2f} "
                             f"bearing={math.degrees(det.bearing_rad):+.1f}° "
                             f"dist={det.distance_m:.2f}m", C_OK)
                return det
            t = self._Twist()
            t.angular.z = self.args.search_wz
            self.cmd_pub.publish(t)
        self.stop()
        log("找球失败", f"{timeout:.0f}s 内没看到 {self.args.ball_label}", C_BAD)
        return None

    def close(self) -> None:
        try:
            self.rclpy.shutdown()
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass


def takeover_mission() -> bool:
    """SIGTERM 掉 mission 节点（它现在会优雅退出并发零速）。"""
    r = subprocess.run(["pgrep", "-f", "[m]ission_node"], capture_output=True, text=True)
    pids = [p for p in r.stdout.split() if p.isdigit()]
    if not pids:
        print(f"      {C_DIM}mission 没在跑，不用接管{C_OFF}")
        return True
    for p in pids:
        try:
            os.kill(int(p), signal.SIGTERM)
        except Exception as e:  # noqa: BLE001
            print(f"      {C_BAD}kill {p} 失败: {e}{C_OFF}")
            return False
    time.sleep(2.0)
    left = subprocess.run(["pgrep", "-f", "[m]ission_node"], capture_output=True, text=True)
    ok = not left.stdout.strip()
    print(f"      {C_OK if ok else C_BAD}"
          f"{'已接管 /cmd_vel（mission 已退出）' if ok else 'mission 没退干净'}{C_OFF}")
    return ok


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="整车测试：绕场 → 找篮球 → 走到球前 → 背对球")
    ap.add_argument("--dry", action="store_true", help="只检查前置条件，不动车")
    ap.add_argument("--yes", action="store_true", help="跳过确认")
    ap.add_argument("--takeover", action="store_true",
                    help="自动 SIGTERM mission 节点（否则要求你先手动停它）")
    ap.add_argument("--skip-perimeter", action="store_true", help="跳过绕场，直接找球")
    ap.add_argument("--ball-label", default="ball_basketball", help="目标球标签")
    ap.add_argument("--speed", type=float, default=0.25, help="平移速度上限 m/s")
    ap.add_argument("--kp", type=float, default=1.5)
    ap.add_argument("--kp-yaw", type=float, default=2.0)
    ap.add_argument("--max-ang", type=float, default=0.8, help="角速度上限 rad/s")
    ap.add_argument("--yaw-tol", type=float, default=0.05, help="朝向容差 rad")
    ap.add_argument("--tol", type=float, default=0.10, help="到点容差 m")
    ap.add_argument("--approach-radius", type=float, default=0.45)
    ap.add_argument("--approach-speed", type=float, default=0.12)
    ap.add_argument("--inset", type=float, default=0.70, help="绕场时离边界距离 m")
    ap.add_argument("--search-wz", type=float, default=0.4, help="找球自转角速度 rad/s")
    ap.add_argument("--cam-yaw-offset", type=float, default=0.0,
                    help="相机光轴相对车头的偏置 rad")
    ap.add_argument("--stop-distance", type=float, default=0.60,
                    help="走到离球多远停下 m")
    ap.add_argument("--max-approaches", type=int, default=3, help="接近球最多试几次")
    args = ap.parse_args(argv)

    os.chdir(WS)
    print("🤖 整车测试：绕场一圈 → 回出发点 → 找篮球 → 走到球前 → 转身背对球")

    # ── [1/5] 节点 ──────────────────────────────────────────────────────
    print("\n[1/5] 检查节点")
    need = {"底盘": "[c]hassis_node", "定位": "[l]ocalization_node",
            "感知": "[p]erception_node", "FAST-LIO": "[f]astlio_mapping"}
    miss = []
    for n, pat in need.items():
        ok = subprocess.run(["pgrep", "-f", pat], capture_output=True).returncode == 0
        print(f"      {C_OK+'✓'+C_OFF if ok else C_BAD+'✗'+C_OFF} {n}")
        if not ok:
            miss.append(n)
    if miss:
        print(f"\n  {C_BAD}缺: {', '.join(miss)}{C_OFF} —— 先跑  rbws && bash tools/boot.sh")
        return 1

    # ── [2/5] 接管 /cmd_vel ─────────────────────────────────────────────
    print("\n[2/5] 接管 /cmd_vel（mission 在 IDLE 时也会发零速，会抢话题）")
    mission_alive = subprocess.run(["pgrep", "-f", "[m]ission_node"],
                                   capture_output=True, text=True).stdout.strip() != ""
    if mission_alive:
        if args.takeover:
            if not takeover_mission():
                return 1
        else:
            print(f"      {C_WARN}mission 还在跑{C_OFF} —— 加 --takeover 自动停，"
                  f"或手动：pkill -f '[m]ission_node'")
            if not args.dry:
                return 1
    else:
        print(f"      {C_OK}✓{C_OFF} mission 没在跑，可安全接管")

    run = Runner(args)
    try:
        # ── [3/5] 位姿 + 场地 ───────────────────────────────────────────
        print("\n[3/5] 读位姿与场地")
        if not run.wait_pose(8.0):
            print(f"      {C_BAD}✗ 读不到 /localization/pose{C_OFF}")
            return 1
        sx, sy, syaw = run.pose
        import yaml
        fld = (yaml.safe_load(open("src/rb_mission/config/mission.yaml",
                                   encoding="utf-8")) or {}).get("field", {}) or {}
        L = float(fld.get("length_m", 14.0))
        W = float(fld.get("width_m", 7.5))
        log("位姿", f"出发点 ({sx:+.2f}, {sy:+.2f}, {math.degrees(syaw):+.1f}°)")
        log("场地", f"{L:.1f} × {W:.1f} m，绕场 inset={args.inset:.2f}m")

        if args.dry:
            print(f"\n  {C_OK}--dry：前置条件都过了，没动车{C_OFF}")
            return 0

        if not args.yes:
            print(f"\n  {C_WARN}接下来车会真的动。急停准备好（另开终端 bash tools/panic.sh）{C_OFF}")
            if input("  输入 y 开始，其它取消 > ").strip().lower() != "y":
                print("  已取消。")
                return 0

        # ── [4/5] 绕场一圈 ──────────────────────────────────────────────
        if args.skip_perimeter:
            log("跳过", "不绕场，直接找球")
        else:
            print("\n[4/5] 沿场地外围走一圈（最后回到出发点）")
            k = args.inset
            wps = [(k, k), (L - k, k), (L - k, W - k), (k, W - k), (sx, sy)]
            for i, (tx, ty) in enumerate(wps, 1):
                log("绕场", f"[{i}/{len(wps)}] 目标 ({tx:.2f},{ty:.2f})")
                if not run.goto(tx, ty, timeout=90.0):
                    log("中止", "绕场未完成，停车退出", C_BAD)
                    return 1
            log("绕场完成", "已回到出发点", C_OK)

        # ── [5/5] 找球 → 走到球前 → 背对球 ──────────────────────────────
        print("\n[5/5] 找球 → 走到球前 → 转身背对球")
        det = run.find_ball(timeout=45.0)
        if det is None:
            return 1

        arrived = False
        for attempt in range(1, args.max_approaches + 1):
            det = run.best_ball() or det
            goal = run.ball_field_xy(det)
            if goal is None:
                # 没有距离估计：按方位角盲走近一点，再重看
                log("无测距", "distance_m 无效，按方位角靠近后重看", C_WARN)
                x, y, _ = run.pose
                ang = run.ball_bearing_world(det)
                step = 0.5
                run.goto(x + step * math.cos(ang), y + step * math.sin(ang), timeout=25.0)
                det = run.find_ball(timeout=15.0)
                if det is None:
                    break
                continue

            bx, by = goal
            x, y, _ = run.pose
            d = math.hypot(bx - x, by - y)
            # 停在球前 stop_distance 处：从当前位置朝球走，但少走那一段
            if d > args.stop_distance:
                f = (d - args.stop_distance) / d
                tx, ty = x + (bx - x) * f, y + (by - y) * f
            else:
                tx, ty = x, y
            log("走向球", f"第 {attempt}/{args.max_approaches} 次：球≈({bx:.2f},{by:.2f})，"
                          f"目标点 ({tx:.2f},{ty:.2f})，剩余 {d:.2f}m")
            if not run.goto(tx, ty, timeout=45.0):
                log("中止", "走到球前失败", C_BAD)
                return 1

            det2 = run.find_ball(timeout=12.0)
            if det2 is not None:
                det = det2
                dm = det.distance_m
                if dm == dm and dm <= args.stop_distance + 0.30:
                    log("已到位", f"离球 {dm:.2f}m（目标 {args.stop_distance:.2f}m）", C_OK)
                    arrived = True
                    break
            log("重试", "还没到球前，再修正一次", C_WARN)

        # ── 转身：让背面对着球 ──────────────────────────────────────────
        det = run.best_ball() or det
        run.spin(0.3)
        if run.pose is None:
            log("失败", "位姿丢了", C_BAD)
            return 1
        if det is None:
            log("丢球", "转身前看不到球，无法确定方向 → 停", C_WARN)
            return 1

        ball_ang = run.ball_bearing_world(det)
        target_yaw = yaw_to_put_ball_behind(run.pose[2], det.bearing_rad,
                                            args.cam_yaw_offset)
        log("转身", f"球在场地系方位 {math.degrees(ball_ang):+.1f}° → "
                    f"车头转到 {math.degrees(target_yaw):+.1f}°（背面朝球）")
        ok = run.rotate_to(target_yaw, timeout=40.0)

        print()
        if ok and arrived:
            log("完成", "绕场 ✓  找球 ✓  走到球前 ✓  背面朝球 ✓", C_OK)
        elif ok:
            log("部分完成", "绕场 ✓  找球 ✓  转身 ✓（但没确认走到球前）", C_WARN)
        else:
            log("部分完成", "转身没到位", C_WARN)
        log("注意", "车已停。恢复任务：重启 boot.sh，或直接 set_mission")
        return 0 if ok else 1

    except KeyboardInterrupt:
        print(f"\n  {C_WARN}Ctrl-C —— 发零速停车{C_OFF}")
        run.stop()
        return 130
    except Exception as e:  # noqa: BLE001
        print(f"\n  {C_BAD}异常：{type(e).__name__}: {e}{C_OFF}")
        run.stop()
        return 1
    finally:
        run.close()


if __name__ == "__main__":
    sys.exit(main())
