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


def median_ball(samples):
    """把若干 (bearing, distance) 采样合成一个稳健估计。

    为什么用中位数：单目检测偶尔会把干扰物/局部特征当球，给出离谱的距离或方位；
    中位数对这类离群值免疫，平均会被带偏（现场"找球非常不精准"的一部分原因）。

    返回 (bearing, distance, n) 或 None（样本不足/全非法）。
    """
    ok = [(float(b), float(d)) for b, d in (samples or [])
          if float(b) == float(b) and float(d) == float(d) and float(d) > 0.05]
    if len(ok) < 2:
        return None
    bs = sorted(x[0] for x in ok)
    ds = sorted(x[1] for x in ok)
    mid = len(ok) // 2
    return (bs[mid], ds[mid], len(ok))


def approach_axis(yaw: float, bearing_rad: float) -> float:
    """球在场地系的方位角。

    bearing 是【右正左负】（相机约定），场地系是逆时针为正 ⇒ 方位 = yaw − bearing。
    沿这个角的单位向量 (cos, sin) 就是从车指向球的水平方向。
    """
    return wrap_pi(yaw - bearing_rad)


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

    def measure_ball_stable(self, timeout: float = 6.0, n: int | None = None):
        """连续采几帧取【中位数】。

        为什么要中位数不要平均：单帧偶尔会把干扰物当成球（距离/方位离谱），
        平均会被它带偏；中位数对离群值免疫。
        """
        want = int(n or self.args.stable_frames)
        got = []
        t0 = time.time()
        while len(got) < want and time.time() - t0 < timeout:
            d = self.best_ball()
            if d is not None:
                b = float(d.bearing_rad)
                dm = float(d.distance_m)
                if b == b and dm == dm and dm > 0.05:
                    got.append((b, dm))
            self.spin(0.05)
        return median_ball(got)

    def servo_to_ball(self, target_d: float, tol_b_deg: float, tol_d: float,
                      steps: int, max_move: float, max_turn_deg: float):
        """闭环把车摆到"正对球、且旋转中心距球 target_d"。

        每一步都重新量，不靠一次性解算 —— 单目测距在近距离本来就有几个百分点
        误差，只有闭环才能收敛到"正对着"。
        返回 (ok, 最后一次测量)
        """
        last = None
        for i in range(1, steps + 1):
            m = self.measure_ball_stable()
            if m is None:
                log("丢球", f"第 {i}/{steps} 步：看不到球了，先找", C_WARN)
                if self.find_ball(timeout=12.0) is None:
                    return False, last
                continue
            bearing, dist, nf = m
            last = m
            db = math.degrees(bearing)
            dd = dist - target_d
            log("伺服", f"第 {i}/{steps} 步：方位 {db:+.2f}°（容差 ±{tol_b_deg:.1f}°）、"
                        f"距离 {dist:.3f}m（目标 {target_d:.2f}±{tol_d:.2f}，"
                        f"差 {dd:+.3f}） [{nf} 帧中位数]")
            if abs(db) <= tol_b_deg and abs(dd) <= tol_d:
                log("对准完成", f"方位 {db:+.2f}°，距离 {dist:.3f}m —— 摄像头正对球", C_OK)
                return True, m

            # 先转：把球拉到画面正中（bearing → 0）
            # bearing 右正左负 ⇒ 往右偏就右转（wz 取 bearing 同号）
            turn = max(-math.radians(max_turn_deg),
                       min(math.radians(max_turn_deg), bearing))
            if abs(math.degrees(turn)) > 0.3:
                if self.pose is not None:
                    self.rotate_to(wrap_pi(self.pose[2] + turn), timeout=15.0)

            # 再走：朝球方向把距离补上（正数=还远，前进；负数=太近，后退）
            move = max(-max_move, min(max_move, dd))
            if abs(move) > 0.01 and self.pose is not None:
                yaw = self.pose[2]
                # 球在世界系的方位 = yaw - bearing（右正左负）
                ang = wrap_pi(yaw - bearing)
                x, y, _ = self.pose
                self.goto(x + move * math.cos(ang), y + move * math.sin(ang),
                          timeout=20.0)
        log("未收敛", f"伺服 {steps} 步没进容差，以最后一次为准", C_WARN)
        return False, last

    def backup_along(self, dist: float, timeout: float = 25.0) -> bool:
        """沿【车头方向】平移 dist（正=前进；负=后退）。用里程计闭环。"""
        if self.pose is None:
            return False
        x0, y0, yaw = self.pose
        tx = x0 + dist * math.cos(yaw)
        ty = y0 + dist * math.sin(yaw)
        log("平移", f"{'前进' if dist > 0 else '后退'} {abs(dist):.2f}m "
                    f"→ ({tx:.2f},{ty:.2f})")
        return self.goto(tx, ty, timeout=timeout)

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
    # ── 现场规格：对准 → 停在球前 70cm → 转 180° → 后退 20cm ──
    ap.add_argument("--align-dist", type=float, default=0.70,
                    help="对准到位时【车旋转中心】到球的距离 m（现场要求 70cm）")
    ap.add_argument("--align-tol-deg", type=float, default=1.5,
                    help="摄像头对准球的方位角容差（度）。越小越正对，但更难收敛")
    ap.add_argument("--align-dist-tol", type=float, default=0.05,
                    help="距离容差 m")
    ap.add_argument("--servo-steps", type=int, default=14,
                    help="视觉伺服最多迭代几步")
    ap.add_argument("--servo-move", type=float, default=0.12,
                    help="每步最大平移 m（小步逼近，避免冲过）")
    ap.add_argument("--servo-turn-deg", type=float, default=12.0,
                    help="每步最大转角（度）")
    ap.add_argument("--stable-frames", type=int, default=5,
                    help="取几帧做中位数（抗抖动/漏检）")
    ap.add_argument("--turn-deg", type=float, default=180.0,
                    help="对准后原地转多少度（现场要求 180）")
    ap.add_argument("--backup", type=float, default=0.20,
                    help="转身后退多少 m（现场要求 20cm；朝球方向退）")
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

        # ── [5/5] 找球 → 精密对准（70cm）→ 转 180° → 后退 20cm ─────────────
        print("\n[5/5] 找球 → 摄像头正对球 → 旋转中心停在球前 "
              f"{args.align_dist:.2f}m → 转 {args.turn_deg:.0f}° → "
              f"后退 {args.backup:.2f}m")
        det = run.find_ball(timeout=45.0)
        if det is None:
            return 1

        # ① 闭环视觉伺服：摄像头正对球，且旋转中心距球 align_dist
        ok_align, last = run.servo_to_ball(
            target_d=args.align_dist,
            tol_b_deg=args.align_tol_deg,
            tol_d=args.align_dist_tol,
            steps=args.servo_steps,
            max_move=args.servo_move,
            max_turn_deg=args.servo_turn_deg)
        if not ok_align:
            log("对准失败", "视觉伺服没收敛 —— 不盲目转身（宁可不做也别做错）", C_WARN)
            return 1

        # ② 原地转 turn_deg（默认 180°），用里程计 yaw 精确转，不靠视野反推
        if run.pose is None:
            log("失败", "位姿丢了，无法转身", C_BAD)
            return 1
        yaw0 = run.pose[2]
        target_yaw = wrap_pi(yaw0 + math.radians(args.turn_deg))
        log("转身", f"车头 {math.degrees(yaw0):+.1f}° → {math.degrees(target_yaw):+.1f}°"
                    f"（转 {args.turn_deg:.0f}°）")
        ok_turn = run.rotate_to(target_yaw, timeout=45.0)
        if not ok_turn:
            log("转身未到位", "角度没到，停车退出", C_WARN)
            return 1

        # ③ 朝球方向退 backup（现场要 20cm）
        #    ⚠️ 转完 180° 后车头【背对】球，所以"朝球方向"= 车头反方向
        ok_back = run.backup_along(-abs(args.backup), timeout=30.0) \
            if args.turn_deg >= 90 else run.backup_along(abs(args.backup), timeout=30.0)

        # ④ 收尾确认：再用相机看一眼（转完后相机应该看不到球了）
        run.spin(0.4)
        after = run.measure_ball_stable(timeout=4.0, n=3)
        if after is None:
            log("确认", "转身后相机看不到球 —— 符合预期（背面朝球）", C_OK)
        else:
            log("注意", f"转身后相机还能看到球，方位 {math.degrees(after[0]):+.1f}°"
                        f" 距离 {after[1]:.2f}m —— 可能球在侧面，核对一下", C_WARN)

        final_d = None
        if run.pose is not None:
            # 用最后一次伺服的球位置 + 现在的位移推算旋转中心到球的距离
            final_d = args.align_dist - abs(args.backup)

        print()
        if ok_turn and ok_back:
            log("完成", f"找球 ✓  对准(方位容差±{args.align_tol_deg:.1f}°) ✓  "
                        f"旋转中心距球 {args.align_dist:.2f}m ✓  "
                        f"转 {args.turn_deg:.0f}° ✓  后退 {args.backup:.2f}m ✓", C_OK)
            if final_d:
                log("终点", f"旋转中心距球约 {final_d:.2f}m，背面朝球", C_OK)
        elif ok_turn:
            log("部分完成", "对准 ✓  转身 ✓  但后退没完成", C_WARN)
        else:
            log("部分完成", "对准 ✓  转身没到位", C_WARN)
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
