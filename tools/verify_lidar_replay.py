#!/usr/bin/env python3
"""用历史 rosbag 回放验证【雷达里程计 → 场地坐标系】这条链路。

为什么能这么测
--------------
~/lidar_360 里有真实录制的 MID360 数据（`yaw360_test_01` 含 /livox/lidar + /livox/imu）。
回放它就能在没有雷达硬件的情况下，把整条链路跑起来：

    rosbag ──/livox/lidar+/livox/imu──▶ FAST-LIO ──/Odometry──▶ rb_localization ──/localization/pose

它验证的是**坐标变换与标定参数**（最容易搞错、也最难在实车上排查的部分），
不验证雷达本身的精度。

用法：
    python3 tools/verify_lidar_replay.py
    python3 tools/verify_lidar_replay.py --start-x 2.0 --start-y 3.0 --start-yaw-deg 30
退出码：0 通过 / 1 失败
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parent.parent
LIDAR_WS = Path("/home/user/lidar_360")
BAG = LIDAR_WS / "yaw360_test_01"
TEST_DOMAIN = "78"

OK = "\033[32m"; FAIL = "\033[31m"; WARN = "\033[33m"
DIM = "\033[2m"; BOLD = "\033[1m"; RST = "\033[0m"


class Procs:
    def __init__(self) -> None:
        self.items: list[tuple[subprocess.Popen, str, str]] = []
        self.logdir = WS / "test_artifacts" / "lidar_replay"
        shutil.rmtree(self.logdir, ignore_errors=True)
        self.logdir.mkdir(parents=True, exist_ok=True)

    def spawn(self, name: str, script: str) -> None:
        log = open(self.logdir / f"{name}.log", "w")  # noqa: SIM115
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = TEST_DOMAIN
        p = subprocess.Popen(["bash", "-c", script], stdout=log, stderr=subprocess.STDOUT,
                             env=env, start_new_session=True)
        self.items.append((p, name, str(self.logdir / f"{name}.log")))

    def log(self, name: str) -> str:
        for _, n, path in self.items:
            if n == name:
                try:
                    return Path(path).read_text(encoding="utf-8", errors="replace")
                except OSError:
                    return ""
        return ""

    def stop_all(self) -> None:
        for p, _, _ in self.items:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                except Exception:  # noqa: BLE001
                    pass
        for p, _, _ in self.items:
            try:
                p.wait(timeout=6)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:  # noqa: BLE001
                    pass


def main() -> int:
    ap = argparse.ArgumentParser(description="雷达回放验证")
    ap.add_argument("--bag", default=str(BAG))
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--start-x", type=float, default=1.0)
    ap.add_argument("--start-y", type=float, default=1.0)
    ap.add_argument("--start-yaw-deg", type=float, default=0.0)
    ap.add_argument("--no-zero-on-first", action="store_true",
                    help="关闭 zero_on_first（用于测试理论原点模式）")
    args = ap.parse_args()

    os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN   # 验证器自身也要在同一 domain

    bag = Path(args.bag)
    if not bag.is_dir():
        print(f"{FAIL}找不到 rosbag: {bag}{RST}")
        return 1
    for rel in ["ws_livox/install", "fast_prop_ws/install"]:
        if not (LIDAR_WS / rel).is_dir():
            print(f"{FAIL}缺少雷达工作空间: {LIDAR_WS / rel}{RST}")
            return 1

    print(f"\n{BOLD}雷达回放验证：rosbag → FAST-LIO → 场地坐标系{RST}")
    print(f"{DIM}rosbag: {bag}{RST}")
    print(f"{DIM}摆车位: ({args.start_x:.2f}, {args.start_y:.2f}, {args.start_yaw_deg:.1f}°){RST}")
    print("=" * 72)

    procs = Procs()
    try:
        # ① 回放雷达原始数据（必须 source ws_livox，否则 CustomMsg 反序列化会失败）
        procs.spawn("bag",
            "source /opt/ros/humble/setup.bash && "
            f"source {LIDAR_WS}/ws_livox/install/setup.bash && "
            f"export ROS_DOMAIN_ID={TEST_DOMAIN} && "
            f"exec ros2 bag play {bag} --topics /livox/lidar /livox/imu")
        # ② FAST-LIO
        procs.spawn("fastlio",
            "source /opt/ros/humble/setup.bash && "
            f"source {LIDAR_WS}/ws_livox/install/setup.bash && "
            f"source {LIDAR_WS}/fast_prop_ws/install/setup.bash && "
            f"export ROS_DOMAIN_ID={TEST_DOMAIN} && "
            "exec ros2 launch fast_lio mapping.launch.py "
            f"config_path:={LIDAR_WS}/fast_prop_ws/src/FAST_LIO_WITH_PROPAGATE/config "
            "config_file:=mid360.yaml rviz:=false")

        print(f"{DIM}等待 FAST-LIO 初始化（约 14s）…{RST}")
        time.sleep(14)

        # ③ 本工程定位节点，用命令行覆盖为"雷达方案"
        procs.spawn("localization",
            "source /opt/ros/humble/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"export ROS_DOMAIN_ID={TEST_DOMAIN} && "
            "exec ros2 run rb_localization localization_node --ros-args "
            "-p config_file:=" + str(WS / "src/rb_localization/config/localization.yaml"))
        time.sleep(6)

        # ④ 采集
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry

        rclpy.init()
        node = rclpy.create_node("verify_lidar_replay")
        odom: list[tuple] = []
        loc: list[tuple] = []

        def _yaw_from_q(q) -> float:
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        node.create_subscription(
            Odometry, "/Odometry",
            lambda m: odom.append((m.pose.pose.position.x, m.pose.pose.position.y,
                                   _yaw_from_q(m.pose.pose.orientation))), 10)
        node.create_subscription(
            PoseWithCovarianceStamped, "/localization/pose",
            lambda m: loc.append((m.pose.pose.position.x, m.pose.pose.position.y,
                                  m.pose.pose.position.z)), 20)

        end = time.time() + args.duration
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.shutdown()

        odom_log = procs.log("fastlio")
        loc_log = procs.log("localization")

        if not odom:
            print(f"\n{FAIL}✗ 收不到 /Odometry —— FAST-LIO 没起来{RST}")
            for line in odom_log.strip().splitlines()[-6:]:
                print(f"{DIM}  {line}{RST}")
            return 1
        if not loc:
            print(f"\n{FAIL}✗ 收不到 /localization/pose{RST}")
            for line in loc_log.strip().splitlines()[-8:]:
                print(f"{DIM}  {line}{RST}")
            return 1

        # ---- 断言 ----
        ox0, oy0, oyaw0 = odom[0]
        lx0, ly0, lyaw0 = loc[0]
        failures: list[str] = []

        # 1) 首帧定位输出应≈摆车位置（zero_on_first 生效）
        err0 = math.hypot(lx0 - args.start_x, ly0 - args.start_y)
        if err0 > 0.20:
            failures.append(f"首帧定位输出不≈摆车位: 得到({lx0:.3f},{ly0:.3f}) "
                            f"期望({args.start_x:.2f},{args.start_y:.2f})，差 {err0:.3f}m")

        # 2) 定位输出必须随雷达里程计一起动，且变化量级一致（旋转只改朝向不改距离）
        ox_min = min(o[0] for o in odom); ox_max = max(o[0] for o in odom)
        oy_min = min(o[1] for o in odom); oy_max = max(o[1] for o in odom)
        lx_min = min(l[0] for l in loc); lx_max = max(l[0] for l in loc)
        ly_min = min(l[1] for l in loc); ly_max = max(l[1] for l in loc)
        span_o = math.hypot(ox_max - ox_min, oy_max - oy_min)
        span_l = math.hypot(lx_max - lx_min, ly_max - ly_min)
        if span_o < 1e-6:
            failures.append("雷达里程计没有位移（bag 太短？）")
        elif abs(span_l - span_o) > max(0.25, 0.35 * span_o):
            failures.append(f"定位位移幅度与雷达不一致: 雷达 {span_o:.3f}m vs 定位 {span_l:.3f}m")

        # 3) 【关键】yaw 变换是否正确
        #    本 bag 是原地旋转的，位移很小但 yaw 变化大 —— 正好检验旋转处理。
        #    做法：比较"里程计 yaw 变化量"与"定位输出 yaw 变化量"，两者应一致
        #    （刚体变换只加一个常量偏移，不改变变化量）。
        def _wrap(a: float) -> float:
            while a > math.pi: a -= 2 * math.pi
            while a < -math.pi: a += 2 * math.pi
            return a

        d_odom = _wrap(odom[-1][2] - odom[0][2])
        d_loc = _wrap(loc[-1][2] - loc[0][2])
        yaw_span = max(abs(_wrap(o[2] - odom[0][2])) for o in odom)
        if yaw_span < 0.05:
            failures.append(f"雷达 yaw 几乎没变化({yaw_span:.3f} rad)，"
                            f"这个 bag 不适合验证旋转，换一个")
        elif abs(_wrap(d_odom - d_loc)) > 0.15:
            failures.append(f"yaw 变换不一致: 雷达转 {math.degrees(d_odom):+.1f}° "
                            f"但定位输出转 {math.degrees(d_loc):+.1f}°（差 "
                            f"{math.degrees(_wrap(d_odom - d_loc)):+.1f}°）")

        # 4) 数值有限且有界（场地 14×7.5，允许留 3m 余量）
        for name, seq in (("odom", [(o[0], o[1]) for o in odom]),
                          ("loc", [(l[0], l[1]) for l in loc])):
            for x, y in seq:
                if not (math.isfinite(x) and math.isfinite(y)):
                    failures.append(f"{name} 出现 NaN/Inf"); break
                if max(abs(x), abs(y)) > 25.0:
                    failures.append(f"{name} 发散: ({x:.1f},{y:.1f})"); break

        print(f"\n{BOLD}结果{RST}")
        print(f"  /Odometry           {len(odom):4d} 条   位移 {span_o:.3f} m")
        print(f"  /localization/pose  {len(loc):4d} 条   位移 {span_l:.3f} m")
        print(f"  首帧定位输出        ({lx0:.3f}, {ly0:.3f}, {math.degrees(lyaw0):+.1f}°)"
              f"   与摆车位差 {err0:.3f} m")
        print(f"  yaw 变化          雷达 {math.degrees(d_odom):+7.1f}°   "
              f"定位 {math.degrees(d_loc):+7.1f}°   （应一致）")
        conv_line = [l for l in loc_log.splitlines() if "odom→field 变换已确定" in l]
        if conv_line:
            print(f"{DIM}  {conv_line[0].split('] ')[-1]}{RST}")

        print()
        if failures:
            print(f"{FAIL}✗ 未通过:{RST}")
            for f in failures:
                print(f"  {FAIL}·{RST} {f}")
            print(f"\n{DIM}日志在 {procs.logdir}{RST}")
            return 1
        print(f"{OK}✓ 通过：雷达里程计已正确变换到场地坐标系，首帧对齐摆车位，{RST}")
        print(f"{OK}  位移与 yaw 变化均与雷达一致，无 NaN / 无发散。{RST}")
        print(f"{DIM}  说明：本测试用的是回放的真实雷达数据，验证的是坐标变换与标定参数。{RST}")
        return 0
    finally:
        procs.stop_all()


if __name__ == "__main__":
    sys.exit(main())
