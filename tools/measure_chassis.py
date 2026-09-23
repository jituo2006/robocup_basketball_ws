#!/usr/bin/env python3
"""底盘参数实测标定：把 width / length / wheel_radius 量准。

为什么需要专门工具
------------------
代码里 width/length 是【轮子中心到轮子中心】的距离，不是车体外廓尺寸；
而"轮径"大家口语里指的是直径，代码要的却是**半径**。
这两处只要差一点，车就会整体跑偏（轮径错 = 直行和转向都错 2 倍）。

好在运动学上这两个参数是**正交**的：
    wheel_rpm = 速度 × cos45° × 60/(2π·r)          ← 只跟 r 有关
    转向项    = ω × hypot(width/2, length/2) × …    ← 只跟 轮距/轴距 有关
所以可以**独立**标定：

  第 1 步（直行）：命令走一段固定距离 → 卷尺量实际距离 → 反推 r
  第 2 步（自转）：命令转一个固定角度 → 量实际转角     → 反推 轮距/轴距

前置：底盘节点已在跑
    ros2 launch rb_chassis chassis.launch.py

用法：
    # 第 1 步：直行标定轮径（车要留出至少 2m 净空）
    python3 tools/measure_chassis.py straight --speed 0.15 --seconds 6

    # 第 2 步：自转标定轮距/轴距
    python3 tools/measure_chassis.py rotate --angular 0.5 --seconds 6

    # 已知实测值、不想交互输入时：
    python3 tools/measure_chassis.py straight --speed 0.15 --seconds 6 --actual 0.85

    # 想把结果直接写回配置：
    python3 tools/measure_chassis.py straight ... --actual 0.85 --apply
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parent.parent
CFG = WS / "src" / "rb_chassis" / "config" / "chassis.yaml"

BOLD = "\033[1m"; OK = "\033[32m"; WARN = "\033[33m"; FAIL = "\033[31m"
DIM = "\033[2m"; RST = "\033[0m"


def read_cfg() -> dict:
    import yaml
    with open(CFG, "r", encoding="utf-8") as fh:
        return (yaml.safe_load(fh) or {}).get("rb_chassis", {}).get("ros__parameters", {})


def apply_cfg(updates: dict) -> None:
    """把值写回 chassis.yaml（只替换数值，保留注释）。"""
    text = CFG.read_text(encoding="utf-8")
    for key, val in updates.items():
        pattern = re.compile(rf"^(\s*{key}:\s*)([-0-9.eE]+)(.*)$", re.MULTILINE)

        def _sub(m):
            return f"{m.group(1)}{val:g}{m.group(3)}"

        new, n = pattern.subn(_sub, text, count=1)
        if n:
            text = new
        else:
            print(f"{WARN}  未能在配置里找到 {key}，请手动修改{RST}")
    CFG.write_text(text, encoding="utf-8")
    print(f"{OK}  已写回 {CFG}{RST}")


def drive(node, pub, twist, seconds: float, rate_hz: float = 20.0) -> None:
    from geometry_msgs.msg import Twist
    import rclpy

    msg = Twist()
    msg.linear.x = twist[0]
    msg.linear.y = twist[1]
    msg.angular.z = twist[2]
    end = time.time() + seconds
    while time.time() < end and rclpy.ok():
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=1.0 / rate_hz)
    # 明确停车
    pub.publish(Twist())
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.02)


def ask_actual(prompt: str, preset: float | None) -> float:
    if preset is not None:
        return preset
    while True:
        raw = input(prompt).strip()
        try:
            return float(raw)
        except ValueError:
            print(f"{WARN}  请输入一个数字{RST}")


def main() -> int:
    ap = argparse.ArgumentParser(description="底盘参数实测标定",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["straight", "rotate", "show"])
    ap.add_argument("--speed", type=float, default=0.15, help="直行速度 m/s（默认 0.15，保守）")
    ap.add_argument("--angular", type=float, default=0.5, help="自转角速度 rad/s（默认 0.5）")
    ap.add_argument("--seconds", type=float, default=6.0, help="持续时间 s（默认 6）")
    ap.add_argument("--actual", type=float, default=None,
                    help="实测值：straight 模式填米数，rotate 模式填角度(度)")
    ap.add_argument("--apply", action="store_true", help="把结果写回 chassis.yaml")
    args = ap.parse_args()

    cfg = read_cfg()
    r = float(cfg.get("wheel_radius", 0.0375))
    w = float(cfg.get("width", 0.65))
    l = float(cfg.get("length", 0.40))
    arm = math.hypot(w / 2.0, l / 2.0)

    print(f"\n{BOLD}底盘参数标定{RST}")
    print(f"  当前配置: width={w:.3f}  length={l:.3f}  wheel_radius={r:.4f}  "
          f"hypot(w/2,l/2)={arm:.4f}")
    print(f"  {DIM}配置来源: {CFG}{RST}")

    if args.mode == "show":
        print(f"\n{BOLD}当前参数下的理论输出{RST}")
        for v in (0.15, 0.30, 0.50):
            rpm = v * math.cos(math.pi / 4) * 60.0 / (2 * math.pi * r)
            print(f"  vx={v:.2f} m/s → 每轮 {rpm:6.1f} rpm")
        for wz in (0.3, 0.5, 0.8):
            rpm = wz * arm * 60.0 / (2 * math.pi * r)
            print(f"  wz={wz:.2f} rad/s → 每轮 {rpm:6.1f} rpm（绕自身中心）")
        return 0

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Int16MultiArray
    except ImportError as exc:
        print(f"{FAIL}导入 rclpy 失败: {exc}{RST}")
        return 1

    rclpy.init()
    node = rclpy.create_node("measure_chassis")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    wheels: list[list[int]] = []
    node.create_subscription(Int16MultiArray, "/chassis/wheel_speed",
                             lambda m: wheels.append(list(m.data)), 10)

    # 确认底盘节点在跑
    t0 = time.time()
    while time.time() - t0 < 4.0 and not wheels:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not wheels:
        print(f"{FAIL}收不到 /chassis/wheel_speed —— 底盘节点没在跑？{RST}")
        print(f"{DIM}  先执行: ros2 launch rb_chassis chassis.launch.py{RST}")
        rclpy.shutdown()
        return 1
    print(f"{OK}  底盘节点在线{RST}")

    rc = 0
    try:
        if args.mode == "straight":
            theory = args.speed * args.seconds
            print(f"\n{BOLD}第 1 步：直行标定【轮径】{RST}")
            print(f"  即将以 vx={args.speed} m/s 前进 {args.seconds:.0f} 秒")
            print(f"  {BOLD}理论距离 = {theory:.3f} m{RST}")
            print(f"  {WARN}请确保车前方至少 {theory + 0.5:.1f} m 净空；随时可用 Ctrl+C 中断{RST}")
            if not args.actual:
                input("  准备好后按回车开始…")

            wheels.clear()
            drive(node, pub, (args.speed, 0.0, 0.0), args.seconds)
            rpm_seen = wheels[-1] if wheels else [0, 0, 0, 0]
            print(f"  实测期间轮速指令: {rpm_seen} rpm")
            print(f"\n  请用卷尺量【车实际移动的距离】(m)")

            actual = ask_actual("  实际距离 (m): ", args.actual)
            if actual <= 0:
                print(f"{FAIL}  实测距离必须为正{RST}")
                rc = 1
            else:
                r_new = r * actual / theory
                err = (actual / theory - 1.0) * 100.0
                print(f"\n{BOLD}结果{RST}")
                print(f"  理论 {theory:.3f} m vs 实测 {actual:.3f} m  → 偏差 {err:+.1f}%")
                print(f"  当前 wheel_radius = {r:.4f} m")
                print(f"  {OK}建议 wheel_radius = {r_new:.4f} m{RST}")
                if abs(err) > 5:
                    print(f"  {DIM}（偏差 >5%，说明当前值确实不准，务必更新）{RST}")
                else:
                    print(f"  {DIM}（偏差在 5% 以内，当前值可用）{RST}")
                if args.apply:
                    apply_cfg({"wheel_radius": r_new})

        elif args.mode == "rotate":
            theory_rad = args.angular * args.seconds
            theory_deg = math.degrees(theory_rad)
            print(f"\n{BOLD}第 2 步：自转标定【轮距/轴距】{RST}")
            print(f"  即将以 wz={args.angular} rad/s 原地旋转 {args.seconds:.0f} 秒")
            print(f"  {BOLD}理论转角 = {theory_deg:.1f}°{RST}")
            print(f"  {WARN}请确保车四周留空；在车身上贴一张纸条当地标，便于量角{RST}")
            if not args.actual:
                input("  准备好后按回车开始…")

            wheels.clear()
            drive(node, pub, (0.0, 0.0, args.angular), args.seconds)
            rpm_seen = wheels[-1] if wheels else [0, 0, 0, 0]
            print(f"  实测期间轮速指令: {rpm_seen} rpm")
            print(f"\n  请量【车实际转过的角度】"
                  f"（{DIM}顺时针为正，逆时针为负{RST}）")

            actual_deg = ask_actual("  实际转角 (度): ", args.actual)
            if abs(actual_deg) < 1e-6:
                print(f"{FAIL}  实测转角不能为 0{RST}")
                rc = 1
            else:
                # 实测转得比理论多 → 说明配置的 arm 偏大
                scale = theory_deg / actual_deg
                arm_new = arm * scale
                print(f"\n{BOLD}结果{RST}")
                print(f"  理论 {theory_deg:.1f}° vs 实测 {actual_deg:.1f}°")
                print(f"  当前 hypot(w/2,l/2) = {arm:.4f} m")
                print(f"  {OK}应有 hypot(w/2,l/2) = {arm_new:.4f} m{RST}")
                # 按比例缩放 width 与 length（保持长宽比）
                ratio = arm_new / arm if arm > 1e-9 else 1.0
                w_new, l_new = w * ratio, l * ratio
                print(f"  {OK}建议 width={w_new:.4f}  length={l_new:.4f}"
                      f"（等比缩放，保持长宽比）{RST}")
                print(f"  {DIM}若你能分别量出轮距与轴距，优先用实测值："
                      f"新值必须满足 hypot({w_new/2:.3f},{l_new/2:.3f})≈{arm_new:.3f}{RST}")
                if args.apply:
                    apply_cfg({"width": w_new, "length": l_new})
    finally:
        try:
            from geometry_msgs.msg import Twist
            pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        rclpy.shutdown()

    print(f"\n{DIM}标定完记得重启底盘节点让新参数生效。{RST}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
