#!/usr/bin/env python3
"""底盘单独测试工具 —— 先验证软件，再动真轮子。

为什么需要它：底盘是唯一"做错就会撞坏东西"的节点。2025 届就出过
"发了指令当时不动、随后突然窜车"的事故（见 omni_chassis.cpp 里的注释）。
所以这里把测试分成两级，**默认只跑安全的那一级**：

    python3 tools/test_chassis.py              # ① 干跑：只解算，不下发 CAN（默认，绝对安全）
    python3 tools/test_chassis.py --drive      # ② 实动：真的驱动电机（需要二次确认）

① 干跑验证什么（不需要底盘上电）
    前进/后退/左移/右移/左转/右转 六种指令下，四个电机的目标转速是否合理，
    以及"收不到指令时是否自动归零"这条安全网是否生效。

② 实动验证什么（必须先把车架起来、轮子悬空！）
    轮子转的方向对不对。这是软件验证不了的 —— 只取决于轮子实际怎么装的。

⚠️ 实动前务必确认：
    1. 四个轮子悬空（垫砖/上支架），否则车会窜出去
    2. 人能随时断电（手放电源开关上）
    3. 底盘已上电，且 `candump can0` 能看到数据
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

# 与 tools/ 下其它脚本一致：按目录导入，别指望它是包
sys.path.insert(0, str(Path(__file__).resolve().parent))

WHEEL_TOPIC = "/chassis/wheel_speed"
CMD_TOPIC = "/cmd_vel"

# 测试用的速度。**故意取小值**：0.15 m/s 和 0.3 rad/s，
# 远低于 limit_vel=1.0，即使方向判错也不会造成严重后果。
TEST_VX = 0.15
TEST_VY = 0.15
TEST_WZ = 0.30

CASES = [
    ("前进", Twist(linear=Vector3(x=TEST_VX))),
    ("后退", Twist(linear=Vector3(x=-TEST_VX))),
    ("左移", Twist(linear=Vector3(y=TEST_VY))),
    ("右移", Twist(linear=Vector3(y=-TEST_VY))),
    ("原地左转", Twist(angular=Vector3(z=TEST_WZ))),
    ("原地右转", Twist(angular=Vector3(z=-TEST_WZ))),
]

class Probe(Node):
    def __init__(self) -> None:
        super().__init__("chassis_probe")
        self.wheels: list[int] | None = None
        self._pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Int16MultiArray, WHEEL_TOPIC, self._on_wheel, 10)

    def _on_wheel(self, msg: Int16MultiArray) -> None:
        self.wheels = list(msg.data)

    def hold(self, twist: Twist, seconds: float) -> None:
        """持续发布指令（底盘 200ms 收不到就归零，所以必须持续发）。"""
        t0 = time.time()
        while time.time() - t0 < seconds and rclpy.ok():
            self._pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)

    def read(self, timeout: float = 1.0) -> list[int] | None:
        self.wheels = None
        t0 = time.time()
        while time.time() - t0 < timeout and self.wheels is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.wheels


def _fmt(w: list[int] | None) -> str:
    return "无数据" if w is None else "[" + ", ".join(f"{v:4d}" for v in w) + "]"


def dry_run_test(duration: float) -> int:
    """① 只解算测试。返回失败项数。"""
    node = Probe()
    print("\n  ① 干跑测试（不下发 CAN，不需要底盘上电）")
    print("  " + "-" * 62)

    # 等订阅建立，否则第一次读会是 None
    node.read(timeout=2.0)

    idle = node.read(timeout=2.0)
    print(f"     {'不发指令':<16} {_fmt(idle)}")
    fails = 0
    if idle and any(idle):
        print("       ❌ 没有指令时轮速应该全是 0（安全网失效！）")
        fails += 1

    results: dict[str, list[int] | None] = {}
    for name, twist in CASES:
        node.hold(twist, duration)
        w = node.read()
        results[name] = w
        print(f"     {name:<16} {_fmt(w)}")
        if w is None:
            print("       ❌ 收不到 /chassis/wheel_speed")
            fails += 1
        elif not any(w):
            print("       ❌ 指令下发了但轮速全是 0")
            fails += 1

    # 方向自洽性检查（纯软件，能抓出运动学写错）
    def sign(w):
        return [1 if v > 0 else (-1 if v < 0 else 0) for v in w]

    checks = [
        ("前进/后退 应符号相反", "前进", "后退"),
        ("左移/右移 应符号相反", "左移", "右移"),
        ("左转/右转 应符号相反", "原地左转", "原地右转"),
    ]
    print("  " + "-" * 62)
    for desc, a, b in checks:
        wa, wb = results.get(a), results.get(b)
        if wa and wb:
            ok = sign(wa) == [-s for s in sign(wb)]
            print(f"     {'✅' if ok else '❌'} {desc}")
            if not ok:
                fails += 1

    # 超时归零
    time.sleep(0.4)
    after = node.read(timeout=2.0)
    print(f"     {'停发 0.4s 后':<16} {_fmt(after)}")
    if after and any(after):
        print("       ❌ 停发后没有归零（cmd_vel_timeout_ms 失效！）")
        fails += 1

    node.destroy_node()
    return fails


def drive_test(duration: float) -> int:
    """② 实动测试。每个方向单独跑，方向由你肉眼判断。"""
    print("\n  ⚠️  实动测试：先确认四个轮子**已经悬空**！")
    print("      测试中车速会很低（0.15 m/s / 0.3 rad/s），但方向判错会窜车。")
    if input("      轮子悬空了吗？输入 yes 继续: ").strip().lower() not in ("yes", "y"):
        print("      已取消。")
        return 0

    node = Probe()
    node.read(timeout=2.0)
    print("\n  ① 实动（看着轮子，判断方向对不对）")
    print("  " + "-" * 62)
    for name, twist in CASES:
        print(f"\n     >>> {name}（{duration:.1f} 秒）")
        node.hold(twist, duration)
    # 收尾务必归零
    node.hold(Twist(), 0.5)
    print("\n  已停。")
    node.destroy_node()
    return 0


def can_state() -> str:
    """看一眼 CAN 总线，实动前先确认它不是死的。"""
    try:
        out = subprocess.run(["ip", "-details", "link", "show", "can0"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return "查不到"
    if not out.strip():
        return "can0 不存在"
    state = "?"
    for tok in out.split():
        if tok.startswith("ERROR") or tok == "BUS-OFF":
            state = tok
    bitrate = next((out.split("bitrate")[1].split()[0] for _ in [0] if "bitrate" in out), "?")
    return f"{state} @ {bitrate} bps"


def main() -> int:
    ap = argparse.ArgumentParser(description="底盘单独测试（默认只干跑，安全）")
    ap.add_argument("--drive", action="store_true",
                    help="真的驱动电机（默认只干跑解算，绝不动车）")
    ap.add_argument("--duration", type=float, default=1.5, help="每个方向持续秒数，默认 1.5")
    args = ap.parse_args()

    print("=" * 66)
    print("  底盘单独测试")
    print("=" * 66)
    print(f"  CAN 状态: {can_state()}")
    if not args.drive:
        print("  模式    : 干跑（只解算，不下发 CAN）")
        print("  ⚠️  需要底盘节点已在**干跑**模式运行：")
        print("        ros2 launch rb_chassis chassis.launch.py dry_run:=true")
    else:
        print("  模式    : 实动（会真的转轮子）")
        print("  ⚠️  需要底盘节点在正常模式运行：")
        print("        ros2 launch rb_chassis chassis.launch.py")

    rclpy.init()
    try:
        fails = drive_test(args.duration) if args.drive else dry_run_test(args.duration)
    finally:
        rclpy.shutdown()

    print("=" * 66)
    if not args.drive:
        if fails == 0:
            print("  ✅ 干跑全部通过 —— 运动学解算与安全网都没问题。")
            print("     接下来可以实动验证轮子方向：")
            print("       ros2 launch rb_chassis chassis.launch.py")
            print("       python3 tools/test_chassis.py --drive")
        else:
            print(f"  ❌ 有 {fails} 项没过，先别实动。")
    print("=" * 66)
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
