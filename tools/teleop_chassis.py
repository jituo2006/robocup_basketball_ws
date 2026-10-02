#!/usr/bin/env python3
"""键盘遥控底盘 —— 松手即停，双保险。

用法：
    python3 tools/teleop_chassis.py            # 默认低速
    python3 tools/teleop_chassis.py --fast     # 启动就是高速

按键（单字符，避免方向键转义序列在 SSH 下失灵）：
    w 前进    s 后退
    a 左移    d 右移          （全向轮，可横着走）
    q 左转    e 右转          （逆时针 / 顺时针）
    f 高低速切换              （低 0.3 m/s，高 0.8 m/s；限幅 1.0 以内）
    space 全停
    Esc / Ctrl+C 退出

安全设计：
    1. 速度故意取小值，默认低速，撞了也基本无损伤。
    2. 松开所有方向键 → 立刻发 0 速度（本脚本），
       再加底盘侧的 cmd_vel_timeout_ms=200 兜底：即使脚本崩了，200ms 后自动停。
    3. 需要底盘节点在正常模式跑着：
           ros2 launch rb_chassis chassis.launch.py
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node

# 键 -> (名字, vx系数, vy系数, wz系数)
KEYMAP = {
    "w": ("前进", 1.0, 0.0, 0.0),
    "s": ("后退", -1.0, 0.0, 0.0),
    "a": ("左移", 0.0, 1.0, 0.0),
    "d": ("右移", 0.0, -1.0, 0.0),
    "q": ("左转", 0.0, 0.0, 1.0),
    "e": ("右转", 0.0, 0.0, -1.0),
}

LOW = (0.30, 0.60)   # 线速度 0.30 m/s，角速度 0.60 rad/s
HIGH = (0.80, 1.00)  # 线速度 0.80 m/s，角速度 1.00 rad/s


class _KeyReader:
    """非阻塞读单个字符（原始模式），退出时恢复终端。"""

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)

    def __enter__(self):
        tty.setraw(self._fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def read(self) -> str | None:
        if select.select([sys.stdin], [], [], 0.02)[0]:
            return sys.stdin.read(1)
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description="键盘遥控底盘")
    ap.add_argument("--fast", action="store_true", help="启动即高速")
    args = ap.parse_args()

    # 键盘读原始模式需要真实 TTY。SSH 不带 -t、IDE 内嵌终端、管道等都会
    # 拿不到 TTY，termios 会抛 "Inappropriate ioctl for device" —— 提前给清晰提示。
    if not sys.stdin.isatty():
        print("❌ 键盘遥控需要真实终端（TTY）。", file=sys.stderr)
        print("   - SSH 远程请加 -t：  ssh -t user@host", file=sys.stderr)
        print("   - 或在本机图形终端的独立窗口里运行。", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("teleop_chassis")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    linear, angular = HIGH if args.fast else LOW
    running = True
    last_desc = "停"

    print("=" * 60)
    print("  键盘遥控底盘")
    print("  w前进 s后退 | a左移 d右移 | q左转 e右转 | f变速 space停 | Esc退出")
    print(f"  当前速度: {linear} m/s, {angular} rad/s  {'（高速）' if args.fast else '（低速）'}")
    print("  ⚠️ 松手即停；Ctrl+C 随时退出")
    print("=" * 60)

    try:
        with _KeyReader() as kb:
            while running and rclpy.ok():
                key = kb.read()
                twist = Twist()
                if key is None:
                    pass  # 没按键 → 保持全停（松手即停）
                elif key == "\x1b":  # Esc 退出
                    running = False
                elif key == " ":
                    last_desc = "停"
                elif key == "f":
                    if (linear, angular) == LOW:
                        linear, angular = HIGH
                    else:
                        linear, angular = LOW
                    print(f"\r  速度切到 {linear} m/s, {angular} rad/s          ", end="", flush=True)
                    last_desc = "停"
                elif key in KEYMAP:
                    name, kx, ky, kz = KEYMAP[key]
                    twist = Twist(
                        linear=Vector3(x=kx * linear, y=ky * linear),
                        angular=Vector3(z=kz * angular),
                    )
                    last_desc = name

                pub.publish(twist)
                print(f"\r  [{last_desc}] vx={twist.linear.x:+.2f} vy={twist.linear.y:+.2f} "
                      f"wz={twist.angular.z:+.2f}      ", end="", flush=True)
                rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        # 无论如何，退出前发一条零速
        pub.publish(Twist())
        print("\n  已停，退出。")
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
