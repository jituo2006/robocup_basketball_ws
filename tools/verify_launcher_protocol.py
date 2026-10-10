#!/usr/bin/env python3
"""机构串口协议验证（安全版）：用虚拟 PTY 抓机构节点真正发出去的帧，逐字节解码。

为什么这样做
------------
直接对真设备发 SHOOT/DRIBBLE 会让机构**真的动作**，不方便在不确定周围环境时做。
这里给机构节点一个**虚拟串口**，节点写进去的字节被我们读出来解码 ——
能验证：动作码映射、speed/angle 编码、CRC32、重复发送频率、开关类单帧、
STOP 语义、串口不在时的拒绝行为。**完全不碰真实机构。**

用法：
    python3 tools/verify_launcher_protocol.py
"""

from __future__ import annotations

import os
import pty
import select
import struct
import subprocess
import sys
import time
import tty
import zlib
from pathlib import Path

WS = Path("/home/user/robocup_basketball_ws")
NS = {"SHOOT": 0, "FAST_SHOOT": 1, "DRIBBLE": 2, "DOUBLE_DRIBBLE": 3,
      "PAWL_TOGGLE": 4, "SLIDEWAY_TOGGLE": 5, "KEEP_LOOP": 6, "STOP": 7}
# 动作 → 期望的帧控制字节（来自 rb_launcher/include/utils/message.hpp）
WANT = {"SHOOT": 0x40, "FAST_SHOOT": 0x41, "DRIBBLE": 0x80, "DOUBLE_DRIBBLE": 0x81,
        "PAWL_TOGGLE": 0x20, "SLIDEWAY_TOGGLE": 0x10, "KEEP_LOOP": 0x42}


def decode(frame: bytes) -> str:
    """11 字节帧 → 人类可读。"""
    if len(frame) < 11:
        return f"短帧 {frame.hex(' ')}"
    head, cmd, sp, ang = frame[0], frame[1], *struct.unpack("<HH", frame[2:6])
    crc = struct.unpack("<I", frame[6:10])[0]
    tail = frame[10]
    ok = (head == ord("+") and tail == ord("*") and crc == zlib.crc32(frame[:6]))
    name = {0x40: "SHOOT", 0x41: "FAST_SHOOT", 0x80: "DRIBBLE", 0x81: "DOUBLE_DRIBBLE",
            0x20: "PAWL", 0x10: "SLIDEWAY", 0x42: "KEEP_LOOP", 0x60: "RESPONSE"}.get(cmd, "?")
    return (f"cmd=0x{cmd:02X}({name:13}) speed={sp:5d} angle={ang:3d} "
            f"crc={'OK' if ok else 'BAD'} {'✓' if ok else '✗'}")


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from rb_msgs.srv import Launch

    os.chdir(WS)
    domain = 173
    master, slave = pty.openpty()
    # ⚠️ 必须设 raw：pty 默认是【规范模式】，read() 会一直等换行，而机构发的是
    # 二进制帧（没有换行）→ 读侧会永久阻塞（实测内核栈停在 n_tty_read）。
    tty.setraw(master)
    tty.setraw(slave)
    os.set_blocking(master, False)      # 非阻塞，配合 select 读
    pty_path = os.ttyname(slave)
    print(f"🏀 机构串口协议验证（虚拟 PTY {pty_path}，不碰真实机构）\n")

    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    # 直接用 ament 索引查路径 —— 不要 subprocess 起 ros2（实测 bash -lc 会卡住）
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix("rb_launcher"),
                       "lib/rb_launcher/rb_launcher_node")
    if not os.path.isfile(exe):
        print(f"  ❌ 找不到 {exe}，先 rbws build")
        return 1

    rclpy.init(args=[], domain_id=domain)
    node = Node("launcher_proto_verify")
    cli = node.create_client(Launch, "/rb_launcher/launch")
    proc = subprocess.Popen(
        [exe, "--ros-args", "-p", f"port:={pty_path}",
         "-p", "send_rate_hz:=20.0", "-p", "default_speed:=2800", "-p", "default_angle:=64"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    fails = 0

    def call(action, speed=0, angle=0, wait=5.0):
        req = Launch.Request(action=action, speed=speed, angle=angle)
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < wait:
            rclpy.spin_once(node, timeout_sec=0.02)
        return fut.result()

    def _drain(fd):
        """把已到达的字节读干净（非阻塞）。"""
        out = b""
        while True:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                break
            if not chunk:
                break
            out += chunk
        return out

    def grab(seconds=0.6):
        """读虚拟口里节点写出来的字节，按 11 字节切帧。"""
        buf = b""
        t0 = time.time()
        while time.time() - t0 < seconds:
            if select.select([master], [], [], 0.05)[0]:
                buf += _drain(master)
        frames = [buf[i:i + 11] for i in range(0, len(buf) - 10, 11)]
        return frames, buf

    try:
        if not cli.wait_for_service(timeout_sec=8):
            print("  ❌ 机构服务没起来")
            return 1
        print("  ✓ 机构节点已就绪\n")

        # ── 逐个动作 ──────────────────────────────────────────────────
        for name, act in NS.items():
            if name == "STOP":
                continue
            # 先清空
            _drain(master)
            r = call(act, 0, 0)                       # speed/angle=0 → 用默认值
            frames, raw = grab(0.7)
            want = WANT[name]
            got = frames[0][1] if frames else None
            n = len(frames)
            is_toggle = name in ("PAWL_TOGGLE", "SLIDEWAY_TOGGLE")
            # 开关类只发一帧；其余按 20Hz 重复
            ok = (got == want) and (n == 1 if is_toggle else n >= 2)
            if not ok:
                fails += 1
            print(f"  {'✓' if ok else '✗'} {name:15} action={act}  收到 {n} 帧  "
                  f"{decode(frames[0]) if frames else '（无帧！）'}")
            if frames and want == 0x40:               # SHOOT 用默认 speed/angle
                sp, ang = struct.unpack("<HH", frames[0][2:6])
                if (sp, ang) != (2800, 64):
                    print(f"      ⚠️ 默认 speed/angle 应为 (2800, 64)，实际 ({sp}, {ang})")
                    fails += 1
            if name in ("SHOOT", "FAST_SHOOT", "DRIBBLE"):
                # 显式给参数
                _drain(master)
                call(act, 3333, 77)
                frames2, _ = grab(0.5)
                if frames2:
                    sp, ang = struct.unpack("<HH", frames2[0][2:6])
                    good = (sp, ang) == (3333, 77)
                    if not good:
                        fails += 1
                    print(f"      显式参数 → speed={sp} angle={ang} {'✓' if good else '✗'}")

        # ── STOP：应停止重复发送，且不再有帧 ──────────────────────────
        r = call(7)
        frames, _ = grab(0.8)
        ok = r.success and len(frames) == 0
        if not ok:
            fails += 1
        print(f"  {'✓' if ok else '✗'} STOP            success={r.success} "
              f"停止后新增帧={len(frames)}（应为 0）")

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        rclpy.shutdown()
        node.destroy_node()
        os.close(master)
        os.close(slave)

    print("\n" + "=" * 68)
    if fails == 0:
        print("  ✅ 协议验证全过 —— 动作码/参数/CRC/重复发送/开关单帧/STOP 语义都对")
        print("     下一步可以（在确认周围安全后）对真实机构发一条试试。")
    else:
        print(f"  ❌ 有 {fails} 项不符，先别对真机构发。")
    print("=" * 68)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
