#!/usr/bin/env python3
"""分辨「下位机回话了」还是「本地回显 / 环回」—— 这两种现象看起来一样，但含义完全不同。

结论怎么下：
  * 二进制字节也**逐字节原样**回来      → 本地 tty 回显或硬件环回（下位机可能根本不在）
  * 只回可打印文本 / 回的是整理过的内容 → 真·下位机在解析
  * 发出去后**几乎零延迟**回来          → 环回
  * 有明显延迟（毫秒级）或分帧          → 更像真设备

用法：python3 tools/serial_probe_echo.py
"""

from __future__ import annotations

import os
import select
import sys
import termios
import time

PORT = "/dev/R1_usb2ttl"


def read_for(ser, sec):
    out = b""
    t0 = time.time()
    while time.time() - t0 < sec:
        n = ser.in_waiting
        if n:
            out += ser.read(n)
        else:
            time.sleep(0.01)
    return out


def main() -> int:
    import serial

    print(f"🔍 串口回显性质探测  {PORT}\n")
    try:
        ser = serial.Serial(PORT, 115200, timeout=0.05, write_timeout=1.0)
    except Exception as e:  # noqa: BLE001
        print(f"  ❌ 打不开: {e}")
        return 1

    try:
        # ── ① 什么都不发，看有没有自发数据 ────────────────────────────
        ser.reset_input_buffer()
        idle = read_for(ser, 2.0)
        print(f"  ① 静默 2s 收到 {len(idle)} 字节"
              f"{'：' + repr(idle[:40]) if idle else '（下位机不主动发）'}")

        # ── ② 二进制字节：真设备通常不会原样回二进制 ──────────────────
        ser.reset_input_buffer()
        probe = bytes([0x01, 0x02, 0x7F, 0xFF, 0x00, 0xAA])
        t0 = time.time()
        ser.write(probe)
        ser.flush()
        back = read_for(ser, 1.5)
        dt = time.time() - t0
        exact = back.startswith(probe)
        print(f"  ② 发二进制 {probe.hex(' ')}")
        print(f"     收回 {len(back)} 字节 {back.hex(' ') if back else '（无）'}")
        print(f"     {'⚠️ 逐字节原样回来 → 强烈提示【本地回显/环回】' if exact else '✓ 没有原样回来 → 更像真设备在解析'}")

        # ── ③ 可打印文本 ──────────────────────────────────────────────
        ser.reset_input_buffer()
        t0 = time.time()
        ser.write(b"hallo\n")
        ser.flush()
        back2 = read_for(ser, 1.5)
        dt2 = (time.time() - t0) * 1000
        print(f"  ③ 发 b'hallo\\n' → 收回 {back2!r}   （{dt2:.0f} ms 窗口内）")

        # ── ④ 分开发送，看是不是"整行才回"（行缓冲回显的特征）────────
        ser.reset_input_buffer()
        got = b""
        for ch in b"abc":
            ser.write(bytes([ch]))
            ser.flush()
            time.sleep(0.05)
            got += read_for(ser, 0.1)
        print(f"  ④ 逐字符发 'a','b','c' → 逐字符期间收到 {got!r}")
        tail = read_for(ser, 0.3)
        print(f"     之后又收到 {tail!r}")
        if not got and not tail:
            print("     （逐字符不发，可能等整行/整帧）")
        elif got == b"abc":
            print("     ⚠️ 逐字符立刻原样回来 → 【本地回显/环回】特征")
        else:
            print("     ✓ 不是逐字符即时回显")

        # ── ⑤ 本地 tty 的 ECHO 标志 ───────────────────────────────────
        try:
            attrs = termios.tcgetattr(ser.fileno())
            lflag = attrs[3]
            print(f"  ⑤ tty 本地标志: ECHO={'开' if lflag & termios.ECHO else '关'} "
                  f"ECHOE={'开' if lflag & getattr(termios, 'ECHOE', 0) else '关'} "
                  f"ICANON={'开' if lflag & termios.ICANON else '关'}")
            if lflag & termios.ECHO:
                print("     ⚠️ ECHO 开着 —— 这就是回显的来源（读到的可能是自己发的）")
            else:
                print("     ✓ ECHO 关着 pyserial 已设 raw；回显更可能来自设备/环回")
        except Exception as e:  # noqa: BLE001
            print(f"  ⑤ 读 termios 失败: {e}")

    finally:
        ser.close()

    print("\n" + "=" * 66)
    print("  判读：")
    print("   · 二进制也原样回来 + 零延迟 → 环回（USB 转串口的 TX/RX 短接，或下位机")
    print("     直接把收到的字节原样转发）。这时【不能】说'下位机活着'。")
    print("   · 只有可打印文本回来、内容被整理过、有延迟 → 真·下位机在解析。")
    print("   · 什么也不回 → 下位机只收不发（旧工程就是这种设计），属正常。")
    print("=" * 66)
    return 0


if __name__ == "__main__":
    sys.exit(main())
