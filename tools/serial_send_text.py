#!/usr/bin/env python3
"""往机构串口发一句文本（默认 "hallo"），并听下位机有没有回话。

用途：现场快速确认「线对不对 / 波特率对不对 / 下位机在不在」。
注意：这不是 11 字节协议帧，而是**纯文本**。旧工程的下位机同时支持字符串命令
      （"catch" / "slideway" / "dribbledouble" / "launchallfast"），所以纯文本
      是会被解析的 —— 但未知字符串通常会被忽略。**发之前确认周围安全。**

用法：
    python3 tools/serial_send_text.py                       # 15200 发 hallo
    python3 tools/serial_send_text.py --baud 115200
    python3 tools/serial_send_text.py --text hello --repeat 3 --listen 3
    python3 tools/serial_send_text.py --sweep               # 常用波特率轮一遍
"""

from __future__ import annotations

import argparse
import sys
import time


def try_baud(port: str, baud: int, text: str, repeat: int, listen: float,
             newline: bool, verbose: bool = True) -> int:
    """返回收到的字节数（-1 = 打不开这个波特率）。"""
    import serial  # pyserial

    try:
        ser = serial.Serial(port, baudrate=baud, bytesize=8, parity="N",
                            stopbits=1, timeout=0.2, write_timeout=1.0)
    except Exception as e:  # noqa: BLE001
        if verbose:
            print(f"  ✗ {baud:>7} 打不开: {e}")
        return -1

    # 实际生效的波特率（cdc_acm 可能被改回去）
    actual = getattr(ser, "baudrate", baud)
    payload = text.encode() + (b"\n" if newline else b"")
    got = b""
    try:
        ser.reset_input_buffer()
        for i in range(repeat):
            ser.write(payload)
            ser.flush()
            if verbose:
                print(f"      → 发送 [{i+1}/{repeat}] {payload!r}  ({len(payload)} 字节)")
            time.sleep(0.25)
        t0 = time.time()
        while time.time() - t0 < listen:
            chunk = ser.read(4096)
            if chunk:
                got += chunk
                if verbose:
                    print(f"      ← 收到 {len(chunk)} 字节: {chunk!r}")
            else:
                time.sleep(0.05)
    except Exception as e:  # noqa: BLE001
        if verbose:
            print(f"      ⚠️ 写/读异常: {e}")
    finally:
        ser.close()

    if verbose:
        flag = "✓ 回话了!" if got else "（静默，无回话）"
        extra = "" if actual == baud else f"  ⚠️ 实际波特率={actual}"
        print(f"  {'✓' if got else '·'} {baud:>7} 发送 {repeat} 次{extra}  "
              f"收回 {len(got)} 字节 {flag}")
    return len(got)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="往机构串口发文本并听回话")
    ap.add_argument("--port", default="/dev/R1_usb2ttl")
    ap.add_argument("--text", default="hallo")
    ap.add_argument("--baud", type=int, default=15200)
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--listen", type=float, default=2.5, help="发完听多久（秒）")
    ap.add_argument("--no-newline", action="store_true", help="不加换行")
    ap.add_argument("--sweep", action="store_true", help="常用波特率轮一遍找对的")
    args = ap.parse_args(argv)

    nl = not args.no_newline
    print(f"📤 串口文本发送  port={args.port}  text={args.text!r}  "
          f"换行={'加' if nl else '不加'}\n")

    if args.sweep:
        # 先试非标准 15200（cdc_acm 才可能支持），再轮标准值
        cands = [15200, 115200, 57600, 38400, 19200, 9600, 460800, 230400, 128000, 256000]
        hits = []
        for b in cands:
            n = try_baud(args.port, b, args.text, args.repeat, 1.5, nl)
            if n > 0:
                hits.append(b)
        print()
        if hits:
            print(f"  ✅ 有回话的波特率: {hits}  ← 用这个")
        else:
            print("  ⚠️ 所有波特率都没回话。可能：")
            print("     ① 下位机本来就不回话（只单向接收指令）—— 旧工程就是这种设计")
            print("     ② 线序不对 / 没接通 / 下位机没上电")
            print("     ③ 文本命令不被识别（需要 11 字节协议帧）")
        return 0

    n = try_baud(args.port, args.baud, args.text, args.repeat, args.listen, nl)
    if n < 0:
        return 1
    if n == 0:
        print("\n  ℹ️ 没回话不一定是坏事：旧工程的下位机是「收到指令才动作」的单向设计，")
        print("     正常工作时也不回数据。要确认它收到了，看机构有没有实际动作。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
