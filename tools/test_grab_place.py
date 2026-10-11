#!/usr/bin/env python3
"""抓球并放下 —— 机构动作序列调试。

为什么做成「步骤表」而不是写死流程：
  运球/双运球/夹爪/滑轨的**物理语义在下位机固件里**，本仓库看不到。
  到底哪一步是"抓住"、哪一步是"松开"，只有上车试了才知道。
  所以这里给一个可配置的步骤序列 + 几个预设，现场改 JSON 就能试各种组合。

动作码（来自 ROBOCON 2025 ballrobot 的 Message::FrameControl）：
  0=SHOOT  1=FAST_SHOOT  2=DRIBBLE  3=DOUBLE_DRIBBLE
  4=PAWL_TOGGLE  5=SLIDEWAY_TOGGLE  6=KEEP_LOOP  7=STOP
⚠️ PAWL/ SLIDEWAY 在下位机是**开关语义**，每发一次翻转一次 ——
   所以脚本会**记录自己发过几次翻转**，方便现场判断当前状态。

用法：
    python3 tools/test_grab_place.py --preset grab_place
    python3 tools/test_grab_place.py --preset grab_hold --repeats 3
    python3 tools/test_grab_place.py --steps '[{"action":2,"hold_s":1.5},{"action":7,"hold_s":1}]'
    python3 tools/test_grab_place.py --preset grab_place --dry-run     # 只打印不发

⚠️ 会真动机构！务必先确认机械周围没人、球在正确位置、随时能断电。
   脚本开始前会等 3 秒倒计时（--no-countdown 可跳过）。
"""
from __future__ import annotations

import argparse
import json
import sys
import time

# ⚠️ ROS 依赖【延迟导入】：--help / --dry-run 不该要求 ROS 环境，
#    否则没 source 工作空间时连"看看步骤表长什么样"都做不到。
rclpy = None
Node = None
Launch = None


def _ensure_ros() -> None:
    global rclpy, Node, Launch
    if rclpy is not None:
        return
    try:
        import rclpy as _r
        from rclpy.node import Node as _N
        from rb_msgs.srv import Launch as _L
    except ImportError as e:  # noqa: BLE001
        print(f"无法导入 ROS 依赖: {e}\n请先 source 工作空间（rbws）", file=sys.stderr)
        raise SystemExit(2)
    rclpy, Node, Launch = _r, _N, _L

ACTION_NAMES = {
    0: "SHOOT 发射", 1: "FAST_SHOOT 快发", 2: "DRIBBLE 运球",
    3: "DOUBLE_DRIBBLE 双运球", 4: "PAWL_TOGGLE 夹爪翻转",
    5: "SLIDEWAY_TOGGLE 滑轨翻转", 6: "KEEP_LOOP 循环",
    7: "STOP 停",
}

# 预设：现场按实际机构语义再调
PRESETS = {
    # 抓球 → 保持 → 放下。默认用「运球开 / 停」当抓放，因为下位机把 DRIBBLE
    # 当持续动作（节点会按 send_rate 重复发），比 toggle 好控制。
    "grab_place": [
        {"action": 2, "hold_s": 1.5, "note": "抓球（运球动作）"},
        {"action": 7, "hold_s": 0.5, "note": "停一下"},
        {"action": 4, "hold_s": 0.8, "note": "夹爪翻转（抓/放取决于固件当前态）"},
        {"action": 7, "hold_s": 0.5, "note": "停"},
    ],
    # 只抓不放，验证抓取牢不牢
    "grab_hold": [
        {"action": 2, "hold_s": 2.0, "note": "抓球"},
        {"action": 4, "hold_s": 3.0, "note": "夹爪翻转并保持 3s（看夹没夹住）"},
    ],
    # 只放，验证释放
    "place_only": [
        {"action": 4, "hold_s": 0.8, "note": "夹爪翻转（释放）"},
        {"action": 7, "hold_s": 0.5, "note": "停"},
    ],
    # 慢速单步：每步之间停久一点，方便人盯着看
    "slow_walk": [
        {"action": 2, "hold_s": 2.0, "note": "运球开"},
        {"action": 7, "hold_s": 1.5, "note": "停"},
        {"action": 4, "hold_s": 2.0, "note": "夹爪翻转"},
        {"action": 7, "hold_s": 1.5, "note": "停"},
        {"action": 5, "hold_s": 2.0, "note": "滑轨翻转"},
        {"action": 7, "hold_s": 1.5, "note": "停"},
    ],
}


def call_launch(node, cli, action: int, speed: int, angle: int,
                timeout: float = 3.0) -> tuple[bool, str]:
    if not cli.wait_for_service(timeout_sec=timeout):
        return False, "服务 /rb_launcher/launch 不可用（机构节点没起？）"
    req = Launch.Request()
    req.action, req.speed, req.angle = int(action), int(speed), int(angle)
    fut = cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not fut.done():
        return False, "服务调用超时"
    try:
        r = fut.result()
        return bool(r.success), str(r.message)
    except Exception as e:  # noqa: BLE001
        return False, f"服务异常: {e}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="抓球并放下 —— 机构动作序列调试")
    ap.add_argument("--preset", choices=sorted(PRESETS), default="grab_place")
    ap.add_argument("--steps", help="自定义步骤 JSON，覆盖 preset")
    ap.add_argument("--speed", type=int, default=0, help="转速 rpm（0=用节点默认）")
    ap.add_argument("--angle", type=int, default=0, help="角度（0=用节点默认）")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--interval", type=float, default=1.0, help="两遍之间的间隔秒")
    ap.add_argument("--dry-run", action="store_true", help="只打印不发指令")
    ap.add_argument("--no-countdown", action="store_true")
    args = ap.parse_args(argv)

    steps = PRESETS[args.preset]
    if args.steps:
        try:
            steps = json.loads(args.steps)
        except json.JSONDecodeError as e:
            print(f"--steps 不是合法 JSON: {e}", file=sys.stderr)
            return 2
    if not isinstance(steps, list) or not steps:
        print("步骤表为空", file=sys.stderr)
        return 2
    for s in steps:
        if int(s.get("action", -1)) not in ACTION_NAMES:
            print(f"步骤里动作码非法: {s}", file=sys.stderr)
            return 2

    print("═" * 58)
    print(f"  抓球并放下 · 机构序列调试   预设/自定义 = {args.preset}")
    print("═" * 58)
    for i, s in enumerate(steps, 1):
        a = int(s["action"])
        print(f"  {i}. {ACTION_NAMES[a]:26} 保持 {s.get('hold_s', 0.5)}s"
              f"   {s.get('note', '')}")
    print(f"  重复 {args.repeats} 遍，间隔 {args.interval}s，"
          f"speed={args.speed or '默认'} angle={args.angle or '默认'}")
    print("═" * 58)

    if args.dry_run:
        print("  --dry-run：只打印，不发任何指令 ✓")
        return 0

    print("\n⚠️  即将真动机构 —— 确认机械周围没人、球在位、随时能断电")
    if not args.no_countdown:
        for k in (3, 2, 1):
            print(f"  {k}...", flush=True)
            time.sleep(1.0)

    _ensure_ros()
    rclpy.init()
    node = Node("grab_place_debug")
    cli = node.create_client(Launch, "/rb_launcher/launch")
    flips = {4: 0, 5: 0}          # 记录 toggle 次数（下位机是开关语义）
    ok_all = True
    try:
        for rep in range(1, args.repeats + 1):
            if args.repeats > 1:
                print(f"\n── 第 {rep}/{args.repeats} 遍 ──")
            for i, s in enumerate(steps, 1):
                a = int(s["action"])
                hold = float(s.get("hold_s", 0.5))
                ok, msg = call_launch(node, cli, a, args.speed, args.angle)
                if a in flips:
                    flips[a] += 1
                mark = "✓" if ok else "✗"
                print(f"  [{i}/{len(steps)}] {mark} {ACTION_NAMES[a]}"
                      f"   → {msg}" + (f"   （已翻转 {flips[a]} 次）" if a in flips else ""))
                if not ok:
                    ok_all = False
                    print("      服务失败，中止序列（避免机械停在不确定状态）")
                    break
                time.sleep(hold)
            else:
                time.sleep(max(0.0, args.interval))
                continue
            break
    except KeyboardInterrupt:
        print("\n  收到 Ctrl-C → 发 STOP 并退出")
        call_launch(node, cli, 7, 0, 0)
        return 130
    finally:
        # 无论如何都收尾：停动作，避免机械停在高位
        ok, msg = call_launch(node, cli, 7, 0, 0, timeout=2.0)
        print(f"\n  收尾 STOP: {'✓' if ok else '✗'} {msg}")
        print(f"  toggle 累计: 夹爪 {flips[4]} 次、滑轨 {flips[5]} 次"
              f"（下位机是开关语义，偶数=回到初始态）")
        node.destroy_node()
        rclpy.shutdown()

    print("\n  " + ("全部步骤成功 ✓" if ok_all else "有步骤失败 ✗（见上面输出）"))
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
