#!/usr/bin/env python3
"""内参自检：把球放在相机正前方【已知距离】，看检测读数准不准。

为什么需要这个：内参（fx/cx…）没法靠"型号一样"推断 ——
  * fx 取决于镜头焦距 + 传感器像素间距
  * cx/cy（主点）是【单台】属性：每台传感器粘接位置都略有不同
实测这台前摄标出来的主点就偏了 -62.6px ≈ -4.5°。把它的内参抄给后摄，
后摄的方位角就会系统偏几度 —— 而"让摄像头正对球"靠的正是方位角。

所以抄内参【可以救急】，但必须用这个工具验一次：
  距离读数偏差 <5%  且  球在画面正中时 bearing ≈ 0°  → 能凑合用
  否则老老实实标定。

用法：
    # 车不要动，把球放在相机正前方 1.00m（卷尺量），然后：
    python3 tools/check_intrinsics.py                 # 前摄
    python3 tools/check_intrinsics.py --rear          # 后摄
    python3 tools/check_intrinsics.py --rear --truth 1.00
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import yaml

WS = __import__("pathlib").Path(__file__).resolve().parents[1]

CFG = {
    "front": (WS / "src/rb_camera/config/camera.yaml",
              "/perception/detections", "前摄"),
    "rear": (WS / "src/rb_camera/config/camera_rear.yaml",
             "/perception/detections_rear", "后摄"),
}


def load_intr(slot: str):
    cam, _, _ = CFG[slot]
    d = yaml.safe_load(cam.read_text(encoding="utf-8")) or {}
    i = d.get("intrinsics", {}) or {}
    r = d.get("camera", {}) or {}
    return i, r


def main() -> int:
    ap = argparse.ArgumentParser(description="内参自检：已知距离放球，看读数准不准")
    ap.add_argument("--rear", action="store_true", help="检后摄（默认前摄）")
    ap.add_argument("--truth", type=float, default=1.00,
                    help="球到相机光心的真实距离 m（卷尺量，默认 1.00）")
    ap.add_argument("--seconds", type=float, default=8.0, help="采样时长")
    ap.add_argument("--label", default="ball_basketball")
    args = ap.parse_args()

    slot = "rear" if args.rear else "front"
    intr, rcfg = load_intr(slot)
    _, topic, cn = CFG[slot]

    print("═" * 62)
    print(f"  内参自检 · {cn}")
    print("═" * 62)
    fx = float(intr.get("fx") or 0.0)
    if fx <= 0:
        print(f"  ⚠️ {slot} 的 fx = 0（未标定）—— 现在是用假定 HFOV 反推的，")
        print("     /cos(b) 的换算会整段偏。建议先标定或从另一摄抄一份再来自检。")
    print(f"  fx={fx}  fy={intr.get('fy')}  cx={intr.get('cx')}  cy={intr.get('cy')}")
    print(f"  分辨率 {rcfg.get('width')}x{rcfg.get('height')}")
    if fx > 0:
        hfov = math.degrees(2 * math.atan((float(rcfg.get("width", 1280)) / 2) / fx))
        print(f"  推算 HFOV = {hfov:.1f}°")
    print(f"  目标：球放在相机正前方 {args.truth:.2f}m（卷尺量准），画面尽量居中")
    print(f"  订阅：{topic}")
    print("═" * 62)

    try:
        import rclpy
        from rclpy.node import Node
        from rb_msgs.msg import DetectionArray
    except ImportError as e:  # noqa: BLE001
        print(f"  无法导入 ROS 依赖: {e}\n  先 source 工作空间", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("check_intrinsics")
    samples = []

    def cb(m):
        for d in m.detections:
            if d.label != args.label:
                continue
            b = float(d.bearing_rad)
            dist = float(d.distance_m)
            w = float(getattr(d, "bbox_px_w", 0.0))
            if b == b and dist == dist and dist > 0:
                samples.append((b, dist, w))

    node.create_subscription(DetectionArray, topic, cb, 10)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    if len(samples) < 3:
        print(f"  ✗ 只收到 {len(samples)} 个「{args.label}」检测 —— 球在画面里吗？")
        print(f"    标签对不对？先 ros2 topic echo {topic} 看一眼")
        return 1

    bs = sorted(s[0] for s in samples)
    ds = sorted(s[1] for s in samples)
    b_med = bs[len(bs) // 2]
    d_med = ds[len(ds) // 2]
    b_deg = math.degrees(b_med)
    err = (d_med - args.truth) / args.truth * 100.0
    # 光轴深度 → 斜距 → 球到光心的距离
    slant = d_med / max(math.cos(b_med), 1e-3)

    print(f"  样本 {len(samples)} 帧")
    print(f"  方位角中位数   {b_deg:+.2f}°")
    print(f"  距离读数中位数 {d_med:.3f} m")
    print(f"  斜距           {slant:.3f} m   ← 球到相机光心的真实距离应接近这个")
    print(f"  与真值 {args.truth:.2f}m 的偏差 {err:+.1f}%")
    print()
    ok_d = abs(err) < 5.0
    # 球摆正时 bearing 应≈0；这里只提示，不作为硬判据（人摆不准）
    ok_b = abs(b_deg) < 5.0
    print(f"  {'✓' if ok_d else '✗'} 距离误差 {abs(err):.1f}%  {'（<5%，可用）' if ok_d else '（>5%，内参不对）'}")
    print(f"  {'✓' if ok_b else '⚠'} 方位角 {b_deg:+.2f}°  "
          f"{'（球基本在正中）' if ok_b else '（球没摆正在正中也可能是这个原因，先摆正再看）'}")
    print()
    if ok_d and ok_b:
        print("  → 内参可用，可以先用着（精准起见有空还是标一次）")
    else:
        print("  → 建议重新标定：python3 tools/calibrate_camera.py "
              f"--slot {slot} --auto")
        if not ok_b:
            print("     若距离准但方位角一直偏固定角度，也可以用检测器的 "
                  "camera_yaw_offset_rad 补掉这个常数偏移")
    return 0 if (ok_d and ok_b) else 1


if __name__ == "__main__":
    raise SystemExit(main())
