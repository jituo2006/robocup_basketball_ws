#!/usr/bin/env python3
"""生成一张合成测试图，用于在没有相机的情况下验证视觉检测逻辑。

图上包含规则里会出现的几种目标：
  * 橙色篮球（圆）
  * 白/蓝排球（圆）
  * 红/橙色篮筐（环）
  * 蓝绿相间定位柱（竖条）

默认还会直接跑一遍检测器并把结果打出来 —— 这样"改阈值后是否还能检出"
不需要任何硬件就能验证。

用法：
    ros2 run rb_tests make_test_image.py --out /tmp/rb_test.png
    ros2 run rb_tests make_test_image.py --out /tmp/rb_test.png --check
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

from rb_tests.ws import WS  # 工作空间定位

from rb_perception.detectors import CameraModel, build_detectors  # noqa: E402


def make_image(w: int = 640, h: int = 480) -> np.ndarray:
    img = np.full((h, w, 3), 60, dtype=np.uint8)  # 深灰背景（模拟体育馆地面/墙）

    # 橙色篮球（直径约 70px）
    cv2.circle(img, (110, 360), 35, (0, 140, 255), -1)

    # 白/蓝排球
    cv2.circle(img, (230, 350), 30, (240, 240, 240), -1)
    cv2.circle(img, (230, 350), 30, (200, 120, 60), 6)

    # 红橙色篮筐（环）
    cv2.circle(img, (470, 150), 55, (0, 60, 230), 6)
    # 篮板
    cv2.rectangle(img, (440, 40), (600, 120), (230, 230, 230), -1)

    # 蓝绿相间定位柱（竖条）
    cv2.rectangle(img, (300, 180), (330, 300), (200, 120, 40), -1)   # 蓝
    cv2.rectangle(img, (300, 300), (330, 420), (60, 200, 80), -1)    # 绿

    return img


def main() -> int:
    ap = argparse.ArgumentParser(description="生成合成测试图并可选自检")
    ap.add_argument("--out", default=os.path.join(WS, "test_artifacts", "rb_test.png"),
                    help="输出图片路径（默认放在工作空间内，避免 /tmp 被清）")
    ap.add_argument("--config", default=os.path.join(
        WS, "src", "rb_perception", "config", "perception.yaml"))
    ap.add_argument("--check", action="store_true", help="生成后直接跑一遍检测器")
    args = ap.parse_args()

    img = make_image()
    cv2.imwrite(args.out, img)
    print(f"已生成测试图: {args.out}  ({img.shape[1]}x{img.shape[0]})")

    if not args.check:
        return 0

    import yaml

    with open(args.config, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh) or {}
    detectors, notes = build_detectors(cfg.get("perception", {}))
    for n in notes:
        print(f"  [note] {n}")

    cam = CameraModel.from_config(cfg.get("camera", {}) or {}, img.shape[1], img.shape[0])
    print(f"  相机模型: fx={cam.fx:.1f} cx={cam.cx:.1f} (未标定，按假定 FOV 反推)")

    total = 0
    for det in detectors:
        try:
            results = det.detect(img, cam)
        except Exception as exc:  # noqa: BLE001
            print(f"  ✗ {det.label}: 检测异常 {type(exc).__name__}: {exc}")
            continue
        if results:
            total += len(results)
            for r in results:
                dist = "nan" if r.distance_m != r.distance_m else f"{r.distance_m:.2f}m"
                print(f"  ✓ {r.label:<18} conf={r.confidence:.2f} "
                      f"px=({r.px:6.1f},{r.py:6.1f}) bearing={r.bearing_rad:+.3f}rad dist={dist}")
        else:
            print(f"  · {det.label:<18} 未检出")

    print(f"\n共检出 {total} 个目标")
    return 0 if total > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
