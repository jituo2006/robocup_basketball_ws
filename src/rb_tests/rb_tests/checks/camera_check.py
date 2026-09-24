#!/usr/bin/env python3
"""相机链路自检：起 rb_camera，订阅图像/内参，报告实测帧率并存一帧快照。

⚠️ 为什么测压缩话题而不是原始图：
   本机实测原始 640x480 (921KB) 图像消息投递只有 ~7fps（大消息吞吐瓶颈），
   这不代表相机坏，而是"大图没压缩"。实际给 rb_perception 用的是
   /camera/image_raw/compressed（JPEG，~40KB，能跑满 30fps）。
   所以自检以压缩话题为准，同时兼顾原始的 CameraInfo 频率。

用法：
    ros2 run rb_tests camera_check.py [--device /dev/video0] [--seconds 6]

退出码 0 = 相机链路正常；1 = 有问题。
产物：test_artifacts/camera/snapshot.jpg（肉眼确认画面，别是黑屏/雪花）
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from rb_tests.ws import WS  # 工作空间定位（原来靠 __file__ 推算，搬包后会错）
TEST_DOMAIN = "79"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, CompressedImage

    outdir = WS / "test_artifacts" / "camera"
    outdir.mkdir(parents=True, exist_ok=True)
    logf = open(outdir / "camera_node.log", "w")  # noqa: SIM115
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = TEST_DOMAIN
    proc = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/humble/setup.bash && "
                       f"source {WS}/install/setup.bash && "
                       f"exec ros2 run rb_camera camera_node --ros-args -p device:={args.device}"],
        cwd=str(WS), stdout=logf, stderr=subprocess.STDOUT, env=env, start_new_session=True)

    rclpy.init()
    node = rclpy.create_node("camera_check")
    stats = {"img": 0, "info": 0, "shape": None, "frame_id": "", "k": None, "frame": None}

    def on_img(m: CompressedImage) -> None:
        stats["img"] += 1
        stats["frame_id"] = m.header.frame_id
        frame = cv2.imdecode(np.frombuffer(bytes(m.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            stats["shape"] = (frame.shape[0], frame.shape[1])
            stats["frame"] = frame
            stats["kb"] = len(m.data) / 1024.0

    # best_effort 订阅对 reliable / best_effort 两种发布者都兼容，自检更稳
    sub_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    node.create_subscription(CompressedImage, "/camera/image_raw/compressed", on_img, sub_qos)
    node.create_subscription(CameraInfo, "/camera/camera_info",
                             lambda m: (stats.__setitem__("info", stats["info"] + 1),
                                        stats.__setitem__("k", list(m.k))), 2)

    time.sleep(4.0)  # 预热
    stats["img"] = stats["info"] = 0
    t0 = time.time()
    while time.time() - t0 < args.seconds and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
    elapsed = time.time() - t0

    saved = ""
    if stats["frame"] is not None:
        saved = str(outdir / "snapshot.jpg")
        cv2.imwrite(saved, stats["frame"])

    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=8)
    except Exception:  # noqa: BLE001
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:  # noqa: BLE001
            pass

    print(f"  /camera/image_raw/compressed : {stats['img']} 帧 / {elapsed:.1f}s = "
          f"{stats['img'] / elapsed:.1f} fps (每帧约 {stats.get('kb', 0):.0f} KB)")
    print(f"  /camera/camera_info          : {stats['info']} 条 = {stats['info'] / elapsed:.1f} fps")
    print(f"  分辨率                       : {stats['shape']} frame_id={stats['frame_id']}")
    if stats["k"]:
        print(f"  内参 K                       : fx={stats['k'][0]:.1f} fy={stats['k'][4]:.1f} "
              f"cx={stats['k'][2]:.1f} cy={stats['k'][5]:.1f}")
    if saved:
        print(f"  快照                         : {saved}  ← 打开看一眼，别是黑屏")

    if stats["img"] == 0:
        print("\033[31m✗ 失败\033[0m  收不到图像。看 test_artifacts/camera/camera_node.log")
        return 1
    if stats["info"] == 0:
        print("\033[33m! 警告\033[0m  没有 CameraInfo")
    rate = stats["img"] / elapsed
    if rate < 15:
        print(f"\033[33m! 警告\033[0m  只有 {rate:.1f} fps，偏低（期望 ≥25）。"
              f"可能是 CPU 被占满或相机被别的进程抢用")
    print("\033[32m✓ 通过\033[0m  相机链路正常")
    return 0


if __name__ == "__main__":
    sys.exit(main())
