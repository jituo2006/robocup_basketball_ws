#!/usr/bin/env python3
"""视觉吞吐率测量：相机发布 vs 感知输出，用来定分辨率。

为什么要测：相机能 720p@30，但要看 rb_perception 跟不跟得上。
分辨率选错，机器人看到的就是"慢动作"。

测量技巧
--------
不用图像话题测相机帧率！Python 订阅 720p 图像本身只有几 fps，
测出来是"订阅者的能力"，不是相机的。改用**小的 CameraInfo**
（每帧随图像一起发、体积可忽略），它的频率 == 图像发布频率。

用法：
    ros2 run rb_tests vision_rate_check.py                 # 测当前配置
    ros2 run rb_tests vision_rate_check.py -w 640 -H 480
    ros2 run rb_tests vision_rate_check.py --sweep         # 480p / 720p 对比
    ros2 run rb_tests vision_rate_check.py --no-debug      # 关掉调试图再测（看 A/B 差异）
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
TEST_DOMAIN = "83"


def measure(width: int, height: int, seconds: float, debug: bool) -> dict:
    os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN
    import rclpy
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import CameraInfo
    from rb_msgs.msg import DetectionArray

    outdir = WS / "test_artifacts" / "camera"
    outdir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = TEST_DOMAIN

    tag = f"{width}x{height}_{'dbg' if debug else 'nodb'}"
    cam_log = open(outdir / f"rate_cam_{tag}.log", "w")  # noqa: SIM115
    perc_log = open(outdir / f"rate_perc_{tag}.log", "w")  # noqa: SIM115

    cam = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/humble/setup.bash && source "
                       f"{WS}/install/setup.bash && exec ros2 run rb_camera camera_node "
                       f"--ros-args -p width:={width} -p height:={height}"],
        cwd=str(WS), stdout=cam_log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    time.sleep(3.5)  # 等相机打开
    perc = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/humble/setup.bash && source "
                       f"{WS}/install/setup.bash && exec ros2 run rb_perception perception_node "
                       f"--ros-args -p publish_debug_image:={'true' if debug else 'false'}"],
        cwd=str(WS), stdout=perc_log, stderr=subprocess.STDOUT, env=env, start_new_session=True)

    rclpy.init()
    node = rclpy.create_node("vision_rate_check")
    counts = {"cam": 0, "det": 0}
    last = {"det": None}
    # CameraInfo：小消息，可靠传输，能准确反映图像发布频率
    node.create_subscription(CameraInfo, "/camera/camera_info",
                             lambda m: counts.__setitem__("cam", counts["cam"] + 1), 10)
    # 感知话题在 rb_mission 里是 best_effort 订阅；这里也用 best_effort 避免干扰
    det_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
    node.create_subscription(DetectionArray, "/perception/detections",
                             lambda m: (counts.__setitem__("det", counts["det"] + 1),
                                        last.__setitem__("det", m)), det_qos)

    time.sleep(6.0)          # 预热
    counts["cam"] = counts["det"] = 0
    t0 = time.time()
    while time.time() - t0 < seconds and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
    dt = time.time() - t0

    result = {
        "width": width, "height": height,
        "cam_fps": counts["cam"] / dt, "det_fps": counts["det"] / dt,
        "detections": len(last["det"].detections) if last["det"] is not None else -1,
        "debug": debug,
    }

    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass
    for proc in (perc, cam):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=6)
        except Exception:  # noqa: BLE001
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:  # noqa: BLE001
                pass
    time.sleep(1.2)
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-w", "--width", type=int, default=640)
    ap.add_argument("-H", "--height", type=int, default=480)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--sweep", action="store_true", help="依次测 640x480 和 1280x720")
    ap.add_argument("--no-debug", action="store_true", help="关掉调试图发布")
    args = ap.parse_args()

    combos = [(640, 480), (1280, 720)] if args.sweep else [(args.width, args.height)]
    print(f"调试图发布: {'关' if args.no_debug else '开'}")
    print("分辨率        相机发布    感知输出    可视化目标数")
    print("-" * 54)
    results = []
    for w, h in combos:
        r = measure(w, h, args.seconds, not args.no_debug)
        results.append(r)
        flag = "" if r["det_fps"] >= 10 else "  ← 偏慢"
        print(f"{w}x{h:<8}  {r['cam_fps']:6.1f} fps  {r['det_fps']:6.1f} fps  "
              f"{r['detections']:>6}{flag}")

    print("-" * 54)
    slowest = min(results, key=lambda r: r["det_fps"])
    if slowest["det_fps"] < 10:
        print(f"\033[33m提示\033[0m 感知在 {slowest['width']}x{slowest['height']} 下只有 "
              f"{slowest['det_fps']:.1f}fps。\n"
              f"     感知算法本身很快（480p 约 10ms/帧 = 90fps 上限），瓶颈多在 ROS 消息路径。\n"
              f"     ① 优先用 640x480（远处识别靠标定/镜头，不靠分辨率堆）\n"
              f"     ② 关调试图：perception.yaml 的 publish_debug_image 或 -p publish_debug_image:=false\n"
              f"     ③ 关掉用不到的 circle 检测器（hoop/rack_ring 最贵）\n"
              f"     ④ 上 YOLO ONNX（比 HSV+霍夫圆更快，还更稳）")
    else:
        print("\033[32m✓\033[0m 感知帧率够用（≥10fps）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
