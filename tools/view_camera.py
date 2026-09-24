#!/usr/bin/env python3
"""实时看图：订阅相机话题并在窗口里显示，可叠加视觉检测框。

为什么需要它：本机**没装** rqt_image_view / rqt / image_view，
所以 `ros2 launch rb_camera camera.launch.py` 之后图是发出去了，但没有东西显示它。
本脚本只用已装好的 OpenCV(GTK3) + rclpy，不依赖任何新包。

用法
----
    cd ~/robocup_basketball_ws && source install/setup.bash
    python3 tools/view_camera.py                  # 默认看压缩图 + 检测框
    python3 tools/view_camera.py --raw            # 看原始图（720p 下帧率会低，现象正常）
    python3 tools/view_camera.py --no-detections  # 只看图不叠框

快捷键
------
    q / ESC   退出
    s         把当前画面存到 test_artifacts/camera/view_snapshot.jpg
    d         切换是否显示检测框

⚠️ 必须在有图形界面的机器上跑（本机 DISPLAY=:0 可用）。
   如果报 "cannot open display"，说明你是在 SSH 里跑——加 -X 转发或改在车机本机跑。
"""

from __future__ import annotations

import argparse
import math
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from rb_msgs.msg import DetectionArray

WS = Path(__file__).resolve().parent.parent

# 与 perception_node.py 里 _DEBUG_COLORS 保持一致，方便对照
COLORS = {
    "ball_basketball": (0, 140, 255),
    "ball_volleyball": (255, 200, 0),
    "ball_unknown": (0, 255, 255),
    "hoop": (0, 0, 255),
    "rack_ring": (255, 0, 255),
    "pass_rack": (255, 0, 128),
    "pillar": (0, 255, 0),
    "obstacle": (128, 128, 128),
}


class CameraViewer(Node):
    def __init__(self, compressed: bool, show_dets: bool) -> None:
        super().__init__("camera_viewer")
        self.show_dets = show_dets
        self._lock = threading.Lock()
        self._frame = None
        self._frame_t = 0.0
        self._arrivals: list[float] = []
        self._dets: list = []
        self._count = 0

        # BEST_EFFORT 订阅对 reliable / best_effort 两种发布者都兼容
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        if compressed:
            self.topic = "/camera/image_raw/compressed"
            self.create_subscription(CompressedImage, self.topic, self.on_compressed, qos)
        else:
            self.topic = "/camera/image_raw"
            self.create_subscription(Image, self.topic, self.on_image, qos)
        self.create_subscription(DetectionArray, "/perception/detections", self.on_dets, 5)

        # ⚠️ 千万不能写成 self.executor = ...：rclpy 的 Node.executor 是**弱引用** property，
        #    赋进去的执行器会立刻被 GC 回收，随后 self.executor 返回 None
        #    （症状：'NoneType' object has no attribute 'add_node'）。
        #    必须用普通属性保存强引用。
        self._exec = MultiThreadedExecutor(num_threads=2)
        self._exec.add_node(self)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    # -- 回调（在 ROS 线程里）----------------------------------------------
    def on_compressed(self, msg: CompressedImage) -> None:
        frame = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self._store(frame)

    def on_image(self, msg: Image) -> None:
        try:
            if msg.encoding in ("bgr8", "8UC3"):
                frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                    (msg.height, msg.step // 3, 3))[:, : msg.width, :]
                frame = np.ascontiguousarray(frame)
            else:
                frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                    (msg.height, msg.width, -1))
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self._store(frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"图像解析失败: {exc}", throttle_duration_sec=5.0)

    def on_dets(self, msg: DetectionArray) -> None:
        with self._lock:
            self._dets = list(msg.detections)

    def _store(self, frame) -> None:
        now = time.time()
        with self._lock:
            self._frame = frame
            self._frame_t = now
            self._arrivals.append(now)
            self._arrivals = [t for t in self._arrivals if now - t <= 2.0]
            self._count += 1

    # -- 主线程取用 --------------------------------------------------------
    def snapshot(self):
        with self._lock:
            frame = self._frame
            dets = list(self._dets)
            arrivals = list(self._arrivals)
        return frame, dets, arrivals

    def stop(self) -> None:
        # 顺序很重要：先让 spin() 返回，再 join 线程，最后才能销毁节点。
        # 否则执行器在自旋中被析构会抛 "terminate called without an active exception"。
        try:
            self._exec.shutdown(timeout_sec=1.0)
        except Exception:  # noqa: BLE001
            pass
        try:
            self._thread.join(timeout=2.0)
        except Exception:  # noqa: BLE001
            pass


def draw(frame, dets: list, topic: str, fps: float, count: int, show_dets: bool):
    vis = frame.copy()
    h, w = vis.shape[:2]
    if show_dets:
        for d in dets:
            color = COLORS.get(d.label, (255, 255, 255))
            x0, y0 = int(d.x_min * w), int(d.y_min * h)
            x1, y1 = int(d.x_max * w), int(d.y_max * h)
            cv2.rectangle(vis, (x0, y0), (x1, y1), color, 2)
            text = f"{d.label} {d.confidence:.2f}"
            if d.distance_m == d.distance_m:  # not NaN
                text += f" {d.distance_m:.2f}m"
            if d.bearing_rad == d.bearing_rad:
                text += f" {math.degrees(d.bearing_rad):+.0f}deg"
            cv2.putText(vis, text, (x0, max(16, y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

    # 顶部信息条
    bar = f"{topic}  {fps:.1f} fps  frames={count}  dets={len(dets)}"
    cv2.putText(vis, bar, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(vis, bar, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(vis, "q=quit  s=save  d=toggle dets", (8, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(vis, "q=quit  s=save  d=toggle dets", (8, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def main() -> int:
    ap = argparse.ArgumentParser(description="实时查看相机画面")
    ap.add_argument("--raw", action="store_true",
                    help="订阅原始图 /camera/image_raw（720p 下帧率会明显偏低，属正常）")
    ap.add_argument("--no-detections", action="store_true", help="不叠加检测框")
    ap.add_argument("--window-scale", type=float, default=1.0, help="窗口缩放，如 0.5 看全画面")
    ap.add_argument("--duration", type=float, default=0.0,
                    help=">0 时跑这么多秒后自动退出（无人值守/自检用）")
    ap.add_argument("--save", default="",
                    help="退出时把最后一帧（含检测框）存到该路径")
    args = ap.parse_args()

    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("✗ 没有图形界面（DISPLAY/WAYLAND_DISPLAY 都没设）。")
        print("  如果是在 SSH 里，请加 -X 做 X11 转发，或直接在车机本机桌面上跑。")
        return 2

    rclpy.init()
    node = CameraViewer(compressed=not args.raw, show_dets=not args.no_detections)
    win = "rb_camera view (q=quit, s=save, d=toggle dets)"
    print(f"订阅 {node.topic} ，窗口打开中… 没画面时先确认：")
    print("  ros2 launch rb_camera camera.launch.py")
    print("  python3 tools/camera_check.py     # 确认相机在出图")
    waited = 0.0
    saved_note = ""
    t_start = time.time()
    was_visible = False
    try:
        while rclpy.ok():
            if args.duration > 0 and time.time() - t_start >= args.duration:
                break
            frame, dets, arrivals = node.snapshot()
            if frame is None:
                # 还没收到图：显示一张占位图，别让用户以为程序卡死
                placeholder = np.zeros((240, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "waiting for image...", (30, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2, cv2.LINE_AA)
                cv2.putText(placeholder, node.topic, (30, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1, cv2.LINE_AA)
                cv2.imshow(win, placeholder)
                waited += 0.05
                if waited > 5.0:
                    waited = 0.0
                    print(f"  …仍在等 {node.topic}（5s 无图）")
            else:
                fps = len(arrivals) / 2.0
                vis = draw(frame, dets, node.topic, fps, node._count, node.show_dets)
                if args.window_scale != 1.0:
                    vis = cv2.resize(vis, None, fx=args.window_scale, fy=args.window_scale)
                cv2.imshow(win, vis)

            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s") and frame is not None:
                out = WS / "test_artifacts" / "camera" / "view_snapshot.jpg"
                out.parent.mkdir(parents=True, exist_ok=True)
                vis = draw(frame, dets, node.topic, len(arrivals) / 2.0, node._count, node.show_dets)
                cv2.imwrite(str(out), vis)
                saved_note = str(out)
                print(f"  已保存 {out}")
            if key == ord("d"):
                node.show_dets = not node.show_dets
                print(f"  检测框显示: {'开' if node.show_dets else '关'}")
            # 只在"窗口曾经可见、现在不可见"时才认定用户关了窗口。
            # 直接判断 <1 会在窗口尚未映射时误触发（无头/自动化环境必现）。
            try:
                visible = cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE)
            except Exception:  # noqa: BLE001
                visible = 1.0
            if visible >= 1:
                was_visible = True
            elif was_visible:
                break  # 用户点了窗口关闭按钮
    except KeyboardInterrupt:
        pass
    finally:
        if args.save:
            frame, dets, arrivals = node.snapshot()
            if frame is not None:
                vis = draw(frame, dets, node.topic, len(arrivals) / 2.0, node._count, node.show_dets)
                Path(args.save).parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(args.save, vis)
                saved_note = args.save
                print(f"  已保存 {args.save}")
            else:
                print("  --save 失败：全程没收到图像")
        cv2.destroyAllWindows()
        node.stop()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    if saved_note:
        print(f"快照: {saved_note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
