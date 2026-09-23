#!/usr/bin/env python3
"""假检测：根据模拟位姿生成几何正确的检测结果，用于在无相机时验证完整状态机。

与 publish_test_image.py 的区别
-------------------------------
  publish_test_image.py  发一张**静态图**，只能验证"视觉能不能检出目标"，
                         无法推进 SEEK→ACQUIRE→NAV→ALIGN→LAUNCH 的完整流程
                         （因为图不动，球的距离永远不变）。
  fake_detections.py     直接按 /odom 的位姿算出"球在哪个方向、多远"，
                         于是状态机能真正走完整条链路。

用途：只验证**决策与运动逻辑**，不验证视觉。视觉用 make_test_image.py 单独验。
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Header

from rb_msgs.msg import Detection, DetectionArray


class FakeDetections(Node):
    def __init__(self, ball_label: str, rate: float) -> None:
        super().__init__("fake_detections")
        self.ball_label = ball_label
        self.pose = (1.0, 1.0, 0.0)

        # 目标物在场地里的"真实"位置（与 mission.yaml 的 field 对齐）
        self.ball_xy = (4.0, 3.0)      # 一颗球的位置
        self.hoop_xy = (13.5, 3.75)    # 篮筐
        self.rack_xy = (10.5, 3.75)    # 传球架

        self.ball_seq = [(4.0, 3.0), (2.5, 6.0)]   # 两颗球，模拟"吸走一颗再来一颗"
        self.ball_idx = 0
        self.pickup_distance = 0.85
        self.pub = self.create_publisher(DetectionArray, "/perception/detections", 5)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_timer(1.0 / max(rate, 1.0), self.tick)
        self.get_logger().info(f"假检测启动：目标球={ball_label}")

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = p.z
        self.pose = (p.x, p.y, yaw)

    def _det(self, label: str, target_xy, real_size: float, has_ball: bool) -> Detection:
        x, y, yaw = self.pose
        dx, dy = target_xy[0] - x, target_xy[1] - y
        dist = math.hypot(dx, dy)
        bearing_world = math.atan2(dy, dx)
        bearing_body = math.atan2(math.sin(bearing_world - yaw), math.cos(bearing_world - yaw))

        # 用固定内参反推像素：x = cx + fx*tan(bearing)
        fx, cx, cy = 554.3, 320.0, 240.0
        px = cx + fx * math.tan(max(-1.2, min(1.2, bearing_body)))
        py = cy
        size_px = fx * real_size / max(dist, 0.05)

        d = Detection()
        d.stamp = self.get_clock().now().to_msg()
        d.label = label
        d.confidence = 0.9
        d.px, d.py = float(px), float(py)
        d.bbox_px_w = d.bbox_px_h = float(size_px)
        d.cx = float(px / 640.0)
        d.cy = float(py / 480.0)
        d.width = d.height = float(size_px / 640.0)
        d.x_min = d.cx - d.width / 2
        d.y_min = d.cy - d.height / 2
        d.x_max = d.cx + d.width / 2
        d.y_max = d.cy + d.height / 2
        d.bearing_rad = float(bearing_body)
        d.distance_m = float(dist)
        d.diameter_m = float(real_size)
        return d

    def _maybe_pickup(self) -> None:
        """机器人靠近当前球时，把它"拿走"，换下一颗（模拟吸取成功）。"""
        if self.ball_idx >= len(self.ball_seq):
            return
        x, y, _ = self.pose
        bx, by = self.ball_seq[self.ball_idx]
        if math.hypot(bx - x, by - y) < self.pickup_distance:
            self.ball_idx += 1
            if self.ball_idx < len(self.ball_seq):
                self.ball_xy = self.ball_seq[self.ball_idx]
                self.get_logger().info(f"模拟吸取：换到第 {self.ball_idx + 1} 颗球 {self.ball_xy}")
            else:
                self.get_logger().info("模拟吸取：球已全部取完")

    def tick(self) -> None:
        self._maybe_pickup()
        arr = DetectionArray()
        arr.header = Header()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "camera"
        arr.fx, arr.fy, arr.cx_cam, arr.cy_cam = 554.3, 554.3, 320.0, 240.0
        arr.img_width, arr.img_height = 640, 480

        dets = []
        if self.ball_idx < len(self.ball_seq):
            dets.append(self._det("ball_basketball", self.ball_xy, 0.24, True))
        dets += [
            self._det("ball_volleyball", (2.0, 5.5), 0.21, False),
            self._det("hoop", self.hoop_xy, 0.40, False),
            self._det("rack_ring", self.rack_xy, 0.55, False),
            self._det("pillar", (10.5, 3.75), 0.20, False),
        ]
        arr.detections = dets
        self.pub.publish(arr)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ball", default="ball_basketball")
    ap.add_argument("--rate", type=float, default=10.0)
    args = ap.parse_args()
    rclpy.init()
    node = FakeDetections(args.ball, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
