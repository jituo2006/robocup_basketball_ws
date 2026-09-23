#!/usr/bin/env python3
"""把一张图片按固定频率发布成 sensor_msgs/Image，用于无相机时的视觉联调。"""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
DEFAULT_IMAGE = os.path.join(WS, "test_artifacts", "rb_test.png")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=DEFAULT_IMAGE)
    ap.add_argument("--topic", default="/camera/image_raw")
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--frame-id", default="camera")
    args = ap.parse_args()

    # 图片不存在就自动生成 —— 这样离线拉起时不依赖启动顺序，
    # 也避免 /tmp 被清理导致找不到图。
    if not os.path.isfile(args.image):
        print(f"图片不存在，自动生成: {args.image}")
        os.makedirs(os.path.dirname(args.image), exist_ok=True)
        try:
            sys.path.insert(0, HERE)
            from make_test_image import make_image

            cv2.imwrite(args.image, make_image())
        except Exception as exc:  # noqa: BLE001
            print(f"自动生成失败: {exc}，请手动运行 tools/make_test_image.py", file=sys.stderr)
            return 2

    img = cv2.imread(args.image)
    if img is None:
        print(f"无法读取图片: {args.image}", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("publish_test_image")
    bridge = CvBridge()
    pub = node.create_publisher(Image, args.topic, 5)

    msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    msg.header.frame_id = args.frame_id

    period = 1.0 / max(args.rate, 0.1)
    node.get_logger().info(f"以 {args.rate} Hz 发布 {args.image} 到 {args.topic}")
    try:
        while rclpy.ok():
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
