#!/usr/bin/env python3
"""假底盘：把 /cmd_vel 积分成 /odom，让定位与任务状态机在无硬件时也能跑。

用途：离线联调。真实车上应该由底盘/全场定位板/雷达提供 /odom，本脚本只用于
在没有硬件的情况下验证"速度指令 → 位姿变化 → 状态机推进"这条闭环。

约定与真车一致：
  position.x/y 为平面坐标，**position.z 携带 yaw**；
  twist 里放车体系速度。
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FakeBoard(Node):
    def __init__(self, x: float, y: float, yaw: float) -> None:
        super().__init__("fake_board")
        self.x, self.y, self.yaw = x, y, yaw
        self.vx = self.vy = self.wz = 0.0
        self.last = time.time()

        self.pub = self.create_publisher(Odometry, "/odom", 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz
        self.get_logger().info(f"假底盘启动于 ({x:.2f}, {y:.2f}, {yaw:.2f})")

    def on_cmd(self, msg: Twist) -> None:
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z

    def tick(self) -> None:
        now = time.time()
        dt = now - self.last
        self.last = now
        if dt <= 0 or dt > 0.5:
            return

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (self.vx * c - self.vy * s) * dt
        self.y += (self.vx * s + self.vy * c) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.wz * dt), math.cos(self.yaw + self.wz * dt))

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.yaw          # 本队约定：yaw 塞在 z
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.wz
        self.pub.publish(msg)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=1.0)
    ap.add_argument("--y", type=float, default=1.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    args = ap.parse_args()

    rclpy.init()
    node = FakeBoard(args.x, args.y, args.yaw)
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
