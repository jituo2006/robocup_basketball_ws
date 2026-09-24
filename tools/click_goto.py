#!/usr/bin/env python3
"""电脑端"点地图"操控：在场地俯视图上点一个点，机器人自主走过去。

这是给 rb_mission 的 /rb_mission/goto_pose 服务包的一个图形前端。
机器人端（NUC）跑着 rb_mission，本脚本跑在**你的电脑**上，通过 DDS 通信。

用法
----
    cd ~/robocup_basketball_ws
    source install/setup.bash
    python3 tools/click_goto.py [--config src/rb_mission/config/mission.yaml]

交互
----
    左键单击   走到该点（保持当前朝向）
    右键单击   走到该点并对准 yaw=0（即朝向场地 +x 方向，通常朝篮筐）
    按 c       取消当前 GOTO（切回 IDLE 并停车）
    按 q / 关窗 退出（**不会**停车：机器人会继续走完最后一个目标点）

没有 matplotlib 时，也能用命令行直接操控（效果一样）：
    ros2 service call /rb_mission/goto_pose rb_msgs/srv/GotoPose \
        "{x: 5.0, y: 3.0, yaw: 0.0, align_yaw: false}"
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rb_msgs.srv import GotoPose, SetMission


def _yaw_from(msg: PoseWithCovarianceStamped) -> float:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # 兼容本队两种约定：yaw 塞在 position.z，或写四元数
    if abs(math.hypot(q.x, q.y, q.z)) < 1e-6:
        return p.z
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ClickGotoNode(Node):
    def __init__(self, config_path: str) -> None:
        super().__init__("click_goto")
        self.cfg: dict = {}
        self.field: dict = {}
        self.home = (0.0, 0.0)
        self.hoop = (0.0, 0.0)
        self.pass_zone: list[list[float]] = []
        self.load_config(config_path)

        self.pose: tuple[float, float, float] | None = None
        self.goal: tuple[float, float] | None = None
        self.last_result = ""

        self.create_subscription(PoseWithCovarianceStamped, "/localization/pose", self.on_pose, 10)
        self.goto_client = self.create_client(GotoPose, "/rb_mission/goto_pose")
        self.mission_client = self.create_client(SetMission, "/rb_mission/set_mission")

        # ⚠️ 千万不能写成 self.executor = ...：rclpy 的 Node.executor 是**弱引用** property，
        #    赋进去的执行器会立刻被 GC 回收，随后 self.executor 返回 None。
        #    必须用普通属性保存强引用。
        self._exec = MultiThreadedExecutor(num_threads=2)
        self._exec.add_node(self)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()
        self.get_logger().info("click_goto 就绪，等待 /localization/pose …")

    def load_config(self, path: str) -> None:
        if not path or not os.path.isfile(path):
            self.get_logger().warn(f"配置文件不存在: {path}，用默认场地尺寸")
            self.field = {"length_m": 14.0, "width_m": 7.5}
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                self.cfg = yaml.safe_load(fh) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"读配置失败: {exc}")
            self.cfg = {}
        self.field = self.cfg.get("field", {}) or {}
        h = self.field.get("home", {}) or {}
        self.home = (float(h.get("x", 0.0)), float(h.get("y", 0.0)))
        hp = self.field.get("hoop", {}) or {}
        self.hoop = (float(hp.get("x", 0.0)), float(hp.get("y", 0.0)))
        self.pass_zone = (self.field.get("pass_zone", {}) or {}).get("polygon", []) or []

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, _yaw_from(msg))

    def send_goal(self, x: float, y: float, yaw: float, align: bool) -> str:
        if not self.goto_client.wait_for_service(timeout_sec=0.5):
            return "❌ /rb_mission/goto_pose 服务未就绪（机器人端 rb_mission 没起？）"
        req = GotoPose.Request()
        req.x, req.y, req.yaw, req.align_yaw = float(x), float(y), float(yaw), bool(align)
        future = self.goto_client.call_async(req)

        def _done(fut) -> None:
            try:
                self.last_result = fut.result().message
                self.get_logger().info(f"GOTO 结果: {self.last_result}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"GOTO 调用失败: {exc}")

        future.add_done_callback(_done)
        return f"已发送 GOTO ({x:.2f}, {y:.2f})" + (f" yaw={math.degrees(yaw):.0f}°" if align else "")

    def cancel(self) -> str:
        if not self.mission_client.wait_for_service(timeout_sec=0.5):
            return "❌ set_mission 服务未就绪"
        req = SetMission.Request()
        req.mission = "IDLE"
        self.mission_client.call_async(req)
        return "已发送取消（IDLE）"


def run_map(node: ClickGotoNode) -> int:
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        node.get_logger().warn(f"matplotlib 不可用（{exc}），改用命令行 service call 操控")
        return 0

    fl = float(node.field.get("length_m", 14.0))
    fw = float(node.field.get("width_m", 7.5))

    fig, ax = plt.subplots(figsize=(9, 5))
    fig.canvas.manager.set_window_title("RoboCup 篮球 · 点地图操控 (click_goto)")
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, fl + 0.5)
    ax.set_ylim(-0.5, fw + 0.5)
    ax.set_xlabel("场地 x (m)")
    ax.set_ylabel("场地 y (m)")
    ax.grid(True, alpha=0.25)

    # 静态要素：场地边界、home、篮筐、传球区
    ax.add_patch(plt.Rectangle((0, 0), fl, fw, fill=False, edgecolor="black", lw=1.5))
    ax.plot(*node.home, "g*", ms=16, label="home 出发区")
    ax.plot(*node.hoop, "rD", ms=12, label="迷你篮筐")
    if node.pass_zone:
        xs = [p[0] for p in node.pass_zone] + [node.pass_zone[0][0]]
        ys = [p[1] for p in node.pass_zone] + [node.pass_zone[0][1]]
        ax.plot(xs, ys, "b--", lw=1.5, label="传球区")

    robot_arrow = None
    goal_marker = None
    goal_heading = None

    def redraw():
        nonlocal robot_arrow, goal_marker, goal_heading
        if robot_arrow is not None:
            robot_arrow.remove()
        if goal_marker is not None:
            goal_marker.remove()
        if goal_heading is not None:
            goal_heading.remove()

        if node.pose is not None:
            x, y, yaw = node.pose
            robot_arrow = ax.quiver(x, y, math.cos(yaw), math.sin(yaw),
                                    color="g", scale=8, width=0.02, zorder=5,
                                    label="机器人(位置+朝向)")
        if node.goal is not None:
            gx, gy = node.goal
            goal_marker = ax.plot(gx, gy, "rx", ms=14, mew=3, label="目标点")[0]
        ax.legend(loc="upper right", fontsize=8)
        if node.pose is not None:
            x, y, yaw = node.pose
            fig.canvas.manager.set_window_title(
                f"click_goto · 当前位置 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")
        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes is not ax or event.xdata is None or event.ydata is None:
            return
        x, y = float(event.xdata), float(event.ydata)
        align = (event.button == 3)  # 右键：对准 yaw=0
        node.goal = (x, y)
        msg = node.send_goal(x, y, 0.0, align)
        node.get_logger().info(msg)
        ax.set_title(msg, fontsize=9)
        redraw()

    def on_key(event):
        if event.key in ("q", "Q"):
            plt.close(fig)
        elif event.key in ("c", "C"):
            node.get_logger().info(node.cancel())
            node.goal = None
            ax.set_title("已取消 GOTO", fontsize=9)
            redraw()

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    redraw()

    ax.set_title("左键=走到该点 · 右键=走到并对准 +x · c=取消 · q=退出", fontsize=9)
    plt.show()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="RoboCup 篮球 点地图操控")
    parser.add_argument("--config", default="",
                        help="mission.yaml 路径（读场地尺寸；默认取 rb_mission 包的 share 配置）")
    args = parser.parse_args()

    config = args.config
    if not config:
        try:
            from ament_index_python.packages import get_package_share_directory
            config = os.path.join(get_package_share_directory("rb_mission"),
                                  "config", "mission.yaml")
        except Exception:  # noqa: BLE001
            config = ""

    rclpy.init(args=sys.argv[1:])
    node = ClickGotoNode(config)
    try:
        return run_map(node)
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
