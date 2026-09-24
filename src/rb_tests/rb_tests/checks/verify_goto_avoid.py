#!/usr/bin/env python3
"""离线验证新加的两个功能：goto_pose 服务 + 视觉势场避障。

不需要任何硬件：直接给 rb_mission 喂假位姿、假检测，断言 /cmd_vel 的行为。

用法：
    ros2 run rb_tests verify_goto_avoid.py

断言：
    1. /rb_mission/goto_pose 服务存在且接受合法请求；
    2. 从 (1,1) goto (5,1) 时，机器人向前（linear.x > 0）且无侧向分量；
    3. 正前方 0.5m 出现障碍物（IDLE 模式下任何球/obstacle 都躲）时，
       linear.x 显著下降（被排斥速度抵消）；
    4. 障碍物在正右方时，产生向左的侧向速度（linear.y < 0）。
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from rb_tests.ws import WS  # 工作空间定位（原来靠 __file__ 推算，搬包后会错）
TEST_DOMAIN = "78"


def main() -> int:
    os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from std_msgs.msg import Header
    from rb_msgs.msg import Detection, DetectionArray
    from rb_msgs.srv import GotoPose

    (WS / "test_artifacts").mkdir(parents=True, exist_ok=True)
    logf = open(WS / "test_artifacts" / "goto_avoid.log", "w")  # noqa: SIM115
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = TEST_DOMAIN
    proc = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/humble/setup.bash && "
                       f"source {WS}/install/setup.bash && "
                       "exec ros2 run rb_mission mission_node"],
        cwd=str(WS), stdout=logf, stderr=subprocess.STDOUT,
        env=env, start_new_session=True)

    rclpy.init()
    node = rclpy.create_node("verify_goto_avoid")
    cmds: list[Twist] = []
    node.create_subscription(Twist, "/cmd_vel", lambda m: cmds.append(m), 20)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/localization/pose", 10)
    det_pub = node.create_publisher(DetectionArray, "/perception/detections", 10)
    cli = node.create_client(GotoPose, "/rb_mission/goto_pose")

    def spin_for(sec: float) -> None:
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

    def pub_pose(x: float, y: float, yaw: float) -> None:
        m = PoseWithCovarianceStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = "field"
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.position.z = yaw  # 本队约定 yaw 塞 z
        pose_pub.publish(m)

    def pub_det(label: str, bearing: float, dist: float) -> None:
        m = DetectionArray()
        m.header = Header()
        m.header.stamp = node.get_clock().now().to_msg()
        d = Detection()
        d.label = label
        d.confidence = 0.9
        d.bearing_rad = bearing
        d.distance_m = dist
        m.detections = [d]
        det_pub.publish(m)

    def latest_cmd() -> Twist | None:
        return cmds[-1] if cmds else None

    fail = ""
    try:
        # 等服务
        spin_for(3.0)
        if not cli.wait_for_service(timeout_sec=6.0):
            fail = "找不到 /rb_mission/goto_pose 服务"
            return _finish(node, proc, fail)

        # 1) 服务接受请求
        pub_pose(1.0, 1.0, 0.0)
        spin_for(0.3)
        req = GotoPose.Request()
        req.x, req.y, req.yaw, req.align_yaw = 5.0, 1.0, 0.0, False
        fut = cli.call_async(req)
        spin_for(2.0)
        if not (fut.done() and fut.result() is not None and fut.result().accepted):
            fail = "goto_pose 服务未接受请求"
            return _finish(node, proc, fail)

        # 2) 无障碍：向前走，无侧向
        cmds.clear()
        for _ in range(30):
            pub_pose(1.0, 1.0, 0.0)
            spin_for(0.1)
        c = latest_cmd()
        if c is None:
            fail = "goto 后收不到 /cmd_vel"
            return _finish(node, proc, fail)
        if c.linear.x <= 0.05:
            fail = f"goto (5,1) 应向前，实际 linear.x={c.linear.x:.3f}"
            return _finish(node, proc, fail)
        if abs(c.linear.y) > 0.05:
            fail = f"无侧向障碍时 linear.y 应≈0，实际 {c.linear.y:.3f}"
            return _finish(node, proc, fail)
        base_vx = c.linear.x

        # 3) 正前方障碍 → 前进被抵消
        cmds.clear()
        for _ in range(30):
            pub_pose(1.0, 1.0, 0.0)
            pub_det("ball_basketball", 0.0, 0.5)  # IDLE 模式：任何球都躲
            spin_for(0.1)
        c = latest_cmd()
        if c is None or not (c.linear.x < base_vx - 0.10):
            fail = f"正前方 0.5m 障碍未削弱前进速度（base={base_vx:.2f}, now={c.linear.x if c else '无':.2f})"
            return _finish(node, proc, fail)

        # 4) 右侧障碍 → 向左躲（linear.y < 0）
        cmds.clear()
        for _ in range(30):
            pub_pose(1.0, 1.0, 0.0)
            pub_det("obstacle", math.pi / 2, 0.5)  # 右正左负，正右方
            spin_for(0.1)
        c = latest_cmd()
        if c is None or not (c.linear.y < -0.05):
            fail = f"正右方障碍应产生向左速度，实际 linear.y={c.linear.y if c else '无':.3f}"
            return _finish(node, proc, fail)

        return _finish(node, proc, "", base_vx)
    finally:
        pass


def _finish(node, proc, fail: str, base_vx: float | None = None) -> int:
    import rclpy
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

    if fail:
        print(f"\033[31m✗ 失败\033[0m  {fail}")
        print("  日志: test_artifacts/goto_avoid.log")
        return 1
    print("\033[32m✓ 通过\033[0m  goto_pose 服务 + 视觉势场避障均正常"
          + (f"（无障前进 vx={base_vx:.2f}，前障/侧障均正确避让）" if base_vx else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
