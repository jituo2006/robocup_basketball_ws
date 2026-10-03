#!/usr/bin/env python3
"""测量感知/定位链路的**反馈延迟**：话题的 header.stamp 比"现在"老多少。

为什么需要它：导航"冲过目标点"和"地图位置严重滞后"都是反馈延迟的症状。
超调 ≈ 延迟 × 接近速度。中位 100ms 以内算健康。

⚠️⚠️ 测量本身的坑（实测踩到）：
   同时订阅**大点云**（/cloud_registered、/livox/lidar）会让本脚本自己处理不过来，
   读出来的数字被自己抬高 5~10 倍（实测：只订阅 /Odometry 是 26ms，
   同时订阅 5 个话题变成 141ms）。所以默认**只订阅轻量话题**（定位输出 + 里程计），
   这才是控制实际用的延迟。要点云延迟用 --with-cloud，并记住那是**上界**。

用法：
    rbws
    python3 tools/check_latency.py                 # 默认：定位/里程计（准）
    python3 tools/check_latency.py --with-cloud    # 加测点云（读数偏大，是上界）
    python3 tools/check_latency.py --watch         # 每秒刷新

退出码：0 = 健康；1 = 延迟过大。
"""

from __future__ import annotations

import argparse
import sys
import time

_HEALTHY_MS = 100.0
_BAD_MS = 400.0


def _plan(with_cloud: bool):
    """返回 [(话题, 显示名)]；点云是重负载，读数会被本脚本抬高。"""
    plan = [
        ("/Odometry", "FAST-LIO 里程计（定位的输入）"),
        ("/localization/pose", "定位输出（任务实际用的）"),
    ]
    if with_cloud:
        plan.append(("/cloud_registered", "点云（RViz 地图；读数是上界）"))
    return plan


def _make_sub(node, topic, store):
    """按话题名选类型（只有点云才 import 大消息类型）。"""
    if topic == "/Odometry":
        from nav_msgs.msg import Odometry
        typ = Odometry
    elif topic == "/localization/pose":
        from geometry_msgs.msg import PoseWithCovarianceStamped
        typ = PoseWithCovarianceStamped
    else:
        from sensor_msgs.msg import PointCloud2
        typ = PointCloud2

    def cb(msg) -> None:
        now = node.get_clock().now().nanoseconds * 1e-9
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        store.setdefault(topic, []).append((now - st) * 1000.0)

    node.create_subscription(typ, topic, cb, 10)


def measure(seconds: float, with_cloud: bool) -> dict[str, float]:
    import rclpy

    rclpy.init()
    node = rclpy.create_node("check_latency")
    store: dict[str, list[float]] = {}
    for topic, _name in _plan(with_cloud):
        try:
            _make_sub(node, topic, store)
        except Exception:  # noqa: BLE001 - 类型不可用就跳过
            pass

    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    rclpy.shutdown()

    out: dict[str, float] = {}
    for topic, vals in store.items():
        vals.sort()
        if vals:
            out[topic] = vals[len(vals) // 2]
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="测反馈链路延迟")
    ap.add_argument("--seconds", type=float, default=10.0, help="采样秒数（默认 10）")
    ap.add_argument("--with-cloud", action="store_true",
                    help="加测点云（读数会被本脚本抬高，只当上界看）")
    ap.add_argument("--watch", action="store_true", help="每秒刷新")
    args = ap.parse_args(argv)

    rc = 0
    while True:
        print(f"⏱  延迟测量（{args.seconds:.0f}s 采样）")
        res = measure(args.seconds, args.with_cloud)
        names = dict(_plan(args.with_cloud))
        if not res:
            print("  ❌ 没收到任何话题 —— 整套软件在跑吗？")
            rc = 1
        else:
            rc = 0
            for topic in sorted(res, key=lambda k: -res[k]):
                ms = res[topic]
                if ms <= _HEALTHY_MS:
                    mark, color = "✓", "\033[32m"
                elif ms <= _BAD_MS:
                    mark, color = "△", "\033[33m"
                else:
                    mark, color = "✗", "\033[31m"
                    rc = 1
                print(f"  {color}{mark}\033[0m {topic:20s} {ms:8.1f} ms   {names.get(topic, '')}")
            if rc == 0:
                print("  ✅ 延迟健康（<100ms）")
            else:
                print("  ⚠️ 延迟过大 → 看 docs/07 §5.4：")
                print("     ① CPU 被吃光（uptime 看 load、ps 看谁在吃）")
                print("     ② FAST-LIO 跟不上（ros2 topic hz /cloud_registered 应 ~10Hz）")
        if not args.watch:
            break
        time.sleep(1.0)
        print()
    return rc


if __name__ == "__main__":
    sys.exit(main())
