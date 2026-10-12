#!/usr/bin/env python3
"""测雷达链路的消息年龄 —— 判断云点图/里程计"不顺畅"的量化依据。

为什么要测这个：找球的精确阶段（precision_ball_approach / precision_ball.py）
要求 /Odometry 与 /cloud_registered_body 的
    * 自身年龄     ≤ max_age_s  (0.3s)
    * 两者时间差   ≤ max_sync_s (0.10s)
超过就判 stale_or_unsynchronised ✗，脚本只发停车、车不动。

现场曾经实测到【中位 1.379s】的积压（FAST-LIO 订阅队列写死 20 +
map_en:true 不停发累积地图），改完两处后降到 0.024s ✓。

用法:
    python3 tools/check_lidar_latency.py [--seconds 8]
"""
from __future__ import annotations

import argparse
import math
import statistics as st
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="测雷达消息年龄（新鲜度）")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--age-max", type=float, default=0.30, help="判据上限（与 max_age_s 一致）")
    ap.add_argument("--sync-max", type=float, default=0.10, help="云/里程计时间差上限")
    args = ap.parse_args(argv)

    rclpy.init()
    n = Node("lidar_latency_probe")
    samples: dict[str, list[tuple[float, float]]] = {"Odometry": [], "cloud": []}

    def mk(key):
        def cb(m):
            now = n.get_clock().now().nanoseconds * 1e-9
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            samples[key].append((now, stamp))
        return cb

    n.create_subscription(Odometry, "/Odometry", mk("Odometry"), qos_profile_sensor_data)
    n.create_subscription(PointCloud2, "/cloud_registered_body", mk("cloud"),
                          qos_profile_sensor_data)

    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(n, timeout_sec=0.05)

    ok = True
    for key in ("Odometry", "cloud"):
        v = samples[key]
        if not v:
            print(f"  {key:9} ✗ 没收到任何消息 —— 雷达链路没通"
                  f"（先跑 bash tools/lidar_net_check.sh）")
            ok = False
            continue
        ages = sorted(now - stamp for now, stamp in v)
        med = ages[len(ages) // 2]
        good = med <= args.age_max
        ok &= good
        print(f"  {key:9} {len(v):4} 条  年龄 min={ages[0]:.3f}s "
              f"中位={med:.3f}s max={ages[-1]:.3f}s  "
              f"[≤{args.age_max:g}s → {'✓' if good else '✗ 太旧，找球会拒停'}]")

    # 云和里程计的时间戳差（找球的 max_sync_s 判据）
    if samples["Odometry"] and samples["cloud"]:
        o = [s for _, s in samples["Odometry"]]
        c = [s for _, s in samples["cloud"]]
        diffs = [min(abs(a - b) for b in c) for a in o]
        med = st.median(diffs)
        good = med <= args.sync_max
        ok &= good
        print(f"  {'同步':9} 云↔里程计时间差 中位={med:.3f}s "
              f"[≤{args.sync_max:g}s → {'✓' if good else '✗ 不同步，找球会拒停'}]")

    n.destroy_node()
    rclpy.shutdown()
    print("  " + ("✅ 雷达新鲜度达标 ✓" if ok else
                  "❌ 不达标 —— 检查 tools/apply_lidar_patch.sh 是否已应用并重启 FAST-LIO"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
