#!/usr/bin/env python3
"""定位柱坐标测量：把视觉检测反算成场地坐标，直接填进 landmarks。

为什么需要它
------------
`localization.yaml` 的 `landmarks` 要的是定位柱在**场地坐标系**下的 (x, y)，
但那个坐标系是"开机时车摆在哪"定义的，用卷尺直接量很不方便。
本工具换个思路：**让车自己当一把尺子。**

    柱子位置 = 车的位置 + 距离 × (朝向 − 相机方位角 的方向)

其中车的位姿来自 `/localization/pose`，距离与方位角来自 `/perception/detections`
的 pillar 检测（距离用柱子的实际宽度做单目测距）。

用法
----
    cd ~/robocup_basketball_ws && source install/setup.bash

    # ① 先跑，看看测出来的坐标稳不稳（不动配置）
    python3 tools/measure_landmark.py

    # ② 满意了再写进配置（会自动备份原文件）
    python3 tools/measure_landmark.py --write

    # 换配置文件 / 调采样时间 / 换标签
    python3 tools/measure_landmark.py --config src/rb_localization/config/localization.yaml
    python3 tools/measure_landmark.py --seconds 8
    python3 tools/measure_landmark.py --label pillar

前提
----
1. 雷达 + FAST-LIO + rb_localization 都在跑（需要 /localization/pose）
2. 相机 + rb_perception 在跑，柱子在**视野内**且被检出为 pillar
3. `pillar` 检测器的 `real_width_m` 要等于柱子的**真实宽度**（默认 0.20m），
   否则距离会按比例偏 —— 这是本工具唯一的系统误差来源。

⚠️ 使用时**车不要动**：车一动，"车当尺子"的基准就变了。
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import statistics
import sys
import time
from pathlib import Path

try:
    import rclpy
    import yaml
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rb_msgs.msg import DetectionArray
    # ⚠️ 复用 rb_localization 里的换算，**不要在这里重写一遍公式** ——
    #    bearing 的符号约定是全项目最容易写反的地方（见 docs/02 §2.5），
    #    ekf2d.camera_bearing_to_world 有单测锁住，这里直接调它。
    from rb_localization.ekf2d import camera_bearing_to_world
except ImportError as exc:  # pragma: no cover - 环境没 source 时的友好提示
    print(f"导入失败：{exc}\n请先 source install/setup.bash（或跑 rbws）", file=sys.stderr)
    raise SystemExit(2)


def landmark_xy(pose: tuple[float, float, float], bearing_rad: float,
                distance_m: float, camera_yaw_offset: float = 0.0) -> tuple[float, float]:
    """由车位姿 + 一次检测，反算地标在场地坐标系下的 (x, y)。

    这是纯函数（无 ROS 依赖），便于单测；坐标换算复用
    `camera_bearing_to_world`，与 rb_localization 保持同一套符号约定。

    车当尺子：地标 = 车的位置 + 距离 × 方向，方向角 = yaw + 相机偏置 − bearing。
    """
    x, y, yaw = pose
    world_angle = camera_bearing_to_world(yaw, camera_yaw_offset, bearing_rad)
    return x + distance_m * math.cos(world_angle), y + distance_m * math.sin(world_angle)


class LandmarkMeasurer(Node):
    def __init__(self, label: str, camera_yaw_offset: float) -> None:
        super().__init__("measure_landmark")
        self.label = label
        self.camera_yaw_offset = camera_yaw_offset
        self.pose: tuple[float, float, float] | None = None
        self.samples: list[tuple[float, float, float, float]] = []  # (lx, ly, dist, conf)
        self.create_subscription(
            PoseWithCovarianceStamped, "/localization/pose", self._on_pose, 10)
        self.create_subscription(
            DetectionArray, "/perception/detections", self._on_dets, 5)
        self.get_logger().info(
            f"等待 /localization/pose 与 /perception/detections 里的 '{label}' 检测 …")

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw 优先取四元数；本队老约定把 yaw 塞在 position.z 也兼容
        if abs(math.hypot(q.x, q.y)) < 1e-9 and abs(q.z) < 1e-9:
            yaw = float(p.z)
        else:
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (float(p.x), float(p.y), yaw)

    def _on_dets(self, msg: DetectionArray) -> None:
        if self.pose is None:
            return
        for d in msg.detections:
            if d.label != self.label:
                continue
            dist = float(d.distance_m)
            bearing = float(d.bearing_rad)
            if dist != dist or dist <= 0.0 or bearing != bearing:
                continue
            # ⚠️ 符号：bearing_rad 是"右正左负"，场地系是"y 朝左、逆时针为正"。
            #    换算复用 rb_localization 的 camera_bearing_to_world（有单测锁住），
            #    不要在这里重写公式。
            lx, ly = landmark_xy(self.pose, bearing, dist, self.camera_yaw_offset)
            self.samples.append((lx, ly, dist, float(d.confidence)))

    def estimate(self) -> dict | None:
        if not self.samples:
            return None
        lxs = [s[0] for s in self.samples]
        lys = [s[1] for s in self.samples]
        dists = [s[2] for s in self.samples]
        confs = [s[3] for s in self.samples]
        n = len(self.samples)
        return {
            "n": n,
            "x": statistics.median(lxs),
            "y": statistics.median(lys),
            "x_spread": max(lxs) - min(lxs),
            "y_spread": max(lys) - min(lys),
            "dist": statistics.median(dists),
            "conf": statistics.fmean(confs),
            "pose": self.pose,
        }


def _write_landmarks(cfg_path: Path, points: list[dict]) -> None:
    """把 landmarks 写回 YAML。用逐行替换而不是 yaml.dump —— 保住文件里的注释。"""
    text = cfg_path.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    inserted = False
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("landmarks:"):
            # 吃掉原来的列表项（以 "- " 开头的续行）
            j = i + 1
            while j < len(lines) and (lines[j].startswith("  -") or
                                      lines[j].startswith("    ") or
                                      lines[j].startswith("  #") or
                                      lines[j].strip() == ""):
                if lines[j].lstrip().startswith("-") or lines[j].startswith("  #"):
                    j += 1
                else:
                    break
            out.append("landmarks:\n")
            for p in points:
                out.append(f"  - {{x: {p['x']:.3f}, y: {p['y']:.3f}}}\n")
            if not points:
                out.append("  []\n")
            inserted = True
            i = j
            continue
        out.append(line)
        i += 1
    if not inserted:
        # 文件里没有 landmarks 段，就追加
        out.append("\nlandmarks:\n")
        for p in points:
            out.append(f"  - {{x: {p['x']:.3f}, y: {p['y']:.3f}}}\n")
        if not points:
            out.append("  []\n")
    cfg_path.write_text("".join(out), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="把定位柱的视觉检测反算成场地坐标，填入 landmarks",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="src/rb_localization/config/localization.yaml",
                    help="localization.yaml 路径")
    ap.add_argument("--label", default="pillar", help="要测的检测标签（默认 pillar）")
    ap.add_argument("--seconds", type=float, default=6.0, help="采样时长（默认 6s）")
    ap.add_argument("--write", action="store_true", help="把结果写回配置（会先备份）")
    ap.add_argument("--min-samples", type=int, default=10, help="少于这个样本数就拒绝写入")
    ap.add_argument("--max-spread", type=float, default=0.15,
                    help="采样抖动超过这个值（米）就拒绝写入，默认 0.15")
    args = ap.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.exists():
        print(f"找不到配置文件：{cfg_path}", file=sys.stderr)
        return 2

    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    cam_off = float((cfg.get("sources") or {}).get("camera_yaw_offset_rad", 0.0))
    existing = cfg.get("landmarks") or []
    if existing:
        print(f"⚠️ 配置里已有 {len(existing)} 个地标，--write 会**替换**它们。")

    print(f"采样 {args.seconds:.1f}s（标签 '{args.label}'，相机偏置 {cam_off:+.3f} rad）…")
    rclpy.init()
    node = LandmarkMeasurer(args.label, cam_off)
    t0 = time.time()
    while time.time() - t0 < args.seconds and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

    est = node.estimate()
    pose = node.pose
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if pose is None:
        print("❌ 没收到 /localization/pose —— 定位起了吗？", file=sys.stderr)
        return 1
    print(f"车位姿 = ({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):+.1f}°)")
    if est is None:
        print(f"❌ 没检出 '{args.label}' —— 柱子在视野里吗？"
              f"用 python3 tools/view_camera.py 看看，"
              f"或先跑 tools/calibrate_color.py --label {args.label} 标颜色。",
              file=sys.stderr)
        return 1

    print(f"\n样本 {est['n']} 个，平均置信度 {est['conf']:.2f}，"
          f"中位距离 {est['dist']:.2f} m")
    print(f"  → 地标坐标 = ({est['x']:.3f}, {est['y']:.3f})")
    print(f"     抖动 x±{est['x_spread'] / 2:.3f} m  y±{est['y_spread'] / 2:.3f} m")

    ok = True
    if est["n"] < args.min_samples:
        print(f"❌ 样本太少（{est['n']} < {args.min_samples}）")
        ok = False
    if max(est["x_spread"], est["y_spread"]) > args.max_spread:
        print(f"❌ 抖动太大（{max(est['x_spread'], est['y_spread']):.3f} m "
              f"> {args.max_spread}）—— 车是不是动了？或者柱子时隐时现？")
        ok = False

    if not args.write:
        print("\n（未写入。确认无误后加 --write）")
        print("\n复制到 localization.yaml：")
        print("landmarks:")
        print(f"  - {{x: {est['x']:.3f}, y: {est['y']:.3f}}}")
        return 0

    if not ok:
        print("\n❌ 质量不达标，**拒绝写入**（避免把噪声写进配置文件）", file=sys.stderr)
        return 1

    backup = cfg_path.with_suffix(cfg_path.suffix + f".bak_{time.strftime('%Y%m%d_%H%M%S')}")
    shutil.copy2(cfg_path, backup)
    points = [{"x": est["x"], "y": est["y"]}] if not existing else (
        existing + [{"x": est["x"], "y": est["y"]}])
    _write_landmarks(cfg_path, points)
    print(f"\n✅ 已写入 {cfg_path}（共 {len(points)} 个地标；备份 {backup.name}）")
    print("   重启定位生效：ros2 launch rb_localization localization.launch.py")
    print("   验证日志应出现：定位柱更新: 地标(...) 观测方位=... 累计=N")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
