#!/usr/bin/env python3
"""Stationary camera/LiDAR observation with candidate geometry; no actuator publisher.

No checkerboard or ruler input required. This is NOT accuracy acceptance and never
changes calibration flags. Camera models may contain old or estimated intrinsics.
"""
import argparse
import collections
import json
import sys
import time
from pathlib import Path
import numpy as np
import yaml

WS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS / 'src/rb_perception'))
from rb_perception.detectors import CameraModel, Detection
from rb_perception.precision_ball import diagnose_geometry, rigid


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=15.)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if not 1 <= args.seconds <= 60:
        raise ValueError('seconds must be 1..60')
    import rclpy
    from rb_msgs.msg import DetectionArray
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from rclpy.qos import qos_profile_sensor_data
    configs = {role: yaml.safe_load((WS / ('src/rb_perception/config/perception.yaml'
               if role == 'front' else 'src/rb_perception/config/perception_rear.yaml')).read_text())
               for role in ('front', 'rear')}
    tip = np.asarray(yaml.safe_load((WS / 'src/rb_perception/config/ball_approach.yaml').read_text())
                     ['intake_position_base_m'], dtype=float)
    if tip.shape != (3,) or not np.isfinite(tip).all():
        raise ValueError('missing candidate rail-tip midpoint')
    for cfg in configs.values():
        rigid(cfg['perception']['lidar_ball']['base_from_camera'])
        rigid(cfg['perception']['lidar_ball']['base_from_cloud'])
    rclpy.init()
    node = rclpy.create_node('observe_ball_geometry')
    cloud, cloud_stamp, frame = None, 0., ''
    counts = collections.Counter()
    reasons = {r: collections.Counter() for r in configs}
    samples = {r: [] for r in configs}
    stamps = {r: 0. for r in configs}
    last = {}

    def on_cloud(msg):
        nonlocal cloud, cloud_stamp, frame
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= cloud_stamp:
            return
        cloud_stamp, frame = stamp, msg.header.frame_id
        cloud = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        counts['cloud'] += 1

    def on_detection(role, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= stamps[role]:
            return
        stamps[role] = stamp; counts[role] += 1
        cfg = configs[role]
        cam = CameraModel.from_config(cfg['camera'], msg.img_width, msg.img_height)
        dets = [Detection(d.label, d.confidence, (d.x_min, d.y_min, d.x_max, d.y_max))
                for d in msg.detections]
        now = node.get_clock().now().nanoseconds * 1e-9
        _, report = diagnose_geometry(dets, cloud, cloud_stamp, frame, stamp, now, cam,
                                     cfg['camera'], cfg['perception']['lidar_ball'])
        reasons[role][report['reason']] += 1
        for target in report['targets']:
            target['rail_tip_to_centre_m'] = float(np.linalg.norm(np.asarray(target['centre_base_m']) - tip))
            samples[role].append(target)
        last[role] = dict(report, detections=[{'label': d.label, 'confidence': d.confidence,
                                             'bbox': list(d.bbox)} for d in dets],
                          image_age_s=now-stamp, cloud_age_s=now-cloud_stamp)

    node.create_subscription(PointCloud2, '/cloud_registered_body', on_cloud, qos_profile_sensor_data)
    for role in configs:
        topic = '/perception/detections' if role == 'front' else '/perception_rear/detections'
        node.create_subscription(DetectionArray, topic,
                                 lambda msg, role=role: on_detection(role, msg), qos_profile_sensor_data)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node(); rclpy.shutdown()
    result = dict(diagnostic_only=True, accuracy_verified=False, counts=dict(counts),
                  reasons={r: dict(v) for r, v in reasons.items()}, last=last, samples=samples)
    Path(args.out).write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps({k: v for k, v in result.items() if k != 'samples'}, ensure_ascii=False))
    return 0 if counts['cloud'] and all(counts[r] for r in configs) else 1


if __name__ == '__main__':
    sys.exit(main())
