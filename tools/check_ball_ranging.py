#!/usr/bin/env python3
"""Stationary ranging acceptance. READ ONLY: no command publisher or actuator.

After intrinsic and extrinsic candidates exist, --camera-role front/rear samples
live data using the SAME production fitter (without changing confirmation flags).
--expected-base X Y Z must be an independently measured ball centre in base_link.
Use several distances/positions. No single trial automatically enables control.
"""
import argparse
import copy
import json
import sys
import time
from pathlib import Path
import numpy as np
import yaml

WS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS/'src/rb_perception'))
from rb_perception.detectors import CameraModel, Detection
from rb_perception.precision_ball import fuse_precision, rigid


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--camera-role', required=True, choices=['front', 'rear'])
    ap.add_argument('--expected-base', nargs=3, type=float, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=20.)
    args = ap.parse_args()
    import rclpy
    from rb_msgs.msg import DetectionArray
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from rclpy.qos import qos_profile_sensor_data
    role = args.camera_role
    path = WS / ('src/rb_perception/config/perception.yaml' if role == 'front'
                 else 'src/rb_perception/config/perception_rear.yaml')
    cfg = yaml.safe_load(path.read_text())
    lidar = copy.deepcopy(cfg['perception']['lidar_ball'])
    rigid(lidar['base_from_camera']); rigid(lidar['base_from_cloud'])
    lidar['extrinsics_confirmed'] = True  # Only in this read-only verifier's memory.
    rclpy.init()
    node = rclpy.create_node('check_ball_ranging')
    cloud = None
    cloud_stamp = 0.
    frame = ''
    samples = []
    last_pair = 0.

    def on_cloud(msg):
        nonlocal cloud, cloud_stamp, frame
        cloud = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        frame = msg.header.frame_id

    def on_image(msg):
        nonlocal last_pair
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        cam = CameraModel.from_config(cfg['camera'], msg.img_width, msg.img_height)
        dets = [Detection(d.label, d.confidence, (d.x_min, d.y_min, d.x_max, d.y_max))
                for d in msg.detections]
        _, result = fuse_precision(dets, cloud, cloud_stamp, frame, stamp,
                                    node.get_clock().now().nanoseconds*1e-9, cam, cfg['camera'], lidar)
        pair = min(stamp, cloud_stamp)
        targets = [t for t in result['targets'] if t['label']=='ball_basketball']
        if len(targets) == 1 and pair > last_pair:
            last_pair = pair
            samples.append(targets[0]['centre_base_m'])

    topic = '/perception/detections' if role == 'front' else '/perception_rear/detections'
    node.create_subscription(PointCloud2, lidar['cloud_topic'], on_cloud, qos_profile_sensor_data)
    node.create_subscription(DetectionArray, topic, on_image, qos_profile_sensor_data)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node(); rclpy.shutdown()
    report = {'role': role, 'expected_base_m': args.expected_base, 'samples': samples,
              'accepted': False}
    if len(samples) >= 20:
        p = np.asarray(samples)
        error = np.linalg.norm(p - args.expected_base, axis=1)
        report.update(p95_error_m=float(np.percentile(error, 95)),
                      span_m=np.ptp(p, axis=0).tolist(),
                      accepted=bool(np.percentile(error, 95) <= .02))
    Path(args.out).write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}))
    return 0 if report['accepted'] else 1


if __name__=='__main__':
    sys.exit(main())
