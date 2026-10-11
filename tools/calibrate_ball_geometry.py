#!/usr/bin/env python3
"""Solve camera mounting from a checkerboard with a MEASURED base-frame pose.

Input geometry JSON: base_from_board (4x4), square_m, base_from_cloud (4x4),
intake_position_base_m (3). Board origin = first inner corner, X along columns,
Y along rows. --write stores candidates but does NOT confirm ranging accuracy.
"""
import argparse
import json
import sys
from pathlib import Path
import cv2
import numpy as np
import yaml

WS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS / "src/rb_perception"))
from rb_perception.precision_ball import rigid
from calibrate_camera import find_corners, atomic_backup_write


def solve(points_base, pixels, K, D):
    ok, r, t = cv2.solvePnP(np.asarray(points_base, dtype=np.float64),
                          np.asarray(pixels, dtype=np.float64), K, D)
    if not ok:
        raise ValueError("solvePnP failed")
    camera_from_base = np.eye(4)
    camera_from_base[:3, :3] = cv2.Rodrigues(r)[0]
    camera_from_base[:3, 3] = t.ravel()
    projected = cv2.projectPoints(points_base, r, t, K, D)[0].reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((projected-pixels.reshape(-1, 2))**2, axis=1))))
    if not np.isfinite(rms) or rms > .7:
        raise ValueError(f"extrinsic reprojection RMS {rms:.3f} px exceeds 0.7 px")
    return rigid(np.linalg.inv(camera_from_base)), rms


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--camera-role", choices=["front", "rear"], required=True)
    ap.add_argument("--image", required=True)
    ap.add_argument("--geometry", required=True)
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()
    path = WS / ("src/rb_perception/config/perception.yaml" if args.camera_role == "front"
                 else "src/rb_perception/config/perception_rear.yaml")
    cfg = yaml.safe_load(path.read_text())
    c = cfg["camera"]
    if not c.get("calibrated"):
        raise ValueError("complete intrinsic calibration first")
    image = cv2.imread(args.image)
    if image is None or [image.shape[1], image.shape[0]] != c["calibration_size"]:
        raise ValueError("missing image or incorrect calibration resolution")
    corners = find_corners(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (args.cols, args.rows))
    if corners is None:
        raise ValueError("checkerboard not detected")
    # Annotated first corner lets the operator verify ordering against the marked board.
    preview = image.copy()
    for i in (0, args.cols-1, args.cols*(args.rows-1)):
        pixel = tuple(int(v) for v in corners[i, 0])
        cv2.circle(preview, pixel, 7, (0, 0, 255), 2)
        cv2.putText(preview, str(i), pixel, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    preview_path = Path(args.image).with_suffix(".corners.png")
    cv2.imwrite(str(preview_path), preview)
    g = json.loads(Path(args.geometry).read_text())
    position = np.asarray(g['intake_position_base_m'], dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("invalid measured intake centre")
    board = rigid(g["base_from_board"])
    lidar = rigid(g["base_from_cloud"])
    square = float(g["square_m"])
    if not .005 <= square <= .1:
        raise ValueError("square_m must be measured in metres")
    points = np.zeros((args.cols*args.rows, 3), dtype=np.float64)
    points[:, :2] = np.mgrid[:args.cols, :args.rows].T.reshape(-1, 2)*square
    points = points @ board[:3, :3].T + board[:3, 3]
    K = np.array([[c['fx'], 0, c['cx']], [0, c['fy'], c['cy']], [0, 0, 1]], dtype=float)
    camera, rms = solve(points, corners, K, np.array(c['distortion'], dtype=float))
    result = {"base_from_camera": camera.tolist(), "base_from_cloud": lidar.tolist(),
              "rms_px": rms, "corner_preview": str(preview_path), "confirmed": False}
    result_path = Path(args.image).with_suffix(".extrinsics.json")
    result_path.write_text(json.dumps(result, indent=2))
    print(f"Candidate saved: {result_path}; check corner ordering and measured board pose")
    if args.write:
        cfg['perception']['lidar_ball'].update(base_from_camera=camera.tolist(),
                                              base_from_cloud=lidar.tolist(),
                                              extrinsics_confirmed=False)
        atomic_backup_write(path, yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
        approach_path = WS / "src/rb_perception/config/ball_approach.yaml"
        approach = yaml.safe_load(approach_path.read_text())
        approach.update(intake_position_base_m=position.tolist(), intake_confirmed=False)
        atomic_backup_write(approach_path, yaml.safe_dump(approach, sort_keys=False, allow_unicode=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
