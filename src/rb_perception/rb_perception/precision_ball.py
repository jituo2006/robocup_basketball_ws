"""Calibrated camera/LiDAR ball ranging. No ROS or actuator dependency.

Transforms map sensor coordinates TO base_link (x forward, y left, z up).
Camera coordinates follow OpenCV (x right, y down, z forward).
Uncertain/stale measurements never become authoritative ranges.
"""
from dataclasses import replace
import copy
import math

import cv2
import numpy as np

BALL_RADII = {"ball_basketball": 0.12, "ball_volleyball": 0.105}


def rigid(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not math.isclose(np.linalg.det(t[:3, :3]), 1, abs_tol=1e-5)):
        raise ValueError("invalid rigid transform")
    return t


def transform(points, t):
    return np.asarray(points) @ t[:3, :3].T + t[:3, 3]


def fresh(stamp, now, timeout):
    return math.isfinite(stamp) and stamp > 0 and 0 <= now - stamp <= timeout


def fit_sphere(points, radius, cfg):
    """Fit surface samples, rejecting flat or poorly constrained patches."""
    p = np.asarray(points, dtype=float)
    if len(p) < int(cfg.get("min_points", 12)):
        return None
    a = 2 * (p[1:] - p[0])
    b = np.sum(p[1:] ** 2, axis=1) - np.sum(p[0] ** 2)
    centre, _, rank, _ = np.linalg.lstsq(a, b, rcond=None)
    if rank < 3:
        return None
    # Fixed measured radius stabilises fitting to partial sphere surfaces.
    for _ in range(12):
        delta = centre - p
        lengths = np.linalg.norm(delta, axis=1)
        if np.min(lengths) < 1e-6:
            return None
        j = delta / lengths[:, None]
        error = lengths - radius
        step = np.linalg.lstsq(j, error, rcond=None)[0]
        if np.linalg.norm(step) > radius:
            return None
        centre -= step
        if np.linalg.norm(step) < 1e-6:
            break
    lengths = np.linalg.norm(centre - p, axis=1)
    rms = float(np.sqrt(np.mean((lengths - radius) ** 2)))
    j = (centre - p) / lengths[:, None]
    singular = np.linalg.svd(j, compute_uv=False)
    if singular[-1] < 1e-8:
        return None
    condition = float(singular[0] / singular[-1])
    # Includes a sensor noise floor; fit residual alone understates precision.
    noise = max(rms, float(cfg.get("sensor_noise_m", .01)))
    uncertainty = noise / float(singular[-1])
    if (rms > float(cfg.get("max_residual_m", .012))
            or condition > float(cfg.get("max_condition", 8))
            or uncertainty > float(cfg.get("max_uncertainty_m", .02))
            or np.max(np.ptp(p, axis=0)) < radius * .6):
        return None
    return centre, rms, uncertainty


def fuse_precision(detections, cloud, cloud_stamp, cloud_frame, image_stamp, now, cam,
                   camera_cfg, cfg):
    """Return display detections and base-frame range records for control.

    Detection.distance_m retains its existing optical-depth contract. The new
    precision topic explicitly carries base_link centres and ball radius.
    """
    out = [replace(d, distance_m=float("nan")) if d.label in BALL_RADII else d
           for d in detections]
    result = {"camera_stamp": image_stamp, "cloud_stamp": cloud_stamp,
              "valid": False, "reason": "calibration_required", "targets": []}
    if not (camera_cfg.get("calibrated", False) and cfg.get("extrinsics_confirmed", False)):
        return out, result
    if [cam.width, cam.height] != list(camera_cfg.get("calibration_size", [])):
        result["reason"] = "calibration_resolution_mismatch"
        return out, result
    if (cloud is None or not fresh(image_stamp, now, cfg.get("max_age_s", .3))
            or not fresh(cloud_stamp, now, cfg.get("max_age_s", .3))
            or abs(cloud_stamp - image_stamp) > cfg.get("max_sync_s", .10)):
        result["reason"] = "stale_or_unsynchronised"
        return out, result
    if cloud_frame != cfg.get("cloud_frame", "body"):
        result["reason"] = "wrong_cloud_frame"
        return out, result
    try:
        lidar_t = rigid(cfg["base_from_cloud"])
        camera_t = rigid(cfg["base_from_camera"])
    except (KeyError, ValueError, TypeError):
        result["reason"] = "invalid_extrinsics"
        return out, result
    base = transform(cloud, lidar_t)
    camera = transform(base, np.linalg.inv(camera_t))
    mask = np.isfinite(camera).all(axis=1) & (camera[:, 2] > .1)
    mask &= np.linalg.norm(camera, axis=1) <= cfg.get("max_range_m", 6.)
    base, camera = base[mask], camera[mask]
    if len(base) == 0:
        result["reason"] = "no_cloud_points"
        return out, result
    pixels = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), cam._K(),
                               np.asarray(cam.distortion) if cam.distortion else None)[0].reshape(-1, 2)
    # Spatial components prevent a wall behind the image box entering the fit.
    from .lidar_ball import _grid_clusters
    result["reason"] = "no_unambiguous_sphere"
    for i, d in enumerate(detections):
        if d.label not in BALL_RADII or d.confidence < cfg.get("min_confidence", .6):
            continue
        bounds = np.asarray(d.bbox) * [cam.width, cam.height, cam.width, cam.height]
        inside = ((pixels[:, 0] >= bounds[0]) & (pixels[:, 0] <= bounds[2])
                  & (pixels[:, 1] >= bounds[1]) & (pixels[:, 1] <= bounds[3]))
        radius = float(cfg.get("radii_m", BALL_RADII).get(d.label, BALL_RADII[d.label]))
        if not .05 <= radius <= .2:
            continue
        candidates = []
        points = base[inside]
        for cluster in _grid_clusters(points, float(cfg.get("cluster_tolerance_m", .065)),
                                      int(cfg.get("min_points", 12))):
            fitted = fit_sphere(cluster, radius, cfg)
            if fitted is None:
                continue
            centre, rms, uncertainty = fitted
            # Whole centre must project into its own box as well.
            cc = transform(centre[None, :], np.linalg.inv(camera_t))[0]
            if cc[2] <= .1:
                continue
            uv = cv2.projectPoints(cc[None, :], np.zeros(3), np.zeros(3), cam._K(),
                                  np.asarray(cam.distortion) if cam.distortion else None)[0].reshape(2)
            if not (bounds[0] <= uv[0] <= bounds[2] and bounds[1] <= uv[1] <= bounds[3]):
                continue
            candidates.append((centre, cc, rms, uncertainty, len(cluster)))
        if len(candidates) != 1:
            continue
        centre, cc, rms, uncertainty, n = candidates[0]
        measured = replace(d, distance_m=float(cc[2]), bearing_rad=math.atan2(cc[0], cc[2]))
        measured.source = "camera_lidar"
        out[i] = measured
        result["targets"].append({"label": d.label, "confidence": float(d.confidence),
                                  "centre_base_m": centre.tolist(), "radius_m": radius,
                                  "residual_m": rms, "uncertainty_m": uncertainty, "points": n})
    # Overlapping detector boxes can assign one sphere twice: refuse duplicates.
    targets = result["targets"]
    duplicates = {i for i, a in enumerate(targets) for j, b in enumerate(targets)
                  if i != j and np.linalg.norm(np.array(a["centre_base_m"]) - b["centre_base_m"]) < .10}
    if duplicates:
        result["targets"] = []
        out = [replace(d, distance_m=float("nan")) if d.label in BALL_RADII else d for d in detections]
        result["reason"] = "duplicate_association"
    if result["targets"]:
        result.update(valid=True, reason="ok")
    return out, result


def diagnose_geometry(detections, cloud, cloud_stamp, cloud_frame, image_stamp, now, cam,
                      camera_cfg, cfg):
    """Observe measured/CAD candidates without authorising motion or claiming calibration.

    Uses the production projection and sphere fitter. Unknown optical calibration
    remains an explicit limitation, even if a fit passes the point-quality gates.
    Returned control distances stay NaN and report.valid is ALWAYS False.
    """
    candidate_camera, candidate_lidar = copy.deepcopy(camera_cfg), copy.deepcopy(cfg)
    candidate_camera['calibrated'] = True
    candidate_lidar['extrinsics_confirmed'] = True
    _, report = fuse_precision(detections, cloud, cloud_stamp, cloud_frame, image_stamp,
                               now, cam, candidate_camera, candidate_lidar)
    report.update(diagnostic_only=True, calibration_verified=False,
                  fit_quality_passed=bool(report['valid']),
                  reason='diagnostic:' + report['reason'], valid=False)
    out = [replace(d, distance_m=float('nan')) if d.label in BALL_RADII else d
           for d in detections]
    return out, report
