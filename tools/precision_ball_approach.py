#!/usr/bin/env python3
"""Front approach -> relative 180 degree turn -> NEW rear camera/LiDAR servo.

No chassis enable/disable, driver configuration, process killing or bringup.
Default invocation only validates config. --execute deliberately sends cmd_vel.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

WS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WS / "src/rb_perception"))
from rb_perception.ball_approach import Arrival, command, geometry, wrap
from rb_perception.precision_ball import fresh, rigid


def validate(cfg):
    geometry(cfg)
    transforms = []
    for role in ("front", "rear"):
        p = yaml.safe_load((WS / cfg[role + "_perception_config"]).read_text())
        lidar = p["perception"]["lidar_ball"]
        if not (p["camera"].get("calibrated") and lidar.get("extrinsics_confirmed")
                and lidar.get("enabled") and lidar.get("mode") == "precision"):
            raise ValueError(f"{role} camera/LiDAR calibration incomplete")
        rigid(lidar["base_from_camera"])
        transforms.append(rigid(lidar["base_from_cloud"]))
    if not np.allclose(transforms[0], transforms[1], atol=1e-6):
        raise ValueError("front/rear must share one IMU-to-base calibration")
    return transforms[0]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(WS / "src/rb_perception/config/ball_approach.yaml"))
    ap.add_argument("--execute", action="store_true")
    args = ap.parse_args(argv)
    cfg = yaml.safe_load(Path(args.config).read_text())
    try:
        base_from_cloud = validate(cfg)
    except (ValueError, KeyError, TypeError) as exc:
        print(f"REFUSED: {exc}; see docs/ball_precision_setup.md")
        return 1
    if not args.execute:
        print("Calibration configuration passed; no command publisher created.")
        return 0
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node("precision_ball_approach")
    pub = node.create_publisher(Twist, "/cmd_vel", 1)
    readings = {"front": None, "rear": None}
    pose = None
    last_log = 0.
    gate = Arrival(cfg)
    phase = "front"
    stage_start = time.monotonic()
    turn_goal = None
    predicted_ball_world = None
    tracked_world = None
    last_observation = 0.
    done = False

    def on_range(role, msg):
        try:
            readings[role] = json.loads(msg.data)
        except (ValueError, TypeError):
            readings[role] = None

    def on_pose(msg):
        nonlocal pose
        q = msg.pose.pose.orientation
        # Full IMU->base installation transform, not just raw IMU yaw.
        quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1) > .02:
            pose = None
            return
        x, y, z, w = quat / np.linalg.norm(quat)
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        world_from_imu = np.eye(4)
        world_from_imu[:3, :3] = rotation
        p = msg.pose.pose.position
        world_from_imu[:3, 3] = [p.x, p.y, p.z]
        world_from_base = world_from_imu @ np.linalg.inv(base_from_cloud)
        v, a = msg.twist.twist.linear, msg.twist.twist.angular
        angular_base = base_from_cloud[:3, :3] @ np.array([a.x, a.y, a.z])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if pose is not None and stamp <= pose[0]:
            return
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        wz = float(angular_base[2])
        # FAST-LIO builds sometimes leave Odometry.twist all zero. Derive
        # motion from successive source-stamped poses before accepting standstill.
        if pose is None or stamp - pose[0] > .5:
            speed = float('inf')
        else:
            dt = stamp - pose[0]
            speed = max(speed, float(np.linalg.norm(world_from_base[:3, 3] - pose[1][:3, 3]))/dt)
            previous_yaw = math.atan2(pose[1][1, 0], pose[1][0, 0])
            current_yaw = math.atan2(world_from_base[1, 0], world_from_base[0, 0])
            wz = max(abs(wz), abs(wrap(current_yaw - previous_yaw))/dt)
        pose = (stamp, world_from_base, speed, wz)

    for role in readings:
        node.create_subscription(String, cfg[role + "_range_topic"],
                                 lambda msg, role=role: on_range(role, msg), qos_profile_sensor_data)
    node.create_subscription(Odometry, cfg["odom_topic"], on_pose, qos_profile_sensor_data)

    def send(values=(0., 0., 0.)):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = values
        pub.publish(msg)

    def status(message):
        nonlocal last_log
        if time.monotonic() - last_log >= 5:
            last_log = time.monotonic()
            node.get_logger().info(f"{phase}: {message}")

    try:
        # Reject competing control publishers; this script never stops them.
        rclpy.spin_once(node, timeout_sec=1.)
        if node.count_publishers("/cmd_vel") != 1:
            raise RuntimeError("another /cmd_vel publisher exists; stop its mission before testing")
        last_ownership_check = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.025)
            if time.monotonic() - last_ownership_check >= 1.:
                last_ownership_check = time.monotonic()
                if node.count_publishers("/cmd_vel") != 1:
                    raise RuntimeError("a competing /cmd_vel publisher appeared")
            now = node.get_clock().now().nanoseconds * 1e-9
            elapsed = time.monotonic() - stage_start
            if elapsed > cfg.get("stage_timeout_s", 45.):
                raise RuntimeError(f"{phase}: timeout without confirmed arrival")
            if pose is None or not fresh(pose[0], now, .3):
                send(); gate.reset()
                status("waiting for fresh radar odometry; stopped")
                continue
            stamp_pose, world_from_base, speed, wz = pose
            yaw = math.atan2(world_from_base[1, 0], world_from_base[0, 0])
            if phase == "turn":
                err = wrap(turn_goal - yaw)
                if abs(err) < .035:
                    send()
                    if speed < .03 and abs(wz) < .05:
                        phase, stage_start = "rear", time.monotonic()
                        readings["rear"] = None
                        gate.reset()
                        last_observation = 0.
                        node.get_logger().info("rear: require a new rear camera/LiDAR observation")
                else:
                    send((0., 0., float(np.clip(err * 1.2, -.18, .18))))
                continue
            observation = readings[phase]
            valid = (observation and observation.get("valid")
                     and fresh(observation.get("camera_stamp", 0), now, .3)
                     and fresh(observation.get("cloud_stamp", 0), now, .3))
            targets = ([t for t in observation.get("targets", [])
                        if t.get("label") == cfg["ball_label"]] if valid else [])
            if len(targets) != 1:
                send(); gate.reset()
                reason = observation.get("reason", "stale_or_ambiguous_target") if observation else "no_range_data"
                status(f"waiting for one valid {phase} target ({reason}); stopped")
                continue
            target = targets[0]
            centre = np.asarray(target["centre_base_m"])
            current_world = (world_from_base @ np.r_[centre, 1.])[:3]
            reference = predicted_ball_world if phase == "rear" else tracked_world
            if reference is not None and np.linalg.norm(current_world - reference) > .3:
                send(); gate.reset()
                status("target differs from tracked ball; stopped")
                continue  # Never silently switch to a different basketball after turning.
            # Process each source pair once; repeated packets do not extend dwell.
            obs_stamp = min(observation["camera_stamp"], observation["cloud_stamp"])
            if obs_stamp <= last_observation:
                continue
            last_observation = obs_stamp
            tracked_world = current_world
            send(command(target, cfg, rear=phase == "rear"))
            if gate.update(target, obs_stamp, now, speed, wz, rear=phase == "rear"):
                send()
                if phase == "rear":
                    node.get_logger().info("arrived: rail-tip-to-ball-centre distance 0.60 m, stable")
                    done = True
                    break
                predicted_ball_world = current_world
                turn_goal = wrap(yaw + math.pi)
                phase, stage_start = "turn", time.monotonic()
                gate.reset()
                node.get_logger().info("turn: front stage settled, relative 180 degree turn")
            from rb_perception.ball_approach import errors
            e, lateral, heading, _ = errors(target, cfg, rear=phase == "rear")
            status(f"centre_distance_error={e:.3f} lateral={lateral:.3f} heading={heading:.3f}")
    except (KeyboardInterrupt, RuntimeError, ValueError, KeyError, TypeError) as exc:
        node.get_logger().error(str(exc))
    finally:
        for _ in range(5):
            send()
            rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if done else 1


if __name__ == "__main__":
    sys.exit(main())
