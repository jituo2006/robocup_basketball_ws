"""连续转场：有限前视、制动包络、限加速度和终点新鲜反馈确认。寻球不使用这里。"""

import math
import time

DEFAULTS = dict(
    accel_mps2=0.6,
    decel_mps2=0.7,
    brake_mps2=0.3,
    lookahead_m=0.12,
    corner_mps=0.18,
    corner_blend_m=0.08,
    terminal_radius_m=0.40,
    terminal_kp=2.0,
    velocity_damping=0.45,
    prediction_max_s=0.15,
    arrival_mps=0.05,
    arrival_frames=2,
    correction_mps=0.12,
)


def parameters(options=None):
    if options is not None and not isinstance(options, dict):
        raise ValueError("transit_control 须为字典")
    cfg = {**DEFAULTS, **(options or {})}
    if set(cfg) != set(DEFAULTS):
        raise ValueError("未知 transit_control 参数")
    for key, v in cfg.items():
        if (
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
            or v <= 0
        ):
            raise ValueError("transit_control 参数须为正有限数: " + key)
    if not isinstance(cfg["arrival_frames"], int):
        raise ValueError("arrival_frames 须为正整数")
    if cfg["corner_blend_m"] > cfg["lookahead_m"]:
        raise ValueError("corner_blend_m 不能大于 lookahead_m")
    if cfg["prediction_max_s"] > 0.5:
        raise ValueError("定位外推最多允许 0.5 秒")
    return cfg


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def limit(x, y, maximum):
    length = math.hypot(x, y)
    scale = min(1.0, maximum / length) if length else 1.0
    return x * scale, y * scale


def follow_path(
    r, targets, tolerance, options=None, heading=None, stop_at_goal=True, emit=None
):
    success = False
    try:
        cfg = parameters(options)
        speed = r.args.speed
        if not all(math.isfinite(v) and v > 0 for v in (tolerance, speed)):
            raise ValueError("转场速度和容差须为正有限数")
        points = []
        for p in targets:
            if len(p) != 2 or not all(math.isfinite(v) for v in p):
                raise ValueError("转场点须为有限 [x,y]")
            if not points or math.dist(p, points[-1]) > 1e-8:
                points.append(tuple(p))
        if not points:
            success = True
            return True
        r.require_fresh_pose()
        start = r.pose[:2]
        heading = r.pose[2] if heading is None else heading
        if not math.isfinite(heading):
            raise ValueError("转场朝向无效")
        # 原路掉头必须停车，不能把180度尖点当作可高速通过的拐弯。
        chain = [start] + points
        for k in range(1, len(chain) - 1):
            ax, ay = chain[k][0] - chain[k - 1][0], chain[k][1] - chain[k - 1][1]
            bx, by = chain[k + 1][0] - chain[k][0], chain[k + 1][1] - chain[k][1]
            den = math.hypot(ax, ay) * math.hypot(bx, by)
            if den and (ax * bx + ay * by) / den < -0.85:
                if not follow_path(r, points[:k], tolerance, cfg, heading, True, emit):
                    return False
                success = follow_path(
                    r, points[k:], tolerance, cfg, heading, stop_at_goal, emit
                )
                return success
        i = 0
        vx = vy = cx = cy = 0.0
        last_pose = tuple(start)
        last_stamp = getattr(r, "pose_stamp", time.monotonic())
        previous = time.monotonic()
        last_emit = -math.inf
        stable = 0
        stable_stamp = None
        while True:
            r.require_fresh_pose()
            now = time.monotonic()
            dt = max(0.001, min(0.1, now - previous))
            previous = now
            x, y, yaw = r.pose
            stamp = getattr(r, "pose_stamp", now)
            if stamp > last_stamp:
                elapsed = stamp - last_stamp
                alpha = 1.0 - math.exp(-elapsed / 0.08)
                vx += alpha * ((x - last_pose[0]) / elapsed - vx)
                vy += alpha * ((y - last_pose[1]) / elapsed - vy)
                last_pose, last_stamp = (x, y), stamp
            age = 0.0
            if hasattr(r, "node"):
                age = max(
                    0.0,
                    min(
                        cfg["prediction_max_s"],
                        r.node.get_clock().now().nanoseconds * 1e-9 - stamp,
                    ),
                )
            px, py = x + vx * age, y + vy * age
            while i < len(points) - 1:
                b = points[i]
                dx, dy = b[0] - start[0], b[1] - start[1]
                length = math.hypot(dx, dy)
                along = (
                    ((px - start[0]) * dx + (py - start[1]) * dy) / length
                    if length
                    else 0.0
                )
                lateral = (
                    abs((px - start[0]) * dy - (py - start[1]) * dx) / length
                    if length
                    else math.dist((px, py), b)
                )
                if math.dist((px, py), b) <= cfg["corner_blend_m"] or (
                    along >= length and lateral <= cfg["corner_blend_m"]
                ):
                    if emit:
                        emit(
                            "transit_waypoint",
                            index=i,
                            target=list(b),
                            pose=list(r.pose),
                        )
                    start = b
                    i += 1
                else:
                    break
            target = points[i]
            remaining = math.dist((px, py), target) + sum(
                math.dist(points[j], points[j + 1]) for j in range(i, len(points) - 1)
            )
            final_error = math.dist((x, y), points[-1])
            measured = math.hypot(vx, vy)
            if i == len(points) - 1:
                if stop_at_goal:
                    # 终点不能越线放行；两帧位置和测量速度同时满足，不用固定秒数停留。
                    if final_error <= tolerance and measured <= cfg["arrival_mps"]:
                        if stamp != stable_stamp:
                            stable += 1
                            stable_stamp = stamp
                        if stable >= cfg["arrival_frames"]:
                            if emit:
                                emit(
                                    "transit_arrival",
                                    target=list(points[-1]),
                                    pose=list(r.pose),
                                    error_m=final_error,
                                    measured_mps=measured,
                                )
                            success = True
                            return True
                    else:
                        stable = 0
                        stable_stamp = None
                elif final_error <= max(tolerance, cfg["corner_blend_m"]):
                    success = True
                    return True
            dx, dy = target[0] - start[0], target[1] - start[1]
            length = math.hypot(dx, dy)
            along = (
                max(
                    0.0,
                    min(length, ((px - start[0]) * dx + (py - start[1]) * dy) / length),
                )
                if length
                else 0.0
            )
            ahead = along + cfg["lookahead_m"]
            j = i
            a, b = start, target
            while ahead > length and j < len(points) - 1:
                ahead -= length
                a = b
                j += 1
                b = points[j]
                length = math.dist(a, b)
            fraction = min(1.0, ahead / length) if length else 1.0
            gx, gy = a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1])
            ex, ey = gx - px, gy - py
            direction = math.hypot(ex, ey)
            cap = speed
            if stop_at_goal:
                cap = min(
                    cap,
                    math.sqrt(
                        2 * cfg["brake_mps2"] * max(0.0, remaining - tolerance * 0.5)
                    ),
                )
            to_corner = math.dist((px, py), target)
            for k in range(i, len(points) - 1):
                a = start if k == i else points[k - 1]
                b, c = points[k], points[k + 1]
                ux, uy = b[0] - a[0], b[1] - a[1]
                wx, wy = c[0] - b[0], c[1] - b[1]
                den = math.hypot(ux, uy) * math.hypot(wx, wy)
                if den and (ux * wx + uy * wy) / den < 0.94:
                    cap = min(
                        cap,
                        math.sqrt(
                            cfg["corner_mps"] ** 2
                            + 2
                            * cfg["brake_mps2"]
                            * max(0.0, to_corner - cfg["lookahead_m"])
                        ),
                    )
                to_corner += math.dist(b, c)
            terminal = (
                i == len(points) - 1
                and remaining < cfg["terminal_radius_m"]
                and stop_at_goal
            )
            if terminal:
                ex, ey = points[-1][0] - px, points[-1][1] - py
                ux = cfg["terminal_kp"] * ex - cfg["velocity_damping"] * vx
                uy = cfg["terminal_kp"] * ey - cfg["velocity_damping"] * vy
                if ux * ex + uy * ey < 0 or final_error < 0.12:
                    cap = min(cap, cfg["correction_mps"])
                ux, uy = limit(ux, uy, cap)
                v = math.hypot(ux, uy)
                if (
                    final_error > tolerance
                    and ux * ex + uy * ey > 0
                    and 0 < v < min(0.05, cap)
                ):
                    ux, uy = limit(ux / v * 0.05, uy / v * 0.05, cap)
                if final_error <= tolerance:
                    ux = uy = 0.0
            else:
                ux, uy = (
                    (cap * ex / direction, cap * ey / direction)
                    if direction
                    else (0.0, 0.0)
                )
            acceleration = (
                cfg["decel_mps2"]
                if math.hypot(ux, uy) < math.hypot(cx, cy)
                else cfg["accel_mps2"]
            )
            dx, dy = limit(ux - cx, uy - cy, acceleration * dt)
            cx += dx
            cy += dy
            tw = r._Twist()
            cs, sn = math.cos(yaw), math.sin(yaw)
            tw.linear.x = cx * cs + cy * sn
            tw.linear.y = -cx * sn + cy * cs
            error = wrap(heading - yaw)
            rate = r.yaw_rate if math.isfinite(r.yaw_rate) else 0.0
            tw.angular.z = max(
                -r.args.max_ang,
                min(r.args.max_ang, r.args.kp_yaw * error - r.args.kd_yaw * rate),
            )
            if abs(error) > r.args.route_heading_tol:
                tw.linear.x = tw.linear.y = 0.0
                cx = cy = 0.0
            r.publish_command(tw)
            if emit and now - last_emit >= 0.25:
                emit(
                    "transit_progress",
                    index=i,
                    target=list(target),
                    final_target=list(points[-1]),
                    pose=list(r.pose),
                    remaining_m=remaining,
                    final_error_m=final_error,
                    measured_mps=measured,
                    command_mps=math.hypot(tw.linear.x, tw.linear.y),
                    pose_age_s=(
                        max(0.0, r.node.get_clock().now().nanoseconds * 1e-9 - stamp)
                        if hasattr(r, "node")
                        else 0.0
                    ),
                    prediction_s=age,
                    terminal=terminal,
                )
                last_emit = now
            r.spin(0.02)
    finally:
        if stop_at_goal or not success:
            r.stop()


def run_segment(
    runner, tx, ty, tolerance, stop_at_goal=True, heading=None, options=None, emit=None
):
    return follow_path(
        runner, [(tx, ty)], tolerance, options, heading, stop_at_goal, emit
    )
