"""Horizontal shot range and measured launcher-table interpolation (no ROS)."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ShotSolution:
    distance_m: float
    speed: int
    angle: int
    source: str


def finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} 必须为有限数值")
    return value


def shot_solution(cfg, pose, hoop, bearing, camera_depth):
    """Camera depth is optical-axis Z=fx*diameter/pixels, not slant range.

    Offsets are in base frame (x forward, y left). Camera bearing is right-positive.
    Table angles are MCU units, not assumed degrees. Ball-specific table is required.
    """
    if not cfg.get("map_confirmed", False):
        raise ValueError("篮筐地图坐标尚未确认")
    x, y, yaw = [finite(v, "位姿") for v in pose]
    hx, hy = finite(hoop["x"], "篮筐 x"), finite(hoop["y"], "篮筐 y")
    dx, dy = hx - x, hy - y
    bx = math.cos(yaw) * dx + math.sin(yaw) * dy
    by = -math.sin(yaw) * dx + math.cos(yaw) * dy
    lx = finite(cfg.get("launch_x_m", 0), "发射口 x")
    ly = finite(cfg.get("launch_y_m", 0), "发射口 y")
    cx = finite(cfg.get("camera_x_m", 0), "相机 x")
    cy = finite(cfg.get("camera_y_m", 0), "相机 y")
    cyaw = finite(cfg.get("camera_yaw_rad", 0), "相机朝向")
    distance = math.hypot(bx - lx, by - ly)
    if distance <= 0:
        raise ValueError("目标距离必须大于零")
    aim_tolerance = finite(cfg.get("aim_tolerance_rad", math.pi), "发射朝向容差")
    if aim_tolerance <= 0 or abs(math.atan2(by - ly, bx - lx)) > aim_tolerance:
        raise ValueError("出球方向尚未对准篮筐")
    bearing = finite(bearing, "相机方位角")
    expected = math.atan2(by - cy, bx - cx) - cyaw
    error = math.atan2(math.sin(expected + bearing), math.cos(expected + bearing))
    tolerance = finite(cfg.get("bearing_tolerance_rad", 0.15), "方位容差")
    if tolerance <= 0 or abs(error) > tolerance:
        raise ValueError("相机目标方位与地图篮筐不一致")
    source = "map+camera_bearing"
    if math.isfinite(float(camera_depth)) and float(camera_depth) > 0:
        z = float(camera_depth)
        if abs(bearing) >= math.pi / 2:
            raise ValueError("相机方位角超出前向视场")
        camera_y = -z * math.tan(bearing)
        vx = cx + math.cos(cyaw) * z - math.sin(cyaw) * camera_y
        vy = cy + math.sin(cyaw) * z + math.cos(cyaw) * camera_y
        max_error = finite(cfg.get("range_tolerance_m", 0.5), "测距容差")
        if max_error <= 0 or math.hypot(vx - bx, vy - by) > max_error:
            raise ValueError("相机测距与地图篮筐位置不一致")
        weight = finite(cfg.get("vision_weight", 0.3), "视觉权重")
        if not 0 <= weight <= 1:
            raise ValueError("视觉权重必须在 0..1")
        distance = (1 - weight) * distance + weight * math.hypot(vx - lx, vy - ly)
        source = "map+camera_range"
    elif cfg.get("require_camera_range", False):
        raise ValueError("缺少有效相机测距")
    else:
        # Without depth, use map range only to compensate camera/launcher offsets.
        radius = math.hypot(bx - cx, by - cy)
        vx = cx + radius * math.cos(cyaw - bearing)
        vy = cy + radius * math.sin(cyaw - bearing)
    if abs(math.atan2(vy - ly, vx - lx)) > aim_tolerance:
        raise ValueError("视觉目标尚未对准出球方向")

    rows = cfg.get("table", [])
    if len(rows) < 2:
        raise ValueError("至少需要两条实测投篮标定记录")
    table = []
    for row in rows:
        d = finite(row["distance_m"], "标定距离")
        speed = finite(row["speed"], "标定转速")
        angle = finite(row["angle"], "标定角度指令")
        if d <= 0 or not 1 <= speed <= 65535 or not 1 <= angle <= 65535:
            raise ValueError("标定距离/转速/角度指令越界")
        if table and d <= table[-1][0]:
            raise ValueError("标定距离必须严格递增")
        table.append((d, speed, angle))
    if not table[0][0] <= distance <= table[-1][0]:
        raise ValueError(f"距离 {distance:.2f}m 超出实测射程表范围")
    for low, high in zip(table, table[1:]):
        if distance <= high[0]:
            fraction = (distance - low[0]) / (high[0] - low[0])
            return ShotSolution(distance, round(low[1] + fraction * (high[1] - low[1])),
                                round(low[2] + fraction * (high[2] - low[2])), source)
    raise ValueError("无法计算投篮参数")
