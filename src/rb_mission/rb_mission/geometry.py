"""场地几何工具：区域判定与运动原语用的数学。"""

from __future__ import annotations

import math
from typing import Sequence


def point_in_polygon(px: float, py: float, polygon: Sequence[Sequence[float]]) -> bool:
    """射线法。polygon 为 [[x,y], ...]，可以是凸或凹多边形。"""
    if len(polygon) < 3:
        return False
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i][0], polygon[i][1]
        xj, yj = polygon[j][0], polygon[j][1]
        if (yi > py) != (yj > py):
            x_cross = (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi
            if px < x_cross:
                inside = not inside
        j = i
    return inside


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def goto_command(cur_x: float, cur_y: float, cur_yaw: float,
                 tgt_x: float, tgt_y: float,
                 kp_lin: float, kp_yaw: float,
                 max_lin: float, max_ang: float,
                 face_travel: bool = True) -> tuple[float, float, float, float]:
    """朝目标点走的 P 控制器（场地坐标系 → 车体系速度）。

    返回 (vx, vy, wz, distance)。vx/vy/wz 已经是车体系。
    """
    dx = tgt_x - cur_x
    dy = tgt_y - cur_y
    dist = math.hypot(dx, dy)

    # 世界系期望速度方向
    vwx = kp_lin * dx
    vwy = kp_lin * dy
    speed = math.hypot(vwx, vwy)
    if speed > max_lin and speed > 1e-9:
        vwx *= max_lin / speed
        vwy *= max_lin / speed

    # 转到车体系
    c, s = math.cos(cur_yaw), math.sin(cur_yaw)
    vx = vwx * c + vwy * s
    vy = -vwx * s + vwy * c

    wz = 0.0
    if face_travel and dist > 0.15:
        desired_yaw = math.atan2(dy, dx)
        wz = clamp(kp_yaw * wrap_pi(desired_yaw - cur_yaw), -max_ang, max_ang)

    return vx, vy, wz, dist


def face_bearing_command(cur_yaw: float, target_bearing_body: float,
                         kp_yaw: float, max_ang: float) -> tuple[float, float]:
    """把车头转向某个视觉目标。返回 (wz, 剩余角度误差)。

    target_bearing_body 是目标相对**车体前进方向**的方位角（右正左负）。
    """
    err = wrap_pi(target_bearing_body)
    return clamp(kp_yaw * err, -max_ang, max_ang), err
