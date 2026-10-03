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
                 face_travel: bool = True,
                 approach_radius: float = 0.0,
                 approach_speed: float = 0.0) -> tuple[float, float, float, float]:
    """朝目标点走的 P 控制器（场地坐标系 → 车体系速度）。

    返回 (vx, vy, wz, distance)。vx/vy/wz 已经是车体系。

    ⚠️ 为什么需要 approach_radius / approach_speed（接近限速）：
      反馈链路有 ~300ms 延迟（雷达扫描 100ms → FAST-LIO ~200ms → 定位 2ms，
      实测 /Odometry 消息时延中位 304ms）。P 控制器在离目标 0.27m 时仍是
      满速 max_lin，于是"指令停下时车已经冲过头"：
          超调 ≈ 延迟 × 接近速度
      限速后：0.3s × 0.12m/s ≈ 3.6cm（原来 0.3s × 0.40 ≈ 12cm）。
      在 approach_radius 内把速度上限压到 approach_speed，换取到位精度。
    """
    dx = tgt_x - cur_x
    dy = tgt_y - cur_y
    dist = math.hypot(dx, dy)

    # 接近限速：进入 approach_radius 后把速度上限压到 approach_speed
    eff_max = max_lin
    if approach_radius > 0.0 and approach_speed > 0.0 and dist < approach_radius:
        eff_max = min(max_lin, approach_speed)

    # 世界系期望速度方向
    vwx = kp_lin * dx
    vwy = kp_lin * dy
    speed = math.hypot(vwx, vwy)
    if speed > eff_max and speed > 1e-9:
        vwx *= eff_max / speed
        vwy *= eff_max / speed

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

    target_bearing_body 是目标相对**车体前进方向**的方位角（**右正左负**，
    即相机 bearing_rad 的约定）。

    ⚠️ 符号：cmd_vel.angular.z 用的是 ROS 约定（**逆时针为正**），
    与输入的"右正"相反，所以这里要取负。
    若忘了取负 → 目标在右边时车往左转（背道而驰）。
    """
    err = wrap_pi(-target_bearing_body)
    return clamp(kp_yaw * err, -max_ang, max_ang), err
