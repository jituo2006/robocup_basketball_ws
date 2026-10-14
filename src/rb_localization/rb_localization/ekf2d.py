"""2D EKF（x、y、yaw）：运动预测与位姿观测更新，坐标系为场地坐标系。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def world_vel_to_body(vx_w: float, vy_w: float, body_yaw: float
                      ) -> tuple[float, float]:
    """把【世界(里程计)系】速度转成【车体系】速度。

    ⚠️ 第三个容易写错的地方：**FAST-LIO 的 /Odometry.twist 是世界系速度**。

    证据（`~/lidar_360/fast_prop_ws/.../include/common_lib.h:147`）：
        V3D vel_end;   // the estimated velocity at the end lidar point (world frame)
    而 `laserMapping.cpp` 把它直接塞进 twist，且 `header.frame_id = "camera_init"`。

    但 `Ekf2D.predict()` 的入参约定是**车体系**（它内部会再按 yaw 旋转一次）。
    若把世界系速度直接喂进去，等于**多转了一次 yaw** —— 预测方向完全错，
    表现为定位在车运动时抖动/发飘。

        v_body = R(−body_yaw) · v_world
    """
    c, s = math.cos(body_yaw), math.sin(body_yaw)
    return vx_w * c + vy_w * s, -vx_w * s + vy_w * c


def sensor_to_base_xy(ox: float, oy: float, oyaw: float,
                      s2b_x: float, s2b_y: float, s2b_yaw: float
                      ) -> tuple[float, float, float]:
    """把雷达（sensor）在里程计系下的位姿，换算成车体（base）位姿。

    ⚠️ 这里是第二个最容易写错的地方：**偏移必须按「车体 yaw」旋转，不是雷达 yaw**。

    为什么：物理上"雷达装在车体中心前方 30cm"这件事固定在**车体系**里；
    而雷达本身可能装得转了角度（本车 `sensor_to_base_yaw_deg = -85°`，几乎转了 90°）。
    若用雷达 yaw 去转这个偏移，等于把"前 30cm"转成了"侧 30cm" ——
    实测症状：RViz 里位姿箭头跑到车体的左上角。

    关系式：

        byaw = wrap_pi(oyaw + s2b_yaw)              # 车体朝向
        base = sensor + R(byaw) · (s2b_x, s2b_y)

    其中 `s2b` 是「**从雷指向车体旋转中心**」的向量，表达在**车体系**。
    注意方向是反的：雷达在中心【前方 30cm】⇒ `s2b_x = −0.30`。
    """
    byaw = wrap_pi(oyaw + s2b_yaw)
    c, s = math.cos(byaw), math.sin(byaw)
    return (ox + (s2b_x * c - s2b_y * s),
            oy + (s2b_x * s + s2b_y * c),
            byaw)




@dataclass
class EkfConfig:
    # 过程噪声（每步）
    sigma_v: float = 0.08          # 线速度噪声 (m/s)
    sigma_w: float = 0.06          # 角速度噪声 (rad/s)
    # 初始不确定度
    init_std_xy: float = 0.30
    init_std_yaw: float = 0.20
    # 观测噪声
    obs_std_xy: float = 0.15       # 绝对位姿 xy (m)
    obs_std_yaw: float = 0.08      # 绝对位姿 yaw (rad)


class Ekf2D:
    def __init__(self, cfg: EkfConfig | None = None) -> None:
        self.cfg = cfg or EkfConfig()
        self.x = np.zeros(3)  # x, y, yaw
        self.P = np.diag([
            self.cfg.init_std_xy ** 2,
            self.cfg.init_std_xy ** 2,
            self.cfg.init_std_yaw ** 2,
        ])
        self.initialized = False

    # -- 设定初始位姿 ------------------------------------------------------
    def set_pose(self, x: float, y: float, yaw: float,
                 std_xy: float | None = None, std_yaw: float | None = None) -> None:
        self.x = np.array([x, y, wrap_pi(yaw)], dtype=float)
        self.P = np.diag([
            (std_xy if std_xy is not None else self.cfg.init_std_xy) ** 2,
            (std_xy if std_xy is not None else self.cfg.init_std_xy) ** 2,
            (std_yaw if std_yaw is not None else self.cfg.init_std_yaw) ** 2,
        ])
        self.initialized = True

    # -- 预测（输入为车体系速度）------------------------------------------
    def predict(self, vx: float, vy: float, wz: float, dt: float) -> None:
        if dt <= 0.0:
            return
        if not self.initialized:
            self.set_pose(0.0, 0.0, 0.0)

        x, y, yaw = self.x
        c, s = math.cos(yaw), math.sin(yaw)

        # 运动模型
        self.x[0] = x + (vx * c - vy * s) * dt
        self.x[1] = y + (vx * s + vy * c) * dt
        self.x[2] = wrap_pi(yaw + wz * dt)

        # 雅可比 F = ∂f/∂x
        F = np.array([
            [1.0, 0.0, (-vx * s - vy * c) * dt],
            [0.0, 1.0, (vx * c - vy * s) * dt],
            [0.0, 0.0, 1.0],
        ])

        # 过程噪声（把速度噪声投影到状态空间）
        q_v = self.cfg.sigma_v ** 2
        q_w = self.cfg.sigma_w ** 2
        G = np.array([
            [c * dt, -s * dt, 0.0],
            [s * dt, c * dt, 0.0],
            [0.0, 0.0, dt],
        ])
        Q = G @ np.diag([q_v, q_v, q_w]) @ G.T

        self.P = F @ self.P @ F.T + Q

    def predict_pose_delta(self, previous, current, dt: float) -> None:
        """Predict from base poses without also integrating odometry twist.

        Independent filter corrections remain in the predicted state.
        """
        if dt <= 0.0:
            return
        vx, vy = world_vel_to_body((current[0] - previous[0]) / dt,
                                  (current[1] - previous[1]) / dt, previous[2])
        self.predict(vx, vy, wrap_pi(current[2] - previous[2]) / dt, dt)

    # -- 更新：绝对位姿 ----------------------------------------------------
    def update_pose(self, mx: float, my: float, myaw: float) -> None:
        if not self.initialized:
            self.set_pose(mx, my, myaw)
            return
        z = np.array([mx, my, wrap_pi(myaw)])
        h = np.array([self.x[0], self.x[1], self.x[2]])
        residual = np.array([z[0] - h[0], z[1] - h[1], wrap_pi(z[2] - h[2])])

        H = np.eye(3)
        R = np.diag([self.cfg.obs_std_xy ** 2, self.cfg.obs_std_xy ** 2, self.cfg.obs_std_yaw ** 2])
        self._kalman_update(residual, H, R)


    # -- 卡尔曼更新 --------------------------------------------------------
    def _kalman_update(self, residual: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        self.x = self.x + K @ residual
        self.x[2] = wrap_pi(self.x[2])
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

    # -- 输出 --------------------------------------------------------------
    @property
    def pose(self) -> tuple[float, float, float]:
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    @property
    def std(self) -> tuple[float, float, float]:
        return (float(math.sqrt(max(self.P[0, 0], 0.0))),
                float(math.sqrt(max(self.P[1, 1], 0.0))),
                float(math.sqrt(max(self.P[2, 2], 0.0))))
