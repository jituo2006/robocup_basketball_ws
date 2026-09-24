"""极简 2D EKF（状态 x, y, yaw）。

为什么自己写而不用 robot_localization
-------------------------------------
robot_localization 只能吃"位姿/速度"类输入，而本项目最关键的观测是
**视觉看到的定位柱方位角**（bearing-only landmark），
robot_localization 表达不了。所以这里写一个最小的 3 维 EKF，
predict 用底盘里程计，update 支持两类观测：
  1. 绝对位姿（雷达 FAST-LIO 或全场定位板）
  2. 已知位置的地标方位角（定位柱）

状态约定
--------
坐标系为**场地坐标系**（右手，x 沿场地长边 14m，y 沿 7.5m，yaw 逆时针为正）。
"""

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
    bearing_std: float = 0.05      # 方位角 (rad)
    bearing_min_dist: float = 0.3  # 太近的地标方位角不可靠
    bearing_max_dist: float = 8.0


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

    # -- 更新：已知地标方位角（bearing-only）-------------------------------
    def update_bearing(self, landmark_xy: tuple[float, float], bearing_meas_world: float) -> bool:
        """bearing_meas_world = 机器人观测到的、**已在场地坐标系下**的方位角。

        即：观测到的目标方向角（相对机器人朝向） + 当前 yaw 估计。
        调用方负责把相机 bearing 通过安装外参转换到车体、再叠加 yaw。
        """
        if not self.initialized:
            return False

        lx, ly = landmark_xy
        dx = lx - self.x[0]
        dy = ly - self.x[1]
        d2 = dx * dx + dy * dy
        d = math.sqrt(d2)
        if d < self.cfg.bearing_min_dist or d > self.cfg.bearing_max_dist:
            return False

        # 残差 = 观测到的世界方位角 − 地标在当前估计下的几何方位角
        # （原来这里还多写了一句基于 predicted 的 residual，随即被本行覆盖，
        #   属于死代码；等价形式反而更容易看错，故删除。）
        geometric = math.atan2(dy, dx)
        residual = np.array([wrap_pi(bearing_meas_world - geometric)])

        # ⚠️ H 的符号曾经是反的，会让方位角更新**发散**（实测估计跑到 (10,4) )。
        # 关键在于：卡尔曼更新要的是 H = ∂h_pred/∂x，其中 h_pred 是"给定状态时
        # 传感器**应该**读到什么"。相机读数是相对车体的：
        #     b_pred = atan2(ly−y, lx−x) − yaw − 安装偏置
        # 所以
        #     ∂b_pred/∂x   = +dy/d²
        #     ∂b_pred/∂y   = −dx/d²
        #     ∂b_pred/∂yaw = −1
        # 而 S = H P Hᵀ + R 对 H 取负不变，符号会原样传到增益上 →
        # 用 ∂ν/∂x（即上面三个分量取反）会让每次修正都朝背离真值的方向走。
        H = np.array([[dy / d2, -dx / d2, -1.0]])
        R = np.array([[self.cfg.bearing_std ** 2]])
        self._kalman_update(residual, H, R)
        return True

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
