#!/usr/bin/env bash
# RoboCup 篮球机器人 · 主机重启后的一键启动
#
# 前置（做一次就好，见 docs/07 §1）：
#   ① 雷达网卡 IP：NetworkManager 的 "Wired connection 1" 已是静态
#      192.168.1.110/24 + autoconnect（插上网线自动配，不用手动）
#   ② CAN 自动拉起：/etc/udev/rules.d/99-can0.rules（插上 PCAN 自动 UP）
#
# 每次开机的正确顺序：
#   ① 主机开机
#   ② 合上总开关（雷达 + 电机板 + 相机一起上电）
#   ③ bash tools/boot.sh         ← 本脚本
#   ④ 等约 15 秒，再开地图（见末尾提示）
#
# ⚠️ 为什么顺序是"先合开关、再跑本脚本"：
#   底盘节点启动时会发一次 MotorOn 使能电机 —— 必须在电机板已经上电之后发，
#   否则 MotorOn 落到"还没上电的板子"上就丢了，板子不会使能。
#   雷达则相反：驱动启动后广播找雷达，雷达上电后回应握手即可，顺序无碍
#   （只要雷达不是"上一个会话残留的流状态"）。
set -uo pipefail

WS=/home/user/robocup_basketball_ws
cd "$WS" || exit 1

echo "🤖 篮球机器人一键启动 $(date '+%H:%M:%S')"
echo ""

# ── ① 确保 CAN 起来（udev 规则应已自动做，这里兜底复查）────────────────────
echo "[1/2] 检查 CAN …"
if ip link show can0 >/dev/null 2>&1; then
  state=$(ip -details link show can0 2>/dev/null | grep -oE "state [A-Z-]+" | head -1)
  bitrate=$(ip -details link show can0 2>/dev/null | grep -oE "bitrate [0-9]+" | head -1)
  echo "      can0 已存在: $state $bitrate"
  # 状态不对就重配
  if ! ip -details link show can0 2>/dev/null | grep -q "state UP"; then
    echo "      can0 不是 UP，重新配置 …"
    bash tools/can_recover.sh || true
  fi
else
  echo "      can0 不存在，尝试拉起 …"
  bash tools/can_recover.sh || true
fi
echo ""

# ── ② 起整套软件（前台，Ctrl-C 停）──────────────────────────────────────────
echo "[2/2] 起整套软件（雷达驱动 → FAST-LIO → 相机/感知/定位/底盘/机构/任务）"
echo "      看到「底盘已使能 4 个电机」「mission 就绪」就是成功了"
echo "      Ctrl-C 停止（会走优雅退出、先发零速）"
echo ""

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
source "$WS/install/setup.bash"

exec ros2 launch rb_bringup bringup_lidar_vision.launch.py
