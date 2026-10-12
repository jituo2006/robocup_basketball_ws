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
set -o pipefail

# PTP uses this host as master; it does not require Internet/NTP to align sensors.
# The patched driver only admits device-PTP samples and converts TAI to ROS UTC.
# Service active means installed/running; actual LiDAR lock is checked by data.
if [ -f /etc/robocup/livox_ptp_utc_offset ]; then
  _ptp_iface=$(cat /etc/robocup/ptp_interface 2>/dev/null || echo enp114s0)
  for _service in "robocup-ptp4l@$_ptp_iface.service" "robocup-phc2sys@$_ptp_iface.service"; do
    if ! systemctl is-active --quiet "$_service"; then
      if ! sudo -n systemctl start "$_service"; then
        echo "PTP 服务启动失败：${_service}；请检查 tools/install_ptp.sh 安装。" >&2
        exit 1
      fi
    fi
  done
  echo "已启用硬件 PTP；等待雷达真实 PTP 数据后启动 FAST-LIO。"
  if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" != "yes" ]; then
    echo "系统暂未连接 NTP，使用本机时钟进行雷达相对同步。"
  fi
elif command -v timedatectl >/dev/null 2>&1; then
  # Legacy installation: retain the previous boot-time wall-clock check.
  for (( _sync_wait=0; _sync_wait<45; _sync_wait++ )); do
    [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ] && break
    sleep 1
  done
  if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" != "yes" ]; then
    echo "系统校时未完成，请先完成校时再启动雷达。" >&2
    exit 1
  fi
fi

WS=/home/user/robocup_basketball_ws
cd "$WS" || exit 1

echo "🤖 篮球机器人一键启动 $(date '+%H:%M:%S')"
echo ""

# ── ⓪ 清理残留进程（**最关键的一步**）──────────────────────────────────────
#
# ⚠️ 这是"雷达反复没数据"的真正根因：
#   launch 重启时 livox 驱动常常不被带走，变成孤儿进程（ppid=1）。
#   第二个驱动起来后，两个进程**同时持有 56301/56401 端口**（SO_REUSEPORT），
#   雷达 UDP 数据被内核随机分给两个进程 → 点云时有时无、IMU 丢失、地图消失。
#   症状看起来像"雷达坏了"，其实是重复驱动抢端口。
#
#   解决：每次启动前把要起的节点全部清干净，保证"只有一个"。
#   （用 -9：livox 驱动卡死时 SIGTERM 可能不生效。）
#
# ⚠️⚠️ 必须排除"自己 + 所有祖先进程"：
#   `pkill -9 -f <模式>` 会匹配*整条命令行*。如果调用 boot.sh 的那个 shell
#   （或它的父进程）命令行里恰好含 "mission_node" 之类字样，就会把自己杀掉
#   —— 实测踩过：`bash -c '... mission_node ...'` 调用 boot.sh，boot.sh
#   把自己的父 shell SIGKILL 了。所以下面逐 PID 判断，跳过祖先链。
_self=$$
_anc=" $_self "
_p=$_self
while :; do
  _p=$(ps -o ppid= -p "$_p" 2>/dev/null | tr -d ' ')
  [ -z "$_p" ] && break
  [ "$_p" = "0" ] && break
  [ "$_p" = "1" ] && break
  _anc="$_anc$_p "
done

# 安全清理：返回实际杀掉的进程数（排除祖先链）
safe_kill() {
  local pat="$1" pid cnt=0
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    case "$_anc" in *" $pid "*) continue ;; esac   # 跳过自己/祖先
    kill -9 "$pid" 2>/dev/null && cnt=$((cnt + 1))
  done
  echo "$cnt"
}

echo "[1/3] 清理残留进程（防止重复驱动抢端口）…"
killed=0
for pat in \
    lidar_session.py \
    livox_ros_driver2_node \
    fastlio_mapping \
    mapping.launch.py \
    msg_MID360_launch.py \
    localization_node \
    rb_chassis_node \
    mission_node \
    camera_node \
    perception_node \
    rb_launcher_node ; do
  n=$(safe_kill "$pat")
  killed=$((killed + n))
done
sleep 2
if [ "$killed" -gt 0 ]; then
  echo "      已清理 $killed 个残留进程"
else
  echo "      干净，无残留"
fi
left=$(pgrep -f "livox_ros_driver2_node" 2>/dev/null | wc -l)
for pid in $(pgrep -f "livox_ros_driver2_node" 2>/dev/null); do
  case "$_anc" in *" $pid "*) left=$((left - 1)) ;; esac
done
if [ "$left" -gt 0 ]; then
  echo "      ⚠️ 还有 $left 个雷达驱动残留，手动确认：pgrep -af livox_ros_driver2_node"
fi
echo ""

# ── ① 确保 CAN 起来（udev 规则应已自动做，这里兜底复查）────────────────────
echo "[2/3] 检查 CAN …"
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
echo "[3/3] 起整套软件（雷达驱动 → FAST-LIO → 相机/感知/定位/底盘/机构/任务）"
echo "      看到「底盘已使能 4 个电机」「mission 就绪」就是成功了"
echo "      Ctrl-C 停止（会走优雅退出、先发零速）"
echo ""

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
source "$WS/install/setup.bash"

# 保持足够的 CAN 发送队列，避免队列过小导致 ENOBUFS 丢指令。
if ip link show can0 >/dev/null 2>&1; then
  _q=$(cat /sys/class/net/can0/tx_queue_len 2>/dev/null || echo 0)
  if [ "${_q:-0}" -lt 500 ] 2>/dev/null; then
    if ! ip link set can0 txqueuelen 1000 2>/dev/null; then
      if ! sudo -n ip link set can0 txqueuelen 1000; then
        echo "CAN 发送队列调整失败，启动中止；请先设置 can0 txqueuelen 1000。" >&2
        exit 1
      fi
    fi
    echo "can0 txqueuelen: $_q -> $(cat /sys/class/net/can0/tx_queue_len)"
  fi
fi

exec ros2 launch rb_bringup bringup_lidar_vision.launch.py "$@"
