#!/usr/bin/env bash
# RoboCup 篮球机器人 · 主机重启后的一键启动
#
# 前置（做一次就好，见 docs/07 §1）：
#   ① 雷达网卡 IP：NetworkManager 的 "Wired connection 1" 已是静态
#      192.168.1.110/24 + autoconnect（插上网线自动配，不用手动）
#   ② CAN 自动拉起：/etc/udev/rules.d/99-can0.rules（插上 PCAN 自动 UP）
#
# 主机和雷达可一起上电；DJI板及四路电调可之后独立上电。
# 软件启动后持续握手，板端重新上电后重新确认，不自动沿用旧运动目标。
# 雷达断电仍由 pass_boot_watch 管理整车重启；底盘独立断电不重启雷达。
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
# The desktop console owns a persistent chassis service. Lidar restarts must
# neither kill its node nor touch its CAN interface.
console_start_chassis=true
for console_arg in "$@"; do
  case "$console_arg" in
    start_chassis:=false|start_chassis:=False|start_chassis:=0) console_start_chassis=false ;;
    start_chassis:=true|start_chassis:=True|start_chassis:=1) console_start_chassis=true ;;
  esac
done
if [[ "$console_start_chassis" == true ]] && systemctl --user is-active --quiet robocup-console-chassis.service; then
  echo '共享底盘服务正在运行，请指定 start_chassis:=false 复用该驱动。' >&2
  exit 1
fi

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
  [[ "$pat" != rb_chassis_node || "$console_start_chassis" == true ]] || continue
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
if [[ "$console_start_chassis" == false ]]; then
  echo '      复用桌面共享底盘服务，不重配 CAN'
elif ip link show can0 >/dev/null 2>&1; then
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
echo "      雷达定位和底盘握手分别就绪后，才可点击任务出发"
echo "      Ctrl-C 停止（会走优雅退出、先发零速）"
echo ""

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
source "$WS/install/setup.bash"

# Before any chassis node: bound pending frames and recover bus-off in the kernel.
if [[ "$console_start_chassis" == true ]]; then
  python3 "$WS/tools/configure_chassis_can.py" || exit 1
fi

exec ros2 launch rb_bringup bringup_lidar_vision.launch.py "$@"
