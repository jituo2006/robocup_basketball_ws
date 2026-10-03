#!/usr/bin/env bash
# 就绪检查：一条命令告诉你"现在能不能开车/看地图"
#
# 用法：bash tools/check_ready.sh
# 退出码：0 = 就绪；1 = 还没就绪
set -o pipefail

WS=/home/user/robocup_basketball_ws
cd "$WS" || exit 1

source /opt/ros/humble/setup.bash 2>/dev/null
# ⚠️ 必须 source 雷达工作空间：/livox/lidar 是 livox_ros_driver2/msg/CustomMsg，
#    不 source 的话 ros2 topic hz 认不出类型，会误报"没数据"。
source /home/user/lidar_360/ws_livox/install/setup.bash 2>/dev/null
source /home/user/lidar_360/fast_prop_ws/install/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=2

G=$'\033[32m'; R=$'\033[31m'; Y=$'\033[33m'; N=$'\033[0m'
ok=0; bad=0

check_topic() {  # $1=话题  $2=期望最低频率
  local t="$1" min="$2"
  local rate
  # ⚠️ 超时给足 12 秒：点云是大消息，ros2 topic hz 建立订阅 + 收到第一个包
  #    要好几秒（会先打一行 "does not appear" 再出 rate）。4 秒会误报"没数据"。
  rate=$(timeout 12 ros2 topic hz "$t" 2>/dev/null | grep -oE "average rate: [0-9.]+" | head -1 | awk '{print $3}')
  if [ -z "$rate" ]; then
    printf "  ${R}✗${N} %-24s 没数据\n" "$t"
    bad=$((bad+1))
  elif awk "BEGIN{exit !($rate >= $min)}"; then
    printf "  ${G}✓${N} %-24s ${rate} Hz\n" "$t"
    ok=$((ok+1))
  else
    printf "  ${Y}△${N} %-24s ${rate} Hz（偏低）\n" "$t"
    bad=$((bad+1))
  fi
}

echo "🤖 就绪检查 $(date '+%H:%M:%S')"
echo ""
echo "  ── 硬件 ──"
if ip link show can0 >/dev/null 2>&1 && ip -details link show can0 2>/dev/null | grep -q "state UP"; then
  printf "  ${G}✓${N} %-24s UP\n" "CAN (can0)"
  ok=$((ok+1))
else
  printf "  ${R}✗${N} %-24s 没 UP（bash tools/can_recover.sh）\n" "CAN (can0)"
  bad=$((bad+1))
fi

echo "  ── 数据链 ──"
check_topic /livox/lidar 5
check_topic /livox/imu 100
check_topic /cloud_registered 5
check_topic /Odometry 5
check_topic /localization/pose 10

echo "  ── 定位可信度 ──"
lok=$(timeout 4 ros2 topic echo /localization/ok --once 2>/dev/null | grep -oE "data: (true|false)" | awk '{print $2}')
case "$lok" in
  true)  printf "  ${G}✓${N} %-24s true\n" "/localization/ok"; ok=$((ok+1));;
  false) printf "  ${R}✗${N} %-24s false\n" "/localization/ok"; bad=$((bad+1));;
  *)     printf "  ${R}✗${N} %-24s 无\n" "/localization/ok"; bad=$((bad+1));;
esac

echo ""
if [ "$bad" -eq 0 ]; then
  echo "  ${G}✅ 全部就绪，可以开车 / 开地图了${N}"
  exit 0
else
  echo "  ${R}⚠️ 还有 $bad 项没就绪${N}"
  echo "     雷达/IMU 没数据 → 断电重启雷达，等 15 秒（点云有但 imu 无，也是断电重启雷达）"
  echo "     CAN 没 UP → bash tools/can_recover.sh"
  echo "     定位没起来 → 确认整套软件（boot.sh）在跑"
  exit 1
fi
