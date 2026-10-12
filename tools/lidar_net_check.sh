#!/usr/bin/env bash
# 雷达网络一键检查/修复
#
# ⚠️ 为什么需要它：机器人断电 → 雷达断电 → 网卡 enp114s0 失去载波自动 DOWN ✗，
#    雷达再上电时【网卡不会自己回来】✗ → Livox SDK 报
#        bind failed / Failed to init livox lidar sdk / Init lds lidar fail!
#    → /livox/lidar、/Odometry、/cloud_registered(_body) 全没数据 ✗
#    现场反复出现，症状极像"软件坏了"，其实是物理链路 ✗。
#
# 用法: bash tools/lidar_net_check.sh [--fix]
set -u
IFACE=${LIDAR_IFACE:-enp114s0}
HOST_IP=${LIDAR_HOST_IP:-192.168.1.110}
LIDAR_IP=${LIDAR_IP:-192.168.1.138}
FIX=0
[ "${1:-}" = "--fix" ] && FIX=1

echo "── 雷达网络检查 ($IFACE ↔ $LIDAR_IP) ──"
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "  ✗ 网卡 $IFACE 不存在"; exit 1
fi
carrier=$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)
echo "  carrier  = $carrier   ($([ "$carrier" = "1" ] && echo '有链路 ✓' || echo '【链路断 ✗ 物理问题:雷达没上电/网线松】'))"
echo "  operstate= $(cat /sys/class/net/$IFACE/operstate 2>/dev/null)"
ip -4 addr show "$IFACE" 2>/dev/null | grep -q "$HOST_IP" \
  && echo "  IP       = $HOST_IP ✓" || echo "  IP       = 【缺 $HOST_IP ✗】"

if [ "$FIX" = "1" ]; then
  echo "── 修复 ──"
  sudo ip link set "$IFACE" up 2>/dev/null
  ip -4 addr show "$IFACE" 2>/dev/null | grep -q "$HOST_IP" || \
    sudo ip addr add "$HOST_IP/24" dev "$IFACE" 2>/dev/null
  echo "  已拉起 $IFACE 并补齐 IP"
fi

if ping -c1 -W1 "$LIDAR_IP" >/dev/null 2>&1; then
  echo "  ping     = 通 ✓  雷达在线"
else
  echo "  ping     = 【不通 ✗】"
  echo "     → carrier=0 时:【先去查雷达上电和网线】✗ 软件无解"
  echo "     → carrier=1 时:试试 bash tools/lidar_net_check.sh --fix 后再 ping"
fi
