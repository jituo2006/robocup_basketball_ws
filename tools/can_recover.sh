#!/usr/bin/env bash
# CAN 掉线恢复助手 —— 拔插过 CAN 适配器、或"车突然不动了"时跑这个
#
# 做三件事：
#   ① 确认 can0 是 UP（DOWN 就自动拉起，含波特率与 txqueuelen）
#   ② 清掉多余的底盘节点（同时跑多个会互相抢 CAN）
#   ③ 打印必须【手动】做的那一步：电机板断电 10 秒
#
# 为什么要 ③：拔插 CAN 只断通信、不断电机板电源。板子在通信丢失后会
# 锁存故障态，软件重发 MotorOn 清不掉，必须物理断电。详见 docs/03_底盘模块.md §2.2
#
# 用法：
#   bash tools/can_recover.sh

set -u

IFACE=can0
BITRATE=1000000
QLEN=20

G=$'\033[32m'; R=$'\033[31m'; Y=$'\033[33m'; B=$'\033[1m'; D=$'\033[2m'; N=$'\033[0m'

echo "══════════════════════════════════════════════════════"
echo "  CAN 掉线恢复"
echo "══════════════════════════════════════════════════════"

# ---------- ① can0 ----------
echo
echo "${B}[1/3] CAN 接口${N}"

need_up=0
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "  ${R}✗${N} $IFACE 不存在"
  # 分辨"没插"和"插了但在反复掉线" —— 后者用户常误判成软件问题
  # 取内核日志：dmesg 无 sudo 常被禁（kernel.dmesg_restrict），退回 kern.log
  _klog() {
    if [ -r /var/log/kern.log ]; then cat /var/log/kern.log
    elif sudo -n dmesg >/dev/null 2>&1; then sudo -n dmesg
    else dmesg 2>/dev/null; fi
  }
  # grep -c 无匹配时输出 0 但退出码为 1，不能再接 `|| echo 0`（会变成两行）
  drops=$(_klog | grep -c "USB disconnect"); drops=${drops:-0}
  usb_seen=$(lsusb 2>/dev/null | grep -c "0c72:000c"); usb_seen=${usb_seen:-0}
  if [ "${usb_seen:-0}" -gt 0 ]; then
    echo "      ${Y}! USB 上能看到 CAN 适配器（lsusb 0c72:000c），但 /sys 里没有接口${N}"
    echo "      ${D}→ 它在枚举/掉线循环中，稍等几秒再跑，或换 USB 口/换线${N}"
  else
    echo "      ${D}lsusb 里看不到 0c72:000c —— 适配器没插好（换线/换口试试）${N}"
  fi
  if [ "${drops:-0}" -gt 3 ]; then
    echo "      ${Y}⚠ 本次开机已记录 $drops 次 USB 掉线 —— 适配器/线在飘！${N}"
    echo "      ${D}建议：换一根 USB 线、换 USB 口；已加 udev 规则关掉自动挂起${N}"
    echo "      ${D}查看：sudo dmesg -T | grep -E '3-|peak|can0' | tail -20${N}"
  fi
  echo "      ${D}插好/稳定后重新跑本脚本${N}"
  exit 1
fi

state=$(ip -br link show "$IFACE" 2>/dev/null | awk '{print $2}')
flags=$(ip -details link show "$IFACE" 2>/dev/null | grep -o 'bitrate [0-9]*' | awk '{print $2}')
qlen=$(ip -details link show "$IFACE" 2>/dev/null | grep -o 'qlen [0-9]*' | awk '{print $2}')

if [ "$state" != "UP" ]; then
  echo "  ${Y}!${N} $IFACE 是 DOWN（拔插适配器后常见）→ 正在拉起"
  need_up=1
elif [ "${flags:-0}" != "$BITRATE" ] || [ "${qlen:-0}" != "$QLEN" ]; then
  echo "  ${Y}!${N} 配置不对（bitrate=${flags:-?} qlen=${qlen:-?}）→ 重新配置"
  need_up=1
else
  echo "  ${G}✓${N} $IFACE 已 UP，bitrate=$flags qlen=$qlen"
fi

if [ "$need_up" = "1" ]; then
  sudo ip link set "$IFACE" down 2>/dev/null
  sudo ip link set "$IFACE" type can bitrate "$BITRATE" || { echo "  ${R}✗ 设置波特率失败${N}"; exit 1; }
  sudo ip link set "$IFACE" txqueuelen "$QLEN"
  sudo ip link set "$IFACE" up
  sleep 1
  err=$(ip -details link show "$IFACE" 2>/dev/null | grep -o 'can state [A-Z-]*')
  echo "  ${G}✓${N} 已拉起：$(ip -br link show "$IFACE")  ${D}$err${N}"
  echo "  ${D}注意：接口编号可能变了，底盘节点必须重启才会绑定新编号${N}"
fi

# ---------- ② 底盘节点去重 ----------
echo
echo "${B}[2/3] 底盘节点${N}"
mapfile -t nodes < <(pgrep -f "[r]b_chassis_node" 2>/dev/null || true)
n=${#nodes[@]}

if [ "$n" -eq 0 ]; then
  echo "  ${Y}!${N} 没有底盘节点在跑 —— 稍后要起一个（见下面第 3 步）"
elif [ "$n" -eq 1 ]; then
  echo "  ${G}✓${N} 只有 1 个（PID ${nodes[0]}）"
else
  echo "  ${Y}!${N} 有 $n 个底盘节点在抢 CAN（PID: ${nodes[*]}）→ 全部清掉"
  pkill -f "[r]b_chassis" 2>/dev/null
  sleep 2
  left=$(pgrep -f "[r]b_chassis_node" 2>/dev/null | wc -l)
  echo "  ${G}✓${N} 已清理，剩余 $left 个"
fi

# ---------- ③ 手动步骤 ----------
echo
echo "${B}[3/3] ⚠️  必须手动做的一步${N}"
echo
echo "  ${Y}把【电机板】彻底断电 10 秒，再上电。${N}"
echo "  ${D}不是关 CAN、不是重启程序 —— 是断电机板那一路电源。${N}"
echo "  ${D}板子在通信丢失后会锁存故障，软件重发 MotorOn 清不掉，必须物理断电。${N}"
echo
echo "  做完之后重启底盘节点："
echo "    ${B}rbws${N}"
echo "    ${B}ros2 launch rb_chassis chassis.launch.py${N}"
echo "  ${D}看到「CAN 已启动: can0」和「底盘已使能 4 个电机（board_id=1）」才算 OK。${N}"
echo
echo "  然后另开终端遥控（终端也要 source 环境）："
echo "    ${B}rbws${N}"
echo "    ${B}python3 tools/teleop_chassis.py${N}"
echo
echo "══════════════════════════════════════════════════════"
