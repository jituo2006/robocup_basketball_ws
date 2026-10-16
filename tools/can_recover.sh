#!/usr/bin/env bash
# CAN 掉线恢复助手 —— 拔插过 CAN 适配器、或"车突然不动了"时跑这个
#
# 做三件事：
#   ① 确认 can0 是 UP（DOWN 就自动拉起，含波特率与 txqueuelen）
#   ② 清掉多余的底盘节点（同时跑多个会互相抢 CAN）
#   ③ 提示新固件握手恢复及USB重绑定要求
#
# 用法：
#   bash tools/can_recover.sh

set -u

IFACE=can0

G=$'\033[32m'; R=$'\033[31m'; Y=$'\033[33m'; B=$'\033[1m'; D=$'\033[2m'; N=$'\033[0m'

echo "══════════════════════════════════════════════════════"
echo "  CAN 掉线恢复"
echo "══════════════════════════════════════════════════════"

# ---------- ① can0 ----------
echo
echo "${B}[1/3] CAN 接口${N}"

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
    echo "      ${Y}! USB 上能看到 CAN 适配器（lsusb 0c72:000c），但没有 can0 接口${N}"
    # 区分两种完全不同的故障：掉线循环 vs 协议不应答
    if _klog | tail -40 | grep -qE "probe of .* failed with error -110|waiting rsp .* failure: -110"; then
      echo "      ${R}✗ 是【协议不应答】: peak_usb 发厂商命令超时(-110 = ETIMEDOUT)${N}"
      echo "      ${D}  设备在 USB 层是活的（描述符能读），但固件不实现 PEAK 协议${N}"
      echo "      ${D}  → 多半是兼容品（lsusb 的 Product 会显示 XCAN-USB，序列号为 0）${N}"
      echo "      ${D}  → 换原厂 PCAN-USB，或换 Linux 原生支持的适配器${N}"
    elif _klog | tail -40 | grep -qE "error -71|EPROTO|USB disconnect"; then
      echo "      ${Y}! 是【掉线循环】: USB 层报错(-71 EPROTO)/反复 disconnect${N}"
      echo "      ${D}  → 换 USB 线、换 USB 口（别插前面板），或加有源 hub${N}"
    else
      echo "      ${D}  → 稍等几秒再跑；仍不行就看 sudo dmesg -T | grep -E 'peak|can0'${N}"
    fi
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

# Shared with boot.sh: no contradictory queue size or restart policy.
python3 "$(dirname "${BASH_SOURCE[0]}")/configure_chassis_can.py" --interface "$IFACE" || exit 1

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

# ---------- ③ 握手恢复 ----------
echo
echo "${B}[3/3] 底盘恢复${N}"
echo "  配套新固件会在DJI板重上电后重新握手，不要求重启主机或雷达。"
echo "  等待四路电调反馈、速度模式和使能确认；新任务先发零速，再发运动指令。"
echo "  若can0设备被USB拔插重建，需另行重启底盘节点以重新绑定设备。"
echo "  此脚本只恢复主机CAN配置，不会发送运动指令或烧录控制板。"
