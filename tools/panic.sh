#!/usr/bin/env bash
# RoboCup 篮球机器人 · 一键急停（panic stop）
#
# 什么时候用
# ----------
#   * 车往不该走的方向走 / 停不下来
#   * 遥控终端眼看要断，你想先让它停下来
#   * 任何"不对劲"的时候 —— 先停，再排查
#
# 用法
# ----
#   bash tools/panic.sh            # 软急停：锁死任务 + 停车 + 杀掉会驱动车的节点
#   bash tools/panic.sh --hard     # 在上面基础上**再把 can0 拉下来**（最彻底）
#   bash tools/panic.sh --quiet    # 少打印
#
# 停止顺序（为什么这么排）
# ------------------------
#   ① 发 estop=true  → rb_mission 立刻不再产生任何速度指令（最快的一刀）
#   ② 发零速多帧     → 万一还有别的 /cmd_vel 发布者，也压成 0
#   ③ 杀 rb_mission  → 断掉指令源头
#   ④ 杀 rb_chassis  → 它退出时会走 on_shutdown 钩子**主动发一次零轮速**，
#                      所以"优雅地杀"比"砍 CAN"更安全，不要先砍 CAN
#   ⑤ --hard: can0 down → 连残余帧的可能都掐掉
#
# 注意：本脚本**默认不动**雷达/相机/定位/感知 —— 它们不驱动车，
#       留着方便你复看位姿、排查原因。
set -uo pipefail

WS=/home/user/robocup_basketball_ws
HARD=0
QUIET=0
for a in "$@"; do
  case "$a" in
    --hard)  HARD=1 ;;
    --quiet) QUIET=1 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数: $a（用 --help）" >&2; exit 2 ;;
  esac
done

say() { [ "$QUIET" -eq 1 ] || echo "$@"; }

say "🛑 RoboCup 急停 $(date '+%H:%M:%S')"
say ""

# ── ① + ② 通过 ROS 发 estop 与零速 ────────────────────────────────────────
# 需要 ROS 环境；拿不到就跳过（后面直接杀节点，同样能停）
if [ -f "$WS/install/setup.bash" ]; then
  if [ "$QUIET" -eq 0 ]; then say "  [1/5] 下发 estop + 零速 …"; fi
  env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin:/opt/ros/humble/bin bash -c "
    source /opt/ros/humble/setup.bash
    source '$WS/install/setup.bash' 2>/dev/null
    export ROS_DOMAIN_ID=2
    # estop=true（锁死任务侧）
    timeout 4 ros2 topic pub --once /mission/estop std_msgs/msg/Bool '{data: true}' \
      >/dev/null 2>&1 || true
    # 零速连发几帧（覆盖 200ms 超时窗口）
    timeout 6 ros2 topic pub -r 20 -t 8 /cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      >/dev/null 2>&1 || true
  " || true
else
  say "  [1/5] 找不到工作空间，跳过 ROS 急停（直接杀节点）"
fi

# ── ③ 杀任务节点（指令源头）───────────────────────────────────────────────
say "  [2/5] 停 rb_mission …"
pkill -f "[r]b_mission/lib/rb_mission/mission_node" 2>/dev/null
pkill -f "[r]os2 run rb_mission" 2>/dev/null
sleep 1

# ── ④ 优雅杀底盘（on_shutdown 会主动发零轮速）─────────────────────────────
say "  [3/5] 停 rb_chassis（退出钩子会主动发零轮速）…"
pkill -f "[r]b_chassis/lib/rb_chassis/rb_chassis_node" 2>/dev/null
pkill -f "[r]os2 launch rb_chassis" 2>/dev/null
sleep 2

# ── 复查 ──────────────────────────────────────────────────────────────────
left=$(ps -eo cmd | grep -cE "[r]b_mission/lib/|[r]b_chassis/lib/")
say "  [4/5] 复查：残留驱动节点 = $left"

# ── ⑤ 可选：把 CAN 拉下来（最彻底）────────────────────────────────────────
if [ "$HARD" -eq 1 ]; then
  say "  [5/5] --hard：拉低 can0（此后不可能有任何 CAN 帧）…"
  if ip link show can0 >/dev/null 2>&1; then
    sudo ip link set can0 down 2>/dev/null && say "        ✅ can0 已 down" \
      || say "        ⚠️ 需要 sudo 权限，can0 未拉低"
  else
    say "        can0 不存在（适配器已拔？）"
  fi
else
  say "  [5/5] 软停完成（未动 can0；要更彻底用 --hard）"
fi

say ""
if [ "$left" -eq 0 ]; then
  say "✅ 已停：没有任何节点在把 /cmd_vel 变成 CAN 帧"
else
  say "⚠️ 仍有 $left 个驱动节点，请手动确认："
  ps -eo pid,cmd | grep -E "[r]b_mission/lib/|[r]b_chassis/lib/" | sed 's/^/     /'
fi
say ""
say "想彻底断电最保险：直接按车上的**电源开关**。软件永远不如拔电可靠。"
