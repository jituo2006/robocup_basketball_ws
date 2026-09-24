#!/usr/bin/env bash
# RoboCup 篮球机器人 · 上车前预检
# 逐项给出红绿灯。✗ 的项必须解决后再上车。
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OK="\033[32m"; WARN="\033[33m"; FAIL="\033[31m"; DIM="\033[2m"; BOLD="\033[1m"; RST="\033[0m"
pass=0; warn=0; fail=0; sw_fail=0; hw_fail=0
ok()   { echo -e "  ${OK}✓${RST} $1"; pass=$((pass+1)); }
wa()   { echo -e "  ${WARN}!${RST} $1"; warn=$((warn+1)); [ -n "${2:-}" ] && echo -e "    ${DIM}→ $2${RST}"; }
no()   { echo -e "  ${FAIL}✗${RST} $1"; fail=$((fail+1)); [ -n "${2:-}" ] && echo -e "    ${DIM}→ $2${RST}"; }
# 只属于"还没接硬件"的失败：这类红是正常的，上车时自然会变绿
no_hw() { no "$1" "$2"; hw_fail=$((hw_fail+1)); }
# 属于"软件/构建/配置"的失败：这类必须现在解决
no_sw() { no "$1" "$2"; sw_fail=$((sw_fail+1)); }

echo -e "${BOLD}RoboCup 篮球机器人 · 上车前预检${RST}   ($WS)"
echo "────────────────────────────────────────────────────────────"

# 1) 环境
echo -e "${BOLD}[环境]${RST}"
if [ "${ROS_DISTRO:-}" = "humble" ]; then ok "ROS_DISTRO=humble"; else no_sw "ROS_DISTRO=${ROS_DISTRO:-<空>}" "source /opt/ros/humble/setup.bash"; fi
if echo "${AMENT_PREFIX_PATH:-}" | grep -q "$WS"; then ok "已加载本工作空间"; else no_sw "未加载 $WS" "source install/setup.bash"; fi
[ "${ROS_DOMAIN_ID:-}" = "2" ] && ok "ROS_DOMAIN_ID=2" || wa "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<未设>}" "export ROS_DOMAIN_ID=2（与队里其它车区分）"

# 2) 构建
echo -e "${BOLD}[构建]${RST}"
for p in rb_msgs rb_chassis rb_launcher rb_camera rb_perception rb_localization rb_mission rb_bringup; do
  if timeout 12 ros2 pkg prefix "$p" >/dev/null 2>&1; then ok "$p 可加载"; else no_sw "$p 找不到" "colcon build --symlink-install"; fi
done

# 3) CAN
echo -e "${BOLD}[CAN / 底盘]${RST}"
if ip link show can0 >/dev/null 2>&1; then
  qlen=$(ip -details link show can0 2>/dev/null | sed -n 's/.*qlen \([0-9]*\).*/\1/p' | head -1)
  br=$(ip -details link show can0 2>/dev/null | sed -n 's/.*bitrate \([0-9]*\).*/\1/p' | head -1)
  ip link show can0 2>/dev/null | grep -q "UP" && ok "can0 已 UP" || no_hw "can0 未 UP" "sudo ip link set can0 up"
  [ "${br:-0}" = "1000000" ] && ok "bitrate=1000000" || no_hw "bitrate=${br:-?}" "sudo ip link set can0 type can bitrate 1000000"
  if [ "${qlen:-1000}" -le 20 ] 2>/dev/null; then ok "txqueuelen=$qlen（安全）"
  else no_hw "txqueuelen=$qlen（危险！默认 1000 会积压旧速度帧导致窜车）" "sudo ip link set can0 txqueuelen 20"; fi
else
  no_hw "can0 不存在（未接 CAN 适配器）" "插好 USB-CAN；PCAN 可试 sudo modprobe peak_usb"
fi

# 4) 串口
echo -e "${BOLD}[串口]${RST}"
PORT=$(grep -oP '^\s*port:\s*"\K[^"]+' "$WS/src/rb_launcher/config/launcher.yaml" 2>/dev/null | head -1)
if [ -n "$PORT" ] && [ -e "$PORT" ]; then ok "机构串口 $PORT 存在"
else no_hw "机构串口 ${PORT:-未配置} 未就绪" "补 udev 规则，或改 launcher.yaml 的 port 为实际设备"; fi
[ -e /dev/usb2ttl ] && ok "手柄串口 /dev/usb2ttl（RoboCup 比赛中不可用，仅调试）" || wa "无 /dev/usb2ttl（正常，比赛禁用遥控）"
systemctl is-active ModemManager >/dev/null 2>&1 && wa "ModemManager 在跑，可能抢串口" "sudo systemctl stop ModemManager"

# 4.5) 雷达网络
echo -e "${BOLD}[雷达网络]${RST}"
if [ -f "$WS/tools/check_lidar_net.py" ]; then
  if python3 "$WS/tools/check_lidar_net.py" >/tmp/lidarnet.txt 2>&1; then
    ok "雷达网络配置一致"
    grep -E "^  !" /tmp/lidarnet.txt | sed 's/^/  /' || true
  else
    # 雷达没接时这里本来就该红，归到硬件侧
    no_hw "雷达网络未就绪（主机 IP 不在网卡上 / ping 不通）" "python3 tools/check_lidar_net.py --ping 看详情"
    sed -n '/^\x1b\[31m·/p' /tmp/lidarnet.txt 2>/dev/null | head -3 | sed 's/^/    /' || true
  fi
else
  wa "缺少 check_lidar_net.py"
fi

# 5) 相机
echo -e "${BOLD}[相机]${RST}"
DEV=$(grep -oP '^\s*device:\s*"\K[^"]+' "$WS/src/rb_camera/config/camera.yaml" 2>/dev/null | head -1)
if [ -n "$DEV" ] && [ -e "$DEV" ]; then
  ok "相机设备 $DEV 存在"
  if command -v v4l2-ctl >/dev/null 2>&1; then
    if v4l2-ctl -d "$DEV" --list-formats 2>/dev/null | grep -q "MJPG"; then
      ok "支持 MJPG（高分辨率下必须用它，YUYV 会掉到 6~9fps）"
    else
      wa "不支持 MJPG" "只能用小分辨率；换相机或降分辨率"
    fi
  else
    wa "无 v4l2-ctl，无法确认格式" "sudo apt install v4l-utils"
  fi
  grep -q 'publish_compressed: true' "$WS/src/rb_camera/config/camera.yaml" 2>/dev/null \
    && ok "JPEG 压缩已开启（原始大图投递只有 ~7fps，必须压缩）" \
    || wa "publish_compressed 不是 true" "原始大图消息投递只有 ~7fps，改回 true"
  # 曝光/白平衡是否已锁（HSV 阈值随光照漂移，锁定后才稳定）
  if command -v v4l2-ctl >/dev/null 2>&1; then
    # 输出形如 "auto_exposure: 1 (Manual Mode)" / "white_balance_automatic: 0"
    # 注意结尾可能是 ")"，不能用 [0-9]*$ 去匹配
    ae=$(v4l2-ctl -d "$DEV" --get-ctrl=auto_exposure 2>/dev/null | sed -n 's/.*: *\([0-9][0-9]*\).*/\1/p')
    awb=$(v4l2-ctl -d "$DEV" --get-ctrl=white_balance_automatic 2>/dev/null | sed -n 's/.*: *\([0-9][0-9]*\).*/\1/p')
    if [ "${ae:-3}" = "1" ] && [ "${awb:-1}" = "0" ]; then
      ok "曝光与白平衡已锁定（颜色标定才能长期有效）"
    else
      wa "曝光/白平衡未锁定 (auto_exposure=${ae:-?} white_balance_automatic=${awb:-?})" \
         "跑 python3 tools/tune_camera.py 调好并写回；见 docs/07 §3.4"
    fi
  fi
  # 内参是否已标定
  if grep -qP '^\s*fx:\s*0\.0' "$WS/src/rb_camera/config/camera.yaml" 2>/dev/null; then
    wa "相机内参未标定（fx=0，测距按假定 FOV 估算）" "python3 tools/calibrate_camera.py；见 docs/07 §3.1"
  else
    ok "相机内参已标定"
  fi
else
  no_hw "相机设备 ${DEV:-未配置} 不存在" "插好相机；确认 /dev/video0（见 docs/07）"
fi

# 6) 配置完整性
echo -e "${BOLD}[配置]${RST}"
grep -q "TODO 实测" "$WS/src/rb_chassis/config/chassis.yaml" 2>/dev/null \
  && no_hw "chassis.yaml 的尺寸还是占位值" "上车后实测宽度/轴距/轮径（见 docs/03 §1）" \
  || ok "chassis.yaml 尺寸已填"
n=$(python3 -c "
import yaml,sys
try:
    d=yaml.safe_load(open('$WS/src/rb_localization/config/localization.yaml'))
    print(len(d.get('landmarks') or []))
except Exception: print(0)
" 2>/dev/null)
[ "${n:-0}" -gt 0 ] && ok "定位柱 landmarks 已配 $n 个" || wa "localization.yaml 的 landmarks 为空" "视觉方位角更新不会生效"

# 7) 残留进程
echo -e "${BOLD}[运行环境]${RST}"
# 只匹配**已安装的可执行入口**，避免 pgrep -f 匹配到调用它的 shell 自身
# （shell 的 argv 里含有这些字样，会造成误报）
leftover=$(pgrep -f "lib/[r]b_(mission|localization|perception)/" 2>/dev/null | wc -l)
leftover=$(( leftover + $(pgrep -f "tools/[f]ake_(board|detections).py" 2>/dev/null | wc -l) ))
[ "$leftover" -eq 0 ] && ok "无残留节点" || wa "有 $leftover 个残留进程" "pkill -f rb_mission 等"

echo "────────────────────────────────────────────────────────────"
echo -e "  ${OK}通过 $pass${RST} / ${WARN}警告 $warn${RST} / ${FAIL}失败 $fail${RST}"
echo
echo -e "  ${BOLD}软件侧${RST}（必须现在解决）  : $( [ "$sw_fail" -eq 0 ] && echo -e "${OK}全部就绪${RST}" || echo -e "${FAIL}$sw_fail 项待修${RST}" )"
echo -e "  ${BOLD}硬件侧${RST}（上车时自然会解决）: $( [ "$hw_fail" -eq 0 ] && echo -e "${OK}全部就绪${RST}" || echo -e "${WARN}$hw_fail 项待接${RST}" )"

if [ "$sw_fail" -gt 0 ]; then
  echo
  echo -e "  ${FAIL}软件侧仍有问题 —— 先解决这些，不要上车。${RST}"
  echo -e "  ${DIM}提示: 先跑 python3 tools/verify_offline.py 做完整离线验证${RST}"
  exit 1
fi

echo
if [ "$hw_fail" -gt 0 ]; then
  echo -e "  ${OK}软件侧已就绪 —— 可以开始上车。${RST}"
  echo -e "  ${DIM}上面 ${WARN}!${RST}${DIM} / ${FAIL}✗${RST}${DIM} 的硬件项，按 docs/02_上车调通.md 逐阶段解决即可。${RST}"
  exit 2          # 2 = 软件就绪，硬件待接
fi
echo -e "  ${OK}软硬件全部就绪。${RST}"
