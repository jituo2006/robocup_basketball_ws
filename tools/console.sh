#!/usr/bin/env bash
# 篮球机器人综合控制台 —— 一键启动
#
# 它会自动 source ROS + 本工作空间，然后起 tools/console_app.py。
# 起好后用浏览器打开下面打印的地址（本机或手机都行）。
#
#   bash tools/console.sh                 # 默认端口 8090
#   bash tools/console.sh --port 9000
#   bash tools/console.sh --no-stack-control   # 禁止在页面上起/停整套软件
#
# ⚠️ 控制台本身**不启动**机器人节点；要么先在另一个终端跑 boot.sh，
#    要么用页面右上角的「▶ 启动整套」按钮（等价于跑 boot.sh）。
set -uo pipefail

WS=/home/user/robocup_basketball_ws
LIDAR_WS=/home/user/lidar_360

cd "$WS" || { echo "找不到 $WS"; exit 1; }

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "⚠️  还没构建过，先跑: rbws build"
    exit 1
fi

# 端口占用检查（避免起了但连的是旧进程）
PORT=8090
for a in "$@"; do
    case "$a" in
        --port) shift_next=1 ;;
        *) [ "${shift_next:-0}" = "1" ] && PORT="$a" && shift_next=0 ;;
    esac
done
if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    echo "⚠️  端口 $PORT 已被占用 —— 可能已有一个控制台在跑。"
    echo "    查看:  ss -ltnp | grep $PORT"
    echo "    杀掉:  pkill -f '[c]onsole_app'"
fi

# 本机 IP（方便手机访问）
IP=$(ip -4 -br addr show wlo1 2>/dev/null | awk '{print $3}' | cut -d/ -f1)
[ -z "$IP" ] && IP=$(ip -4 -br addr show 2>/dev/null | awk '/UP/{print $3}' | head -1 | cut -d/ -f1)

echo "🏀 启动综合控制台 …"
echo "   本机:   http://127.0.0.1:$PORT"
[ -n "$IP" ] && echo "   手机/其它电脑: http://$IP:$PORT"
echo ""

exec env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin:/opt/ros/humble/bin \
    DISPLAY="${DISPLAY:-:2}" bash -c "
        source /opt/ros/humble/setup.bash
        source '$LIDAR_WS/ws_livox/install/setup.bash' 2>/dev/null
        source '$WS/install/setup.bash'
        export ROS_DOMAIN_ID=2
        cd '$WS'
        exec python3 tools/console_app.py $*
    "
