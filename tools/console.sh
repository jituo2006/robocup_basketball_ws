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
        --force) ;;                      # 不是端口值，别让下面的匹配吃掉
        *) [ "${shift_next:-0}" = "1" ] && PORT="$a" && shift_next=0 ;;
    esac
done
# 端口占用检查
#
# ⚠️ 原实现只【打印警告】然后照样往下 exec —— Flask 起不来直接
#    "Address already in use" + terminate + 核心转储，看着像程序崩了，
#    其实只是端口被占。这里改成真正处理：
#      * 占着的是【我们自己的控制台】→ 告诉你地址，直接退出（不重复起）
#      * 是别的程序 → 自动往后找一个空闲端口
#      * --force → 把旧的我们自己的控制台杀掉，接管这个端口
FORCE=0
APP_ARGS=()
# ⚠️ 这些是 console.sh【自己】的开关，透传给 app 会让 argparse 报错或吞错参数：
#   --force      → "unrecognized arguments: --force"
#   --port [值]  → 我又显式传了 --port "$PORT"，再透传一次就变成
#                  "--port 8090 --port 8092" 甚至 "--port --port"，
#                  argparse 直接报 "argument --port: expected one argument"
#                  （现场：bash tools/console.sh --port 8092 起不来）
_pass_next=0
for a in "$@"; do
    if [ "$_pass_next" = "1" ]; then _pass_next=0; continue; fi   # 跳过 --port 的值
    case "$a" in
        --force)     FORCE=1 ;;
        --port)      _pass_next=1 ;;                              # 跳过它和它的值
        *)           APP_ARGS+=("$a") ;;
    esac
done

is_our_console() {
    # ⚠️ 不能用 `curl ... | grep -q ...`：脚本开头是 `set -o pipefail`，
    #    而 grep -q 一找到就退出 → 关掉管道 → curl 收到 SIGPIPE → 退出码非 0
    #    → pipefail 让整条管道判为失败。实测这样"明明是自家控制台却认不出来"，
    #    于是走错分支（去换端口），最后还是崩。
    #    改成先把响应收进变量，再用 shell 自己的匹配 —— 没有管道就没有这个坑。
    local body
    body=$(curl -s --max-time 3 "http://127.0.0.1:$1/" 2>/dev/null) || return 1
    case "$body" in
        *cardLidarBall*|*综合控制台*) return 0 ;;
        *) return 1 ;;
    esac
}

if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    if is_our_console "$PORT"; then
        if [ "$FORCE" = "1" ]; then
            echo "🔁 --force：停掉端口 $PORT 上的旧控制台，接管它"
            for _p in $(pgrep -f "[c]onsole_app"); do kill "$_p" 2>/dev/null; done
            sleep 2
            ss -ltn 2>/dev/null | grep -q ":$PORT " && {
                echo "❌ 旧控制台没停下来（$PORT 仍被占）—— 手动确认："
                ss -ltnp 2>/dev/null | grep ":$PORT "
                exit 1
            }
        else
            echo "✅ 端口 $PORT 上已经有一个控制台在跑 —— 不用重复起。"
            echo "    直接打开:  http://127.0.0.1:$PORT"
            IP0=$(ip -4 -br addr show 2>/dev/null | awk '/UP/{print $3}' | head -1 | cut -d/ -f1)
            [ -n "$IP0" ] && echo "    手机/其它电脑: http://$IP0:$PORT"
            echo "    要接管它:  bash tools/console.sh --force"
            exit 0
        fi
    else
        echo "⚠️  端口 $PORT 被【别的程序】占着（不是我们的控制台）"
        ss -ltnp 2>/dev/null | grep ":$PORT " | sed 's/^/      /'
        OLD_PORT=$PORT
        for cand in 8091 8092 8093 8094 8095 8096 8097 8098 8099; do
            if ! ss -ltn 2>/dev/null | grep -q ":$cand "; then
                PORT=$cand
                echo "    → 自动改用空闲端口 $PORT"
                break
            fi
        done
        if [ "$PORT" = "$OLD_PORT" ]; then
            echo "❌ $OLD_PORT~8099 都被占了 —— 用 --port 指定一个空闲端口"
            exit 1
        fi
    fi
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
        # ⚠️ 必须显式带 --port：上面可能因为端口被占【自动换了端口】，
        #    而 $* 里没有 --port，不显式传的话 app 还是会去绑原来的端口 → 崩。
        # ⚠️ 用 APP_ARGS（已剔掉 --force），不要用 $* ——
        #    否则 app 的 argparse 收到不认识的 --force 直接退出。
        #    显式带 --port：上面可能因端口被占【自动换过端口】。
        exec python3 tools/console_app.py --port "$PORT" "${APP_ARGS[@]}"
    "
