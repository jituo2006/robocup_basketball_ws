#!/usr/bin/env bash
# 六按键界面订阅设备状态；运动仍由按钮调用的原脚本负责。
set -eo pipefail
pass_console_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$pass_console_root"
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$pass_console_root/install/setup.bash" ]; then
  source "$pass_console_root/install/setup.bash"
fi
export ROS_DOMAIN_ID=2
exec /usr/bin/python3 tools/console_app.py --pass-only --task shoot "$@"
