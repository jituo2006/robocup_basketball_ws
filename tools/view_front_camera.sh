#!/usr/bin/env bash
# 在远程主机运行，用浏览器访问 http://主机IP:8082；不启动相机或机器人。
set -eo pipefail
front_view_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$front_view_root/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
exec /usr/bin/python3 "$front_view_root/tools/front_camera_web.py" "$@"
