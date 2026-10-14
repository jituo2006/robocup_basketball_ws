#!/usr/bin/env bash
# 修复 ROS/Livox 日志库 ABI 和断流退出；只编译安装，不启动机器人。
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_root="${1:-/home/user/lidar_360/Livox-SDK2}"
driver_root="${2:-/home/user/lidar_360/ws_livox/src/livox_ros_driver2}"
if pgrep -f '(^|/)livox_ros_driver2_node( |$)|(^|/)lidar_session.py( |$)' >/dev/null; then
  echo '雷达程序仍在运行，请先停止雷达/定位再安装，避免替换正在使用的库。' >&2
  exit 1
fi
python3 "$root/tools/livox_shutdown/patch_shutdown.py" --sdk-root "$sdk_root" --driver-root "$driver_root"
cmake -S "$sdk_root" -B "$sdk_root/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$sdk_root/build" -j 4
backup=/usr/local/lib/robocup-backups/livox_shutdown
sudo -n install -d "$backup"
for library in liblivox_lidar_sdk_shared.so liblivox_lidar_sdk_static.a; do
  if [[ ! -f "$backup/$library" ]]; then
    sudo -n cp -p "/usr/local/lib/$library" "$backup/$library"
  fi
done
sudo -n cmake --install "$sdk_root/build"
sudo -n ldconfig
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$(dirname "$(dirname "$driver_root")")"
colcon build --packages-select livox_ros_driver2 --executor sequential --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
python3 "$root/tools/livox_shutdown/patch_shutdown.py" --sdk-root "$sdk_root" --driver-root "$driver_root" --check
echo 'Livox 修复已安装；没有启动雷达、底盘或机构。'
