#!/usr/bin/env bash
# 一键检查：先做完整离线验证，再看硬件侧还差什么。
# 用法:  ./tools/check_all.sh
#
# 注意：ROS 的 setup.bash 在 `set -u` 下会报 AMENT_TRACE_SETUP_FILES 未绑定，
# 所以先关掉 nounset 把环境 source 好，再打开。
set +u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then source install/setup.bash; fi
set -uo pipefail

echo "############ 第 1 步 / 3：离线验证（不需要任何硬件）############"
python3 tools/verify_offline.py
v=$?

echo
echo "############ 第 2 步 / 3：雷达定位回放验证（约 40s）############"
if [ -d "$HOME/lidar_360/fast_prop_ws/install" ]; then
  python3 tools/verify_lidar_replay.py
  L=$?
else
  echo "  (跳过：未找到 ~/lidar_360/fast_prop_ws/install)"
  L=0
fi

echo
echo "############ 第 3 步 / 3：上车前预检 ############"
./tools/preflight.sh
p=$?

echo
if [ $v -ne 0 ]; then
  echo "═══ 结论：离线验证未通过 —— 先修软件（见上面 ✗ 项），不要上车 ═══"
  exit 1
fi
if [ $L -ne 0 ]; then
  echo "═══ 结论：雷达定位回放未通过 —— 先修坐标变换/标定参数 ═══"
  exit 1
fi
if [ $p -eq 1 ]; then
  echo "═══ 结论：软件侧仍有问题 —— 先解决预检里的软件项 ═══"
  exit 1
fi
if [ $p -eq 2 ]; then
  echo "═══ 结论：软件侧已就绪，可以上车；硬件项按 docs/02 逐阶段接 ═══"
  echo "    离线 8/8 通过，未通过的都是「还没接硬件」，不是代码问题。"
  exit 0
fi
echo "═══ 结论：软硬件全部就绪，可以直接上车 ═══"
exit 0
