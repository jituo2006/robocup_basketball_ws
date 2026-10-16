#!/usr/bin/env bash
# 保留旧安装命令作为兼容入口；按用户要求改为桌面手动启动。
set -euo pipefail
pass_console_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo '传球控制已改为桌面手动启动，不再安装开机自启动。'
exec bash "$pass_console_root/tools/install_pass_console_desktop.sh" "$@"
