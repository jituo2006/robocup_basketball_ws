#!/usr/bin/env bash
# 只启动六按键界面；运动由按钮调用的原脚本负责。
set -euo pipefail
pass_console_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$pass_console_root"
exec /usr/bin/python3 tools/console_app.py --pass-only "$@"
