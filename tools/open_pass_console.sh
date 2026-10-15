#!/usr/bin/env bash
set -euo pipefail
systemctl --user start robocup-pass-console.service
/usr/bin/python3 - <<'PY'
import time
import urllib.request
for attempt in range(100):
    try:
        with urllib.request.urlopen('http://127.0.0.1:8090/api/pass/state', timeout=1):
            break
    except OSError:
        time.sleep(.2)
else:
    raise SystemExit('传球界面服务未启动，请查看 journalctl --user -u robocup-pass-console')
PY
# Chrome app 模式在 NUC 桌面上独立显示，不加载旧 R1 面板。
pass_console_profile="${XDG_CONFIG_HOME:-$HOME/.config}/robocup-pass-browser"
exec /usr/bin/google-chrome --user-data-dir="$pass_console_profile" \
  --app=http://127.0.0.1:8090 --start-maximized --no-first-run --no-default-browser-check
