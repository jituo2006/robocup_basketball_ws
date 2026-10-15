#!/usr/bin/env bash
# 安装服务默认不启用；--enable 才开始后台等待下一次断电→上电。
set -euo pipefail
auto_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
auto_user="${SUDO_USER:-$(id -un)}"
auto_enable=0
for option in "$@"; do
  case "$option" in --enable) auto_enable=1 ;; *) echo "用法: sudo bash tools/install_auto_round1.sh [--enable]"; exit 2 ;; esac
done
if [[ "$EUID" != 0 || "$auto_user" == root ]]; then
  echo '请由运行ROS的普通用户通过sudo安装，以便服务使用该用户。' >&2; exit 2
fi
/usr/bin/python3 - "$auto_root" "$auto_user" <<'PY'
from pathlib import Path
import json, re, sys
root, user = Path(sys.argv[1]), sys.argv[2]
if not re.fullmatch(r'[a-z_][a-z0-9_-]*',user) or any(c.isspace() for c in str(root)):
    raise SystemExit('用户或路径格式不支持')
cfg=json.loads((root/'tools/auto_round1.json').read_text())
if Path(cfg['workspace']) != root or cfg['state_dir'] != '/var/lib/robocup-auto-round1':
    raise SystemExit('auto_round1.json工作空间/状态目录与安装目标不一致')
template=(root/'tools/autostart/robocup-auto-round1.service').read_text()
Path('/etc/systemd/system/robocup-auto-round1.service').write_text(
    template.replace('@USER@',user).replace('@WORKSPACE@',str(root)))
PY
systemctl daemon-reload
if [[ "$auto_enable" == 1 ]]; then
  systemctl enable --now robocup-auto-round1.service
  systemctl is-enabled --quiet robocup-auto-round1.service
  systemctl is-active --quiet robocup-auto-round1.service
  echo '已启用开机启动和后台监视；完成或失败后自动恢复等待新的断电→上电。'
else
  echo '服务已安装，尚未启用：sudo systemctl enable --now robocup-auto-round1.service'
fi
