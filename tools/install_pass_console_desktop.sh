#!/usr/bin/env bash
# 安装手动桌面入口，关闭旧的systemd及桌面登录自启动。
set -euo pipefail
pass_console_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pass_console_config="${XDG_CONFIG_HOME:-$HOME/.config}"
pass_console_desktop=$(xdg-user-dir DESKTOP 2>/dev/null || true)
if [[ -z $pass_console_desktop ]]; then
  if [[ -d $HOME/桌面 ]]; then pass_console_desktop=$HOME/桌面; else pass_console_desktop=$HOME/Desktop; fi
fi
mkdir -p "$pass_console_root/backups" "$pass_console_config/systemd/user" \
  "$pass_console_config/autostart" "$pass_console_desktop"
pass_console_backup=$(mktemp -d "$pass_console_root/backups/console-desktop-$(date +%Y%m%d-%H%M%S)-XXXX")
for item in console stack; do
  source_unit="$pass_console_config/systemd/user/robocup-pass-$item.service"
  [[ ! -f $source_unit ]] || cp -p "$source_unit" "$pass_console_backup/$item.service"
done
[[ ! -f $pass_console_config/autostart/robocup-pass-console.desktop ]] || \
  cp -p "$pass_console_config/autostart/robocup-pass-console.desktop" "$pass_console_backup/autostart.desktop"
[[ ! -f $pass_console_desktop/robocup-pass-console.desktop ]] || \
  cp -p "$pass_console_desktop/robocup-pass-console.desktop" "$pass_console_backup/desktop.desktop"

systemctl --user disable --now robocup-pass-console.service
cat > "$pass_console_config/systemd/user/robocup-pass-stack.service" <<EOF
[Unit]
Description=RoboCup stack prepared by the manually opened passing console

[Service]
Type=simple
WorkingDirectory=$pass_console_root
ExecStart=/usr/bin/python3 "$pass_console_root/tools/pass_boot_watch.py" --run-stack
KillSignal=SIGINT
TimeoutStopSec=30
EOF
cat > "$pass_console_config/systemd/user/robocup-pass-console.service" <<EOF
[Unit]
Description=RoboCup six-button passing console (manual desktop start)
After=network.target

[Service]
Type=simple
WorkingDirectory=$pass_console_root
ExecStart=/bin/bash "$pass_console_root/tools/start_pass_console.sh"
Restart=on-failure
RestartSec=2

[Install]
WantedBy=default.target
EOF
cat > "$pass_console_config/autostart/robocup-pass-console.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=RoboCup Passing Control
Hidden=true
X-GNOME-Autostart-enabled=false
EOF
cat > "$pass_console_desktop/robocup-pass-console.desktop" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Passing Control
Name[zh_CN]=传球控制
Comment=打开传球界面并检查雷达与底盘状态
Exec=/bin/bash "$pass_console_root/tools/open_pass_console.sh"
Path=$pass_console_root
Icon=applications-engineering
Terminal=false
StartupNotify=false
EOF
chmod 755 "$pass_console_desktop/robocup-pass-console.desktop"
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/$(id -u)/bus}"
if command -v gio >/dev/null; then
  gio set "$pass_console_desktop/robocup-pass-console.desktop" metadata::trusted true || \
    echo '桌面环境未设置启动信任，请右键快捷方式选择“允许启动”。' >&2
fi
systemctl --user daemon-reload
echo "已关闭开机及登录自启动。点击桌面“传球控制”启动；备份：$pass_console_backup"
