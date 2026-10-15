#!/usr/bin/env bash
# 在 NUC 当前桌面用户下安装；共用总电源上电后自动启动整车，不自动运行路线。
set -euo pipefail
pass_console_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pass_console_config="${XDG_CONFIG_HOME:-$HOME/.config}"
mkdir -p "$pass_console_config/systemd/user" "$pass_console_config/autostart"
cat > "$pass_console_config/systemd/user/robocup-pass-stack.service" <<EOF
[Unit]
Description=RoboCup stack started after shared hardware power on

[Service]
Type=simple
WorkingDirectory=$pass_console_root
ExecStart=/usr/bin/python3 "$pass_console_root/tools/pass_boot_watch.py" --run-stack
KillSignal=SIGINT
TimeoutStopSec=30
EOF
cat > "$pass_console_config/systemd/user/robocup-pass-console.service" <<EOF
[Unit]
Description=RoboCup six-button passing console
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
Name[zh_CN]=RoboCup 传球控制
Exec=/bin/bash "$pass_console_root/tools/open_pass_console.sh"
Terminal=false
X-GNOME-Autostart-enabled=true
StartupNotify=false
EOF
systemctl --user daemon-reload
systemctl --user enable --now robocup-pass-console.service
echo '已安装六按键传球控制台：桌面登录后自动显示 http://127.0.0.1:8090'
