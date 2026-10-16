#!/usr/bin/env python3
"""Migrate an idle console to one shared driver; retire the standalone fallback."""
import datetime
import json
from pathlib import Path
import shutil
import subprocess
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parent.parent


def command(*args):
    return subprocess.run(list(args), check=True, capture_output=True, text=True, timeout=45)


def active(unit):
    result = subprocess.run(['systemctl','--user','is-active',unit], capture_output=True, text=True)
    return result.stdout.strip() in ('active','activating','reloading','deactivating')


def idle(url):
    with urllib.request.urlopen(url, timeout=4) as response:
        state = json.load(response)
    if not isinstance(state, dict):
        raise RuntimeError('无法确认任务状态')
    task = state.get('task')
    if state.get('status') == 'running' or isinstance(task, dict) and task.get('status') == 'running':
        raise RuntimeError('任务正在执行，不能迁移或重启控制台')
    if state.get('chassis_trip',{}).get('activity','idle') != 'idle':
        raise RuntimeError('任务正在执行，不能迁移或重启控制台')


def main():
    was_active = active('robocup-pass-console.service')
    if was_active:
        try:
            idle('http://127.0.0.1:8090/api/shoot/state')
        except urllib.error.HTTPError as exc:
            if exc.code != 404:
                raise
            idle('http://127.0.0.1:8090/api/pass/state')
    if active('robocup-fallback-console.service'):
        idle('http://127.0.0.1:8091/api/fallback/state')
    if active('robocup-pass-stack.service') and not was_active:
        raise RuntimeError('独立整车程序运行中，无法确认任务空闲，未更改服务')
    backup = ROOT/'backups'/('integrated-trip-install-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    backup.mkdir()
    config = Path.home()/'.config/systemd/user'
    desktop = Path(command('xdg-user-dir','DESKTOP').stdout.strip())
    for name in ('robocup-pass-console.service','robocup-pass-stack.service','robocup-fallback-console.service','robocup-console-chassis.service'):
        if (config/name).exists(): shutil.copy2(config/name,backup/name)
    old_shortcut = desktop/'robocup-fallback.desktop'
    if old_shortcut.exists(): shutil.copy2(old_shortcut,backup/old_shortcut.name)
    if was_active:
        # Freeze admission under the server's task mutex before stopping services.
        def freeze(endpoint):
            request=urllib.request.Request('http://127.0.0.1:8090'+endpoint,data=b'{}',headers={'Content-Type':'application/json'})
            with urllib.request.urlopen(request,timeout=5) as response:
                if json.load(response).get('ok') is not True:
                    raise RuntimeError('控制界面未确认更新交接')
        try:
            freeze('/api/console/prepare-update')
        except urllib.error.HTTPError as exc:
            if exc.code != 404:
                raise
            freeze('/api/console/prepare-fallback')
    command('systemctl','--user','stop','robocup-pass-console.service','robocup-pass-stack.service')
    if (config/'robocup-fallback-console.service').exists():
        command('systemctl','--user','disable','--now','robocup-fallback-console.service')
        (config/'robocup-fallback-console.service').unlink()
    old_shortcut.unlink(missing_ok=True)
    console=(config/'robocup-pass-console.service').read_text()
    if 'Wants=robocup-console-chassis.service' not in console:
        console=console.replace('After=network.target', 'Wants=robocup-console-chassis.service\nAfter=network.target robocup-console-chassis.service')
    if 'KillSignal=' not in console: console=console.replace('RestartSec=2', 'RestartSec=2\nKillSignal=SIGINT\nTimeoutStopSec=12')
    (config/'robocup-pass-console.service').write_text(console)
    (config/'robocup-console-chassis.service').write_text(f"""[Unit]
Description=RoboCup shared chassis driver for the manually opened console
PartOf=robocup-pass-console.service

[Service]
Type=simple
WorkingDirectory={ROOT}
ExecStart=/bin/bash "{ROOT}/tools/start_console_chassis.sh"
Restart=on-failure
RestartSec=2
KillSignal=SIGINT
KillMode=mixed
TimeoutStopSec=8
""")
    retired=['tools/fallback_console.py','tools/fallback_ui.html','tools/prepare_fallback_mode.py',
             'tools/start_fallback_console.sh','tools/open_fallback_console.sh','tools/install_fallback_desktop.sh',
             'src/rb_tests/test/test_fallback_console.py','src/rb_tests/test/test_fallback_ros.py']
    for file in retired:
        source=ROOT/file
        if source.exists():
            target=backup/'retired'/file;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.move(source,target)
    command('systemctl','--user','daemon-reload')
    command('systemctl','--user','disable','robocup-pass-console.service')
    if was_active: command('systemctl','--user','start','robocup-pass-console.service')
    print('迁移完成，独立保底快捷方式和服务已移除。备份：'+str(backup))


if __name__ == '__main__':
    main()
