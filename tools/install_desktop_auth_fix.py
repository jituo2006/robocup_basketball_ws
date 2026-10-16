#!/usr/bin/env python3
"""Target the observed Ubuntu update-lock and Clash service prompts only.

Run via sudo, with the desktop account as the argument. Uses its EXISTING
passwordless sudo for two fixed Clash helper commands; no blanket polkit bypass.
"""
import datetime
import os
from pathlib import Path
import pwd
import shutil
import sys


def install(user):
    if os.geteuid() != 0:
        raise SystemExit('请使用 sudo python3 tools/install_desktop_auth_fix.py user')
    account = pwd.getpwnam(user)
    if account.pw_uid == 0 or not user.replace('_', '').replace('-', '').isalnum():
        raise ValueError('无效的桌面用户名')
    home = Path(account.pw_dir)
    backup = Path('/var/backups') / ('robocup-desktop-auth-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    backup.mkdir(mode=0o700)
    changed = []

    def write(path, text, mode=0o644, owner=None):
        path = Path(path)
        if path.exists():
            target = backup / str(path).lstrip('/')
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')
        path.chmod(mode)
        if owner:
            os.chown(path, owner.pw_uid, owner.pw_gid)
        changed.append(str(path))

    # This action only asks whether apt/dpkg's locks are held. Keep updates on.
    write('/etc/polkit-1/localauthority/50-local.d/60-robocup-update-lock.pkla', f'''[Desktop package-lock status without authentication]
Identity=unix-user:{user}
Action=com.ubuntu.update-notifier.pkexec.package-system-locked
ResultAny=yes
ResultInactive=yes
ResultActive=yes
''')
    # CVR 2.0 invokes pkexec sh -c <fixed root-owned helper>. Rewrite ONLY those
    # two exact argument lists to noninteractive sudo, already allowed on this host.
    for helper in ('install-service', 'uninstall-service'):
        stat = Path('/usr/bin', helper).stat()
        if stat.st_uid != 0 or stat.st_mode & 0o022:
            raise RuntimeError('Clash辅助程序所有者或权限异常：' + helper)
    write('/usr/local/lib/robocup-auth/bin/pkexec', '''#!/usr/bin/python3
import os, sys
args = sys.argv[1:]
if len(args) == 3 and args[0] in ('sh', '/bin/sh', '/usr/bin/sh') and args[1] == '-c' and args[2] in ('/usr/bin/install-service', '/usr/bin/uninstall-service'):
    os.execv('/usr/bin/sudo', ['sudo', '-n', '--', args[2]])
os.execv('/usr/bin/pkexec', ['pkexec', *args])
''', 0o755)
    write('/usr/local/lib/robocup-auth/launch-clash', '''#!/bin/sh
PATH=/usr/local/lib/robocup-auth/bin:$PATH
export PATH
exec /usr/bin/clash-verge "$@"
''', 0o755)
    pairs = [(home/'.config/autostart/Clash Verge.desktop', home/'.config/autostart/Clash Verge.desktop', False),
             (Path('/usr/share/applications/Clash Verge.desktop'), home/'.local/share/applications/Clash Verge.desktop', True)]
    for source, target, uri in pairs:
        if source.exists():
            lines = source.read_text().splitlines()
            text = '\n'.join('Exec=/usr/local/lib/robocup-auth/launch-clash' + (' %u' if uri else '') if line.startswith('Exec=') else line for line in lines) + '\n'
            write(target, text, owner=account)
    (backup/'manifest.txt').write_text('\n'.join(changed) + '\n')
    print('认证修复已安装；备份：' + str(backup))
    print('更新锁状态检查立即生效；Clash提权处理从下次打开或登录生效，当前网络进程未重启。')


if __name__ == '__main__':
    install(sys.argv[1])
