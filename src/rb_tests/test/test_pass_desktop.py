"""Execute the installer in a temporary home, with systemctl/gio replaced."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.skipif(os.name=='nt',reason='Linux desktop installer')
def test_install_and_legacy_entry_leave_only_a_manual_desktop_launcher(tmp_path):
    root=Path(__file__).parents[3]
    workspace=tmp_path/'robot workspace'
    (workspace/'tools').mkdir(parents=True)
    for name in ('install_pass_console_desktop.sh','install_pass_console_autostart.sh'):
        shutil.copy2(root/'tools'/name, workspace/'tools'/name)
    home=tmp_path/'home'; desktop=home/'desktop'; desktop.mkdir(parents=True)
    config=home/'.config'; (config/'autostart').mkdir(parents=True)
    auto=config/'autostart/robocup-pass-console.desktop'
    auto.write_text('[Desktop Entry]\nX-GNOME-Autostart-enabled=true\n')
    commands=tmp_path/'bin'; commands.mkdir()
    for name,body in {
        'systemctl':'printf "%s\\n" "$*" >> "$TEST_UNIT_LOG"',
        'xdg-user-dir':'printf "%s\\n" "$TEST_DESKTOP_DIR"',
        'gio':'exit 0',
    }.items():
        p=commands/name; p.write_text('#!/bin/sh\n'+body+'\n');p.chmod(0o755)
    logfile=tmp_path/'units.log'
    env={**os.environ,'HOME':str(home),'XDG_CONFIG_HOME':str(config),
         'TEST_DESKTOP_DIR':str(desktop),'TEST_UNIT_LOG':str(logfile),
         'PATH':str(commands)+os.pathsep+os.environ['PATH']}
    for name in ('install_pass_console_desktop.sh','install_pass_console_autostart.sh'):
        p=subprocess.run(['bash',str(workspace/'tools'/name)],env=env,capture_output=True,text=True)
        assert p.returncode==0,p.stdout+p.stderr
    launcher=desktop/'robocup-pass-console.desktop'
    text=launcher.read_text()
    assert 'open_pass_console.sh' in text and 'Name[zh_CN]=投篮控制' in text
    assert launcher.stat().st_mode & 0o111
    assert 'Hidden=true' in auto.read_text() and 'enabled=false' in auto.read_text()
    calls=logfile.read_text().splitlines()
    assert calls.count('--user disable --now robocup-pass-console.service')==2
    assert not any(' enable ' in ' '+line+' ' or ' start ' in ' '+line+' ' for line in calls)
    assert len(list((workspace/'backups').iterdir()))==2
