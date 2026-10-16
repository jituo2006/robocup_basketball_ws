from pathlib import Path
import shutil
import subprocess


def test_real_inline_script_handles_ready_failure_and_disconnect():
    root=Path(__file__).parents[3]
    node=shutil.which('node')
    assert node, 'Node is required for the frontend behaviour test'
    result=subprocess.run([node,str(Path(__file__).with_name('pass_ui_check.js')),
                           str(root/'tools/console_ui.html')],capture_output=True,text=True,timeout=10)
    assert result.returncode==0, result.stdout+result.stderr
