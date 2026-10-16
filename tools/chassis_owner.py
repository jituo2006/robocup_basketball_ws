"""Ensure a single process owns the physical chassis driver."""
import os
from pathlib import Path


def acquire(name='driver'):
    import fcntl
    path = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')) / 'robocup-chassis-owner.lock'
    handle = path.open('a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError('另一套整车或保底程序正在使用底盘，请先结束并关闭它')
    handle.seek(0)
    handle.truncate()
    handle.write(name)
    handle.flush()
    return handle  # Keep open for the whole controller lifetime.
