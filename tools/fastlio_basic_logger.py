#!/usr/bin/env python3
"""单独运行 FAST-LIO，保留初始化/状态/异常日志，过滤逐帧调试输出。

使用前应结束旧 FAST-LIO 节点并加载 ROS 环境。本工具不管理其他节点。
"""
import argparse
import datetime
import re
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", default=(
        "/home/user/lidar_360/fast_prop_ws/install/fast_lio/lib/fast_lio/fastlio_mapping"))
    parser.add_argument("--config", default=(
        "/home/user/lidar_360/fast_prop_ws/src/FAST_LIO_WITH_PROPAGATE/config/mid360.yaml"))
    args = parser.parse_args()
    cmd = [args.executable, "--ros-args", "--params-file", args.config, "--log-level", "info"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, errors="replace", bufsize=1)

    def stop(_sig, _frame):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print(datetime.datetime.now().isoformat(timespec="seconds"),
          "FAST-LIO started PID", proc.pid, "config", args.config, flush=True)
    last = {}
    essential = ("IMU Initial Done", "not Synced", "loop back", "Too few input",
                 "Self sync", "failed", "Failed")
    for line in proc.stdout:
        clean = re.sub(r"\x1b\[[0-9;]*m", "", line).strip()
        if not clean:
            continue
        if not (re.search(r"\[(INFO|WARN|ERROR|FATAL)\]", clean)
                or any(tag in clean for tag in essential)):
            continue
        key = re.sub(r"\[\d+\.\d+\]", "[time]", clean)
        now = time.monotonic()
        if now - last.get(key, -100) < 5:
            continue
        last[key] = now
        print(clean, flush=True)
    rc = proc.wait()
    print("FAST-LIO exited", rc, flush=True)
    return rc if rc >= 0 else 128 - rc


if __name__ == "__main__":
    raise SystemExit(main())
