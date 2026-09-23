#!/usr/bin/env python3
"""雷达网络一致性检查。

为什么需要它
------------
Livox MID360 靠 UDP 收发点云/IMU，**主机 IP 必须与驱动配置一致**，否则点云永远
不会到达（而且驱动不一定报错，很容易误判成"雷达坏了"）。

这里踩过/查过的三个坑：
  1. JSON 结构是 `{ "MID360": { "host_net_info": {...} } }`，
     **不是**顶层 `host_net_info`（文档里写错过一次）。
  2. 驱动读的是 **install/share 里那份配置**（launch 用 `os.path.realpath(__file__)`
     定位到 install），**改了 src/ 里的不重建是不生效的**。
  3. 本机存在多份配置副本（ws_livox / ws_livox_38 / .bak），值不一样，容易看错。

用法：
    python3 tools/check_lidar_net.py
    python3 tools/check_lidar_net.py --ping        # 顺便 ping 雷达
退出码：0 一致；1 有问题
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

LIDAR_WS = Path("/home/user/lidar_360")
OK = "\033[32m"; WARN = "\033[33m"; FAIL = "\033[31m"
DIM = "\033[2m"; BOLD = "\033[1m"; RST = "\033[0m"

# 实际被 launch 使用的配置（install）与源头（src）
CANDIDATES = [
    ("生效(install)", LIDAR_WS / "ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"),
    ("源头(src)",     LIDAR_WS / "ws_livox/src/livox_ros_driver2/config/MID360_config.json"),
    ("另一套(38)",    LIDAR_WS / "ws_livox_38/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"),
    ("备份(.bak)",    LIDAR_WS / "ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json.bak_before_r1_lidar_test"),
]


def parse(path: Path) -> dict | None:
    """正确解析 MID360 配置（注意 host_net_info 嵌在 MID360 里）。"""
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return None
    net = (d.get("MID360") or {}).get("host_net_info") or {}
    lidar = (d.get("lidar_configs") or [{}])[0]
    return {
        "host": net.get("cmd_data_ip", "?"),
        "all_host_ips": sorted({v for k, v in net.items()
                                if k.endswith("_ip") and isinstance(v, str) and v}),
        "lidar": lidar.get("ip", "?"),
        "lidar_type": (d.get("lidar_summary_info") or {}).get("lidar_type"),
    }


def iface_ips() -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    try:
        res = subprocess.run(["ip", "-json", "addr"], capture_output=True, text=True, timeout=5)
        for item in json.loads(res.stdout or "[]"):
            name = item.get("ifname", "?")
            addrs = [a.get("local") for a in item.get("addr_info", []) if a.get("family") == "inet"]
            if addrs:
                out[name] = addrs
    except Exception:  # noqa: BLE001
        pass
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ping", action="store_true", help="顺便 ping 一次雷达")
    args = ap.parse_args()

    print(f"\n{BOLD}雷达网络一致性检查{RST}")
    print("=" * 76)

    seen: dict[str, dict] = {}
    for label, path in CANDIDATES:
        if not path.exists():
            continue
        info = parse(path)
        if info is None:
            print(f"  {WARN}!{RST} {label:<16} {DIM}解析失败{RST}  {path}")
            continue
        seen[label] = info
        short = str(path).replace(str(LIDAR_WS) + "/", "")
        print(f"  {label:<16} 雷达={info['lidar']:<16} 主机={info['host']:<16}")
        print(f"  {DIM}{'':16} {short}{RST}")

    problems: list[str] = []
    notes: list[str] = []

    if "生效(install)" not in seen:
        problems.append("找不到 install 里实际生效的 MID360_config.json")
    if "源头(src)" not in seen:
        notes.append("找不到 src 里的配置（不影响运行，但改不了配置）")

    # ① src 与 install 是否一致 —— 这是最容易踩的坑
    if "生效(install)" in seen and "源头(src)" in seen:
        a, b = seen["生效(install)"], seen["源头(src)"]
        if (a["host"], a["lidar"]) != (b["host"], b["lidar"]):
            problems.append(
                f"src 与 install 的配置**不一致**（src: 主机={b['host']} 雷达={b['lidar']}；"
                f"install: 主机={a['host']} 雷达={a['lidar']}）—— "
                f"说明改了 src 但没重建，**驱动实际用的是 install 那份**")

    active = seen.get("生效(install)") or seen.get("源头(src)")
    if not active:
        print(f"\n{FAIL}✗ 读不到任何可用配置{RST}")
        return 1

    # ② 主机 IP 是否真的在本机网卡上
    ips = iface_ips()
    host = active["host"]
    if host in ("?", ""):
        problems.append("配置里的主机 IP 为空")
    else:
        owner = [n for n, a in ips.items() if host in a]
        if owner:
            print(f"\n  {OK}✓{RST} 主机 IP {host} 已在网卡 {owner[0]} 上")
        else:
            problems.append(
                f"主机 IP {host} **不在本机任何网卡上** → 点云不会到达。"
                f"当前网卡: " + ", ".join(f"{n}={','.join(a)}" for n, a in ips.items()))

    # ③ host_net_info 里各个 IP 是否统一
    if len(active["all_host_ips"]) > 1:
        problems.append(f"host_net_info 里 IP 不统一: {active['all_host_ips']}")

    # ④ 雷达型号
    if active.get("lidar_type") not in (8, None):
        notes.append(f"lidar_type={active['lidar_type']}（MID360 应为 8）")

    # ⑤ ws_livox_38 那份是否会被误用
    if "另一套(38)" in seen and seen["另一套(38)"]["lidar"] != active["lidar"]:
        notes.append(f"ws_livox_38 用的是另一台雷达（{seen['另一套(38)']['lidar']}），"
                     f"别 source 错工作空间")

    # ⑥ 可选 ping
    if args.ping and host not in ("?", ""):
        try:
            r = subprocess.run(["ping", "-c", "2", "-W", "1", active["lidar"]],
                               capture_output=True, text=True, timeout=8)
            if r.returncode == 0:
                print(f"  {OK}✓{RST} ping {active['lidar']} 通")
            else:
                problems.append(f"ping {active['lidar']} 不通 —— 检查网线/网段/雷达供电")
        except Exception as exc:  # noqa: BLE001
            notes.append(f"ping 未执行: {exc}")

    print("\n" + "=" * 76)
    for n in notes:
        print(f"  {WARN}!{RST} {n}")
    if problems:
        print(f"\n{FAIL}✗ 发现 {len(problems)} 个问题:{RST}")
        for p in problems:
            print(f"  {FAIL}·{RST} {p}")
        print(f"\n{BOLD}怎么办{RST}")
        print("  ① 想改主机/雷达 IP：改 **install 里那份**才立即生效，或改 src 后重建：")
        print(f"     {DIM}cd {LIDAR_WS}/ws_livox && colcon build --symlink-install --packages-select livox_ros_driver2{RST}")
        print("  ② 让网卡跟上配置（推荐，改 ip 不用重建）：")
        print(f"     {DIM}sudo ip addr add {host}/24 dev <网卡名>{RST}")
        print(f"  ③ 确认雷达供电与网线，然后 python3 tools/check_lidar_net.py --ping")
        return 1

    print(f"\n{OK}✓ 雷达网络配置一致。{RST}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
