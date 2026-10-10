#!/usr/bin/env python3
"""一局一键：把车摆到出发点，跑这一条，整轮任务自动执行。

用法：
    rbws
    python3 tools/match_run.py                 # 默认投篮 SHOOT
    python3 tools/match_run.py --mission PASS  # 传球
    python3 tools/match_run.py --yes           # 跳过确认（脚本化用）
    python3 tools/match_run.py --dry           # 只做检查，不起任务
    python3 tools/match_run.py --practice      # 调试模式，不声明通过参赛验收

它做四件事：
    ① 确认整套软件在跑（否则提示先 bash tools/boot.sh）
    ② 跑 tools/check_ready.sh（CAN/雷达/IMU/点云/里程计/定位/未发散）
    ③ 检查车是否还停在出发点（定位锚点 = start_pose）
    ④ 起任务，然后实时打印阶段变化，直到任务结束

⚠️ 起完任务后 Ctrl-C 只是退出"监听"，任务仍在车上跑。
   要停车用：ros2 service call /rb_mission/set_mission rb_msgs/srv/SetMission "{mission: IDLE}"
   要急停用：bash tools/panic.sh
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

_SOURCE_WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SOURCE_WS / "src" / "rb_mission"))
from rb_mission.match_rules import competition_gaps, validate_match_config  # noqa: E402

WS = "/home/user/robocup_basketball_ws"
NODES = {  # 显示名: pgrep 模式（用 [] 防自匹配）
    "雷达驱动": "[l]ivox_ros_driver2_node",
    "FAST-LIO": "[f]astlio_mapping",
    "定位": "[l]ocalization_node",
    "底盘": "[c]hassis_node",
    "任务": "[m]ission_node",
}

MISSIONS = {
    "SHOOT": "投篮（目标球=排球，干扰=篮球，瞄篮筐）",
    "PASS": "传球（目标球=篮球，干扰=排球，瞄传球架）",
}

PHASE_CN = {
    "IDLE": "待命",
    "START_DELAY": "启动延迟（规则要求，期间不得有动作）",
    "SEEK_BALL": "找球并靠近",
    "ACQUIRE": "吸取",
    "NAV_TO_ZONE": "前往动作区",
    "ALIGN": "对准",
    "LAUNCH": "发射",
    "RETURN_HOME": "回位",
    "DONE": "完成",
    "ESTOP": "急停",
    "FAULT": "故障",
    "GOTO": "点地图导航",
}

C_OK, C_BAD, C_WARN, C_DIM, C_OFF = (
    "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m")


def _run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def step(n: int, total: int, msg: str) -> None:
    print(f"\n[{n}/{total}] {msg}")


def nodes_running() -> dict[str, bool]:
    out = {}
    for name, pat in NODES.items():
        r = _run(["pgrep", "-f", pat])
        out[name] = r.returncode == 0
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="一局一键：起任务并监控整轮流程")
    ap.add_argument("--mission", default="SHOOT", choices=sorted(MISSIONS),
                    help="任务类型（默认 SHOOT）")
    ap.add_argument("--yes", action="store_true", help="跳过确认")
    ap.add_argument("--dry", action="store_true", help="只检查，不起任务")
    ap.add_argument("--practice", action="store_true", help="调试模式；允许未完成参赛验收的工程调试")
    ap.add_argument("--timeout", type=float, default=420.0, help="监控最长时间(s)")
    args = ap.parse_args(argv)

    os.chdir(WS)
    mission = args.mission.upper()

    import yaml
    with open("src/rb_mission/config/mission.yaml", encoding="utf-8") as fh:
        mission_cfg = yaml.safe_load(fh)
    try:
        validate_match_config(mission_cfg)
    except (ValueError, TypeError, AttributeError) as exc:
        print(f"任务规则配置错误：{exc}")
        return 1
    gaps = competition_gaps(mission_cfg)
    if gaps:
        for gap in gaps:
            print(f"待完成：{gap}")
        if not args.practice:
            print("参赛验收未完成；当前只能用 --practice 做工程调试。")
            return 1
        print("--practice：本次仅做工程调试。")

    print("🏀 一局一键" + (f"  ——  任务 {mission}：{MISSIONS[mission]}"))

    # ── ① 整套软件在跑吗 ────────────────────────────────────────────────
    step(1, 4, "检查整套软件")
    st = nodes_running()
    for name, ok in st.items():
        print(f"      {C_OK + '✓' + C_OFF if ok else C_BAD + '✗' + C_OFF} {name}")
    if not all(st.values()):
        print(f"\n  {C_BAD}有节点没在跑{C_OFF} —— 先在另一个终端跑：")
        print("      rbws && bash tools/boot.sh")
        return 1

    # ── ② 就绪检查 ──────────────────────────────────────────────────────
    step(2, 4, "数据链就绪检查（约 1 分钟）")
    r = _run(["bash", "tools/check_ready.sh"])
    print("\n".join("      " + ln for ln in (r.stdout or "").rstrip().splitlines()))
    if r.returncode != 0:
        print(f"\n  {C_BAD}就绪检查没过{C_OFF} —— 按上面的提示处理后重跑。")
        return 1

    # ── ③ 车还在出发点吗 ────────────────────────────────────────────────
    step(3, 4, "检查车是否停在出发点")
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import yaml

    loc_cfg = yaml.safe_load(open("src/rb_localization/config/localization.yaml",
                                  encoding="utf-8")) or {}
    sp = (loc_cfg.get("odom_frame_conversion") or {}).get("start_pose") or {}
    sx, sy = float(sp.get("x", 0.0)), float(sp.get("y", 0.0))

    rclpy.init()
    node = rclpy.create_node("match_run_check")
    got: list[tuple[float, float]] = []

    def on_pose(m) -> None:
        p = m.pose.pose.position
        got.append((p.x, p.y))

    node.create_subscription(PoseWithCovarianceStamped, "/localization/pose", on_pose, 10)
    t0 = time.time()
    while time.time() - t0 < 6 and not got:
        rclpy.spin_once(node, timeout_sec=0.05)
    if got:
        x, y = got[-1]
        d = ((x - sx) ** 2 + (y - sy) ** 2) ** 0.5
        flag = C_OK + "✓" + C_OFF if d < 0.5 else C_WARN + "△" + C_OFF
        print(f"      {flag} 当前 ({x:+.2f}, {y:+.2f})   出发点 ({sx:+.2f}, {sy:+.2f})"
              f"   偏差 {d:.2f} m")
        if d >= 0.5:
            print(f"      {C_WARN}偏差偏大 —— 确认车确实摆在标记处？{C_OFF}")
    else:
        print(f"      {C_BAD}✗{C_OFF} 读不到位姿")
        rclpy.shutdown()
        return 1

    if args.dry:
        rclpy.shutdown()
        node.destroy_node()
        print(f"\n  {C_OK}--dry：检查都过了，没起任务{C_OFF}")
        return 0

    if not args.yes:
        print(f"\n  确认要把车交给任务 {mission} 吗？"
              f"（急停：另一个终端 bash tools/panic.sh）")
        if input("  输入 y 继续，其它取消 > ").strip().lower() != "y":
            print("  已取消。")
            rclpy.shutdown()
            node.destroy_node()
            return 0

    # ── ④ 起任务 + 监控 ─────────────────────────────────────────────────
    step(4, 4, f"起任务 {mission}")
    from rb_msgs.srv import SetMission
    from rb_msgs.msg import MissionStatus

    cli = node.create_client(SetMission, "/rb_mission/set_mission")
    if not cli.wait_for_service(timeout_sec=8.0):
        print(f"      {C_BAD}✗{C_OFF} /rb_mission/set_mission 服务不在")
        rclpy.shutdown()
        node.destroy_node()
        return 1

    req = SetMission.Request()
    req.mission = mission
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    if res is None or not getattr(res, "accepted", False):
        msg = getattr(res, "message", "无响应")
        print(f"      {C_BAD}✗{C_OFF} 任务被拒绝：{msg}")
        rclpy.shutdown()
        node.destroy_node()
        return 1
    print(f"      {C_OK}✓{C_OFF} 已接受：{getattr(res, 'message', '')}")

    delay = mission_cfg["timeouts"]["start_delay_s"]
    actions = mission_cfg["rules"]["actions_per_mission"]
    print(f"\n  自动流程：启动延迟 {C_DIM}{delay:g}s{C_OFF} → 找球/已确认目标球 → 动作区 → 对准 → 发射"
          f"  （{C_DIM}最多 {actions} 次服务动作周期，出球/命中仍需验证{C_OFF}） → 回位")
    print(f"  {C_DIM}下面实时打印阶段；Ctrl-C 只退出监听，任务仍在车上跑{C_OFF}\n")

    status_box: list = []
    node.create_subscription(MissionStatus, "/mission/status",
                             lambda m: status_box.append(m), 10)
    last_phase = None
    t_start = time.time()
    try:
        while time.time() - t_start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not status_box:
                continue
            m = status_box[-1]
            key = (m.phase, m.detail)
            if key != last_phase:
                last_phase = key
                el = time.time() - t_start
                cn = PHASE_CN.get(m.phase, m.phase)
                print(f"      {C_DIM}{int(el)//60:02d}:{int(el)%60:02d}{C_OFF} "
                      f"{m.phase:12} {cn}   {C_DIM}{m.detail}{C_OFF}")
            if m.phase in ("DONE", "IDLE") and time.time() - t_start > 2.0:
                print(f"\n  {C_OK}✅ 任务结束：{m.phase} —— {m.detail}{C_OFF}")
                break
            if m.phase in ("FAULT", "ESTOP"):
                print(f"\n  {C_BAD}⚠️ 任务中止：{m.phase} —— {m.detail}{C_OFF}")
                break
        else:
            print(f"\n  {C_WARN}监控超时（{args.timeout:.0f}s）—— 任务可能还在跑{C_OFF}")
    except KeyboardInterrupt:
        print(f"\n  {C_WARN}已停止监听（任务仍在车上跑）。停车：{C_OFF}")
        print('      ros2 service call /rb_mission/set_mission rb_msgs/srv/SetMission '
              '"{mission: IDLE}"')
    finally:
        rclpy.shutdown()
        node.destroy_node()
    return 0


if __name__ == "__main__":
    sys.exit(main())
