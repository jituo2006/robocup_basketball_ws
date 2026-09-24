#!/usr/bin/env python3
"""RoboCup 篮球机器人 · 离线验证器。

在**没有任何硬件**（无 CAN、无串口、无相机、无雷达）的情况下，逐项验证
软件链路是否正确，直到"可以直接上车"的程度。

为什么用 rclpy 直接断言而不是 shell 里跑 ros2 topic echo
--------------------------------------------------------
本脚本把被测节点作为**子进程**拉起，自己在同一进程里订阅话题做断言。
好处：① 不依赖跨终端/跨命名空间的 DDS 发现，稳定；
     ② 能拿到真实数值做区间判断，而不是肉眼看输出。

用法：
    ros2 run rb_tests verify_offline.py            # 跑全部
    ros2 run rb_tests verify_offline.py -v         # 显示每个节点的日志
    ros2 run rb_tests verify_offline.py --only vision,chassis

退出码：0 = 全部通过；1 = 有失败
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from rb_tests.ws import WS  # 工作空间定位（原来靠 __file__ 推算，搬包后会错）
# 用独立的 ROS_DOMAIN_ID，避免与车上/别的测试互相干扰
TEST_DOMAIN = "77"

OK = "\033[32m"; FAIL = "\033[31m"; WARN = "\033[33m"
DIM = "\033[2m"; BOLD = "\033[1m"; RST = "\033[0m"


# ---------------------------------------------------------------------------
# 通用工具
# ---------------------------------------------------------------------------
@dataclass
class Result:
    name: str
    passed: bool
    detail: str = ""
    notes: list[str] = field(default_factory=list)


class Runner:
    """负责拉起/清理子进程，并收集它们的输出。"""

    def __init__(self, verbose: bool) -> None:
        self.verbose = verbose
        self.procs: list[tuple[subprocess.Popen, str, str]] = []
        self.logdir = WS / "test_artifacts" / "verify"
        shutil.rmtree(self.logdir, ignore_errors=True)
        self.logdir.mkdir(parents=True, exist_ok=True)

    def env(self) -> dict:
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = TEST_DOMAIN
        return env

    def spawn(self, name: str, cmd: list[str], cwd: Path | None = None) -> subprocess.Popen:
        log = open(self.logdir / f"{name}.log", "w")  # noqa: SIM115
        proc = subprocess.Popen(cmd, cwd=str(cwd or WS), stdout=log,
                                stderr=subprocess.STDOUT, env=self.env(),
                                start_new_session=True)
        self.procs.append((proc, name, str(self.logdir / f"{name}.log")))
        return proc

    def spawn_bash(self, name: str, script: str) -> subprocess.Popen:
        return self.spawn(name, ["bash", "-c", script])

    def read_log(self, name: str) -> str:
        for _, n, path in self.procs:
            if n == name:
                try:
                    return Path(path).read_text(encoding="utf-8", errors="replace")
                except OSError:
                    return ""
        return ""

    def alive(self, name: str) -> bool:
        for proc, n, _ in self.procs:
            if n == name:
                return proc.poll() is None
        return False

    def stop(self, name: str) -> None:
        for proc, n, _ in list(self.procs):
            if n != name:
                continue
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=8)
                except Exception:  # noqa: BLE001
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:  # noqa: BLE001
                        pass
            self.procs = [t for t in self.procs if t[0] is not proc]

    def stop_all(self) -> None:
        for proc, _, _ in self.procs:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except Exception:  # noqa: BLE001
                    pass
        deadline = time.time() + 8
        for proc, _, _ in self.procs:
            if proc.poll() is None:
                try:
                    proc.wait(timeout=max(0.1, deadline - time.time()))
                except Exception:  # noqa: BLE001
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:  # noqa: BLE001
                        pass

    def dump_failures(self) -> None:
        """有节点异常退出时，把日志尾部打出来，方便定位。"""
        for proc, name, path in self.procs:
            if proc.poll() is None:
                continue
            try:
                text = Path(path).read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            if "Traceback" in text or "Error" in text or proc.returncode not in (0, -2, -15):
                print(f"{DIM}  --- {name} 日志尾部 ---{RST}")
                for line in text.strip().splitlines()[-6:]:
                    print(f"{DIM}    {line}{RST}")


# ---------------------------------------------------------------------------
# 检查 1：构建完整性
# ---------------------------------------------------------------------------
def check_build(r: Runner) -> Result:
    pkgs = ["rb_msgs", "rb_chassis", "rb_launcher", "rb_perception",
            "rb_localization", "rb_mission", "rb_bringup"]
    missing = []
    for p in pkgs:
        if not (WS / "install" / p).is_dir():
            missing.append(p)
    if missing:
        return Result("构建完整", False, f"install 下缺少: {', '.join(missing)}",
                      ["colcon build --symlink-install"])
    notes = []
    # 关键可执行文件必须存在
    for rel in ["rb_chassis/lib/rb_chassis/rb_chassis_node",
                "rb_launcher/lib/rb_launcher/rb_launcher_node"]:
        if not os.access(WS / "install" / rel, os.X_OK):
            return Result("构建完整", False, f"缺少可执行 {rel}")
    for rel in ["rb_mission/lib/rb_mission/mission_node"]:
        if not (WS / "install" / rel).exists():
            return Result("构建完整", False, f"缺少入口 {rel}")
    return Result("构建完整", True, f"7 个包 + 关键入口齐全", notes)


# ---------------------------------------------------------------------------
# 检查 2：视觉检测（合成图）
# ---------------------------------------------------------------------------
EXPECTED_LABELS = {"ball_basketball", "ball_volleyball", "hoop", "rack_ring", "pillar"}

# 离线自检固定用**冻结的标准色配置**，与生产 perception.yaml 解耦。
# 原因：合成图用的是硬编码标准色，而现场会用 calibrate_color.py 把生产配置
# 改成你们真球的颜色（2026 规则各队自带球，可能完全不是橙色）。若自检读生产
# 配置，标定后就会误报失败。这里只检验"视觉管道通不通"。
TEST_PERCEPTION_CFG = WS / "src" / "rb_perception" / "config" / "perception_test.yaml"


def check_vision(r: Runner) -> Result:
    # 用包内模块的**绝对路径**拉起：不依赖 cwd，也不依赖 PYTHONPATH 是否配好
    import rb_tests.fakes.make_test_image as _mti

    out = subprocess.run([sys.executable, _mti.__file__,
                          "--check", "--config", str(TEST_PERCEPTION_CFG)],
                         capture_output=True, text=True, cwd=str(WS), env=r.env())
    text = out.stdout + out.stderr
    found = set()
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("✓"):
            parts = line[1:].split()
            if parts:
                found.add(parts[0])
    missing = EXPECTED_LABELS - found
    extra = found - EXPECTED_LABELS
    if missing:
        return Result("视觉检测", False, f"漏检: {', '.join(sorted(missing))}",
                      ["调 perception.yaml 的 hsv_ranges，然后重跑本检查"])
    if extra:
        return Result("视觉检测", False, f"出现预期外的类别（误检）: {', '.join(sorted(extra))}")
    return Result("视觉检测", True, f"5 类目标全部检出且无误检")


# ---------------------------------------------------------------------------
# 检查 3/4：底盘运动学 + 超时保护
# ---------------------------------------------------------------------------
def check_chassis(r: Runner, verbose: bool) -> Result:
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Int16MultiArray
    except ImportError as exc:
        return Result("底盘运动学", False, f"导入失败: {exc}")

    # 显式给定几何参数，使期望值确定 —— 否则会随 chassis.yaml 的改动而误报
    TEST_RADIUS = 0.10
    TEST_W = 0.40
    TEST_L = 0.40
    r.spawn_bash("chassis", "source /opt/ros/humble/setup.bash && "
                            f"source {WS}/install/setup.bash && "
                            "exec ros2 run rb_chassis rb_chassis_node "
                            "--ros-args -p dry_run:=true -p limit_acc:=100.0 "
                            f"-p wheel_radius:={TEST_RADIUS} -p width:={TEST_W} -p length:={TEST_L}")

    rclpy.init()
    node = rclpy.create_node("verify_chassis")
    wheels: list[list[int]] = []
    node.create_subscription(Int16MultiArray, "/chassis/wheel_speed",
                             lambda m: wheels.append(list(m.data)), 10)
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def spin_for(sec: float) -> None:
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

    # 等节点起来
    for _ in range(60):
        spin_for(0.1)
        if wheels:
            break
    if not wheels:
        rclpy.shutdown()
        return Result("底盘运动学", False, "收不到 /chassis/wheel_speed",
                      ["看 test_artifacts/verify/chassis.log"])

    # 持续发布直到真正收到"非零"轮速（DDS 发现要 1~2s，不能只看第一帧）
    def drive_until_nonzero(tw: Twist, sec: float):
        wheels.clear()
        end = time.time() + sec
        while time.time() < end:
            pub.publish(tw)
            spin_for(0.05)
            if wheels and any(v != 0 for v in wheels[-1]):
                return wheels[-1]
        return None

    # ---- 直行：vx=+0.5，期望 [+, +, -, -]，每轮约 ±34 ----
    vxmsg = Twist()
    vxmsg.linear.x = 0.5
    got = drive_until_nonzero(vxmsg, 4.0)
    if got is None:
        rclpy.shutdown()
        r.stop("chassis")
        return Result("底盘运动学", False, "发布 /cmd_vel 后收不到非零轮速",
                      ["看 test_artifacts/verify/chassis.log"])
    expect = 0.5 * math.cos(math.pi / 4) * (60.0 / (2 * math.pi * TEST_RADIUS))
    ok_shape = len(got) == 4 and got[0] > 0 and got[1] > 0 and got[2] < 0 and got[3] < 0
    ok_mag = len(got) == 4 and all(abs(abs(v) - expect) <= max(3.0, expect * 0.15) for v in got)

    # ---- 旋转：wz=+0.5。纯旋转时四轮必须【同号】，幅值≈expect_rot。----
    # 注意：具体是"正"还是"负"是物理装轮方向决定的（由 chassis.yaml 的
    # angular_z_sign 控制），**测试只校验同号 + 幅值**，绝对方向上车验证。
    wzmsg = Twist()
    wzmsg.angular.z = 0.5
    got_rot = drive_until_nonzero(wzmsg, 4.0)
    expect_rot = 0.5 * math.hypot(TEST_W / 2.0, TEST_L / 2.0) * (60.0 / (2 * math.pi * TEST_RADIUS))
    rot_ok = (got_rot is not None and len(got_rot) == 4
              and all(v > expect_rot * 0.5 for v in got_rot)     # 同为正
              or (len(got_rot) == 4
                  and all(v < -expect_rot * 0.5 for v in got_rot)))  # 同为负

    # ---- 超时保护：停发后应归零 ----
    stop = Twist()
    timeout_ok = False
    end = time.time() + 1.5
    last = got
    while time.time() < end:
        pub.publish(stop) if time.time() < end - 1.0 else None
        spin_for(0.05)
        if wheels:
            last = wheels[-1]
        if all(v == 0 for v in last):
            timeout_ok = True
            break

    rclpy.shutdown()
    r.stop("chassis")

    if not ok_shape:
        return Result("底盘运动学", False, f"直行符号不对: {got}（期望 [+, +, -, -]）")
    if not ok_mag:
        return Result("底盘运动学", False,
                      f"直行幅值不对: {got}（期望约 ±{expect:.0f}）每个轮子都要同幅值")
    if not rot_ok:
        return Result("底盘运动学", False,
                      f"旋转轮速不对: wz=+0.5 → {got_rot}（期望四轮同号且幅值≈{expect_rot:.0f}）")
    sign_note = "同为正" if got_rot and got_rot[0] > 0 else "同为负"
    detail = (f"vx=0.5→{got}（±{expect:.0f}）；wz=+0.5→{got_rot}（四轮{sign_note}≈{expect_rot:.0f}，"
              f"方向由 angular_z_sign 控制，上车验证）")
    if not timeout_ok:
        return Result("底盘运动学", False, f"{detail}；但超时后未归零（安全网失效！）")
    return Result("底盘运动学", True, f"{detail}；停发后归零（200ms 安全网有效）")


# ---------------------------------------------------------------------------
# 检查 5：机构节点健壮性（无串口也不能崩）
# ---------------------------------------------------------------------------
def check_launcher(r: Runner, verbose: bool) -> Result:
    try:
        import rclpy
        from std_msgs.msg import Bool
        from rb_msgs.srv import Launch
    except ImportError as exc:
        return Result("机构节点健壮性", False, f"导入失败: {exc}")

    r.spawn_bash("launcher", "source /opt/ros/humble/setup.bash && "
                             f"source {WS}/install/setup.bash && "
                             "exec ros2 run rb_launcher rb_launcher_node")

    rclpy.init()
    node = rclpy.create_node("verify_launcher")
    oks: list[bool] = []
    node.create_subscription(Bool, "/rb_launcher/ok", lambda m: oks.append(bool(m.data)), 10)
    cli = node.create_client(Launch, "/rb_launcher/launch")

    def spin_for(sec: float) -> None:
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_for(6.0)
    alive = r.alive("launcher")
    log = r.read_log("launcher")

    resp_success = None
    resp_msg = ""
    if alive and cli.wait_for_service(timeout_sec=6.0):
        req = Launch.Request()
        req.action = 0
        req.speed = 2800
        req.angle = 64
        fut = cli.call_async(req)
        end = time.time() + 6.0
        while time.time() < end and not fut.done():
            spin_for(0.05)
        if fut.done() and fut.result() is not None:
            resp_success = fut.result().success
            resp_msg = fut.result().message

    rclpy.shutdown()
    r.stop("launcher")

    if not alive:
        if "terminate called" in log or "Aborted" in log:
            return Result("机构节点健壮性", False,
                          "节点因打不开串口而崩溃（原版 R1_server 的行为，必须避免）")
        return Result("机构节点健壮性", False, "节点意外退出")
    if oks and oks[-1] is True:
        return Result("机构节点健壮性", True, "/rb_launcher/ok=true（串口实际存在）")
    if resp_success is None:
        return Result("机构节点健壮性", True,
                      "无串口时节点存活、ok=false（服务调用超时属正常）")
    if resp_success:
        return Result("机构节点健壮性", False, "串口不存在却返回 success=true，不合理")
    return Result("机构节点健壮性", True,
                  f"无串口时优雅降级：节点存活、ok=false、服务返回明确错误（{resp_msg[:28]}…）")


# ---------------------------------------------------------------------------
# 检查 6：EKF 数值正确性
# ---------------------------------------------------------------------------
def check_ekf(r: Runner) -> Result:
    sys.path.insert(0, str(WS / "src" / "rb_localization"))
    try:
        from rb_localization.ekf2d import Ekf2D
    except ImportError as exc:
        return Result("EKF 数值", False, f"导入失败: {exc}")

    e = Ekf2D()
    e.set_pose(0.0, 0.0, 0.0)
    for _ in range(100):
        e.predict(0.5, 0.0, 0.0, 0.01)
    x, y, yaw = e.pose
    if abs(x - 0.5) > 0.02 or abs(y) > 0.02 or abs(yaw) > 0.02:
        return Result("EKF 数值", False, f"直行 1s 预测错误: x={x:.3f} y={y:.3f} yaw={yaw:.3f}（期望 x≈0.5）")

    for _ in range(100):
        e.predict(0.0, 0.0, math.pi / 2, 0.01)
    if abs(e.pose[2] - math.pi / 2) > 0.02:
        return Result("EKF 数值", False, f"转向预测错误: yaw={e.pose[2]:.3f}（期望 1.571）")

    std_before = e.std[0]
    e.update_pose(0.55, 0.02, math.pi / 2)
    std_after = e.std[0]
    if not std_after <= std_before + 1e-9:
        return Result("EKF 数值", False, "位姿更新后协方差没有收敛")
    if abs(e.pose[0] - 0.55) > 0.1:
        return Result("EKF 数值", False, f"位姿更新未生效: x={e.pose[0]:.3f}")
    return Result("EKF 数值", True,
                  f"直行/转向/位姿更新正确；更新后 std {std_before:.3f}→{std_after:.3f}")


# ---------------------------------------------------------------------------
# 检查 7：状态机全流程
# ---------------------------------------------------------------------------
PHASE_SEEN = []


def check_mission(r: Runner, verbose: bool) -> Result:
    try:
        import rclpy
        from rb_msgs.msg import MissionStatus
        from rb_msgs.srv import SetMission
    except ImportError as exc:
        return Result("状态机全流程", False, f"导入失败: {exc}")

    r.spawn_bash("fsm", "source /opt/ros/humble/setup.bash && "
                        f"source {WS}/install/setup.bash && "
                        "exec ros2 launch rb_bringup bringup_fsm_test.launch.py")

    rclpy.init()
    node = rclpy.create_node("verify_mission")
    seen: list[str] = []
    details: dict[str, str] = {}

    def on_status(m) -> None:
        if not seen or seen[-1] != m.phase:
            seen.append(m.phase)
        details[m.phase] = m.detail

    node.create_subscription(MissionStatus, "/mission/status", on_status, 20)
    cli = node.create_client(SetMission, "/rb_mission/set_mission")

    def spin_for(sec: float) -> None:
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_for(8.0)
    if not cli.wait_for_service(timeout_sec=8.0):
        rclpy.shutdown()
        r.stop("fsm")
        return Result("状态机全流程", False, "找不到 /rb_mission/set_mission 服务")

    req = SetMission.Request()
    req.mission = "PASS"
    fut = cli.call_async(req)
    spin_for(3.0)

    required = ["START_DELAY", "SEEK_BALL", "ACQUIRE", "NAV_TO_ZONE", "ALIGN", "LAUNCH"]
    # 完整走一次动作循环（含 8s 规则延迟 + 两次跨场地导航）约需 60~120s
    deadline = time.time() + 150.0
    while time.time() < deadline and not all(p in seen for p in required):
        spin_for(0.5)

    rclpy.shutdown()
    r.stop("fsm")

    missing = [p for p in required if p not in seen]
    if missing:
        return Result("状态机全流程", False,
                      f"未到达阶段: {', '.join(missing)}；实际路径 {' → '.join(seen)}",
                      ["看 test_artifacts/verify/fsm.log 里的 [PASS] 迁移日志"])
    # 规则合规：启动延迟必须在 5~15s
    idx = seen.index("START_DELAY")
    return Result("状态机全流程", True,
                  f"{' → '.join(seen[:seen.index('LAUNCH') + 1])} …")


# ---------------------------------------------------------------------------
# 检查 8：整车离线启动（6 进程全起）
# ---------------------------------------------------------------------------
def check_offline_bringup(r: Runner, verbose: bool) -> Result:
    try:
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from rb_msgs.msg import DetectionArray
    except ImportError as exc:
        return Result("整车离线启动", False, f"导入失败: {exc}")

    r.spawn_bash("offline", "source /opt/ros/humble/setup.bash && "
                            f"source {WS}/install/setup.bash && "
                            "exec ros2 launch rb_bringup bringup_offline.launch.py")

    rclpy.init()
    node = rclpy.create_node("verify_offline")
    got = {"odom": 0, "det": 0, "pose": 0}
    node.create_subscription(Odometry, "/odom", lambda m: got.__setitem__("odom", got["odom"] + 1), 20)
    node.create_subscription(DetectionArray, "/perception/detections",
                             lambda m: got.__setitem__("det", got["det"] + 1), 5)
    node.create_subscription(PoseWithCovarianceStamped, "/localization/pose",
                             lambda m: got.__setitem__("pose", got["pose"] + 1), 20)

    end = time.time() + 18.0
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    rclpy.shutdown()
    r.stop("offline")

    missing = [k for k, v in got.items() if v == 0]
    if missing:
        return Result("整车离线启动", False,
                      f"以下话题无数据: {', '.join(missing)}（{got}）",
                      ["看 test_artifacts/verify/offline.log"])
    return Result("整车离线启动", True,
                  f"odom {got['odom']} / detections {got['det']} / pose {got['pose']} 条，全链路通")




# ---------------------------------------------------------------------------
# 检查 9：关机安全性（Ctrl+C 不能抛异常 / 段错误）
# ---------------------------------------------------------------------------
def check_clean_shutdown(r: Runner, verbose: bool) -> Result:
    """节点被 Ctrl+C 打断时必须干净退出：不能有 Traceback、RCLError、段错误。

    这一项很实际：如果关机时抛异常，现场就会看到一堆红字，
    而且"退出时没发出零速"是安全隐患。
    """
    r.spawn_bash("shutdown", "source /opt/ros/humble/setup.bash && "
                             f"source {WS}/install/setup.bash && "
                             "exec ros2 launch rb_bringup bringup_offline.launch.py")
    time.sleep(14)
    if not r.alive("shutdown"):
        return Result("关机安全性", False, "启动阶段就退出了")
    r.stop("shutdown")
    time.sleep(2)

    log = r.read_log("shutdown")
    bad = []
    if "Segmentation fault" in log:
        bad.append("Segmentation fault")
    if "RCLError" in log:
        bad.append("RCLError")
    # Traceback 只看 shutdown 阶段的（importlib 在信号期间偶发的一行不算）
    if "Traceback (most recent call last)" in log:
        bad.append("Traceback")

    if bad:
        return Result("关机安全性", False, f"关机时出现 {', '.join(bad)}",
                      [f"看 {r.logdir}/shutdown.log"])
    return Result("关机安全性", True, "6 个进程均干净退出，无异常/段错误")


# ---------------------------------------------------------------------------
CHECKS = [
    ("build", "构建完整", lambda r, v: check_build(r)),
    ("vision", "视觉检测", lambda r, v: check_vision(r)),
    ("chassis", "底盘运动学", check_chassis),
    ("launcher", "机构节点健壮性", check_launcher),
    ("ekf", "EKF 数值", lambda r, v: check_ekf(r)),
    ("mission", "状态机全流程", check_mission),
    ("bringup", "整车离线启动", check_offline_bringup),
    ("shutdown", "关机安全性", check_clean_shutdown),
]


def main() -> int:
    # ⚠️ 必须在使用 rclpy 之前设置：验证器**自己**也要在测试 domain 上，
    #    否则它订阅 domain 0、子进程发布 domain 77，永远收不到数据。
    os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN

    ap = argparse.ArgumentParser(description="RoboCup 篮球机器人离线验证")
    ap.add_argument("-v", "--verbose", action="store_true", help="显示节点日志尾部")
    ap.add_argument("--only", default="", help="只跑指定检查，逗号分隔（build,vision,chassis,launcher,ekf,mission,bringup）")
    args = ap.parse_args()

    selected = [c for c in CHECKS if not args.only or c[0] in args.only.split(",")]
    runner = Runner(args.verbose)

    print(f"\n{BOLD}RoboCup 篮球机器人 · 离线验证{RST}")
    print(f"{DIM}工作空间: {WS}{RST}")
    print(f"{DIM}ROS_DOMAIN_ID={TEST_DOMAIN}（测试专用，不影响车上）{RST}")
    print("=" * 74)

    results: list[Result] = []
    try:
        for key, name, fn in selected:
            print(f"\n{BOLD}[{name}]{RST}")
            t0 = time.time()
            try:
                res = fn(runner, args.verbose)
            except Exception as exc:  # noqa: BLE001
                res = Result(name, False, f"检查本身异常: {type(exc).__name__}: {exc}")
            res.detail = res.detail or ""
            results.append(res)
            mark = f"{OK}✓ 通过{RST}" if res.passed else f"{FAIL}✗ 失败{RST}"
            print(f"  {mark}  {res.detail}  {DIM}({time.time() - t0:.1f}s){RST}")
            for note in res.notes:
                print(f"    {DIM}→ {note}{RST}")
    finally:
        runner.dump_failures()
        runner.stop_all()

    print("\n" + "=" * 74)
    passed = sum(1 for r in results if r.passed)
    failed = [r for r in results if not r.passed]
    print(f"{BOLD}结果: {passed}/{len(results)} 通过{RST}")
    if failed:
        print(f"\n{FAIL}未通过的项:{RST}")
        for r in failed:
            print(f"  {FAIL}✗{RST} {r.name}: {r.detail}")
        print(f"\n{DIM}节点日志在 {runner.logdir}{RST}")
        return 1

    print(f"\n{OK}全部离线检查通过 —— 软件链路已就绪。{RST}")
    print(f"{DIM}下一步：跑 tools/preflight.sh 看硬件侧还差什么，然后按 docs/02 上车。{RST}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
