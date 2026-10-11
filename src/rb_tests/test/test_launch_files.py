"""launch 文件的静态/渲染检查。

为什么需要
----------
`bringup_lidar_vision.launch.py` 曾经用 f-string 把 `LaunchConfiguration` 拼进
bash 命令：

    domain = LaunchConfiguration("ros_domain_id")
    setup = f"export ROS_DOMAIN_ID={domain} && "      # ← 调的是 __repr__！

f-string 不会"替换"LaunchConfiguration，只会调它的 `__repr__`，于是 bash 收到

    export ROS_DOMAIN_ID=<launch.substitutions...LaunchConfiguration object at 0x...>

这条命令非法 → bash 退出码 2 → **进程秒死**。症状极具误导性：
雷达驱动和 FAST-LIO 都没起来，RViz 里地图**一片黑** —— 看起来像"雷达坏了"，
其实是 launch 自己把命令写坏了。

**这类 bug 用眼睛看代码很难发现**（f-string 语法完全合法，也不报错），
所以这里用"把 launch 展开、检查渲染结果"的方式把它钉死。
"""

from __future__ import annotations

import importlib.util
import sys

import pytest
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions

from rb_tests.ws import WS

# 渲染时用到的 launch 参数（保持和 launch 文件里的默认值一致）
_LAUNCH_ARGS = {
    "ros_domain_id": "2",
    "dry_run": "false",
    "rviz": "false",
    "image_topic": "/camera/image_raw",
    "use_compressed": "true",
    "camera_device": "/dev/video0",
    "config_file": "",
}

# 未替换的 Python 对象 repr 一定包含这些片段
_TELLTALES = ("object at 0x", "LaunchConfiguration", "TextSubstitution object")


def _load_launch(name: str):
    """按路径加载 launch 文件（launch/ 不是 Python 包）。"""
    path = WS / "src" / "rb_bringup" / "launch" / f"{name}.launch.py"
    if not path.is_file():
        pytest.skip(f"{path} 不存在")
    spec = importlib.util.spec_from_file_location(f"_launch_{name}", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _walk(actions):
    for a in actions:
        yield a
        yield from _walk(getattr(a, "actions", None) or [])


def _flat(x):
    if isinstance(x, (list, tuple)):
        for i in x:
            yield from _flat(i)
    else:
        yield x


def _rendered_bash_commands(ld) -> list[str]:
    ctx = LaunchContext()
    for k, v in _LAUNCH_ARGS.items():
        ctx.launch_configurations[k] = v
    out = []
    for a in _walk(ld.entities):
        if not isinstance(a, ExecuteProcess):
            continue
        try:
            out.append(perform_substitutions(ctx, list(_flat(a.cmd))))
        except Exception:  # noqa: BLE001 - 某些命令含 locals 替换，渲染不了就跳过
            continue
    return out


def test_bringup_lidar_vision_bash_commands_are_fully_substituted():
    """⭐ 回归测试：launch 拼出来的 bash 命令里不能残留 Python 对象 repr。

    写坏的后果见模块文档：bash 退出码 2、雷达/FAST-LIO 秒死、RViz 一片黑。
    """
    mod = _load_launch("bringup_lidar_vision")
    cmds = _rendered_bash_commands(mod.generate_launch_description())

    assert cmds, "没渲染出任何 ExecuteProcess 命令（launch 结构变了吗？）"
    for cmd in cmds:
        for bad in _TELLTALES:
            assert bad not in cmd, (
                "launch 命令里残留了未替换的对象 repr，bash 会直接报错退出：\n"
                f"  含: {bad}\n  命令: {cmd[:300]}\n"
                "修法：不要把 LaunchConfiguration 塞进 f-string，"
                "改用 ExecuteProcess(additional_env={...})。"
            )


def test_bringup_lidar_vision_sources_lidar_ws():
    """雷达相关命令必须 source 雷达工作空间，否则找不到 livox/fast_lio 包。"""
    mod = _load_launch("bringup_lidar_vision")
    cmds = _rendered_bash_commands(mod.generate_launch_description())
    joined = "\n".join(cmds)
    assert "ws_livox/install/setup.bash" in joined, "没 source 雷达驱动工作空间"
    assert "fast_prop_ws/install/setup.bash" in joined, "没 source FAST-LIO 工作空间"


def test_all_launch_files_render_without_object_repr():
    """扫一遍所有 launch：任何 ExecuteProcess 命令都不该出现对象 repr。"""
    launch_dir = WS / "src" / "rb_bringup" / "launch"
    if not launch_dir.is_dir():
        pytest.skip("找不到 launch 目录")

    checked = 0
    for path in sorted(launch_dir.glob("*.launch.py")):
        name = path.name[: -len(".launch.py")]
        mod = _load_launch(name)
        try:
            ld = mod.generate_launch_description()
        except Exception:  # noqa: BLE001 - 需要外部包/参数时跳过
            continue
        for cmd in _rendered_bash_commands(ld):
            checked += 1
            assert "object at 0x" not in cmd, f"{path.name} 的命令未替换：{cmd[:200]}"
    if checked == 0:
        pytest.skip("没有可渲染的 ExecuteProcess 命令")


# ---------------------------------------------------------------------------
# 前后双摄：launch 必须把两个感知的话题参数分别传对
# ---------------------------------------------------------------------------
def test_rear_camera_in_launch():
    """后摄节点必须在 launch 里，并且用 camera_rear.yaml。"""
    from pathlib import Path
    ws = Path(__file__).resolve().parents[3]
    src = (ws / "src" / "rb_bringup" / "launch"
           / "bringup_lidar_vision.launch.py").read_text(encoding="utf-8")
    assert 'name="rb_camera_rear"' in src, "launch 里没有后摄节点"
    assert "camera_rear.yaml" in src, "后摄没用 camera_rear.yaml"
    assert 'name="rb_perception_rear"' in src, "launch 里没有后摄感知"


def test_rear_perception_gets_all_topic_params():
    """⚠️ 话题是 ROS 参数不是配置文件里的键 —— 必须逐个传对。

    踩过的坑：只传 image_topic 不够，用压缩图时节点读的是 compressed_topic，
    它的默认值指向【前摄】的图 → 后摄感知订了前摄画面、还把结果发到前摄话题上。
    """
    from pathlib import Path
    ws = Path(__file__).resolve().parents[3]
    src = (ws / "src" / "rb_bringup" / "launch"
           / "bringup_lidar_vision.launch.py").read_text(encoding="utf-8")
    i = src.find('name="rb_perception_rear"')
    assert i > 0
    seg = src[i:i + 1200]
    for key, val in (("image_topic", "/camera_rear/image_raw"),
                     ("compressed_topic", "/camera_rear/image_raw/compressed"),
                     ("output_topic", "/perception/detections_rear"),
                     ("debug_image_topic", "/perception/debug_image_rear")):
        assert key in seg, f"后摄感知少传了 {key}"
        assert val in seg, f"{key} 的值不对（应含 {val}）"


def test_rear_configs_have_rear_topics_and_names():
    """后摄的两个配置文件：设备名按【名字】认，话题带 _rear，内参留待标定。"""
    import yaml
    from pathlib import Path
    ws = Path(__file__).resolve().parents[3]
    cam = yaml.safe_load((ws / "src" / "rb_camera" / "config"
                          / "camera_rear.yaml").read_text(encoding="utf-8"))
    assert cam["camera"].get("device_name"), "后摄必须按名字认设备（编号会变）"
    assert "rear" in cam["output"]["image_topic"], "后摄话题应带 _rear"
    # ⚠️ publish_raw 和 publish_compressed 同类，放 camera: 块（output: 只放话题）
    assert cam["camera"].get("publish_raw") is False, \
        "后摄必须关掉原始图发布（1280x720 原始图 30Hz ≈80MB/s，GIL 下会把采集拖到 5fps）"
    # 前摄也要有 device_name，否则会去抢 /dev/video0
    front = yaml.safe_load((ws / "src" / "rb_camera" / "config"
                            / "camera.yaml").read_text(encoding="utf-8"))
    assert front["camera"].get("device_name"), "前摄也必须按名字认设备"


def test_rear_geometry_signs_are_opposite():
    """后摄的方位角换算必须和前摄相反（yaw+bearing vs yaw−bearing）。

    这是整个双摄改动里最容易写错、又最难从现象看出问题的地方：
    写反了车会朝镜像方向走。
    """
    import importlib.util
    import math
    from pathlib import Path
    ws = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        "_tpb2", ws / "tools" / "test_perimeter_ball.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    # 同一侧的目标，前后摄算出来的世界方位角应符号相反
    assert m.rear_world_angle(0.0, 0.3) == pytest.approx(m.bearing_to_world(0.0, -0.3))
    assert m.rear_world_angle(0.0, 0.3) != pytest.approx(m.bearing_to_world(0.0, 0.3))
    # 后摄在画面右侧(bearing>0)的目标 = 车的左侧 ⇒ 车体 y > 0
    pb = m.ball_body_from_rear(0.25, 0.3, 0.90)
    assert pb is not None and pb[1] > 0, "后摄画面右侧目标应在车的左侧(+y)"
    # 正后方：x 应为 -(后摄偏移 + 深度)
    pb0 = m.ball_body_from_rear(0.25, 0.0, 0.90)
    assert pb0[0] == pytest.approx(-1.15) and pb0[1] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# console.sh 的端口占用处理
# ---------------------------------------------------------------------------
def _console_sh() -> str:
    from pathlib import Path
    return (Path(__file__).resolve().parents[3] / "tools" / "console.sh").read_text(
        encoding="utf-8")


def test_console_sh_handles_port_in_use_instead_of_crashing():
    """端口被占时必须真正处理，不能只打印警告然后照样起。

    现场现象：console.sh 打印"端口 8090 已被占用"，然后照样 exec →
    Flask 报 Address already in use + terminate + 核心转储，看着像程序崩了。
    """
    src = _console_sh()
    assert "is_our_console" in src, "应有'是不是自家控制台'的判断"
    assert "--force" in src, "应支持 --force 接管"
    assert "自动改用空闲端口" in src, "被别的程序占用时应自动换端口"


def test_console_sh_console_detect_avoids_pipefail_trap():
    """判断里不能用 `curl ... | grep -q ...`。

    ⚠️ 脚本开头是 `set -o pipefail`，而 grep -q 一找到就退出 → 关掉管道 →
    curl 收 SIGPIPE → 退出码非 0 → pipefail 让整条管道判为失败。
    实测这样"明明是自家控制台却认不出来"，走错分支最后还是崩。
    """
    src = _console_sh()
    i = src.find("is_our_console()")
    assert i > 0
    # ⚠️ 按函数体精确截取（到第一个顶格的 "}"），不要用固定长度 ——
    #    取 900 字符会越界到后面那段，把别人的 grep -q 也算进来（踩过）。
    j = src.find("\n}", i)
    assert j > i, "找不到 is_our_console 的结尾"
    body = src[i:j]
    # ⚠️ 只看【代码行】，跳过注释 —— 注释里正要写"不能用 grep -q"来说明原因。
    #    第一版就是这么误报的（和之前 --params-file 那次一模一样）。
    code = [ln for ln in body.splitlines() if not ln.lstrip().startswith("#")]
    code_txt = "\n".join(code)
    assert "grep -q" not in code_txt, "判断里不能用 grep -q（pipefail 下会误判）"
    assert "case " in code_txt or "grep -c" in code_txt, \
        "应先用变量接住响应再匹配"


def test_console_sh_passes_port_to_app_explicitly():
    """自动换端口后必须把 --port 显式传给 app，否则它还是绑原端口。"""
    src = _console_sh()
    assert 'tools/console_app.py --port "$PORT"' in src, \
        'exec 时必须显式带 --port "$PORT"（否则自动换端口不生效）'


def test_front_camera_does_not_get_hardcoded_device():
    """⚠️ launch 不能给前摄传写死的 device。

    踩过：camera_device 默认 "/dev/video0" 传给前摄 → 前摄绕过了自己的
    device_name 解析、直接去开 /dev/video0；而后摄按名字解析出来也是
    /dev/video0 → 两个节点抢同一个设备：一个占着、另一个 ENODEV 打不开。
    实测现象就是"两个摄像头都打不开"。
    """
    from pathlib import Path
    ws = Path(__file__).resolve().parents[3]
    src = (ws / "src" / "rb_bringup" / "launch"
           / "bringup_lidar_vision.launch.py").read_text(encoding="utf-8")
    i = src.find('name="rb_camera"')
    assert i > 0
    seg = src[i:i + 600]
    assert '"device": camera_device' not in seg, \
        "前摄不能带写死的 device —— 会和后摄抢同一个 /dev/videoN"
    assert "camera.yaml" in seg, "前摄应只用 config_file（里面配 device_name）"
