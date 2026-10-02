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
