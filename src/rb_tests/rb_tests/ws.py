"""rb_tests · RoboCup 篮球机器人集成/系统测试包。

与单元测试的分工
----------------
* `src/<各包>/test/`   单元测试：纯逻辑、无进程、毫秒级，用 colcon test 跑。
* `src/rb_tests/`      集成/系统测试：真的拉起节点、跑数据流，秒级。
                       夹具（假底盘/假检测/合成图）也放这里，供测试与 launch 共用。

为什么单独成包
--------------
这些脚本原来混在 tools/ 里，与"人工工具"（标定/调参/看图）平铺在一起，
既不好找、也不受 colcon test 管理。现在：
    测试 → 这里（colcon test 能跑、能接 CI）
    人工工具 → tools/
"""

from __future__ import annotations

import os
from pathlib import Path


def workspace_root() -> Path:
    """定位本工作空间根目录（含 src/rb_msgs 的那一层）。

    为什么要找而不是写死：包既可能从**源码**（--symlink-install 下 __file__
    指向 src/）被导入，也可能从 **install/** 被导入；colcon test 的 cwd 又可能是
    build/。所以从 cwd 和 __file__ 两个方向向上找标记目录，哪条路都能命中。

    可用 RB_WS 环境变量强制指定（CI 或特殊布局时）。
    """
    env = os.environ.get("RB_WS")
    if env:
        p = Path(env).resolve()
        if (p / "src" / "rb_msgs").is_dir():
            return p

    seen: set[Path] = set()
    for start in (Path.cwd(), Path(__file__).resolve()):
        for candidate in [start, *start.parents]:
            if candidate in seen:
                continue
            seen.add(candidate)
            if (candidate / "src" / "rb_msgs").is_dir():
                return candidate
    # 兜底：返回 cwd，调用方若需要可自行判断
    return Path.cwd()


WS = workspace_root()


def artifacts_dir(name: str) -> Path:
    """测试产物目录（自动创建），例如 artifacts_dir("verify")。"""
    d = WS / "test_artifacts" / name
    d.mkdir(parents=True, exist_ok=True)
    return d
