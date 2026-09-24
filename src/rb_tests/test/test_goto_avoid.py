"""集成测试：go_to_pose 服务 + 视觉势场避障（真的拉起 rb_mission 节点）。

比单元测试慢（约 15 秒），但它是唯一能证明"服务真的被注册、避障真的改了 /cmd_vel"
的一层。运行：colcon test --packages-select rb_tests
"""

from __future__ import annotations

import os
import subprocess
import sys

import pytest

import rb_tests.checks.verify_goto_avoid as vga
from rb_tests.ws import WS


@pytest.mark.integration
def test_goto_pose_and_obstacle_avoidance():
    env = dict(os.environ)
    # 独立 domain，避免与车上/其它测试互相干扰
    env["ROS_DOMAIN_ID"] = "78"
    proc = subprocess.run(
        [sys.executable, vga.__file__],
        capture_output=True, text=True, timeout=180, env=env, cwd=str(WS),
    )
    assert proc.returncode == 0, (
        "goto_pose / 避障集成测试失败：\n"
        + (proc.stdout or "")[-1500:] + (proc.stderr or "")[-800:]
    )
