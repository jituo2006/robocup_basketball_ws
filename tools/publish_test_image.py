#!/usr/bin/env python3
"""兼容入口 —— 实现已迁移到 `rb_tests` 包（测试资产与主代码分离）。

推荐用法（不再依赖 cwd / 相对路径）：

    ros2 run rb_tests publish_test_image

保留本文件只是为了不打断文档与习惯里的 `python3 tools/publish_test_image.py`。
真正实现在 src/rb_tests/rb_tests/fakes/publish_test_image.py
"""

from __future__ import annotations

import sys

try:
    from rb_tests.fakes.publish_test_image import main
except ImportError as exc:  # pragma: no cover
    sys.exit(
        f"导入 rb_tests 失败：{exc}\n"
        "请先加载工作空间：\n"
        "    source /opt/ros/humble/setup.bash && source install/setup.bash"
    )

if __name__ == "__main__":
    sys.exit(main())
