#!/usr/bin/env python3
"""tools/ 下 GUI 脚本共用的窗口守卫。

为什么单独抽出来
----------------
"用户是否点了窗口关闭按钮"这件事，**不能**直接用
`cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1` 判断：
窗口刚被 `cv2.imshow` 创建、还没被窗口管理器**映射**的那一瞬间，该属性也返回 0。
若据此 break，程序会"刚打开就退出"——本项目因此在 view_camera.py、
calibrate_camera.py、tune_camera.py 三处各踩过一次（症状：一张都没采集就结束）。

正确做法：记住"曾经可见"，只有从【可见】变成【不可见】才算用户关闭窗口。

用法
----
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from gui_guard import WindowGuard

    guard = WindowGuard(win_name)
    while True:
        ...
        cv2.imshow(win_name, vis)
        cv2.waitKey(1)
        if guard.closed():
            break
"""

from __future__ import annotations

import cv2


class WindowGuard:
    """判断窗口是否被用户关闭，且不会把"还没映射"误判成"已关闭"。"""

    def __init__(self, name: str) -> None:
        self.name = name
        self.ever_visible = False

    def closed(self) -> bool:
        try:
            v = cv2.getWindowProperty(self.name, cv2.WND_PROP_VISIBLE)
        except Exception:  # noqa: BLE001  窗口不存在等情况，不能崩也不能误判
            return False
        if v >= 1:
            self.ever_visible = True
            return False
        return self.ever_visible


def add_tools_to_path() -> None:
    """把 tools/ 目录加进 sys.path，便于脚本 import 本模块。"""
    import sys
    from pathlib import Path

    here = str(Path(__file__).resolve().parent)
    if here not in sys.path:
        sys.path.insert(0, here)
