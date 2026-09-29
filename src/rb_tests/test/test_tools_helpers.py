"""tools/ 下脚本的辅助逻辑测试。

为什么需要：tools/ 不属于任何 ROS 包，一直**没有测试覆盖**，结果同一类 bug 犯了两次：
  ① `Node.executor` 是弱引用属性 → 赋值后执行器被 GC，GUI 工具起不来
     （view_camera.py 与 click_goto.py 各一次）
  ② `getWindowProperty(WND_PROP_VISIBLE) < 1` 在窗口**还没被映射**时也成立
     → 循环第一次迭代就 break，表现为"刚打开就退出、一张都没采集"
     （view_camera.py 与 calibrate_camera.py 各一次）

这些都不是靠"看代码"能发现的，只能靠测试或实跑。这里把能纯函数化的部分锁住。
运行：colcon test --packages-select rb_tests
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

import pytest

from rb_tests.ws import WS


def _load_tool(name: str):
    """从 tools/ 加载脚本模块（tools/ 不是 Python 包，只能按路径加载）。"""
    path = WS / "tools" / f"{name}.py"
    if not path.is_file():
        pytest.skip(f"{path} 不存在")
    spec = importlib.util.spec_from_file_location(f"_tool_{name}", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# _WindowGuard：窗口"刚创建还没映射"不能当成"用户关了窗口"
# ---------------------------------------------------------------------------
class _FakeCv2:
    """可脚本化返回值的 cv2 替身。"""

    def __init__(self, values):
        self.values = list(values)
        self.calls = 0

    def getWindowProperty(self, name, prop):  # noqa: N802
        v = self.values[min(self.calls, len(self.values) - 1)]
        self.calls += 1
        if isinstance(v, Exception):
            raise v
        return v


def test_window_guard_does_not_close_before_first_map(monkeypatch):
    """⭐ 回归测试：窗口还没被映射（属性=0）时**绝不能**判定为关闭。

    这正是"还没开始采集就结束"的根因。
    """
    gg = _load_tool("gui_guard")
    g = gg.WindowGuard("w")
    monkeypatch.setattr(gg.cv2, "getWindowProperty", lambda n, p: 0.0)
    assert g.closed() is False, "刚创建就判成关闭 → 会立刻退出"
    assert g.closed() is False


def test_window_guard_closes_after_being_visible(monkeypatch):
    """曾经可见 → 变不可见，才算用户点了关闭按钮。"""
    gg = _load_tool("gui_guard")
    g = gg.WindowGuard("w")
    fake = _FakeCv2([1.0, 1.0, 0.0])
    monkeypatch.setattr(gg.cv2, "getWindowProperty", fake.getWindowProperty)
    assert g.closed() is False     # 可见
    assert g.closed() is False     # 仍可见
    assert g.closed() is True      # 变不可见 → 关闭


def test_window_guard_tolerates_exception(monkeypatch):
    """底层抛异常时不能崩，也不能误判为关闭。"""
    gg = _load_tool("gui_guard")
    g = gg.WindowGuard("w")

    def boom(n, p):
        raise RuntimeError("no such window")

    monkeypatch.setattr(gg.cv2, "getWindowProperty", boom)
    assert g.closed() is False


# ---------------------------------------------------------------------------
# 自动采集的姿态去重
# ---------------------------------------------------------------------------
def test_pose_signature_and_dedup():
    """位置挪一点算重复；移到角落/明显拉近/明显倾斜算新姿态。

    标定精度取决于姿态多样性（尤其倾斜，用来打破平面标定的焦距-尺度退化），
    去重写错就会连拍一堆几乎一样的正对姿态。
    """
    import math

    import numpy as np

    cc = _load_tool("calibrate_camera")

    def corners(cx, cy, scale, ang, n=54):
        pts = []
        for i in range(n):
            u, v = (i % 9) - 4, (i // 9) - 3
            x, y = u * scale, v * scale
            pts.append([cx + x * math.cos(ang) - y * math.sin(ang),
                        cy + x * math.sin(ang) + y * math.cos(ang)])
        return np.array(pts, dtype=np.float32).reshape(-1, 1, 2)

    shape = (720, 1280)
    base = cc._pose_signature(corners(640, 360, 40, 0.0), shape)
    for cx, cy, sc, ang, expect_dup in [
        (640, 360, 40, 0.0, True),      # 完全相同
        (660, 370, 40, 0.0, True),      # 挪一点
        (1100, 620, 40, 0.0, False),    # 移到画面角落
        (640, 360, 90, 0.0, False),     # 明显拉近
        (640, 360, 40, 0.6, False),     # 明显倾斜
    ]:
        sig = cc._pose_signature(corners(cx, cy, sc, ang), shape)
        assert cc._too_similar(sig, [base], 0.10) is expect_dup, (cx, cy, sc, ang)


# ---------------------------------------------------------------------------
# 源码守卫：禁止再出现裸的窗口可见性判断
# ---------------------------------------------------------------------------
_GUI_TOOLS = ["calibrate_camera", "calibrate_color", "view_camera",
              "tune_camera", "click_goto"]


@pytest.mark.parametrize("name", _GUI_TOOLS)
def test_no_naive_window_visibility_check(name):
    """源码里不允许再出现裸的 `getWindowProperty(...) < 1 → break`。

    光测 `_WindowGuard` 类挡不住"有人又写回裸判断"。这个坑已经犯过两次
    （view_camera.py、calibrate_camera.py），所以用源码守卫明确禁掉。
    判定"用户关窗"必须走 `_WindowGuard.closed()`（或等价的 was_visible 记法）。
    """
    path = WS / "tools" / f"{name}.py"
    if not path.is_file():
        pytest.skip(f"{path} 不存在")
    bad = [
        (i + 1, line.strip())
        for i, line in enumerate(path.read_text(encoding="utf-8").splitlines())
        if "getWindowProperty" in line and "< 1" in line
    ]
    assert not bad, (
        f"{name}.py 又出现裸的窗口可见性判断（窗口未映射时会误判为关闭，"
        f"导致刚打开就退出）：{bad}"
    )


# ---------------------------------------------------------------------------
# 预览用快速检测：降采样后坐标必须正确放大回原分辨率
# ---------------------------------------------------------------------------
def _synthetic_board_frame():
    """造一张 1280x720、带倾斜棋盘的合成图（供检测类测试用）。"""
    import cv2
    import numpy as np

    cols, rows, sq = 9, 6, 20
    bw, bh = (cols + 1) * sq, (rows + 1) * sq
    board = np.full((bh, bw), 255, np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                board[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = 0
    k = np.array([[1108.0, 0, 640.0], [0, 1108.0, 360.0], [0, 0, 1]])
    o3 = np.float32([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0]])
    o2 = np.float32([[0, 0], [bw, 0], [bw, bh], [0, bh]])
    proj = cv2.projectPoints(o3, np.array([0.35, 0.25, 0.1]),
                             np.array([0.0, 0.0, 520.0]), k, None)[0].reshape(-1, 2)
    h = cv2.getPerspectiveTransform(o2, proj)
    frame = cv2.warpPerspective(board, h, (1280, 720), borderValue=255)
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def test_detect_preview_scales_coords_back():
    """⭐ detect_preview 在降采样图上检测，必须把坐标乘回原分辨率。

    写错的话角点会整体缩到画面中心附近，标定结果全错，但程序**不会报错**
    （只是 RMS 变差）—— 所以必须由测试锁住。
    """
    import cv2
    import numpy as np

    cc = _load_tool("calibrate_camera")
    gray = cv2.cvtColor(_synthetic_board_frame(), cv2.COLOR_BGR2GRAY)

    full = cc.find_corners(gray, (9, 6))
    prev = cc.detect_preview(gray, (9, 6))
    assert full is not None, "全分辨率没检出，合成图有问题"
    assert prev is not None, "降采样预览没检出"
    assert prev.shape == full.shape

    err = float(np.mean(np.linalg.norm(prev.reshape(-1, 2) - full.reshape(-1, 2), axis=1)))
    assert err < 3.0, f"降采样坐标没正确放大回去，平均偏差 {err:.2f}px"


def test_detect_preview_returns_none_without_board():
    import numpy as np

    cc = _load_tool("calibrate_camera")
    assert cc.detect_preview(np.full((720, 1280), 128, np.uint8), (9, 6)) is None


def test_sharpness_and_focus_hint():
    """清晰度指标：模糊图必须明显低于清晰图，分级提示要对得上。

    这个读数就是给用户判断"是不是离太近/失焦"用的（实测失焦时只有 9.9，
    清晰时 141）。
    """
    import cv2
    import numpy as np

    cc = _load_tool("calibrate_camera")
    gray = cv2.cvtColor(_synthetic_board_frame(), cv2.COLOR_BGR2GRAY)
    sharp = cc.sharpness(gray)
    blurry = cc.sharpness(cv2.GaussianBlur(gray, (31, 31), 0))

    assert blurry < sharp, "模糊图的清晰度分数没有下降"
    assert cc.focus_hint(blurry)[0].startswith("严重模糊")
    assert cc.focus_hint(sharp)[0] == "清晰"
    # 无细节的空白画面也应判为模糊
    assert cc.focus_hint(cc.sharpness(np.full((480, 640), 128, np.uint8)))[0].startswith("严重模糊")


# ---------------------------------------------------------------------------
# 棋盘格 PDF 必须是真实物理尺寸（否则打印会被缩放/裁切）
# ---------------------------------------------------------------------------
def test_board_pdf_has_exact_a4_page_size(tmp_path):
    """⭐ PDF 的 MediaBox 必须是精确 A4 210x297mm。

    PNG 不带物理尺寸元数据，打印软件猜 DPI 就会缩放/裁切 —— 本项目因此打印出
    只有 5~6 格宽的板子，导致 9x6 角点全检不出。PDF 是这条问题的根治方案。
    """
    cc = _load_tool("calibrate_camera")
    out = tmp_path / "board.pdf"
    cc.make_board_pdf(str(out), cols=9, rows=6, square_mm=20.0, paper="A4")
    assert out.is_file() and out.stat().st_size > 500, "PDF 没生成或为空"

    data = out.read_bytes()
    m = re.search(rb"/MediaBox\s*\[([^\]]+)\]", data)
    assert m, "PDF 里找不到 MediaBox"
    v = [float(x) for x in m.group(1).split()]
    w_mm = (v[2] - v[0]) / 72 * 25.4
    h_mm = (v[3] - v[1]) / 72 * 25.4
    assert w_mm == pytest.approx(210.0, abs=0.5), f"页宽 {w_mm:.1f}mm ≠ A4"
    assert h_mm == pytest.approx(297.0, abs=0.5), f"页高 {h_mm:.1f}mm ≠ A4"
