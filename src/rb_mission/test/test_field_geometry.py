"""场地几何的配置自洽性测试。

为什么值得单独测：这些数字直接决定**得分档位**（10 分 / 30 分）和"位置是否合规"。
2026 规则正文不给半径（只说"详见场地示意图"），所以这些值来自 2025 规则文本
（第 253-256、366-367 行）。一旦有人改回占位值或改错圆心，分数就悄悄算错了——
不会有任何报错，只会"打得好但分数低"。

本测试锁住三件事：
  1. 规则正文给出的关键数值没被改回占位值；
  2. 配置里的导航目标点与半径**自洽**（目标点该在线外的在线外、该在线内的在线内）；
  3. 三分线圆心 ≠ 篮筐这个容易搞错的点。

历史占位值（错误的）：hoop=13.5, shoot_line_radius=3.0, three_point_radius=6.0
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

# 优先读**已安装**的配置（那才是节点运行时真正读的那份，能抓出"改了源码没重新 build"）；
# 没安装时退回源码路径。不依赖 rb_tests —— 那会造成包间循环依赖。
try:
    from ament_index_python.packages import get_package_share_directory
    CFG = Path(get_package_share_directory("rb_mission")) / "config" / "mission.yaml"
except Exception:  # noqa: BLE001
    CFG = Path(__file__).resolve().parents[1] / "config" / "mission.yaml"


@pytest.fixture(scope="module")
def field() -> dict:
    cfg = yaml.safe_load(CFG.read_text(encoding="utf-8"))
    return cfg["field"]


def _d(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


# -- 规则正文给出的数值 -----------------------------------------------------
def test_court_size(field):
    """§2.4-1：1/4 标准篮球场 14m x 7.5m。"""
    assert field["length_m"] == 14.0
    assert field["width_m"] == 7.5


def test_shoot_line_radius_is_from_rules(field):
    """2025 规则第 367 行：投篮边界线半径 2 米（原占位值 3.0 是错的）。"""
    assert field["shoot_line_radius_m"] == pytest.approx(2.0)


def test_three_point_radius_is_from_rules(field):
    """2025 规则第 254-255 行：三分线半径 3.75 米（原占位值 6.0 是错的）。"""
    assert field["three_point_radius_m"] == pytest.approx(3.75)


def test_hoop_is_not_the_old_placeholder(field):
    """篮筐位置：初赛与标准篮筐投影同心，距底线 1.575m → x = 14 − 1.575。

    原占位值 13.5（距底线 0.5m）在篮板后面，是错的。
    """
    assert field["hoop"]["x"] == pytest.approx(14.0 - 1.575, abs=0.01)
    assert field["hoop"]["y"] == pytest.approx(7.5 / 2)


# -- 三分线圆心 ≠ 篮筐 ------------------------------------------------------
def test_three_point_center_differs_from_hoop(field):
    """三分线圆心是"右侧底线中点向场内 3 米" → (11.0, 3.75)，不是篮筐。

    这个 1.425m 的差很容易被当成同一个点；代码里 is_three_point() 必须用
    three_point_center，否则 30 分档会判错。
    """
    c = field["three_point_center"]
    assert c["x"] == pytest.approx(14.0 - 3.0)
    assert c["y"] == pytest.approx(7.5 / 2)
    assert _d((c["x"], c["y"]), (field["hoop"]["x"], field["hoop"]["y"])) > 1.0


# -- 配置自洽：导航目标点与半径必须匹配 -------------------------------------
def test_shoot_zone_target_is_outside_shoot_line(field):
    """动作点必须在投篮边界线**外**，否则 LAUNCH 阶段会判"位置不合规"→ 永远退回导航。

    _zone_ok() 就是这么判的（允许 1cm 浮点余量）。
    """
    t = field["shoot_zone_target"]
    hoop = field["hoop"]
    assert _d((t["x"], t["y"]), (hoop["x"], hoop["y"])) >= field["shoot_line_radius_m"] - 0.01


def test_shoot_zone_target_is_inside_three_point_line(field):
    """默认动作点走**10 分档**：在三分线内（也必须在投篮线外）。"""
    t = field["shoot_zone_target"]
    c = field["three_point_center"]
    assert _d((t["x"], t["y"]), (c["x"], c["y"])) < field["three_point_radius_m"]


def test_shoot_zone_target_30_is_outside_three_point_line(field):
    """30 分档动作点必须在三分线**外**（且仍在投篮线外）。"""
    t = field["shoot_zone_target_30"]
    c = field["three_point_center"]
    hoop = field["hoop"]
    assert _d((t["x"], t["y"]), (c["x"], c["y"])) >= field["three_point_radius_m"]
    assert _d((t["x"], t["y"]), (hoop["x"], hoop["y"])) >= field["shoot_line_radius_m"]


def test_pass_zone_polygon_is_well_formed(field):
    """传球区多边形至少要能构成多边形（3 个点以上），否则 point_in_polygon 恒 False。"""
    poly = field["pass_zone"]["polygon"]
    assert len(poly) >= 3
    assert all(len(p) == 2 for p in poly)


def test_xy_are_inside_court_bounds(field):
    """所有动作点都应落在场地内（x∈[0,14], y∈[0,7.5]），否则是明显的坐标写错。"""
    for key in ("home", "hoop", "pass_zone_target", "shoot_zone_target",
                "shoot_zone_target_30", "three_point_center"):
        p = field[key]
        assert 0.0 <= p["x"] <= field["length_m"], f"{key}.x 越界"
        assert 0.0 <= p["y"] <= field["width_m"], f"{key}.y 越界"
