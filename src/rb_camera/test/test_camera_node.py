"""rb_camera 驱动的单元测试（不需要相机）。

重点覆盖「锁定自动曝光/白平衡」用到的解析与别名解析：
这两处一旦写错，锁定会**悄无声息地失败**（读不到值就走默认分支），
而现场表现只是"颜色阈值偶尔不好使"，极难排查，所以必须由测试锁住。

下面 REAL_OUTPUT 是本机 HD USB Camera (05a3:9230) 的**真实** --list-ctrls 输出。
运行：colcon test --packages-select rb_camera
"""

from __future__ import annotations

import pytest
from rb_camera.camera_node import (
    _AUTO_EXPOSURE_MANUAL,
    _V4L2_ALIASES,
    parse_v4l2_ctrls,
    resolve_ctrl_name,
)

REAL_OUTPUT = """
User Controls

                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
                       contrast 0x00980901 (int)    : min=0 max=64 step=1 default=32 value=32
                     saturation 0x00980902 (int)    : min=0 max=128 step=1 default=60 value=60
                            hue 0x00980903 (int)    : min=-40 max=40 step=1 default=0 value=0
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                          gamma 0x00980910 (int)    : min=72 max=500 step=1 default=100 value=100
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=1 value=1 (50 Hz)
          white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=2 value=2
         backlight_compensation 0x0098091c (int)    : min=0 max=2 step=1 default=1 value=1

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=156 flags=inactive
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=1
"""


@pytest.fixture(scope="module")
def ctrls() -> dict[str, int]:
    return parse_v4l2_ctrls(REAL_OUTPUT)


# -- parse_v4l2_ctrls -------------------------------------------------------
def test_parses_all_controls(ctrls):
    assert ctrls["auto_exposure"] == 3
    assert ctrls["exposure_time_absolute"] == 156
    assert ctrls["white_balance_automatic"] == 1
    assert ctrls["white_balance_temperature"] == 4600
    assert ctrls["gain"] == 0
    assert ctrls["power_line_frequency"] == 1
    assert ctrls["brightness"] == 0


def test_ignores_headers_and_blank_lines(ctrls):
    """'User Controls' / 空行不能被当成控件名。"""
    assert "User" not in ctrls
    assert "Controls" not in ctrls
    assert "" not in ctrls
    assert len(ctrls) == 14


def test_handles_negative_values():
    text = "  brightness 0x00980900 (int) : min=-64 max=64 step=1 default=0 value=-12"
    assert parse_v4l2_ctrls(text)["brightness"] == -12


def test_handles_menu_suffix_and_flags(ctrls):
    """值后面跟 '(Aperture Priority Mode)' 和 'flags=inactive' 都要能正确解析。"""
    assert ctrls["auto_exposure"] == 3          # 后面跟着 "(Aperture Priority Mode)"
    assert ctrls["exposure_time_absolute"] == 156   # 后面跟着 "flags=inactive"


def test_uses_value_not_default():
    """输出里同一行既有 default= 又有 value=，必须取 value。

    取错会读到相机默认值而不是当前值，导致"以为读到了实际曝光"。
    """
    text = "  gain 0x00980913 (int) : min=0 max=100 default=7 value=42"
    assert parse_v4l2_ctrls(text)["gain"] == 42
    text2 = "  gain 0x00980913 (int) : min=0 max=100 default=7 value=7"
    assert parse_v4l2_ctrls(text2)["gain"] == 7


def test_empty_and_garbage_input():
    assert parse_v4l2_ctrls("") == {}
    assert parse_v4l2_ctrls("Cannot open device /dev/video0, exiting.") == {}


# -- resolve_ctrl_name ------------------------------------------------------
def test_resolve_exposure_prefers_canonical_name(ctrls):
    assert resolve_ctrl_name("exposure", ctrls) == "exposure_time_absolute"


def test_resolve_falls_back_to_alias():
    """有些驱动只有 exposure_absolute，没有 exposure_time_absolute。"""
    assert resolve_ctrl_name("exposure", {"exposure_absolute": 100}) == "exposure_absolute"
    assert resolve_ctrl_name("auto_exposure", {"exposure_auto": 1}) == "exposure_auto"


def test_resolve_unknown_returns_none(ctrls):
    assert resolve_ctrl_name("no_such_control", ctrls) is None


def test_resolve_all_friendly_names_present_on_this_camera(ctrls):
    """本机相机应能解析出配置里用到的全部友好名。"""
    for friendly in ("auto_exposure", "exposure", "white_balance_automatic",
                     "white_balance_temperature", "gain", "brightness",
                     "contrast", "saturation", "sharpness", "power_line_frequency"):
        assert resolve_ctrl_name(friendly, ctrls) is not None, friendly


# -- 常量约定 ---------------------------------------------------------------
def test_manual_exposure_constant_is_one():
    """UVC 里 auto_exposure=1 才是手动；写成 0 或 3 都会让"锁曝光"失效。"""
    assert _AUTO_EXPOSURE_MANUAL == 1


def test_alias_table_covers_required_controls():
    for name in ("auto_exposure", "exposure", "white_balance_automatic",
                 "white_balance_temperature"):
        assert name in _V4L2_ALIASES and _V4L2_ALIASES[name]
