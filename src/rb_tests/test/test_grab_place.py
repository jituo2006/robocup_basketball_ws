"""抓球并放下（机构序列调试）的测试。

⚠️ 机构动作有危险性，测试**只做静态检查和 dry-run**，绝不下发真实指令。
"""
import json
import subprocess
import sys
from pathlib import Path

import pytest

_WS = Path(__file__).resolve().parents[3]
_SCRIPT = _WS / "tools" / "test_grab_place.py"


def _run(*args):
    return subprocess.run([sys.executable, str(_SCRIPT), *args],
                          capture_output=True, text=True, timeout=30)


def test_script_exists_and_help_works_without_ros():
    """--help 不需要 ROS 环境（ROS 依赖必须延迟导入）。"""
    r = _run("--help")
    assert r.returncode == 0, r.stderr
    assert "preset" in r.stdout


def test_dry_run_needs_no_ros_and_prints_steps():
    """干跑不碰 ROS、不发指令，只打印步骤表 —— 现场先看再决定要不要真跑。"""
    r = _run("--preset", "grab_place", "--dry-run")
    assert r.returncode == 0, r.stderr
    assert "dry-run" in r.stdout
    assert "DRIBBLE" in r.stdout and "PAWL_TOGGLE" in r.stdout


def test_all_presets_are_well_formed():
    """每个预设的每一步动作码都必须是合法动作码（写错会在现场才炸）。"""
    sys.path.insert(0, str(_WS / "tools"))
    import importlib.util
    spec = importlib.util.spec_from_file_location("tgp", _SCRIPT)
    m = importlib.util.module_from_spec(spec)
    # 只执行到定义部分（不跑 main）：模块级没有副作用，直接 exec
    spec.loader.exec_module(m)
    assert m.PRESETS, "预设表不能为空"
    for name, steps in m.PRESETS.items():
        assert steps, f"预设 {name} 是空的"
        for s in steps:
            assert int(s["action"]) in m.ACTION_NAMES, f"{name} 里有非法动作码 {s}"
            assert float(s.get("hold_s", 0)) >= 0, f"{name} 里有负的保持时间"


@pytest.mark.parametrize("preset", ["grab_place", "grab_hold", "place_only", "slow_walk"])
def test_every_preset_dry_runs(preset):
    r = _run("--preset", preset, "--dry-run")
    assert r.returncode == 0, f"{preset} 干跑失败:\n{r.stderr}"


def test_custom_steps_dry_run():
    steps = json.dumps([{"action": 2, "hold_s": 1.0}, {"action": 4, "hold_s": 0.5}])
    r = _run("--steps", steps, "--repeats", "2", "--dry-run")
    assert r.returncode == 0, r.stderr
    assert "重复 2 遍" in r.stdout


def test_invalid_action_code_is_rejected():
    """非法动作码必须在发指令【之前】就被拦下。"""
    r = _run("--steps", '[{"action": 99}]', "--dry-run")
    assert r.returncode != 0
    assert "非法" in (r.stdout + r.stderr)


def test_bad_json_is_rejected():
    r = _run("--steps", "not-json", "--dry-run")
    assert r.returncode != 0


# ── 界面接线 ──────────────────────────────────────────────────────────────
def test_ui_has_grab_place_card():
    ui = (_WS / "tools" / "console_ui.html").read_text(encoding="utf-8")
    for k in ("cardGrabPlace", "gpPreset", "gpRun", "gpPreview", "GRAB_PLACE"):
        assert k in ui, f"界面里缺 {k}"


def test_ui_presets_match_script_presets():
    """前端预览的预设名必须和脚本里的对得上，否则选了会报"未知预设"。"""
    sys.path.insert(0, str(_WS / "tools"))
    import importlib.util
    spec = importlib.util.spec_from_file_location("tgp2", _SCRIPT)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    ui = (_WS / "tools" / "console_ui.html").read_text(encoding="utf-8")
    for name in m.PRESETS:
        assert f"{name}:" in ui, f"界面预览里缺预设 {name}"


def test_console_registers_the_tool():
    src = (_WS / "tools" / "console_app.py").read_text(encoding="utf-8")
    assert '"TEST_GRAB_PLACE"' in src
    assert "tools/test_grab_place.py" in src
    assert 'name == "GRAB_PLACE"' in src
    # ToolRunner 必须支持额外参数，否则界面上选的预设传不下去
    assert "extra_args" in src
