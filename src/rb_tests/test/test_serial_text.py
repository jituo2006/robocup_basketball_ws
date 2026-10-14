"""机构【文本协议】的测试：dji_stop / dji_zero / dji_move 1 90 5（抓）/ dji_move 1 0 5（放）。

⚠️ 这台机构的真实协议是文本行，不是仓库原来那套 11 字节二进制帧 ——
   "指令没反应"就是因为发的协议不对。测试只做静态检查，绝不真发指令。
"""
from pathlib import Path

_WS = Path(__file__).resolve().parents[3]


def test_service_definition_exists_and_documents_the_protocol():
    srv = _WS / "src/rb_msgs/srv/SerialText.srv"
    assert srv.is_file(), "缺 rb_msgs/srv/SerialText.srv"
    txt = srv.read_text(encoding="utf-8")
    assert "string command" in txt and "bool success" in txt
    for cmd in ("dji_stop", "dji_zero", "dji_move 1 90 5", "dji_move 1 0 5"):
        assert cmd in txt, f"服务注释里应写明现场协议，缺 {cmd}"


def test_service_is_registered_in_cmakelists():
    cm = (_WS / "src/rb_msgs/CMakeLists.txt").read_text(encoding="utf-8")
    assert "SerialText.srv" in cm, "CMakeLists 里没注册 SerialText.srv"


def test_node_creates_text_service_and_uses_send_string():
    src = (_WS / "src/rb_launcher/src/rb_launcher_node.cpp").read_text(encoding="utf-8")
    assert '#include "rb_msgs/srv/serial_text.hpp"' in src, "少了 include（编译会报 SerialText is not a member）"
    assert "create_service<rb_msgs::srv::SerialText>" in src
    assert "send_string" in src, "应复用已有的 send_string()"
    assert "line_ending_" in src, "行尾应可配置（固件可能要 \\r\\n）"


def test_line_ending_lives_under_ros_parameters():
    """⚠️ 参数必须在 rb_launcher.ros__parameters 下，写到顶层节点读不到。"""
    import yaml
    cfg = yaml.safe_load((_WS / "src/rb_launcher/config/launcher.yaml").read_text(encoding="utf-8"))
    params = cfg["rb_launcher"]["ros__parameters"]
    assert "line_ending" in params, "line_ending 得挂在 ros__parameters 下"
    assert params["line_ending"] in ("\n", "\r\n")


def test_console_exposes_text_actions():
    src = (_WS / "tools/console_app.py").read_text(encoding="utf-8")
    for name in ("LAUNCH_TEXT", "LAUNCH_INIT", "LAUNCH_GRAB", "LAUNCH_PLACE"):
        assert f'name == "{name}"' in src, f"后端缺动作 {name}"
    assert "def launcher_text" in src
    assert "/rb_launcher/text" in src


def test_ui_has_text_command_buttons():
    ui = (_WS / "tools" / "console_ui.html").read_text(encoding="utf-8")
    for k in ("cardSerialText", "LAUNCH_TEXT", "LAUNCH_INIT", "LAUNCH_GRAB",
              "LAUNCH_PLACE", "stCmd"):
        assert k in ui, f"界面缺 {k}"


def test_grab_place_presets_use_text_protocol():
    """抓球脚本的预设必须用文本指令（那才是这台机构的协议）。"""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "tgp3", _WS / "tools" / "test_grab_place.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    for name, steps in m.PRESETS.items():
        assert any(str(s.get("text", "")).strip() for s in steps), \
            f"预设 {name} 里一条文本指令都没有 —— 是不是又用回二进制动作码了？"
        for s in steps:
            txt = str(s.get("text", "")).strip()
            if txt.startswith("dji_move"):
                parts = txt.split()
                assert len(parts) == 4, f"{name} 里 MOVE 参数个数不对: {txt}"
