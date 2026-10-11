"""Video overlay boundaries, freshness, per-camera isolation and HTTP controls."""
import importlib.util
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest


@pytest.fixture
def console():
    spec = importlib.util.spec_from_file_location("_overlay_console", Path(__file__).resolve().parents[3] / "tools/console_app.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def jpeg():
    return cv2.imencode(".jpg", np.zeros((180, 320, 3), dtype=np.uint8))[1].tobytes()


def detection(label="ball_basketball", **updates):
    values = dict(label=label, confidence=0.8, source="camera", x_min=0.1,
                  y_min=0.3, x_max=0.4, y_max=0.8)
    values.update(updates)
    return SimpleNamespace(**values)


def bridge(console, monkeypatch):
    monkeypatch.setattr(console.time, "time", lambda: 100)
    node = console.Bridge.__new__(console.Bridge)
    node.lock = threading.Lock()
    node._overlay_lock = threading.Lock()
    node._overlay_options = console.overlay_options()
    node._overlay_cache = {}
    node.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    node.state = {"cam": jpeg(), "cam_stamp": 100, "cam_source_stamp": 100,
                  "dets": [detection()], "dets_stamp": 100, "dets_source_stamp": 100,
                  "cam_rear": jpeg(), "cam_rear_stamp": 100, "cam_rear_source_stamp": 100,
                  "dets_rear": [detection("ball_volleyball")], "dets_rear_stamp": 100,
                  "dets_rear_source_stamp": 100}
    return node


def test_both_class_boxes_have_distinct_colors(console):
    output = console.draw_ball_boxes(jpeg(), [detection(), detection("ball_volleyball", x_min=0.6, x_max=0.9)])
    image = cv2.imdecode(np.frombuffer(output, np.uint8), cv2.IMREAD_COLOR)
    orange, cyan = image[100, 32], image[100, 191]
    assert orange[2] > 150 and orange[0] < 80
    assert cyan[0] > 150 and cyan[2] < 80


@pytest.mark.parametrize("bad", [detection("hoop"), detection(source="lidar"),
    detection(confidence=0.1), detection(x_min=float("nan")),
    detection(x_min=0.5, x_max=0.2), detection(confidence=float("inf"))])
def test_non_camera_or_invalid_boxes_leave_jpeg_untouched(console, bad):
    raw = jpeg()
    assert console.draw_ball_boxes(raw, [bad]) is raw


def test_corrupt_jpeg_does_not_break_stream(console):
    assert console.draw_ball_boxes(b"not jpeg", [detection()]) == b"not jpeg"


def test_disabled_is_exact_passthrough_without_decode(console, monkeypatch):
    node = bridge(console, monkeypatch)
    node.camera_overlay_settings({"front": False})
    monkeypatch.setattr(console, "draw_ball_boxes", lambda *args: pytest.fail("disabled must not draw"))
    assert node.camera_frame("front") is node.state["cam"]


@pytest.mark.parametrize("changes", [dict(dets_stamp=99), dict(dets_source_stamp=99),
    dict(dets_source_stamp=101), dict(cam_source_stamp=99)])
def test_old_future_and_mismatched_detection_times_do_not_draw(console, monkeypatch, changes):
    node = bridge(console, monkeypatch)
    node.state.update(changes)
    assert node.camera_frame("front") is node.state["cam"]


def test_camera_disconnect_stops_replaying_old_image(console, monkeypatch):
    node = bridge(console, monkeypatch)
    node.state["cam_stamp"] = 98
    assert node.camera_frame("front") is None


def test_each_camera_uses_own_detections_and_render_cache(console, monkeypatch):
    node = bridge(console, monkeypatch)
    calls = []
    def draw(raw, detections, *args):
        label = detections[0].label
        calls.append(label)
        return label.encode()
    monkeypatch.setattr(console, "draw_ball_boxes", draw)
    assert node.camera_frame("front") == b"ball_basketball"
    assert node.camera_frame("front") == b"ball_basketball"
    assert node.camera_frame("rear") == b"ball_volleyball"
    assert calls == ["ball_basketball", "ball_volleyball"]


@pytest.mark.parametrize("changes", [{"front": "false"}, {"unknown": True},
    {"max_age_s": 0}, {"min_confidence": float("nan")}, {"jpeg_quality": 101}])
def test_invalid_settings_are_atomic(console, monkeypatch, changes):
    node = bridge(console, monkeypatch)
    original = node.camera_overlay_settings()
    with pytest.raises(ValueError):
        node.camera_overlay_settings(changes)
    assert node.camera_overlay_settings() == original


def test_http_switch_changes_existing_stream_and_keeps_detections(console, monkeypatch):
    node = bridge(console, monkeypatch)
    app = console.make_app(node, None, None, None, None, None, {})
    client = app.test_client()
    assert client.get("/api/camera_overlay").json["settings"]["front"] is True
    assert client.post("/api/camera_overlay", json={"front": False, "rear": False}).json["ok"]
    assert client.post("/api/camera_overlay", json={"front": "false"}).status_code == 400
    response = client.get("/api/camera.mjpg", buffered=False)
    chunk = next(response.response)
    assert node.state["cam"] in chunk and node.state["dets"][0].label == "ball_basketball"
    response.close()
    assert client.post("/api/camera_overlay", json={"front": True, "rear": True}).json["ok"]
    response = client.get("/api/camera_rear.mjpg", buffered=False)
    assert node.state["cam_rear"] not in next(response.response)
    response.close()
