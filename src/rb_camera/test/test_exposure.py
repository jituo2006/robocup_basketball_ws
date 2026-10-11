"""Exposure range validation and runtime writes, without a physical camera."""
import threading
from types import SimpleNamespace

import pytest
from rb_camera.exposure import exposure_capabilities, validate_exposure
from rb_camera.camera_node import CameraNode


def test_driver_range_and_readback_are_not_defaults():
    caps = exposure_capabilities(" exposure_time_absolute 0x009a0902 (int) : min=1 max=5000 step=1 default=157 value=95 flags=inactive")
    assert caps == dict(control="exposure_time_absolute", min=1, max=5000, step=1, default=157, value=95)
    assert validate_exposure(1, caps) == 1
    assert validate_exposure(5000, caps) == 5000


def test_other_driver_range_and_step():
    caps = exposure_capabilities(" exposure_absolute 0x009a0902 (int) : min=4 max=20000 step=4 value=100")
    assert validate_exposure(20000, caps) == 20000
    with pytest.raises(ValueError):
        validate_exposure(101, caps)
    assert exposure_capabilities("brightness 0x1 (int): min=0 max=255 value=1") is None


@pytest.mark.parametrize("value", [0, 5001, True, 1.5, "100"])
def test_rejects_values_outside_driver_domain(value):
    with pytest.raises(ValueError):
        validate_exposure(value, dict(min=1, max=5000, step=1))


def camera_stub():
    node = CameraNode.__new__(CameraNode)
    node._control_lock = threading.RLock()
    node._ok = True
    node.ae_enabled = True
    node._ae_current = 95
    node.controls_cfg = {"exposure": 95}
    node._ae_buf = [12]
    node.ae_settle_frames = 5
    caps = dict(control="exposure_time_absolute", min=1, max=5000, step=1, value=95)
    node._exposure_caps = lambda: dict(caps)
    node._v4l2_ctrls = lambda: {"auto_exposure": 1}
    writes = []
    def write(name, value):
        writes.append((name, value))
        if name == caps["control"]:
            caps["value"] = value
        return True
    node._set_ctrl = write
    return node, writes


def params(**values):
    return [SimpleNamespace(name=k, value=v) for k, v in values.items()]


def test_manual_max_disables_ae_and_survives_reconnect_config():
    node, writes = camera_stub()
    result = node._on_exposure_parameters(params(**{"exposure.value": 5000, "exposure.auto": False}))
    assert result.successful
    assert writes[-1] == ("exposure_time_absolute", 5000)
    assert not node.ae_enabled and node._ae_current == 5000
    assert node.controls_cfg["exposure"] == 5000 and not node._ae_buf
    node._adjust_auto_exposure = lambda frame: pytest.fail("manual mode must not run AE")
    node._auto_exposure(None)
    assert node._on_exposure_parameters(params(**{"exposure.auto": True})).successful
    assert node.ae_enabled


def test_invalid_atomic_request_has_no_hardware_side_effects():
    node, writes = camera_stub()
    result = node._on_exposure_parameters(params(**{"exposure.value": 5001, "exposure.auto": False}))
    assert not result.successful and not writes and node.ae_enabled
    result = node._on_exposure_parameters(params(**{"exposure.value": 5, "exposure.auto": False, "width": 640}))
    assert not result.successful and not writes


def test_driver_failure_does_not_claim_manual_mode():
    node, writes = camera_stub()
    node._set_ctrl = lambda name, value: False
    result = node._on_exposure_parameters(params(**{"exposure.value": 1, "exposure.auto": False}))
    assert not result.successful and node.ae_enabled and node._ae_current == 95


def test_readback_mismatch_is_rejected():
    node, writes = camera_stub()
    caps = node._exposure_caps()
    node._exposure_caps = lambda: caps
    result = node._on_exposure_parameters(params(**{"exposure.value": 1, "exposure.auto": False}))
    assert not result.successful and node.ae_enabled
    assert writes[-1] == ("exposure_time_absolute", 95)


def test_missing_named_camera_never_opens_other_device(monkeypatch):
    node, _ = camera_stub()
    node.device_name = "missing front camera"
    node.device = "/dev/video0"
    node.get_logger = lambda: SimpleNamespace(warn=lambda message: None)
    monkeypatch.setattr("rb_camera.camera_node.resolve_device_by_name", lambda name, log: None)
    monkeypatch.setattr("rb_camera.camera_node.cv2.VideoCapture",
                        lambda *args: pytest.fail("must not open another camera"))
    assert node._open_capture() is False
