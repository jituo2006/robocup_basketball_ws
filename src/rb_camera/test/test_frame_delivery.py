"""Camera freshness tests without opening a device or starting a ROS node."""
import threading
from types import SimpleNamespace as NS

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from rb_camera.camera_node import CameraNode


def frame_node():
    n = NS(_lock=threading.Lock(), _latest=np.zeros((4, 6, 3), np.uint8),
           _latest_stamp=Time(sec=100), _seq=1, _last_pub_seq=0,
           _frames_published=0, publish_raw=True, frame_id="camera_link")
    n.raw = []; n.info = []; n.compressed = []
    n.bridge = NS(cv2_to_imgmsg=lambda *a, **kw: Image())
    n.image_pub = NS(publish=n.raw.append)
    n.info_pub = NS(publish=n.info.append)
    n.compressed_pub = object()
    n._make_info = lambda stamp: stamp
    n._publish_compressed = lambda frame, stamp: n.compressed.append(stamp)
    return n


def test_old_frame_is_not_republished_or_given_a_new_timestamp():
    n = frame_node()
    for _ in range(20):
        CameraNode.publish_latest(n)
    assert len(n.raw) == len(n.info) == len(n.compressed) == 1
    assert n.raw[0].header.stamp.sec == n.compressed[0].sec == 100
    assert n._frames_published == 1
    n._seq = 2; n._latest_stamp = Time(sec=101)
    CameraNode.publish_latest(n)
    assert len(n.raw) == 2 and n.raw[-1].header.stamp.sec == 101


def test_compressed_only_does_not_convert_to_raw_image():
    n = frame_node(); n.publish_raw = False
    def unexpected_conversion(*args, **kwargs):
        raise AssertionError("raw conversion should not run")
    n.bridge.cv2_to_imgmsg = unexpected_conversion
    CameraNode.publish_latest(n)
    assert not n.raw and len(n.compressed) == 1


def test_capture_timestamp_precedes_exposure_processing():
    n = frame_node()
    n._running = True; n._read_failures = 0; n.flip = 0
    n.ae_enabled = True; n._frames_read = 0; n._ok = False
    clock = NS(sec=100)
    n.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=clock.sec)))
    n._cap = NS(read=lambda: (True, np.zeros((4, 6, 3), np.uint8)))
    def exposure(frame):
        clock.sec = 102
        n._running = False
    n._auto_exposure = exposure
    CameraNode._capture_loop(n)
    assert n._latest_stamp.sec == 100 and n._frames_read == 1
