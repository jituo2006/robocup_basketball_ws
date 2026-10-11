"""Cached YOLO boxes must not claim to come from the current image."""
import time
from types import SimpleNamespace as NS

import numpy as np
from builtin_interfaces.msg import Time
from rb_perception.detectors import Detection, OnnxDetector
from rb_perception.perception_node import PerceptionNode


class CachedDetector(OnnxDetector):
    def __init__(self):
        self.label = "yolo"
        self.inferred_this_frame = False
        self.calls = 0
        self.result = Detection(label="ball_basketball", confidence=.8,
                                bbox=(.1, .1, .5, .5))

    def detect(self, frame, cam):
        self.calls += 1
        self.inferred_this_frame = self.calls != 2
        return [self.result]


def test_cached_box_keeps_inference_stamp_until_next_inference():
    n = NS(cfg={}, detectors=[CachedDetector()], max_per_label=5,
           lidar_cfg={}, frame_count=0, last_summary="", publish_debug=False,
           _stats_n=0, _proc_ms_sum=0.)
    messages = []
    n.pub = NS(publish=messages.append)
    n.get_logger = lambda: NS(info=lambda *a, **kw: None)
    n.to_msg = lambda d, stamp: PerceptionNode.to_msg(n, d, stamp)
    for sec in [100, 101, 102]:
        PerceptionNode.process_frame(n, np.zeros((20, 30, 3), np.uint8),
                                     Time(sec=sec), "camera_link", time.perf_counter())
    assert [m.header.stamp.sec for m in messages] == [100, 101, 102]
    assert [m.detections[0].stamp.sec for m in messages] == [100, 100, 102]
