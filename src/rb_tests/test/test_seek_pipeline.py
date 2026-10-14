"""运行真实回调方法、替换 ROS 消息和设备接口，验证跨层观测时间戳。"""
import ast
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import yaml

from rb_perception.detectors import CameraModel, Detection, OnnxDetector

WS = Path(__file__).resolve().parents[3]


def methods(path, cls, names, namespace):
    tree = ast.parse(path.read_text())
    source = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
    selected = [n for n in source.body if isinstance(n, ast.FunctionDef) and n.name in names]
    unit = ast.Module(body=selected, type_ignores=[])
    exec(compile(unit, str(path), 'exec'), namespace)
    return namespace


def test_seek_config_disables_expensive_non_ball_detectors_and_preserves_calibration():
    root = WS / 'src/rb_perception/config'
    base, seek = (yaml.safe_load((root/name).read_text()) for name in ('perception.yaml', 'perception_seek.yaml'))
    assert seek['camera'] == base['camera']
    enabled = [d for d in seek['perception']['detectors'] if d['enabled']]
    assert len(enabled) == 1 and enabled[0]['type'] == 'onnx'
    assert enabled[0] == next(d for d in base['perception']['detectors'] if d['type'] == 'onnx')
    assert not seek['perception']['lidar_ball']['enabled']


def test_onnx_skip_does_not_report_new_inference():
    detector = OnnxDetector('yolo', {'model_path': '/nonexistent/model.onnx', 'every_n_frames': 2})
    detector.session = NS(run=lambda *a: [np.empty((1, 5, 0))])
    detector.input_name = 'images'
    detector.input_size = 640
    detector._decode = lambda result: (np.empty((0, 4)), np.empty(0), np.empty(0))
    frame = np.zeros((32, 32, 3), dtype=np.uint8)
    camera = CameraModel.from_config({}, 32, 32)
    detector.detect(frame, camera)
    assert not detector.inferred_this_frame
    detector.detect(frame, camera)
    assert detector.inferred_this_frame
    detector.detect(frame, camera)
    assert not detector.inferred_this_frame


def test_cached_detection_keeps_original_stamp_through_actual_process_and_message_conversion():
    ns = methods(WS/'src/rb_perception/rb_perception/perception_node.py', 'PerceptionNode',
                 {'process_frame', 'to_msg'}, dict(CameraModel=CameraModel, Detection=NS,
                                                 DetectionArray=NS, Header=NS, time=time, OnnxDetector=OnnxDetector))
    ball = Detection('ball_basketball', .8, (.1, .1, .2, .2))
    detector = OnnxDetector.__new__(OnnxDetector)
    detector.label, detector.inferred_this_frame = 'yolo', True
    detector.detect = lambda *args: [ball]
    messages = []
    node = NS(cfg={}, detectors=[detector], max_per_label=5, lidar_cfg={'enabled': False},
              frame_count=0, last_summary='', publish_debug=False, _stats_n=0, _proc_ms_sum=0,
              pub=NS(publish=messages.append), get_logger=lambda: NS(info=lambda s: None))
    node.to_msg = lambda d, stamp: ns['to_msg'](node, d, stamp)
    frame = np.zeros((32, 32, 3), dtype=np.uint8)
    ns['process_frame'](node, frame, 100, 'camera', time.perf_counter())
    detector.inferred_this_frame = False
    ns['process_frame'](node, frame, 101, 'camera', time.perf_counter())
    assert messages[-1].header.stamp == 101
    assert messages[-1].detections[0].stamp == 100
    detector.inferred_this_frame = True
    ns['process_frame'](node, frame, 102, 'camera', time.perf_counter())
    assert messages[-1].detections[0].stamp == 102


def test_camera_publication_retains_capture_stamp_and_does_not_repeat_frame():
    ns = methods(WS/'src/rb_camera/rb_camera/camera_node.py', 'CameraNode', {'publish_latest'}, {})
    sent = []
    node = NS(_lock=threading.Lock(), _latest=object(), _seq=1, _last_pub_seq=-1,
              _latest_stamp=123, bridge=NS(cv2_to_imgmsg=lambda *a, **kw: NS(header=NS())),
              frame_id='front', publish_raw=True, image_pub=NS(publish=sent.append),
              info_pub=NS(publish=lambda m: None), _make_info=lambda stamp: stamp,
              compressed_pub=None, _frames_published=0)
    ns['publish_latest'](node)
    ns['publish_latest'](node)
    assert len(sent) == 1 and sent[0].header.stamp == 123
