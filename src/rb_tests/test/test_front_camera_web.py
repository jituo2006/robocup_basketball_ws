"""Read-only camera HTTP viewer; synthetic JPEG/ROS messages only."""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools'))
from front_camera_web import CameraStore, make_app, detection_data


def stamp(n):
    return NS(sec=n//1_000_000_000, nanosec=n % 1_000_000_000)


def image(n=1_000_000_000):
    ok, jpeg = cv2.imencode('.jpg', np.zeros((360, 640, 3), np.uint8))
    assert ok
    return NS(header=NS(stamp=stamp(n)), data=jpeg.tobytes())


def detections(n=1_000_000_000, **kwargs):
    d = NS(label='pillar', source='camera', confidence=.9,
           x_min=.2, y_min=.2, x_max=.7, y_max=.8, distance_m=float('nan'),
           bearing_rad=.1, stamp=stamp(n))
    d.__dict__.update(kwargs)
    return NS(header=NS(stamp=stamp(n)), detections=[d])


def test_exact_inference_image_is_selected_and_overlay_can_be_disabled():
    clock = [0.]
    store = CameraStore(clock=lambda: clock[0])
    store.image(image())
    store.image(image(1_100_000_000))  # a newer camera frame must not receive old boxes
    store.detections(detections())
    store.render()
    client = make_app(store, '/front', '/objects').test_client()
    status = client.get('/status').get_json()
    assert status['aligned'] and status['image_live']
    assert status['detections'][0]['name'] == '定位柱'
    assert status['detections'][0]['distance_m'] is None
    assert status['image_topic'] == '/front'
    json.dumps(status, allow_nan=False)
    drawn = cv2.imdecode(np.frombuffer(client.get('/frame.jpg').data, np.uint8), cv2.IMREAD_COLOR)
    raw = cv2.imdecode(np.frombuffer(client.get('/frame.jpg?overlay=0').data, np.uint8), cv2.IMREAD_COLOR)
    assert drawn[150,128,1] > 100 and raw[150,128,1] < 20
    assert client.post('/api/action', json={'command': 'vesc_reset'}).status_code == 404
    assert client.get('/').headers['Cache-Control'] == 'no-store'


def test_placeholder_is_available_before_ros_data():
    store = CameraStore()
    store.render()
    client = make_app(store, 'image', 'detections').test_client()
    status = client.get('/status').get_json()
    assert not status['image_live'] and not status['detections_live']
    assert status['detections'] == []
    assert client.get('/frame.jpg').status_code == 200
    assert '前置摄像头' in client.get('/').get_data(as_text=True)


def test_stale_detections_are_removed_even_when_camera_is_live():
    clock = [0.]
    store = CameraStore(clock=lambda: clock[0])
    store.image(image());store.detections(detections());store.render()
    clock[0] = 1.1
    store.image(image(2_000_000_000));store.render()
    status = store.status()
    assert status['image_live'] and not status['detections_live']
    assert status['detections'] == [] and not status['aligned']


def test_unmatched_frame_has_no_overlay_and_empty_detection_message_clears_targets():
    store = CameraStore()
    store.image(image());store.detections(detections(2_000_000_000));store.render()
    assert not store.status()['aligned'] and not store.status()['detections']
    empty = detections();empty.detections = []
    store.detections(empty);store.render()
    assert store.status()['aligned'] and store.status()['detections'] == []


@pytest.mark.parametrize('kwargs', [dict(source='lidar'), dict(x_min=float('nan')),
                                  dict(confidence=float('inf')), dict(x_min=.9,x_max=.2)])
def test_invalid_or_lidar_boxes_do_not_appear(kwargs):
    assert detection_data(detections(**kwargs)) == []


def test_cached_inference_observation_too_old_is_not_drawn():
    store = CameraStore()
    store.image(image(2_000_000_000))
    msg = detections(2_000_000_000)
    msg.detections[0].stamp = stamp(1_000_000_000)
    store.detections(msg);store.render()
    assert store.status()['detections'] == []


def test_camera_disconnect_shows_stale_and_removes_boxes():
    clock = [0.]
    store = CameraStore(clock=lambda: clock[0])
    store.image(image());store.detections(detections());store.render()
    clock[0] = 2.
    store.render()
    status = store.status()
    assert not status['image_live'] and not status['detections'] and status['fps'] == 0


def test_corrupted_jpeg_sets_error_and_does_not_crash():
    store = CameraStore()
    msg = image();msg.data = b'not a jpeg'
    store.image(msg);store.render()
    assert store.status()['error'] == 'JPEG图像解码失败'


def test_image_history_is_bounded():
    store = CameraStore()
    msg = image()
    for _ in range(80):store.image(msg)
    assert len(store.images) == 30 and len(store.arrivals) == 60
