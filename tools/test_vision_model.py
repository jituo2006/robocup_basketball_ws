#!/usr/bin/env python3
"""仅检测摄像头画面，可独立网页查看；不发布底盘/机构指令。"""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace as NS

import cv2
import numpy as np
import yaml

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / 'src/rb_perception'))
sys.path.insert(0, str(WS / 'tools'))
from rb_perception.detectors import CameraModel, OnnxDetector
from console_app import draw_ball_boxes


def profile(camera, confidence=None):
    config = WS / 'src/rb_perception/config' / ('perception_rear.yaml' if camera == 'rear' else 'perception_seek.yaml')
    data = yaml.safe_load(config.read_text())
    detector = dict(next(d for d in data['perception']['detectors'] if d['type'] == 'onnx'))
    detector['model_path'] = str((config.parent / detector['model_path']).resolve())
    detector['every_n_frames'] = 1
    if confidence is not None:
        detector['conf_threshold'] = confidence
    return detector, data['camera']


def annotate(frame, detections):
    _, encoded = cv2.imencode('.jpg', frame)
    messages = [NS(label=d.label, source='camera', confidence=d.confidence,
                   x_min=d.bbox[0], y_min=d.bbox[1], x_max=d.bbox[2], y_max=d.bbox[3])
                for d in detections]
    return draw_ball_boxes(encoded.tobytes(), messages)


class View:
    def __init__(self):
        self.jpeg = None
        self.status = {'message': '等待摄像头画面'}
        self.lock = threading.Lock()

    def serve(self, port):
        view = self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                with view.lock:
                    jpeg, status = view.jpeg, dict(view.status)
                if self.path.startswith('/frame.jpg'):
                    if jpeg is None:
                        self.send_error(503, 'Waiting for camera'); return
                    payload, mime = jpeg, 'image/jpeg'
                elif self.path.startswith('/status'):
                    payload, mime = json.dumps(status, ensure_ascii=False).encode(), 'application/json'
                else:
                    payload = ('''<!doctype html><meta charset="utf-8"><title>模型独立检测</title>
<style>body{background:#111;color:#ddd;font:16px sans-serif;margin:24px}img{max-width:100%}pre{white-space:pre-wrap}</style>
<h2>模型独立检测：篮球、排球、定位柱</h2><p>这里只显示识别结果，不驱动车辆。</p>
<img id="frame"><pre id="status"></pre><script>
setInterval(async()=>{document.querySelector('#frame').src='/frame.jpg?t='+Date.now();
document.querySelector('#status').textContent=JSON.stringify(await(await fetch('/status')).json(),null,2)},300);
</script>''').encode()
                    mime = 'text/html; charset=utf-8'
                self.send_response(200); self.send_header('Content-Type', mime)
                self.send_header('Cache-Control', 'no-store'); self.end_headers(); self.wfile.write(payload)
            def log_message(self, *args):
                pass
        server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        return server


def device_frames(camera, device):
    config = WS / 'src/rb_camera/config' / ('camera_rear.yaml' if camera == 'rear' else 'camera.yaml')
    cfg = yaml.safe_load(config.read_text())['camera']
    selected = device or cfg['device']
    if not selected.startswith('/dev/') or not Path(selected).exists():
        raise RuntimeError(f'指定摄像头不存在：{selected}；不回退到数字索引')
    actual = str(Path(selected).resolve())
    for process in Path('/proc').glob('[0-9]*'):
        if process.name == str(os.getpid()): continue
        for descriptor in (process / 'fd').glob('*'):
            try:
                if os.readlink(descriptor) == actual:
                    raise RuntimeError(f'{selected} 正被进程{process.name}使用；系统运行时请改用 --source ros')
            except (FileNotFoundError, PermissionError, OSError):
                pass
    cap = cv2.VideoCapture(selected, cv2.CAP_V4L2)
    if not cap.isOpened(): raise RuntimeError(f'打不开摄像头：{selected}')
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cfg.get('fourcc', 'MJPG')))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg['width']); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg['height'])
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print(f'摄像头 {camera}: {selected}', flush=True)
        while True:
            ok, frame = cap.read()
            if not ok: raise RuntimeError('摄像头读取失败')
            yield frame, time.time()
    finally:
        cap.release()


def ros_frames(camera):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    rclpy.init(); node = rclpy.create_node('vision_model_preview')
    topic = '/camera_rear/image_raw/compressed' if camera == 'rear' else '/camera/image_raw/compressed'
    latest = []
    def receive(message):
        latest[:] = [message]
    subscription = node.create_subscription(CompressedImage, topic, receive, qos_profile_sensor_data)
    print(f'订阅 {topic}（ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}）', flush=True)
    last, received = None, time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if not latest or latest[0] is last:
                if time.monotonic()-received > 8:
                    raise RuntimeError('8秒没有新画面；请启动相机节点，或独立检测改用 --source device')
                continue
            last = latest[0]
            stamp = last.header.stamp.sec + last.header.stamp.nanosec*1e-9
            now = node.get_clock().now().nanoseconds*1e-9
            if not -.05 <= now-stamp <= .5:
                if time.monotonic()-received > 8:
                    raise RuntimeError('摄像头持续发送过期画面，请检查相机源时间戳')
                continue
            received = time.monotonic()
            frame = cv2.imdecode(np.frombuffer(last.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None: yield frame, stamp
    finally:
        node.destroy_node(); rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', choices=['front', 'rear'], default='front')
    parser.add_argument('--source', choices=['ros', 'device'], default='ros')
    parser.add_argument('--device', help='独立检测时明确覆盖固定摄像头设备名')
    parser.add_argument('--confidence', type=float, default=.25)
    parser.add_argument('--seconds', type=float, default=30., help='0表示一直运行，Ctrl+C结束')
    parser.add_argument('--serve', type=int, help='提供独立检测网页的端口，例如8091')
    parser.add_argument('--output', type=Path, default=WS / 'test_artifacts/vision_model')
    args = parser.parse_args(argv)
    if not 0 < args.confidence <= 1 or not np.isfinite(args.seconds) or args.seconds < 0:
        parser.error('置信度须在(0,1]，时间须非负有限数')
    cfg, camera = profile(args.camera, args.confidence)
    detector = OnnxDetector('yolo', cfg)
    if not detector.available: raise RuntimeError(detector.error)
    print(f'模型 {cfg["model_path"]}\n类别 {detector.model_names}\n置信度 {args.confidence}，input_content_scale={cfg.get("input_content_scale", 1.)}', flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    view = View(); server = view.serve(args.serve) if args.serve else None
    if server: print(f'检测网页：http://<Linux主机IP>:{args.serve}/', flush=True)
    stream = device_frames(args.camera, args.device) if args.source == 'device' else ros_frames(args.camera)
    started, frames, last_print = time.monotonic(), 0, 0.
    try:
        for frame, stamp in stream:
            before = time.monotonic()
            dets = detector.detect(frame, CameraModel.from_config(camera, frame.shape[1], frame.shape[0]))
            inference_ms = (time.monotonic()-before)*1000
            frames += 1
            status = {'camera': args.camera, 'source_stamp': stamp, 'frames': frames,
                      'infer_ms': round(inference_ms, 1), 'mean_brightness': round(float(frame.mean()), 1),
                      'fps': round(frames/max(.001, time.monotonic()-started), 1),
                      'detections': [{'label': d.label, 'confidence': round(d.confidence, 3),
                                      'bbox': [round(v, 3) for v in d.bbox]} for d in dets]}
            jpeg = annotate(frame, dets)
            with view.lock: view.jpeg, view.status = jpeg, status
            if time.monotonic()-last_print >= 1:
                print(json.dumps(status, ensure_ascii=False), flush=True)
                (args.output / f'{args.camera}.jpg').write_bytes(jpeg)
                (args.output / f'{args.camera}.json').write_text(json.dumps(status, ensure_ascii=False, indent=2))
                last_print = time.monotonic()
            if args.seconds and time.monotonic()-started >= args.seconds: break
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()
        if server: server.shutdown(); server.server_close()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr); raise SystemExit(1)
