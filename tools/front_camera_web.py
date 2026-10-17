#!/usr/bin/env python3
"""Read-only front/rear camera and recognition web viewer; subscribes to existing ROS data."""
import argparse
from collections import deque
import math
import signal
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request

NAMES = {'ball_basketball': '篮球', 'ball_volleyball': '排球', 'pillar': '定位柱',
         'hoop': '篮筐', 'rack_ring': '传球架圆环', 'ball_unknown': '未知球'}
COLORS = {'ball_basketball': (0, 140, 255), 'ball_volleyball': (255, 200, 0),
          'pillar': (0, 255, 0), 'hoop': (0, 0, 255), 'rack_ring': (255, 0, 255)}


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def detection_data(msg):
    """Ignore non-camera/invalid boxes and make JSON-safe copies of ROS messages."""
    rows = []
    for d in msg.detections[:100]:
        if (getattr(d, 'source', 'camera') or 'camera') != 'camera':
            continue
        values = [finite(v) for v in (d.x_min, d.y_min, d.x_max, d.y_max, d.confidence)]
        if any(v is None for v in values):
            continue
        x0, y0, x1, y1 = [max(0., min(1., v)) for v in values[:4]]
        if x0 >= x1 or y0 >= y1:
            continue
        rows.append(dict(label=d.label, name=NAMES.get(d.label, d.label),
                         confidence=max(0., min(1., values[4])), bbox=[x0, y0, x1, y1],
                         distance_m=finite(d.distance_m), bearing_rad=finite(d.bearing_rad),
                         stamp_ns=stamp_ns(d.stamp)))
    return rows


class CameraStore:
    def __init__(self, *, clock=time.monotonic, freshness=1., width=960, quality=75, name='front'):
        self.name = name
        self.clock, self.freshness = clock, freshness
        self.width, self.quality = width, quality
        self.lock = threading.Lock()
        self.images = deque(maxlen=30)
        self.arrivals = deque(maxlen=60)
        self.latest_detection = None
        self.image_count = self.detection_count = 0
        self.raw_jpeg = self.overlay_jpeg = None
        self.display_rows = []
        self.aligned = False
        self.error = ''

    def image(self, msg):
        now = self.clock()
        with self.lock:
            self.images.append((stamp_ns(msg.header.stamp), now, bytes(msg.data)))
            self.arrivals.append(now)
            self.image_count += 1

    def detections(self, msg):
        rows = detection_data(msg)
        with self.lock:
            self.latest_detection = (stamp_ns(msg.header.stamp), self.clock(), rows)
            self.detection_count += 1

    def render(self):
        now = self.clock()
        with self.lock:
            images, detection = list(self.images), self.latest_detection
        selected = images[-1] if images else None
        rows, aligned = [], False
        if selected and now-selected[1] <= self.freshness and detection and now-detection[1] <= self.freshness:
            # Pair to the actual inference image, rather than drawing old boxes on a newer frame.
            matched = next((im for im in reversed(images)
                            if im[0] == detection[0] and im[0] > 0 and now-im[1] <= self.freshness), None)
            if matched:
                selected, aligned = matched, True
                rows = [d for d in detection[2] if abs(d['stamp_ns']-selected[0]) <= 350_000_000]
        try:
            if selected:
                frame = cv2.imdecode(np.frombuffer(selected[2], np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError('JPEG图像解码失败')
                if frame.shape[1] > self.width:
                    h = max(1, round(frame.shape[0]*self.width/frame.shape[1]))
                    frame = cv2.resize(frame, (self.width, h), interpolation=cv2.INTER_AREA)
            else:
                frame = np.zeros((360, 640, 3), np.uint8)
                cv2.putText(frame, f'Waiting for {self.name} camera...', (24, 175),
                            cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 220, 120), 2)
            overlay = frame.copy()
            h, w = overlay.shape[:2]
            for d in rows:
                x0, y0, x1, y1 = d['bbox']
                a, b = (int(x0*(w-1)), int(y0*(h-1))), (int(x1*(w-1)), int(y1*(h-1)))
                color = COLORS.get(d['label'], (100, 255, 100))
                cv2.rectangle(overlay, a, b, color, 2)
                text = f"{d['label']} {d['confidence']:.0%}"
                if d['distance_m'] is not None:
                    text += f" {d['distance_m']:.2f}m"
                cv2.putText(overlay, text, (a[0], max(18, a[1]-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
            if selected and now-selected[1] > self.freshness:
                for image in (frame, overlay):
                    cv2.putText(image, 'STALE CAMERA - no new image', (12, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 255), 2)
            params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
            ok1, raw = cv2.imencode('.jpg', frame, params)
            ok2, drawn = cv2.imencode('.jpg', overlay, params)
            if not ok1 or not ok2:
                raise ValueError('JPEG编码失败')
            with self.lock:
                self.raw_jpeg, self.overlay_jpeg = raw.tobytes(), drawn.tobytes()
                self.display_rows, self.aligned, self.error = rows, aligned, ''
        except (cv2.error, ValueError) as exc:
            with self.lock:
                self.error, self.display_rows, self.aligned = str(exc), [], False

    def status(self):
        now = self.clock()
        with self.lock:
            image_age = now-self.images[-1][1] if self.images else None
            detection_age = now-self.latest_detection[1] if self.latest_detection else None
            times = list(self.arrivals)
            fps = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.
            image_live = image_age is not None and image_age <= self.freshness and not self.error
            det_live = detection_age is not None and detection_age <= self.freshness
            rows = list(self.display_rows) if image_live and det_live and self.aligned else []
            return dict(service='front_camera_view', image_live=image_live,
                        detections_live=det_live, aligned=self.aligned and image_live and det_live,
                        image_age_s=image_age, detection_age_s=detection_age,
                        fps=round(fps, 1) if image_live else 0., image_count=self.image_count,
                        detection_count=self.detection_count, detections=rows,
                        error=self.error)


HTML = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>前后摄像头 · 实时识别</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#101720;color:#e8eef4;font:15px system-ui,"Microsoft YaHei",sans-serif}
main{max-width:1700px;margin:auto;padding:24px}h1{font-size:25px;margin:0 0 6px}h2{font-size:20px;margin:0}.muted{color:#a5b6c6;font-size:13px}
.top,.toolbar,.heading{display:flex;gap:14px;align-items:center;justify-content:space-between;flex-wrap:wrap}.layout{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:20px;margin-top:22px}
.card{background:#1b2633;border:1px solid #344354;border-radius:12px;overflow:hidden}.video{width:100%;display:block;min-height:180px;background:#05080c}.pad{padding:16px}
.status{margin:0 0 8px;line-height:1.7}.good{color:#73e4b6}.warn{color:#ffca7a}a{color:#dfefff;background:#294358;border:1px solid #4d6c82;border-radius:7px;padding:7px 10px;text-decoration:none;font-size:13px}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:9px 4px;border-bottom:1px solid #344354;text-align:left}label{cursor:pointer;font-size:13px}.topics{font-size:11px;color:#a5b6c6;overflow-wrap:anywhere;margin-top:14px}
.note{font-size:13px;color:#a5b6c6;line-height:1.8;margin-top:18px}.heading{border-bottom:1px solid #344354}@media(max-width:1000px){.layout{grid-template-columns:1fr}main{padding:14px}}
</style><main><div class="top"><div><h1>前后摄像头 · 实时识别</h1><div class="muted">两路画面与主机当前识别结果</div></div><div class="muted">各路识别框和结果独立显示</div></div>
<div class="layout">
<section class="card" id="front-panel"><div class="pad heading"><h2>前置摄像头</h2><div class="toolbar"><label><input type="checkbox" id="front-boxes" checked> 显示识别框</label><a href="/frame.jpg?camera=front" target="_blank">查看单帧</a></div></div><img id="front-video" class="video" alt="正在连接前置摄像头"><div class="pad"><p id="front-state" class="status warn">正在连接…</p><div id="front-metrics" class="muted"></div><h3>前摄识别结果</h3><table><thead><tr><th>目标</th><th>置信度</th><th>距离 / 方位</th></tr></thead><tbody id="front-rows"></tbody></table><p id="front-empty" class="muted">等待识别结果</p><div id="front-topics" class="topics"></div></div></section>
<section class="card" id="rear-panel"><div class="pad heading"><h2>后置摄像头</h2><div class="toolbar"><label><input type="checkbox" id="rear-boxes" checked> 显示识别框</label><a href="/frame.jpg?camera=rear" target="_blank">查看单帧</a></div></div><img id="rear-video" class="video" alt="正在连接后置摄像头"><div class="pad"><p id="rear-state" class="status warn">正在连接…</p><div id="rear-metrics" class="muted"></div><h3>后摄识别结果</h3><table><thead><tr><th>目标</th><th>置信度</th><th>距离 / 方位</th></tr></thead><tbody id="rear-rows"></tbody></table><p id="rear-empty" class="muted">等待识别结果</p><div id="rear-topics" class="topics"></div></div></section>
</div><p class="note">直接显示主机正在运行的前后摄像头及识别节点数据，不重复运行YOLO。两路各自匹配图像时间戳，画面或识别停止更新时显示提示并清除旧框。距离为单目估计；方位角按各摄像头图像右正左负。</p></main>
<script>
const $=id=>document.getElementById(id);const cameras=['front','rear'];
for(const camera of cameras){let timer;const next=()=>{$(camera+'-video').src='/frame.jpg?camera='+camera+'&overlay='+($(camera+'-boxes').checked?'1':'0')+'&t='+Date.now()};
$(camera+'-video').onload=()=>{clearTimeout(timer);timer=setTimeout(next,160)};
$(camera+'-video').onerror=()=>{clearTimeout(timer);timer=setTimeout(next,1000)};
$(camera+'-boxes').onchange=()=>{clearTimeout(timer);next()};next()}
function update(camera,s){
const label=camera==='front'?'前摄':'后摄';
$(camera+'-state').textContent=s.error||(!s.image_live?'等待'+label+'画面 / 画面已停止更新':!s.detections_live?'画面正常；等待识别结果':!s.aligned?'画面正常；等待识别与图像对齐':'画面和识别正常');
$(camera+'-state').className='status '+(s.image_live&&s.aligned?'good':'warn');
const age=v=>v===null?'—':v.toFixed(2)+'s';$(camera+'-metrics').textContent=s.fps+' FPS · 图像延迟 '+age(s.image_age_s)+' · 识别延迟 '+age(s.detection_age_s);
$(camera+'-topics').textContent=s.image_topic+'  |  '+s.detections_topic;$(camera+'-rows').replaceChildren();
for(const d of s.detections){const tr=document.createElement('tr');for(const value of [d.name+' ('+d.label+')',Math.round(d.confidence*100)+'%',(d.distance_m===null?'—':d.distance_m.toFixed(2)+'m')+' / '+(d.bearing_rad===null?'—':(d.bearing_rad*180/Math.PI).toFixed(1)+'°')]){const td=document.createElement('td');td.textContent=value;tr.appendChild(td)}$(camera+'-rows').appendChild(tr)}
$(camera+'-empty').textContent=s.aligned?(s.detections.length?'':'当前没有识别到目标'):'暂无可叠加的实时识别结果';}
async function status(){const controller=new AbortController();const timer=setTimeout(()=>controller.abort(),3500);
try{const r=await fetch('/status',{cache:'no-store',signal:controller.signal});if(!r.ok)throw Error('HTTP '+r.status);const s=await r.json();for(const camera of cameras)update(camera,s.cameras[camera]);
}catch(e){for(const camera of cameras){$(camera+'-state').textContent='连接中断，正在重试…';$(camera+'-state').className='status warn';$(camera+'-rows').replaceChildren();$(camera+'-empty').textContent='等待主机重新连接'}}finally{clearTimeout(timer);setTimeout(status,500)}}status();
</script></html>
'''

def make_app(store, image_topic, detections_topic, rear_store=None,
             rear_image_topic='/camera_rear/image_raw/compressed',
             rear_detections_topic='/perception/detections_rear'):
    app = Flask(__name__)
    if rear_store is None:
        rear_store = CameraStore(name='rear')
        rear_store.render()
    sources = {'front': (store, image_topic, detections_topic),
               'rear': (rear_store, rear_image_topic, rear_detections_topic)}

    @app.after_request
    def headers(response):
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        return response

    @app.get('/')
    def index():
        return Response(HTML, content_type='text/html; charset=utf-8')

    @app.get('/status')
    def status():
        selected = request.args.get('camera', 'front')
        if selected not in sources:
            return jsonify(error='unknown camera'), 404
        cameras = {}
        for key, (camera, image, detections) in sources.items():
            data = camera.status()
            data.update(image_topic=image, detections_topic=detections, camera=key)
            cameras[key] = data
        # Keep legacy front-camera fields/service id for the Windows launcher.
        return jsonify(dict(cameras[selected], cameras=cameras))

    @app.get('/frame.jpg')
    def frame():
        selected = request.args.get('camera', 'front')
        if selected not in sources:
            return jsonify(error='unknown camera'), 404
        camera = sources[selected][0]
        with camera.lock:
            jpeg = camera.raw_jpeg if request.args.get('overlay') == '0' else camera.overlay_jpeg
        return Response(jpeg or b'', status=200 if jpeg else 503, content_type='image/jpeg')

    return app


def create_ros_node(store, image_topic, detections_topic, rear_store=None,
                    rear_image_topic='/camera_rear/image_raw/compressed',
                    rear_detections_topic='/perception/detections_rear'):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from sensor_msgs.msg import CompressedImage
    from rb_msgs.msg import DetectionArray
    node = Node('camera_web_viewer')
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     history=QoSHistoryPolicy.KEEP_LAST)
    node.create_subscription(CompressedImage, image_topic, store.image, qos)
    node.create_subscription(DetectionArray, detections_topic, store.detections, qos)
    if rear_store is not None:
        node.create_subscription(CompressedImage, rear_image_topic, rear_store.image, qos)
        node.create_subscription(DetectionArray, rear_detections_topic, rear_store.detections, qos)
    return node


def main(argv=None):
    parser = argparse.ArgumentParser(description='浏览器同时查看前后摄像头和当前识别结果（只读）')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8082)
    parser.add_argument('--fps', type=float, default=6.)
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--quality', type=int, default=75)
    parser.add_argument('--image-topic', default='/camera/image_raw/compressed')
    parser.add_argument('--detections-topic', default='/perception/detections')
    parser.add_argument('--rear-image-topic', default='/camera_rear/image_raw/compressed')
    parser.add_argument('--rear-detections-topic', default='/perception/detections_rear')
    args = parser.parse_args(argv)
    if not (1 <= args.port <= 65535 and 1 <= args.fps <= 30 and 160 <= args.width <= 1920 and 10 <= args.quality <= 95):
        parser.error('端口1～65535，FPS 1～30，宽度160～1920，JPEG质量10～95')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    from werkzeug.serving import make_server
    cv2.setNumThreads(1)
    store = CameraStore(width=args.width, quality=args.quality, name='front')
    rear = CameraStore(width=args.width, quality=args.quality, name='rear')
    for camera in (store, rear):
        camera.render()
    # Bind before starting ROS so an occupied port does not leave a reader behind.
    server = make_server(args.host, args.port, make_app(store, args.image_topic, args.detections_topic, rear, args.rear_image_topic, args.rear_detections_topic), threaded=True)
    rclpy.init(args=[])
    node = create_ros_node(store, args.image_topic, args.detections_topic, rear, args.rear_image_topic, args.rear_detections_topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    def render(camera):
        while not stop.is_set():
            started = time.monotonic()
            camera.render()
            stop.wait(max(.001, 1./args.fps-(time.monotonic()-started)))

    threads = [threading.Thread(target=spin, daemon=True)]
    threads.extend(threading.Thread(target=render, args=(camera,), daemon=True)
                   for camera in (store, rear))
    for thread in threads:
        thread.start()
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    print(f'前后摄像头查看页：http://127.0.0.1:{args.port} （其他电脑使用主机IP）', flush=True)
    print(f'前摄：{args.image_topic} + {args.detections_topic}', flush=True)
    print(f'后摄：{args.rear_image_topic} + {args.rear_detections_topic}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        executor.shutdown(timeout_sec=2.)
        for thread in threads:
            thread.join(timeout=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
