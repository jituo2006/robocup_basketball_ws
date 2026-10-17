"""Private ROS domain + synthetic front frames only. Never touches a camera."""
import json
import os
from pathlib import Path
import socket
import subprocess
import time
from urllib.request import urlopen
import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rb_msgs.msg import Detection, DetectionArray
from sensor_msgs.msg import CompressedImage

ROOT = Path(__file__).resolve().parents[3]


def test_ros_images_detections_reach_browser_with_no_motion_publishers(tmp_path):
    domain = 202
    sock = socket.socket();sock.bind(('127.0.0.1',0));port = sock.getsockname()[1];sock.close()
    context = Context();rclpy.init(args=[],context=context,domain_id=domain)
    node = Node('synthetic_front_camera',context=context)
    executor = SingleThreadedExecutor(context=context);executor.add_node(node)
    image_pub = node.create_publisher(CompressedImage,'/test/front/compressed',1)
    det_pub = node.create_publisher(DetectionArray,'/test/front/detections',1)
    rear_image_pub = node.create_publisher(CompressedImage,'/test/rear/compressed',1)
    rear_det_pub = node.create_publisher(DetectionArray,'/test/rear/detections',1)
    log = open(tmp_path/'viewer.log','w')
    proc = subprocess.Popen(['/usr/bin/python3',str(ROOT/'tools/front_camera_web.py'),
        '--host','127.0.0.1','--port',str(port),'--image-topic','/test/front/compressed',
        '--detections-topic','/test/front/detections','--rear-image-topic','/test/rear/compressed',
        '--rear-detections-topic','/test/rear/detections'],env=dict(os.environ,ROS_DOMAIN_ID=str(domain)),
        stdout=log,stderr=log)
    try:
        msg = CompressedImage();msg.header.stamp.sec = 123;msg.format = 'jpeg'
        msg.data = cv2.imencode('.jpg',np.zeros((360,640,3),np.uint8))[1].tobytes()
        arr = DetectionArray();arr.header.stamp = msg.header.stamp
        d = Detection();d.stamp = msg.header.stamp;d.source='camera';d.label='ball_basketball'
        d.confidence=.95;d.x_min=.2;d.y_min=.2;d.x_max=.7;d.y_max=.8
        d.distance_m=float('nan');d.bearing_rad=.1;arr.detections=[d]
        rear_arr = DetectionArray();rear_arr.header.stamp = msg.header.stamp
        rear_d = Detection();rear_d.stamp = msg.header.stamp;rear_d.source='camera';rear_d.label='ball_volleyball'
        rear_d.confidence=.8;rear_d.x_min=.4;rear_d.y_min=.2;rear_d.x_max=.8;rear_d.y_max=.8
        rear_d.distance_m=1.2;rear_d.bearing_rad=-.2;rear_arr.detections=[rear_d]
        status = None;end = time.monotonic()+10
        while time.monotonic()<end:
            image_pub.publish(msg);det_pub.publish(arr)
            rear_image_pub.publish(msg);rear_det_pub.publish(rear_arr)
            executor.spin_once(timeout_sec=.03)
            try:
                with urlopen(f'http://127.0.0.1:{port}/status',timeout=.3) as response:
                    status = json.load(response)
                if (status['aligned'] and status['detections']
                        and status['cameras']['rear']['aligned'] and status['cameras']['rear']['detections']):break
            except OSError:pass
            time.sleep(.05)
        assert status and status['aligned'] and status['detections'],(tmp_path/'viewer.log').read_text()
        assert status['detections'][0]['label']=='ball_basketball'
        assert status['cameras']['rear']['detections'][0]['label']=='ball_volleyball'
        assert status['cameras']['rear']['detections_topic']=='/test/rear/detections'
        with urlopen(f'http://127.0.0.1:{port}/frame.jpg',timeout=2) as response:
            image = cv2.imdecode(np.frombuffer(response.read(),np.uint8),cv2.IMREAD_COLOR)
        assert image.shape == (360,640,3)
        assert image[150,128,2]>100
        with urlopen(f'http://127.0.0.1:{port}/frame.jpg?camera=rear',timeout=2) as response:
            rear_image = cv2.imdecode(np.frombuffer(response.read(),np.uint8),cv2.IMREAD_COLOR)
        assert rear_image.shape == (360,640,3) and rear_image[150,255,0]>100
        executor.spin_once(timeout_sec=.1)
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        assert not node.get_publishers_info_by_topic('/rb_launcher/command')
    finally:
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=5)
        log.close();executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)
