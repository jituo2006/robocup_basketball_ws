"""Front/rear image and detection isolation in the same read-only web app."""
import cv2
import numpy as np
from test_front_camera_web import image, detections
from front_camera_web import CameraStore, make_app


def test_each_camera_has_its_own_image_boxes_topics_and_switch():
    front, rear = CameraStore(), CameraStore(name='rear')
    front.image(image());rear.image(image())
    front.detections(detections(label='ball_basketball'))
    rear.detections(detections(label='ball_volleyball',x_min=.4))
    front.render();rear.render()
    app = make_app(front,'/front/image','/front/objects',rear,'/rear/image','/rear/objects')
    client = app.test_client()
    data = client.get('/status').get_json()
    assert data['image_topic']=='/front/image'  # old launcher/API remain usable
    assert data['cameras']['front']['detections'][0]['label']=='ball_basketball'
    assert data['cameras']['rear']['detections'][0]['label']=='ball_volleyball'
    assert data['cameras']['rear']['image_topic']=='/rear/image'
    assert client.get('/status?camera=rear').get_json()['camera']=='rear'
    front_jpeg = client.get('/frame.jpg?camera=front').data
    rear_jpeg = client.get('/frame.jpg?camera=rear').data
    assert front_jpeg != rear_jpeg
    drawn = cv2.imdecode(np.frombuffer(rear_jpeg,np.uint8),cv2.IMREAD_COLOR)
    raw = cv2.imdecode(np.frombuffer(client.get('/frame.jpg?camera=rear&overlay=0').data,np.uint8),cv2.IMREAD_COLOR)
    assert drawn[150,255,0]>100 and raw[150,255,0]<20
    page=client.get('/').get_data(as_text=True)
    assert 'front-panel' in page and 'rear-panel' in page
    assert '前置摄像头' in page and '后置摄像头' in page
    assert 'rear-boxes' in page and 'front-boxes' in page


def test_one_camera_can_be_stale_while_the_other_remains_live():
    clock=[0.]
    front=CameraStore(clock=lambda:clock[0]);rear=CameraStore(clock=lambda:clock[0],name='rear')
    for camera in (front,rear):
        camera.image(image());camera.detections(detections());camera.render()
    clock[0]=1.1
    front.image(image(2_000_000_000));front.detections(detections(2_000_000_000))
    front.render();rear.render()
    data=make_app(front,'front','front-dets',rear).test_client().get('/status').get_json()['cameras']
    assert data['front']['image_live'] and data['front']['detections']
    assert not data['rear']['image_live'] and not data['rear']['detections']


def test_rear_without_detection_data_has_raw_image_and_does_not_show_front_boxes():
    front,rear=CameraStore(),CameraStore(name='rear')
    for camera in (front,rear):camera.image(image())
    front.detections(detections());front.render();rear.render()
    client=make_app(front,'front','front-dets',rear).test_client()
    data=client.get('/status').get_json()['cameras']
    assert data['front']['detections'] and data['rear']['image_live']
    assert not data['rear']['detections_live'] and not data['rear']['detections']
    assert client.get('/frame.jpg?camera=rear').status_code==200


def test_unknown_camera_is_rejected_rather_than_falling_back_to_front():
    store=CameraStore();store.render()
    client=make_app(store,'front','dets').test_client()
    assert client.get('/status?camera=wrong').status_code==404
    assert client.get('/frame.jpg?camera=wrong').status_code==404
