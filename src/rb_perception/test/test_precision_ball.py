import math
from pathlib import Path
import sys
import cv2
import numpy as np
import pytest
import yaml
from rb_perception.detectors import CameraModel, Detection
from rb_perception.precision_ball import fit_sphere, fuse_precision, rigid, transform
from rb_perception.ball_approach import Arrival, errors, command

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS/'tools'))
from calibrate_camera import calibration_quality, load_from_images
from calibrate_ball_geometry import solve


def sphere(centre, radius=.12, n=400):
    rng = np.random.default_rng(18)
    vectors = rng.normal(size=(n, 3))
    vectors /= np.linalg.norm(vectors, axis=1)[:, None]
    vectors = vectors[vectors[:, 2] < -.15]  # Camera-visible surface only.
    return vectors * radius + centre


def scene(rear=False):
    cam = CameraModel(700, 700, 640, 360, 1280, 720, ())
    # Proper optical -> base rotation, front and rear both determinant +1.
    rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    if rear:
        rotation = np.diag([-1., -1., 1.]) @ rotation
    t = np.eye(4); t[:3, :3] = rotation; t[:3, 3] = [.1, -.02, .5]
    optical_centre = np.array([.04, .03, 1.8])
    points = transform(sphere(optical_centre), t)
    d = Detection('ball_basketball', .92, (.45, .41, .59, .65), distance_m=5.)
    cfg = {'extrinsics_confirmed': True, 'base_from_cloud': np.eye(4).tolist(),
           'base_from_camera': t.tolist()}
    c = {'calibrated': True, 'calibration_size': [1280, 720]}
    return cam, points, d, cfg, c, transform(optical_centre[None], t)[0]


def test_partial_surface_centroid_is_biased_but_fit_recovers_centre():
    expected = np.array([1., 2., 3.])
    points = sphere(expected)
    assert np.linalg.norm(points.mean(axis=0)-expected) > .05
    fitted = fit_sphere(points, .12, {})
    assert fitted is not None
    assert np.linalg.norm(fitted[0]-expected) < 1e-5


def test_planar_background_is_not_a_ball():
    x, y = np.meshgrid(np.linspace(-.1, .1, 10), np.linspace(-.1, .1, 10))
    assert fit_sphere(np.c_[x.ravel(), y.ravel(), np.ones(x.size)], .12, {}) is None
    assert fit_sphere(sphere([0, 0, 1.])[:5], .12, {}) is None


@pytest.mark.parametrize('rear', [False, True])
def test_lidar_replaces_bad_monocular_depth_in_both_camera_frames(rear):
    cam, points, d, cfg, c, expected = scene(rear)
    out, report = fuse_precision([d], points, 10., 'body', 10.02, 10.04, cam, c, cfg)
    assert report['valid'], report
    assert out[0].source == 'camera_lidar'
    assert out[0].distance_m == pytest.approx(1.8, abs=1e-4)
    assert np.allclose(report['targets'][0]['centre_base_m'], expected, atol=1e-4)
    assert d.distance_m == 5.  # Caller/display objects are not mutated.


@pytest.mark.parametrize('kind', ['cloud_age', 'image_age', 'future', 'sync', 'frame', 'calib', 'size'])
def test_invalid_measurements_keep_box_but_never_supply_distance(kind):
    cam, points, d, cfg, c, _ = scene()
    cloud_stamp, image_stamp, now, frame = 10., 10.02, 10.04, 'body'
    if kind == 'cloud_age': cloud_stamp = 9.
    if kind == 'image_age': image_stamp = 9.
    if kind == 'future': image_stamp = 11.
    if kind == 'sync': cloud_stamp = 9.8
    if kind == 'frame': frame = 'camera_init'
    if kind == 'calib': c['calibrated'] = False
    if kind == 'size': c['calibration_size'] = [640, 480]
    out, report = fuse_precision([d], points, cloud_stamp, frame, image_stamp, now, cam, c, cfg)
    assert not report['valid'] and not report['targets']
    assert math.isnan(out[0].distance_m)
    assert out[0].bbox == d.bbox


def test_duplicate_image_boxes_cannot_claim_two_lidar_targets():
    cam, points, d, cfg, c, _ = scene()
    out, report = fuse_precision([d, d], points, 10., 'body', 10.02, 10.04, cam, c, cfg)
    assert not report['valid']
    assert all(math.isnan(x.distance_m) for x in out)


def test_two_spheres_in_one_box_are_ambiguous():
    cam, points, d, cfg, c, _ = scene()
    # Second ball behind first shares image ROI but is spatially separate.
    t = np.array(cfg['base_from_camera'])
    points = np.r_[points, transform(sphere([.06, .04, 2.7]), t)]
    _, report = fuse_precision([d], points, 10., 'body', 10.02, 10.04, cam, c, cfg)
    assert not report['valid']


def approach_cfg():
    return {'intake_confirmed': True, 'intake_position_base_m': [-.4, 0, .12],
            'intake_outward_xy': [-1., 0.], 'target_gap_m': .60}


def target(gap=.60, y=0):
    return {'centre_base_m': [-.4-gap-.12, y, .12], 'radius_m': .12}


def test_surface_gap_uses_intake_offset_and_radius_and_single_target():
    cfg = approach_cfg()
    e, lateral, heading, _ = errors(target(), cfg)
    assert abs(e) < 1e-8 and lateral == 0 and abs(heading) < 1e-8
    assert command(target(), cfg) == (0., 0., 0.)
    assert command(target(.9), cfg)[0] < 0  # Rearward when still too far away.
    assert command(target(.3), cfg)[0] > 0  # Forward to correct overshoot.
    assert np.linalg.norm(command(target(2.), cfg)[:2]) <= .08+1e-8
    front = target(); front['centre_base_m'][:2] = [1.12, 0]
    assert errors(front, cfg, rear=False)[0] == pytest.approx(0.)


def test_surface_gap_includes_vertical_intake_offset():
    cfg = approach_cfg(); cfg['intake_position_base_m'][2] = .4
    ball = target()
    assert errors(ball, cfg)[0] == pytest.approx(math.hypot(.72, .28) - .12 - .60)


@pytest.mark.parametrize('bad_target,speed,wz', [(target(.9), 0, 0), (target(.3), 0, 0),
                                               (target(y=.1), 0, 0), (target(), .1, 0),
                                               (target(), 0, .2), (target(), float('nan'), 0)])
def test_not_arrived_with_error_or_motion(bad_target, speed, wz):
    gate = Arrival(approach_cfg())
    for stamp in np.arange(10., 11., .1):
        assert not gate.update(bad_target, stamp, stamp+.01, speed, wz)


def test_arrival_needs_new_observations_and_stable_dwell():
    gate = Arrival(approach_cfg())
    assert not gate.update(target(), 10., 10., 0, 0)
    assert not gate.update(target(), 10., 10.6, 0, 0)  # Reused stale frame.
    for stamp in [11., 11.1, 11.2, 11.3, 11.4]:
        assert not gate.update(target(), stamp, stamp+.01, 0, 0)
    assert gate.update(target(), 11.6, 11.61, 0, 0)


def test_invalid_rigid_transform_and_unmeasured_intake_are_rejected():
    with pytest.raises(ValueError): rigid(np.diag([1., 1., -1., 1.]))
    cfg = approach_cfg(); cfg['intake_confirmed'] = False
    with pytest.raises(ValueError): Arrival(cfg)
    cfg['intake_confirmed'] = True; cfg['target_gap_m'] = .9
    with pytest.raises(ValueError): Arrival(cfg)


def test_extrinsics_solver_recovers_full_mounting_transform():
    rng = np.random.default_rng(7)
    points = rng.uniform(-.2, .2, (54, 3)); points[:, 2] += 2.
    K = np.array([[700., 0, 640], [0, 700., 360], [0, 0, 1.]])
    r = np.array([.04, -.06, .08]); t = np.array([.1, -.12, .2])
    pixels = cv2.projectPoints(points, r, t, K, np.zeros(5))[0]
    fitted, rms = solve(points, pixels, K, np.zeros(5))
    assert rms < 1e-4
    actual = np.eye(4); actual[:3, :3] = cv2.Rodrigues(r)[0]; actual[:3, 3] = t
    assert np.allclose(fitted, np.linalg.inv(actual), atol=1e-5)


def test_mixed_calibration_resolutions_fail(tmp_path):
    cv2.imwrite(str(tmp_path/'a.png'), np.zeros((300, 400, 3), np.uint8))
    cv2.imwrite(str(tmp_path/'b.png'), np.zeros((600, 800, 3), np.uint8))
    with pytest.raises(ValueError, match='mixed image resolutions'):
        load_from_images(str(tmp_path), (9, 6))


def test_weak_intrinsic_calibration_never_gets_a_quality_pass():
    K = np.array([[700., 0, 640], [0, 700., 360], [0, 0, 1.]])
    assert calibration_quality([], [], (1280, 720), .1, K, np.zeros(5), [], [])
    op = np.zeros((54, 3), np.float32)
    ip = np.zeros((54, 1, 2), np.float32)
    assert calibration_quality([op]*15, [ip]*15, (1280, 720), .1, K, np.zeros(5), [], [])


def test_confirmed_rear_device_is_separate_and_geometry_still_requires_calibration():
    cfg = yaml.safe_load((WS/'src/rb_camera/config/camera_rear.yaml').read_text())
    assert cfg['camera']['device'] == '/dev/v4l/by-id/usb-LRCP_V1080P_LRCP_V1080P-video-index0'
    front = yaml.safe_load((WS/'src/rb_camera/config/camera.yaml').read_text())
    assert front['camera']['device'] != cfg['camera']['device']
    assert cfg['output']['image_topic'] == '/camera_rear/image_raw'
    assert cfg['output']['health_topic'] == '/rb_camera_rear/ok'
    cfg = yaml.safe_load((WS/'src/rb_perception/config/perception_rear.yaml').read_text())
    assert not cfg['camera']['calibrated']
    assert len(cfg['perception']['detectors']) == 1


def test_camera_publishes_each_acquisition_once_with_original_stamp():
    # Exercise the real method with fake transport, without importing ROS on macOS.
    import ast
    import threading
    from types import SimpleNamespace as NS
    tree = ast.parse((WS/'src/rb_camera/rb_camera/camera_node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name=='CameraNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name=='publish_latest')
    ns = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), '<camera>', 'exec'), ns)
    capture_stamp = NS(sec=10, nanosec=10)
    messages = []
    n = NS(_lock=threading.Lock(), _latest=np.zeros((2, 2, 3)), _seq=1,
           _latest_ros_stamp=capture_stamp, _published_seq=-1, _frames_published=0,
           bridge=NS(cv2_to_imgmsg=lambda *a, **k: NS(header=NS())), frame_id='camera',
           publish_raw=True, image_pub=NS(publish=messages.append),
           info_pub=NS(publish=lambda m: None), _make_info=lambda s: s,
           compressed_pub=None)
    ns['publish_latest'](n)
    ns['publish_latest'](n)
    assert len(messages)==1 and messages[0].header.stamp is capture_stamp
    n._seq=2; n._latest_ros_stamp=NS(sec=11, nanosec=0)
    ns['publish_latest'](n)
    assert len(messages)==2 and messages[1].header.stamp.sec==11
