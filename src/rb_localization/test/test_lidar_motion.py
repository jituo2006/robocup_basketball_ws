"""Exercise production odometry callbacks with pose-only FAST-LIO messages."""
import math
import pytest
from rb_localization.ekf2d import Ekf2D, EkfConfig, sensor_to_base_xy, world_vel_to_body, wrap_pi
from test_localization_output import node, odom, NS


def motion_node():
    n = node()
    class FakeTime:
        def __init__(self, stamp):
            self.nanoseconds = int((stamp.sec + stamp.nanosec * 1e-9) * 1e9)
        def __sub__(self, other):
            return NS(nanoseconds=self.nanoseconds-other.nanoseconds)
    n.on_odom.__func__.__globals__["Time"] = NS(from_msg=FakeTime)
    n.on_odom.__func__.__globals__.update(world_vel_to_body=world_vel_to_body,
                                        sensor_to_base_xy=sensor_to_base_xy)
    n.ekf = Ekf2D(EkfConfig(sigma_v=.06, sigma_w=.05, obs_std_xy=.10, obs_std_yaw=.06))
    n.ekf.set_pose(1.5, 8., .4)
    n.last_odom_field_pose = None
    n.odom_conv_enabled = True; n.odom_zero_on_first = True
    n.odom_origin = None; n.theta = 0.; n.conv_ready = False
    n.s2b_yaw = math.radians(-85); n.start_pose = (1.5, 8., .4)
    n.odom_to_field = type(n).odom_to_field.__get__(n)
    n.get_logger = lambda: NS(info=lambda *a, **kw: None)
    return n


def feed_field(n, pose, stamp):
    # Independent inverse geometry: first LiDAR pose is (0,0,0), base offset -0.3m.
    theta = n.start_pose[2] - n.s2b_yaw
    c, s = math.cos(theta), math.sin(theta)
    o0x = -.3 * math.cos(n.s2b_yaw); o0y = -.3 * math.sin(n.s2b_yaw)
    tx = 1.5 - (c * o0x - s * o0y); ty = 8. - (s * o0x + c * o0y)
    x, y, yaw = pose
    dx, dy = x-tx, y-ty
    byaw = yaw-theta
    m = odom(stamp)
    m.pose.pose.position.x = c*dx+s*dy + .3*math.cos(byaw)
    m.pose.pose.position.y = -s*dx+c*dy + .3*math.sin(byaw)
    syaw = byaw-n.s2b_yaw
    m.pose.pose.orientation.z = math.sin(syaw/2)
    m.pose.pose.orientation.w = math.cos(syaw/2)
    n.now = stamp + .01
    n.on_odom(m)
    return m


@pytest.mark.parametrize('dx,dy', [(1.,0.), (0.,1.), (-.8,.6)])
def test_translation_after_long_stationary_period_has_no_filter_lag(dx, dy):
    n = motion_node()
    for i in range(300):
        feed_field(n, (1.5,8.,.4),100+i*.1)
    for i in range(1,21):
        target=(1.5+dx*i/20,8.+dy*i/20,.4)
        feed_field(n,target,130+i*.1)
        assert n.ekf.pose == pytest.approx(target,abs=1e-6)


def test_turning_about_base_does_not_translate_base_and_tracks_yaw_wrap():
    n=motion_node()
    for i in range(121):
        target=(1.5,8.,wrap_pi(.4+i*2*math.pi/120))
        feed_field(n,target,100+i*.1)
        assert n.ekf.pose == pytest.approx(target,abs=1e-6)


def test_pose_delta_not_twist_drives_absolute_odometry():
    n=motion_node()
    feed_field(n,(1.5,8.,.4),100)
    feed_field(n,(2.5,8.,.4),102)  # a source gap must not lose accumulated displacement
    assert n.ekf.pose == pytest.approx((2.5,8.,.4),abs=1e-6)
