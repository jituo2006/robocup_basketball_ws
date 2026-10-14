import math
import pytest
from test_lidar_motion import motion_node, feed_field
from test_localization_output import NS, odom


def test_reset_after_manual_return_reanchors_only_from_new_source():
    n = motion_node()
    n.configured_start_pose = n.start_pose
    feed_field(n, (1.5, 8., .4), 100)
    feed_field(n, (2., 8.2, .6), 100.2)
    n.now = 101.
    response = n.reset_experiment_start(None, NS())
    assert response.success
    assert n.ekf.pose == pytest.approx(n.configured_start_pose)
    assert n.odom_origin is None and n.last_odom_field_pose is None
    assert n.last_source_stamp is None
    assert not n.ok_messages[-1].data
    assert n.pose_messages[-1].header.stamp.sec == 0
    n.on_odom(odom(100.9))
    assert n.odom_origin is None and n.odom_count == 0
    # FAST-LIO keeps its map; the robot was carried to a different map pose.
    m = odom(101.1)
    m.pose.pose.position.x = 4.; m.pose.pose.position.y = -2.
    n.now = 101.11
    n.on_odom(m)
    assert n.ekf.pose == pytest.approx(n.configured_start_pose)
    n.publish_state(None)
    assert n.ok_messages[-1].data
    theta = n.theta
    m = odom(101.2)
    m.pose.pose.position.x = 4. + .1 * math.cos(theta)
    m.pose.pose.position.y = -2. - .1 * math.sin(theta)
    n.now = 101.21
    n.on_odom(m)
    assert n.ekf.pose == pytest.approx((1.6, 8., .4), abs=1e-6)


@pytest.mark.parametrize('flag,value', [('relative_mode', True),
                                      ('odom_conv_enabled', False),
                                      ('use_odom_pose_as_abs', False)])
def test_unsupported_reset_does_not_change_pose(flag, value):
    n = motion_node(); n.configured_start_pose = n.start_pose
    setattr(n, flag, value)
    before = n.ekf.pose
    assert not n.reset_experiment_start(None, NS()).success
    assert n.ekf.pose == before
