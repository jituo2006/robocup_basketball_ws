import json
import time
import pytest
from test_lidar_motion import motion_node, feed_field
from test_localization_output import NS, odom


def supervised():
    n=motion_node()
    n.on_lidar_session.__func__.__globals__['time']=time
    n.require_lidar_session=True
    n.lidar_session_ready=False
    n.lidar_session_generation=None
    n.lidar_session_blocked=None
    n.lidar_session_received=0.
    n.lidar_session_cutoff=0.
    n.start_pose=(1.5,8.,.4)
    n.last_pillar_stamp=0.
    return n


def status(n,generation,ready,cutoff=100):
    n.on_lidar_session(NS(data=json.dumps(dict(generation=generation,ready=ready,cutoff=cutoff))))


def test_startup_ignores_pose_until_a_stable_session():
    n=supervised()
    n.on_odom(odom())
    assert n.odom_count==0
    status(n,1,False)
    n.on_odom(odom())
    assert n.odom_count==0
    status(n,1,True)
    n.on_odom(odom())
    assert n.odom_count==1


def test_new_session_preserves_last_good_field_pose_and_clears_baseline():
    n=supervised();status(n,1,True)
    feed_field(n,(1.5,8.,.4),100)
    feed_field(n,(1.6,8.,.4),100.1)
    before=n.ekf.pose
    status(n,2,False)
    assert n.start_pose==pytest.approx(before)
    assert n.last_odom_time is None and n.last_odom_field_pose is None
    assert n.odom_origin is None and n.last_source_stamp is None
    n.publish_state(None)
    assert not n.ok_messages[-1].data
    assert n.pose_messages[-1].header.stamp.sec==0
    status(n,2,True,102)
    n.now=102
    n.on_odom(odom(100.2))  # queued old frame must not seed the new baseline
    assert n.last_odom_time is None
    # Use the real frame conversion after restarting FAST-LIO at a new origin.
    n.odom_conv_enabled=True
    n.odom_to_field=type(n).odom_to_field.__get__(n)
    n.on_odom(odom(102))
    assert n.ekf.pose==pytest.approx(before)


def test_jump_is_rejected_even_if_supervisor_callback_arrives_late():
    n=supervised();status(n,1,True)
    feed_field(n,(1.5,8.,.4),100)
    feed_field(n,(475.,100.,.4),100.1)
    assert n.ekf.pose==pytest.approx((1.5,8.,.4))
    assert not n.lidar_session_ready and n.last_source_stamp is None
    status(n,1,True,100.1)
    assert not n.lidar_session_ready  # unchanged ready heartbeat cannot clear the fault
    status(n,2,False)
    status(n,2,True,102)
    assert n.lidar_session_ready


def test_manager_disappearance_expires_session_and_old_pose():
    n=supervised();status(n,1,True)
    feed_field(n,(1.5,8.,.4),100)
    n.lidar_session_received=time.monotonic()-1
    n.publish_state(None)
    assert not n.ok_messages[-1].data
    assert n.last_source_stamp is None


def test_long_source_gap_does_not_integrate_old_state():
    n=supervised();status(n,1,True)
    feed_field(n,(1.5,8.,.4),100)
    feed_field(n,(2.5,8.,.4),102)
    assert n.ekf.pose==pytest.approx((1.5,8.,.4))
    assert not n.lidar_session_ready
