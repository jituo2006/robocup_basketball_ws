"""Test the production publication/source callbacks with ROS message boundaries replaced."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
from rb_localization.freshness import SourceFreshness


def pose_message():
    return NS(header=NS(stamp=NS(sec=0,nanosec=0),frame_id=''),
              pose=NS(pose=NS(position=NS(x=0,y=0,z=0),
                              orientation=NS(x=0,y=0,z=0,w=0)), covariance=[0]*36))


def node():
    path=Path(__file__).parents[1]/'rb_localization'/'localization_node.py'
    tree=ast.parse(path.read_text())
    statements=[n for n in tree.body if isinstance(n,ast.ClassDef)
                or isinstance(n,ast.FunctionDef) and n.name=='yaw_to_quat']
    ns=dict(Node=object,math=math,SourceFreshness=SourceFreshness,
            PoseWithCovarianceStamped=pose_message,Bool=lambda:NS(data=False),
            SetParametersResult=lambda **kw:NS(**kw),wrap_pi=lambda a:math.atan2(math.sin(a),math.cos(a)),
            Time=NS(from_msg=lambda stamp:NS(nanoseconds=int((stamp.sec+stamp.nanosec*1e-9)*1e9))))
    module=ast.Module(body=[ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0)]+statements,type_ignores=[])
    exec(compile(ast.fix_missing_locations(module),str(path),'exec'),ns)
    cls=ns['LocalizationNode'];n=cls.__new__(cls)
    n.now=100.0
    n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(n.now*1e9)))
    n.get_parameter=lambda name:NS(value=.5)
    n.source_timeout_s=.5;n.source_freshness=SourceFreshness();n.last_source_stamp=None
    n.ekf=NS(pose=(0,0,0),std=(.1,.1,.1),update_pose=lambda *a:None)
    n.pose_messages=[];n.ok_messages=[]
    n.pose_pub=NS(publish=n.pose_messages.append);n.ok_pub=NS(publish=n.ok_messages.append)
    n.tf_broadcaster=None;n.field_frame='field';n.ok_std_threshold=.6
    n.last_odom_time=None;n.use_odom_pose_as_abs=True;n.odom_count=0;n.abs_count=0
    n.odom_to_field=lambda x,y,yaw:(x,y,yaw)
    return n


def odom(stamp=100):
    m=pose_message();m.header.stamp=NS(sec=int(stamp),nanosec=int((stamp-int(stamp))*1e9))
    m.twist=NS(twist=NS(linear=NS(x=0,y=0),angular=NS(z=0)))
    m.pose.pose.orientation.w=1
    return m


def test_timer_does_not_renew_source_stamp_or_health():
    n=node();n.publish_state(None)
    assert not n.ok_messages[-1].data
    n.on_odom(odom());n.publish_state(None)
    assert n.ok_messages[-1].data and n.pose_messages[-1].header.stamp.sec==100
    n.now=100.6;n.publish_state(None)
    assert not n.ok_messages[-1].data
    assert n.pose_messages[-1].header.stamp.sec==100
    n.on_odom(odom());assert n.odom_count==1  # stale input cannot renew filter


def test_identity_quaternion_does_not_turn_lidar_height_into_yaw():
    n=node();m=odom();m.pose.pose.position.z=.7
    assert n._message_yaw(m)==0
    m.pose.pose.orientation.w=0
    assert n._message_yaw(m)==.7  # explicit legacy all-zero quaternion


def test_rejected_parameters_do_not_change_runtime_or_source_state():
    n=node()
    assert not n.on_tuning_parameters([NS(name='source_timeout_s',value=-1)]).successful
    assert n.on_tuning_parameters([NS(name='source_timeout_s',value=1.0)]).successful
    assert n.source_timeout_s==.5  # validation only; applied from accepted ROS values later
