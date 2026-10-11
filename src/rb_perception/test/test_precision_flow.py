"""Run the actual ROS wrapper against simulated transport/kinematics.

These tests do not import ROS, contact a robot, or publish real commands.
"""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import numpy as np
import yaml
import pytest

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS/'tools'))
import precision_ball_approach as flow


@pytest.mark.parametrize('rear_data', [True, False])
def test_actual_flow_requires_new_rear_data_before_success(tmp_path, monkeypatch, rear_data):
    state = NS(t=10., x=0., y=0., yaw=0., cmd=(0., 0., 0.), count=0, moving=False)
    messages = []
    callbacks = {}
    config = {'ball_label':'ball_basketball', 'target_center_distance_m':.60,
              'intake_confirmed':True, 'intake_position_base_m':[-.4, 0., .12],
              'intake_outward_xy':[-1., 0.], 'front_range_topic':'front',
              'rear_range_topic':'rear', 'odom_topic':'odom', 'stage_timeout_s':45.}
    path = tmp_path/'cfg.yaml'; path.write_text(yaml.safe_dump(config))
    monkeypatch.setattr(flow, 'validate', lambda cfg: np.eye(4))
    monkeypatch.setattr(flow.time, 'monotonic', lambda: state.t)

    class Twist:
        def __init__(self):
            self.linear, self.angular = NS(x=0., y=0., z=0.), NS(x=0., y=0., z=0.)

    def publish(msg):
        state.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        state.moving |= any(abs(v)>0 for v in state.cmd)
        messages.append(state.cmd)

    logger = NS(info=lambda s: None, error=lambda s: None)
    node = NS(create_publisher=lambda *a: NS(publish=publish),
              create_subscription=lambda _type, topic, callback, qos: callbacks.update({topic:callback}),
              count_publishers=lambda t:1, get_logger=lambda:logger, destroy_node=lambda:None,
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(state.t*1e9))))

    def spin_once(node, timeout_sec):
        state.count += 1
        assert state.count < 1200, 'flow did not stop or time out'
        dt=.1
        vx, vy, wz = state.cmd
        state.x += (math.cos(state.yaw)*vx - math.sin(state.yaw)*vy)*dt
        state.y += (math.sin(state.yaw)*vx + math.cos(state.yaw)*vy)*dt
        state.yaw += wz*dt
        state.t += dt
        sec=int(state.t); nano=int((state.t-sec)*1e9)
        stamp = NS(sec=sec, nanosec=nano)
        pose = NS(header=NS(stamp=stamp),
                  pose=NS(pose=NS(position=NS(x=state.x,y=state.y,z=0.),
                                  orientation=NS(x=0.,y=0.,z=math.sin(state.yaw/2),w=math.cos(state.yaw/2)))),
                  # FAST-LIO may leave twist zero: pose differentiation must settle.
                  twist=NS(twist=Twist()))
        callbacks['odom'](pose)
        dx,dy=1.0-state.x,-state.y
        ball=[math.cos(state.yaw)*dx+math.sin(state.yaw)*dy,
              -math.sin(state.yaw)*dx+math.cos(state.yaw)*dy,.12]
        report={'valid':True, 'camera_stamp':state.t-.005, 'cloud_stamp':state.t-.005,
                'targets':[{'label':'ball_basketball','centre_base_m':ball,'radius_m':.12}]}
        callbacks['front'](NS(data=json.dumps(report)))
        if rear_data:
            callbacks['rear'](NS(data=json.dumps(report)))

    ros = NS(init=lambda:None, create_node=lambda n:node, spin_once=spin_once,
             ok=lambda:True, shutdown=lambda:None)
    monkeypatch.setitem(sys.modules, 'rclpy', ros)
    monkeypatch.setitem(sys.modules, 'rclpy.qos', NS(qos_profile_sensor_data=None))
    monkeypatch.setitem(sys.modules, 'geometry_msgs.msg', NS(Twist=Twist))
    monkeypatch.setitem(sys.modules, 'nav_msgs.msg', NS(Odometry=object))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=object))
    result = flow.main(['--config',str(path),'--execute'])
    assert result == (0 if rear_data else 1)
    assert state.moving
    assert abs(abs(state.yaw)-math.pi) < .06
    assert all(math.hypot(c[0],c[1])<=.080001 and abs(c[2])<=.180001 for c in messages)
    assert messages[-5:] == [(0.,0.,0.)]*5
