"""Collision vetoes: source freshness, body directions, independent sensors."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
import sys
sys.path.insert(0,str(ROOT/'tools'))
import front_obstacle_guard as g


def cfg():
    return NS(obstacle_guard=True, obstacle_half_length=.5,obstacle_half_width=.5,
              obstacle_stop_gap=.25,obstacle_side_margin=.15,obstacle_timeout=.5,
              obstacle_confidence=.5,obstacle_camera_x=.33,obstacle_depth_scale=.8,
              obstacle_box_size=.22,obstacle_deceleration=.3,obstacle_lidar_cells=3)


def det(label='ball_volleyball',x=.5,distance=.6,stamp=10.,size=.1):
    return NS(label=label,source='camera',confidence=.9,
              x_min=x-size/2,x_max=x+size/2,y_min=.4,y_max=.4+size,
              stamp=NS(sec=int(stamp),nanosec=int((stamp%1)*1e9)),
              distance_m=distance,bearing_rad=math.atan((x-.5)*2),diameter_m=.24)


def cloud():
    floor=np.array([(x,y,-1.2) for x in np.linspace(-3,3,30) for y in np.linspace(-3,3,30)])
    return floor


class TestGuard(unittest.TestCase):
    def test_removed_box_size_parameter_uses_compatible_default(self):
        settings=cfg();del settings.obstacle_box_size
        assert g.camera_hazard([det(size=.25)],10.,.25,0.,settings)=='camera:near_ball:ball_volleyball'

    def test_pointcloud_padding_and_endianness(self):
        for big in (False,True):
            data=bytearray(40)
            for row in range(2):
                a=np.ndarray((1,3),dtype=('>' if big else '<')+'f4',buffer=data,offset=row*20)
                a[:]=[[row+1,2,3]]
            msg=NS(fields=[NS(name=n,offset=i*4,datatype=7) for i,n in enumerate(('x','y','z'))],
                   width=1,height=2,point_step=12,row_step=20,data=data,is_bigendian=big)
            np.testing.assert_allclose(g.pointcloud_xyz(msg),[[1,2,3],[2,2,3]])

    def test_pose_jump_latches_fault_and_does_not_replace_good_pose(self):
        from collections import deque
        r=self.runner();r.pose=(1.5,8.,0);r.pose_stamp=10.;r.pose_history=deque();r.yaw_rate=0
        q=NS(x=0,y=0,z=0,w=1)
        m=NS(pose=NS(pose=NS(position=NS(x=20.,y=8.,z=0),orientation=q)),
             header=NS(stamp=NS(sec=10,nanosec=100000000)))
        r._on_pose(m)
        self.assertIsNotNone(r.motion_fault)
        self.assertEqual(r.pose,(1.5,8.,0))
        with self.assertRaises(RuntimeError):r.require_fresh_pose()

    def test_continuous_pose_is_accepted(self):
        from collections import deque
        r=self.runner();r.pose=(1.5,8.,0);r.pose_stamp=10.;r.pose_history=deque();r.yaw_rate=0
        m=NS(pose=NS(pose=NS(position=NS(x=1.52,y=8.,z=0),orientation=NS(x=0,y=0,z=0,w=1))),
             header=NS(stamp=NS(sec=10,nanosec=100000000)))
        r._on_pose(m)
        self.assertEqual(r.pose,(1.52,8.,0))

    def test_both_ball_classes_veto_forward(self):
        for label in ['ball_basketball','ball_volleyball']:
            self.assertIsNotNone(g.camera_hazard([det(label)],10.1,.15,0,cfg()))

    def test_far_ball_and_out_of_path_not_blocking(self):
        self.assertIsNone(g.camera_hazard([det(distance=3)],10.1,.15,0,cfg()))
        self.assertIsNone(g.camera_hazard([det(x=.95,distance=2)],10.1,.15,0,cfg()))

    def test_large_box_stops_even_with_missing_or_overestimated_depth(self):
        for depth in [float('nan'),10.]:
            self.assertIsNotNone(g.camera_hazard([det(distance=depth,size=.3)],10.1,.15,0,cfg()))

    def test_stale_future_low_confidence_and_lidar_labels_not_camera_veto(self):
        for d in [det(stamp=8),det(stamp=11),det()]:
            if d.stamp.sec==10:d.confidence=.1
            self.assertIsNone(g.camera_hazard([d],10.1,.15,0,cfg()))
        d=det();d.source='lidar'
        self.assertIsNone(g.camera_hazard([d],10.1,.15,0,cfg()))

    def test_camera_right_is_body_negative_y(self):
        d=det(x=.8,distance=.7)
        self.assertIsNotNone(g.camera_hazard([d],10.1,0,-.08,cfg()))

    def test_floor_and_behind_obstacle_do_not_stop_forward(self):
        p=np.vstack([cloud(),[[-1,0,-1.05],[-1.1,0,-1.05],[-1.2,0,-1.05]]])
        self.assertEqual(g.lidar_hazard(p,.15,0,cfg()),(None,'usable'))

    def test_front_and_lateral_obstacles_independently_stop(self):
        for vx,vy,ob in [(.15,0,[[.8,0,-1.05],[.9,0,-1.05],[1,0,-1.05]]),
                          (0,.08,[[0,.8,-1.05],[0,.9,-1.05],[0,1,-1.05]])]:
            self.assertEqual(g.lidar_hazard(np.vstack([cloud(),ob]),vx,vy,cfg())[0],'lidar:path_obstacle')

    def test_single_voxel_noise_and_empty_cloud(self):
        self.assertIsNone(g.lidar_hazard(np.vstack([cloud(),[[.8,0,-1.05]]*9]),.15,0,cfg())[0])
        self.assertEqual(g.lidar_hazard([], .15,0,cfg())[1],'insufficient_points')

    def test_dense_local_object_is_not_accepted_as_ground(self):
        local=np.array([(x,y,-1.0) for x in np.linspace(.8,1,10) for y in np.linspace(-.1,.1,10)])
        self.assertEqual(g.lidar_hazard(local,.15,0,cfg())[1],'floor_unknown')

    def test_transform_translation_and_quaternion(self):
        p=g.transform_points([[1,0,0]],(.3,0,0),(0,0,math.sqrt(.5),math.sqrt(.5)))
        np.testing.assert_allclose(p,[[.3,1,0]],atol=1e-9)
        with self.assertRaises(ValueError):g.transform_points([[1,0,0]],(0,0,0),(0,0,0,0))

    def runner(self):
        spec=importlib.util.spec_from_file_location('seek_guard_test',ROOT/'tools/test_perimeter_ball.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        r=mod.Runner.__new__(mod.Runner)
        r.args=cfg();r.dets=[];r.det_stamp=10.;r.obstacle_body_cloud=None
        r.obstacle_cloud_stamp=0.;r.obstacle_scoop=False
        r.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)))
        r.stops=[];r.stop=lambda:r.stops.append(True)
        return r

    def test_forward_uses_camera_when_lidar_unavailable(self):
        r=self.runner();m=NS(linear=NS(x=.15,y=0))
        r.check_obstacle(m)
        r.dets=[det()]
        with self.assertRaises(RuntimeError):r.check_obstacle(m)
        self.assertEqual(r.stops,[True])

    def test_stale_camera_stops_even_with_no_detections(self):
        r=self.runner();r.det_stamp=8
        self.assertFalse(r.check_obstacle(NS(linear=NS(x=.15,y=0))))

    def test_lateral_motion_requires_usable_lidar(self):
        r=self.runner()
        self.assertFalse(r.check_obstacle(NS(linear=NS(x=0,y=.08))))
        r.obstacle_body_cloud=cloud();r.obstacle_cloud_stamp=10
        r.check_obstacle(NS(linear=NS(x=0,y=.08)))

    def test_lidar_veto_without_camera_ball(self):
        r=self.runner();r.obstacle_body_cloud=np.vstack([cloud(),[[.8,0,-1.05],[.9,0,-1.05],[1,0,-1.05]]]);r.obstacle_cloud_stamp=10
        with self.assertRaises(RuntimeError):r.check_obstacle(NS(linear=NS(x=.15,y=0)))

    def test_nonfinite_velocity_stops(self):
        r=self.runner()
        with self.assertRaises(RuntimeError):r.check_obstacle(NS(linear=NS(x=float('nan'),y=0)))

    def test_reverse_and_in_place_turn_outside_front_guard_scope(self):
        r=self.runner();r.det_stamp=0
        r.check_obstacle(NS(linear=NS(x=-.08,y=0)))
        r.check_obstacle(NS(linear=NS(x=0,y=0)))

    def test_blocked_command_never_reaches_publisher(self):
        r=self.runner();r.motion_guard=False;r.visual_guard=False;r.pose=None;r.pose_stamp=0;r.yaw_rate=0
        sent=[];r.cmd_pub=NS(publish=lambda m:sent.append(m));r.dets=[det()]
        with self.assertRaises(RuntimeError):r.publish_command(NS(linear=NS(x=.15,y=0),angular=NS(z=0)))
        self.assertEqual(sent,[])


if __name__=='__main__':unittest.main()
