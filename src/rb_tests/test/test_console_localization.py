"""Regression coverage for stale localization and field display coordinates."""
import math
import threading
from types import SimpleNamespace as NS
import numpy as np
import pytest
from test_console_app import _load


def live_state():
    return dict(pose=(4.,3.,math.pi/2), loc_ok=True, odom=(1.,2.,0.), odom_x=1.,
                cloud=np.array([[1.,2.,0.]],dtype=np.float32),cloud_fps=10.,
                pose_source_stamp=100.,pose_stamp=200.,odom_source_stamp=100.,
                odom_stamp=200.,cloud_source_stamp=100.,cloud_stamp=200.)


def test_timer_republication_cannot_renew_stale_pose():
    mod=_load(); s=live_state();s['pose_stamp']=210.
    out=mod.localization_display_state(s,110.,210.)
    assert out['pose'] is None and not out['loc_ok']
    assert out['cloud'] is None and out['cloud_fps']==0.
    assert out['odom_delay']==10.  # recomputed, not frozen receipt delay
    assert s['pose'] is not None  # cached data retained for diagnostics


def test_fresh_measurements_remain_visible_and_future_or_unreceived_do_not():
    mod=_load();s=live_state()
    out=mod.localization_display_state(s,100.1,200.1)
    assert out['pose']==s['pose'] and out['cloud'] is s['cloud'] and out['loc_ok']
    assert mod.localization_display_state(s,99.,200.1)['pose'] is None
    assert mod.localization_display_state(s,100.1,210.)['pose'] is None


def test_candidates_use_tuple_pose_to_transform_front_and_left():
    mod=_load();b=mod.Bridge.__new__(mod.Bridge)
    b.lock=threading.Lock();b.state=live_state()
    assert b.body_to_field_xy(1.,0.)==pytest.approx([4.,4.])
    assert b.body_to_field_xy(0.,1.)==pytest.approx([3.,3.])
    b.state['pose']=None
    assert b.body_to_field_xy(1.,0.) is None


def test_missing_pose_does_not_draw_robot_at_zero_and_missing_tf_hides_cloud(monkeypatch):
    import cv2
    mod=_load();monkeypatch.setattr(mod,'draw_texts',lambda img,texts:img)
    img=cv2.imdecode(np.frombuffer(mod.render_lidar(None,None,[],{},False,False),np.uint8),1)
    red=(img[:,:,2]>200)&(img[:,:,0]<120)&(img[:,:,1]<120)
    assert not red.any()
    empty=mod.render_lidar(None,(4.,3.,0.),[],{},False,True)
    unaligned=mod.render_lidar(np.array([[5.,3.,0.]],dtype=np.float32),(4.,3.,0.),[],{},False,True)
    assert empty==unaligned
