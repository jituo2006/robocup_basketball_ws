"""Frame rotation, heading, first-pass route and radar-offset contracts; no ROS."""
import math
from types import SimpleNamespace as NS
import numpy as np
import pytest
from field_coordinates import to_public_pose,to_legacy_pose,to_public_point,first_leg,public_field
from first_pass_route import FirstPassPlan,load_config
from match_round_route import load_config as load_round


@pytest.mark.parametrize('pose',[(0.,0.,0.),(1.5,8.,-.9),(12.425,3.75,math.pi/2)])
def test_rotation_round_trip_and_heading(pose):
    result=to_public_pose(pose)
    original=to_legacy_pose(result)
    assert original[:2]==pytest.approx(pose[:2])
    assert math.cos(original[2])==pytest.approx(math.cos(pose[2]))
    assert math.sin(original[2])==pytest.approx(math.sin(pose[2]))
    # Vehicle left remains left: this is a proper rotation, not a reflection.
    moved=to_public_point((pose[0]-math.sin(pose[2]),pose[1]+math.cos(pose[2])))
    assert np.subtract(moved,result[:2])==pytest.approx([-math.sin(result[2]),math.cos(result[2])])




def test_first_leg_preserves_old_physical_path_and_user_points_are_public():
    cfg=load_config();leg=first_leg(load_round(),cfg)
    assert leg['start']==pytest.approx([-4.225,1.5])
    assert leg['middle_point']==pytest.approx([-1.025,1.5])
    assert leg['scan_points'][0]==pytest.approx([-1.025,4.4])
    assert leg['heading_rad']==0.
    assert leg['pass_point']==(-1.9,3.45)
    assert cfg['return_point']==[0.,4.8]
    assert cfg['second_scan_end']==[0.,0.]


@pytest.mark.parametrize('local_yaw',[0.,math.pi/2,math.pi])
def test_public_radar_targets_convert_once_and_rotate_offset(local_yaw):
    r=NS(pose=(1.2,8.,local_yaw),args=NS(route_intake=False))
    cfg=load_config();plan=FirstPassPlan(r,load_round(),cfg,lambda s:None)
    for point in [cfg['pass_point'],cfg['second_scan_end']]:
        base=plan.target(point,local_yaw)
        lidar=(base[0]+.3*math.cos(local_yaw),base[1]+.3*math.sin(local_yaw))
        assert plan.frame.nominal(lidar)==pytest.approx(point)
        assert math.sin(plan.frame.yaw(math.pi/2)-local_yaw)==pytest.approx(0.,abs=1e-9)
        assert math.cos(plan.frame.yaw(math.pi/2)-local_yaw)==pytest.approx(1.)


def test_public_geometry_is_a_copy_and_does_not_change_legacy_ros_configuration():
    original={'length_m':14.,'width_m':7.5,'hoop':{'x':12.425,'y':3.75},
              'pass_zone':{'polygon':[[8.,1.],[13.,6.5]]}}
    public=public_field(original)
    assert public['hoop']==pytest.approx({'x':.025,'y':12.425})
    assert public['x_extent_m']==7.5 and public['y_extent_m']==14.
    assert public['pass_zone']['polygon']==pytest.approx([(2.775,8.),(-2.725,13.)])
    assert original['hoop']['x']==12.425


def test_centered_origin_and_midfield_contract():
    from field_coordinates import to_legacy_point, public_bounds, X_ORIGIN_M, MIDFIELD_Y_M
    assert X_ORIGIN_M == 3.725 and MIDFIELD_Y_M == 5.75
    assert public_bounds() == (-3.725, 3.775, 0., 14.)
    assert to_public_point(to_legacy_point((0.,4.8))) == pytest.approx((0.,4.8))
    field=public_field({'length_m':14.,'width_m':7.5})
    assert field['x_min_m']==-3.725 and field['x_max_m']==3.775
    assert field['midfield_y_m']==5.75


@pytest.mark.parametrize('point',[[-3.726,3.45],[3.776,3.45],[0.,-0.01],[0.,14.01]])
def test_route_rejects_points_outside_centered_bounds(tmp_path,point):
    import json
    cfg=load_config();cfg['pass_point']=point
    path=tmp_path/'route.json';path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError):load_config(path)


@pytest.mark.parametrize('point,accepted', [((-4.225,1.5),True),((0.,12.),True),
                                          ((-5.8,1.5),False),((0.,16.1),False)])
def test_motion_bounds_use_centered_route_frame(point,accepted):
    from test_perimeter_ball import Runner
    stops=[]
    r=NS(pose=(*point,0.),pose_stamp=10.,odom_stamp=10.,field_l=14.,field_w=7.5,
         round_coordinate_frame='field_center_y_basket',round_start_frame=NS(nominal=lambda p:p[:2]),
         args=NS(pose_timeout=1.),stop=lambda:stops.append(True),
         node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000))))
    if accepted:
        Runner.require_fresh_pose(r)
        assert not stops
    else:
        with pytest.raises(RuntimeError,match='超出场地'):Runner.require_fresh_pose(r)
        assert stops
