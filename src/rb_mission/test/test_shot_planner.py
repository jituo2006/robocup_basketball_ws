import math
import pytest
from rb_mission.shot_planner import shot_solution


def config():
    # Synthetic fixtures only: these are not robot calibration measurements.
    return dict(map_confirmed=True, table=[dict(distance_m=2, speed=2000, angle=50),
                                          dict(distance_m=4, speed=4000, angle=70)])


def solve(cfg=None, pose=(0, 0, 0), hoop=None, bearing=0, depth=math.nan):
    return shot_solution(cfg or config(), pose, hoop or dict(x=3, y=0), bearing, depth)


def test_map_interpolation_with_camera_bearing():
    result = solve()
    assert (result.distance_m, result.speed, result.angle) == (3, 3000, 60)
    assert result.source == 'map+camera_bearing'


def test_camera_depth_fusion():
    result = solve(depth=3.2)
    assert result.distance_m == pytest.approx(3.06)
    assert result.speed == 3060 and result.angle == 61


def test_body_offsets_and_yaw_transform():
    cfg = config()
    cfg.update(camera_x_m=.2, launch_x_m=.5)
    result = solve(cfg, pose=(1, 2, math.pi/2), hoop=dict(x=1, y=5.5), depth=3.3)
    assert result.distance_m == pytest.approx(3)


def test_camera_right_positive_depth_projection():
    b = math.atan2(1, 3)
    result = solve(hoop=dict(x=3, y=-1), bearing=b, depth=3)
    assert result.distance_m == pytest.approx(math.sqrt(10))


@pytest.mark.parametrize('distance', [2, 4])
def test_table_endpoints(distance):
    assert solve(hoop=dict(x=distance,y=0)).speed == distance * 1000


@pytest.mark.parametrize('distance', [1.99, 4.01])
def test_refuses_extrapolation(distance):
    with pytest.raises(ValueError, match='射程'):
        solve(hoop=dict(x=distance,y=0))


@pytest.mark.parametrize('change', [dict(map_confirmed=False), dict(table=[]),
    dict(table=[dict(distance_m=3,speed=3000,angle=60),dict(distance_m=2,speed=2000,angle=50)]),
    dict(table=[dict(distance_m=2,speed=70000,angle=50),dict(distance_m=4,speed=4000,angle=70)]),
    dict(vision_weight=2), dict(launch_x_m=math.nan), dict(require_camera_range=True)])
def test_invalid_inputs_refused(change):
    cfg = config(); cfg.update(change)
    with pytest.raises(ValueError):
        solve(cfg, depth=3 if 'vision_weight' in change else math.nan)


def test_camera_map_disagreement():
    with pytest.raises(ValueError, match='位置不一致'):
        solve(depth=5)
    with pytest.raises(ValueError, match='方位'):
        solve(bearing=.6)


def test_launcher_heading_must_be_aligned():
    cfg = config(); cfg['aim_tolerance_rad'] = .08
    with pytest.raises(ValueError, match='出球方向'):
        solve(cfg, hoop=dict(x=3, y=1), bearing=-math.atan2(1, 3))


def test_vision_must_also_be_aligned_even_inside_map_consistency_gate():
    cfg = config(); cfg['aim_tolerance_rad'] = .08
    with pytest.raises(ValueError, match='视觉目标'):
        solve(cfg, bearing=.1)
