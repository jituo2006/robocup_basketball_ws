"""单元测试：tools/test_perimeter_ball.py 的方位角/转身几何。

为什么单测这几个纯函数：**方位角符号是整个文件最容易错、又最难从现象看出问题的地方**。
`Detection.bearing_rad` 是相机约定（右正左负），而场地系是 y 朝左、逆时针为正
—— 两者相反。写反了车就会朝镜像方向走，"背对球"会变成"面对球"。

运行：colcon test --packages-select rb_tests
"""

from __future__ import annotations

from pathlib import Path

_WS = Path(__file__).resolve().parents[3]

import importlib.util
import math
import sys

import pytest

from rb_tests.ws import WS


def _load():
    path = WS / "tools" / "test_perimeter_ball.py"
    if not path.is_file():
        pytest.skip(f"{path} 不存在")
    spec = importlib.util.spec_from_file_location("_tool_perimeter_ball", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# bearing_to_world：相机约定 → 场地系
# ---------------------------------------------------------------------------
def test_ball_dead_ahead_is_at_robot_heading():
    """车头朝 0°，球在正前方（bearing=0）→ 球在场地系方位 0（+x 方向）。"""
    m = _load()
    assert abs(m.bearing_to_world(0.0, 0.0)) < 1e-9


def test_ball_on_the_right_maps_to_negative_y():
    """⚠️ 关键符号：相机 bearing 右正，而场地系 y 朝左 → 右边的球应是负方位角。"""
    m = _load()
    # 车头朝 0°，球在右前方 45°（bearing=+π/4）
    got = m.bearing_to_world(0.0, math.pi / 4)
    assert abs(got - (-math.pi / 4)) < 1e-9, f"应为 -45°，实际 {math.degrees(got):.1f}°"
    # 负方位角 → 场地系 y 为负（右侧）✓
    assert math.sin(got) < 0


def test_ball_on_the_left_maps_to_positive_y():
    m = _load()
    got = m.bearing_to_world(0.0, -math.pi / 4)      # 左边
    assert got > 0 and math.sin(got) > 0


def test_bearing_is_relative_to_heading():
    """车头转到 90° 后，正前方的球应落在场地系 90°。"""
    m = _load()
    assert abs(m.bearing_to_world(math.pi / 2, 0.0) - math.pi / 2) < 1e-9


def test_camera_yaw_offset_shifts_result():
    m = _load()
    got = m.bearing_to_world(0.0, 0.0, cam_yaw_offset=math.radians(10))
    assert abs(got - math.radians(10)) < 1e-9


# ---------------------------------------------------------------------------
# yaw_to_put_ball_behind：转身让背面对着球
# ---------------------------------------------------------------------------
def test_ball_ahead_turn_180():
    """球在正前方 → 车头要转到 180°（正后方才是球的方向）。"""
    m = _load()
    got = m.yaw_to_put_ball_behind(0.0, 0.0)
    assert abs(abs(got) - math.pi) < 1e-9, f"应为 ±180°，实际 {math.degrees(got):.1f}°"


def test_back_faces_ball_geometrically():
    """几何验证：转身后，球的方向与车头方向夹角应为 180°。"""
    m = _load()
    for yaw in (0.0, 0.7, -1.3, 2.9):
        for bearing in (0.0, 0.4, -0.6, 1.1):
            target = m.yaw_to_put_ball_behind(yaw, bearing)
            ball_world = m.bearing_to_world(yaw, bearing)
            diff = m.wrap_pi(ball_world - target)          # 球相对车头的夹角
            assert abs(abs(diff) - math.pi) < 1e-9, (
                f"yaw={yaw} bearing={bearing}: 球在车头 {math.degrees(diff):.1f}° 处，"
                f"应恰好 180°（背面）")


# ---------------------------------------------------------------------------
# ball_xy：反算球在场地系的坐标
# ---------------------------------------------------------------------------
def test_ball_xy_dead_ahead():
    m = _load()
    x, y = m.ball_xy(1.0, 2.0, 0.0, 0.0, 3.0)
    assert abs(x - 4.0) < 1e-9 and abs(y - 2.0) < 1e-9


def test_ball_xy_on_the_right_is_minus_y():
    """右前方 45°、光轴深度 2m → 斜距 2/cos45° ≈ 2.828，落在右前方。

    ⚠️ 这里以前按"distance 就是斜距"写（用 90° 直接给 y=-2），是错的：
    Detection.distance_m 是【光轴深度 Z】，90° 时斜距发散、不可信。
    """
    m = _load()
    b = math.radians(45)
    x, y = m.ball_xy(0.0, 0.0, 0.0, b, 2.0)
    r = 2.0 / math.cos(b)
    assert x == pytest.approx(r * math.cos(b))
    assert y == pytest.approx(-r * math.sin(b))
    assert y < 0, "右前方 → 场地系 y 应为负"


def test_ball_xy_uses_slant_not_optical_depth():
    """关键回归：偏轴时不能用光轴深度当斜距（否则球位置偏近）。"""
    m = _load()
    b = math.radians(45)
    x, y = m.ball_xy(0.0, 0.0, 0.0, b, 2.0)
    r_depth_only = math.hypot(x, y)
    assert r_depth_only > 2.0 + 0.5, (
        f"斜距应明显大于光轴深度 2.0（45° 时约 2.83），实际 {r_depth_only}")


def test_ball_xy_rejects_near_horizon_bearing():
    """方位角接近 ±90° 时斜距不可信，应报错而不是给个巨大的坐标。"""
    m = _load()
    with pytest.raises(ValueError):
        m.ball_xy(0.0, 0.0, 0.0, math.radians(89), 2.0)


def test_ball_xy_dead_ahead_is_unchanged_by_slant():
    """正前方时 cos=1，斜距 = 光轴深度 —— 结果应与旧实现一致。"""
    m = _load()
    x, y = m.ball_xy(1.0, 2.0, 0.0, 0.0, 3.0)
    assert abs(x - 4.0) < 1e-9 and abs(y - 2.0) < 1e-9


def test_ball_xy_respects_heading():
    """车头朝 +y（90°），球在正前方 2m → 应落在 (0, 2)。"""
    m = _load()
    x, y = m.ball_xy(0.0, 0.0, math.pi / 2, 0.0, 2.0)
    assert abs(x) < 1e-9 and abs(y - 2.0) < 1e-9


# ---------------------------------------------------------------------------
# 现场规格：找球 → 摄像头正对球 → 旋转中心停在球前 70cm → 转 180° → 后退 20cm
# ---------------------------------------------------------------------------
def _load():
    import importlib.util
    from pathlib import Path
    ws = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        "_tpb", ws / "tools" / "test_perimeter_ball.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_median_ball_rejects_outliers():
    """单帧离群值（把干扰物当球）不能把估计带偏 —— 用中位数。"""
    mod = _load()
    good = [(0.10, 0.70)] * 5
    bad = [(1.20, 4.50)]          # 一帧离谱的
    r = mod.median_ball(good + bad)
    assert r is not None
    b, d, n = r
    assert abs(b - 0.10) < 1e-9, f"方位被离群值带偏: {b}"
    assert abs(d - 0.70) < 1e-9, f"距离被离群值带偏: {d}"
    assert n == 6


def test_median_ball_needs_at_least_two():
    mod = _load()
    assert mod.median_ball([]) is None
    assert mod.median_ball([(0.1, 0.7)]) is None


def test_median_ball_drops_nan_and_tiny_distance():
    """NaN 和过小的距离必须被丢掉，剩下的样本才参与中位数。"""
    mod = _load()
    r = mod.median_ball([(float("nan"), 0.7), (0.1, float("nan")),
                         (0.1, 0.0),                       # 距离 0 非法
                         (0.10, 0.70), (0.12, 0.72), (0.11, 0.71)])
    assert r is not None
    assert r[2] == 3, f"应剩 3 个合法样本，实际 {r[2]}"
    assert abs(r[0] - 0.11) < 1e-9, f"中位数应为 0.11，实际 {r[0]}"
    assert abs(r[1] - 0.71) < 1e-9


def test_approach_axis_is_yaw_minus_bearing():
    """球在场地系的方位 = yaw − bearing（bearing 右正左负）。"""
    import math
    mod = _load()
    assert abs(mod.approach_axis(0.0, 0.0)) < 1e-9
    # 球在正左（车体系 y>0）⇒ bearing 负，方位应为 +90°
    assert abs(mod.approach_axis(0.0, -math.pi / 2) - math.pi / 2) < 1e-9
    # 球在正右 ⇒ bearing 正，方位应为 -90°
    assert abs(mod.approach_axis(0.0, math.pi / 2) + math.pi / 2) < 1e-9


def test_servo_approach_math_moves_toward_ball():
    """沿 approach_axis 走就是朝球走 —— 正对着球时（bearing≈0）应沿车头前进。"""
    import math
    mod = _load()
    yaw = 0.5
    ang = mod.approach_axis(yaw, 0.0)
    assert abs(ang - yaw) < 1e-9, "正对球时前进方向应等于车头朝向"
    # 从 (0,0) 沿该方向走 0.5m，终点应朝球
    tx, ty = 0.5 * math.cos(ang), 0.5 * math.sin(ang)
    assert abs(math.hypot(tx, ty) - 0.5) < 1e-9


def test_shipped_defaults_match_the_site_spec():
    """出厂默认必须就是现场要的：70cm / 180° / 20cm。"""
    src = (_WS / "tools" / "test_perimeter_ball.py").read_text(encoding="utf-8")
    assert '"--align-dist", type=float, default=0.70' in src, "对准距离默认应为 70cm"
    assert '"--turn-deg", type=float, default=180.0' in src, "转身默认应为 180°"
    assert '"--backup", type=float, default=0.20' in src, "后退默认应为 20cm"


def test_align_tolerance_is_tight_enough_to_face_the_ball():
    """对准容差要够紧才叫"正对球"；太松等于没对准。"""
    src = (_WS / "tools" / "test_perimeter_ball.py").read_text(encoding="utf-8")
    i = src.find('"--align-tol-deg"')
    assert i > 0
    seg = src[i:i + 160]
    import re
    m = re.search(r"default=([\d.]+)", seg)
    assert m, "读不到 align-tol-deg 默认值"
    assert float(m.group(1)) <= 3.0, f"对准容差 {m.group(1)}° 太松，谈不上正对球"


def test_ball_runner_callbacks_have_no_undefined_lock():
    """⚠️ Runner 的回调不能引用 self.lock —— 这个类里没有锁。

    踩过：加后摄回调时写了 `with self.lock:`，Runner 没有 lock 属性 →
    回调抛 AttributeError → 直接打断 ros2 执行器 → "找球"刚开始就崩，
    现象是 AttributeError: 'Runner' object has no attribute 'lock'。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[3]
           / "tools" / "test_perimeter_ball.py").read_text(encoding="utf-8")
    # ⚠️ 只看【代码行】、跳过注释 —— 注释里正要写 "self.lock" 来说明这个坑。
    #    这个错我已经犯了三次（--params-file / grep -q / 这次），
    #    凡是"断言某字符串不出现"的测试，一律先滤掉注释。
    code = "\n".join(ln for ln in src.splitlines()
                     if not ln.lstrip().startswith("#"))
    assert "self.lock" not in code, "Runner 里不该出现 self.lock（没有这个属性）"


# ---------------------------------------------------------------------------
# 三级找球：① 前摄转圈 → ② 雷达粗定位 → ③ 进场 + 顺时针旋转
# ---------------------------------------------------------------------------
def _ball_script_src() -> str:
    from pathlib import Path
    return (Path(__file__).resolve().parents[3]
            / "tools" / "test_perimeter_ball.py").read_text(encoding="utf-8")


def test_find_ball_has_three_stages():
    """找球必须是三级回退，而不是只会让摄像头盲转。

    现场要求：前摄没找到 → 用雷达找附近有无球 → 都没有就向场内移动 1m
    再顺时针旋转找球。
    """
    src = _ball_script_src()
    for tag in ("找球①", "找球②", "找球③"):
        assert tag in src, f"缺 {tag} 这一段"
    assert "lidar_find_ball" in src, "缺雷达兜底找球"
    assert "move_into_field" in src, "缺'向场内移动'"
    assert "_scan_for_ball" in src, "缺可复用的转圈找球"


def test_lidar_fallback_then_camera_confirms():
    """雷达是粗定位，转过去之后必须用摄像头复核。

    雷达在地面上找的是一个小点簇，容易有杂点；直接拿它当结果会把车
    引到没球的地方。
    """
    src = _ball_script_src()
    i = src.find("找球②")
    assert i > 0
    seg = src[i:i + 1600]
    assert "rotate_to" in seg, "雷达候选后应转向"
    assert "_scan_for_ball" in seg, "转过去后应用摄像头确认"


def test_clockwise_search_uses_negative_wz():
    """顺时针旋转 = 角速度为负（ROS 里 +z 是逆时针）。"""
    src = _ball_script_src()
    i = src.find('add_argument("--cw-wz"')
    assert i > 0, "缺 --cw-wz 参数"
    seg = src[i:i + 200]
    assert "default=-" in seg, "顺时针角速度应为负"


def test_move_into_field_goes_toward_center_and_stays_in_field():
    """向场内移动要朝场地中心、并且夹在场内（别把自己走出界）。"""
    src = _ball_script_src()
    i = src.find("def move_into_field")
    assert i > 0
    j = src.find("def find_ball", i)
    body = src[i:j]
    assert "field_l / 2.0" in body and "field_w / 2.0" in body, "应朝场地中心"
    assert "min(max(tx" in body and "min(max(ty" in body, "应夹在场内"


def test_pointcloud_parser_is_shared_not_duplicated():
    """点云解析只应有一份实现（liar: 复制两份迟早漂移）。"""
    from pathlib import Path
    ws = Path(__file__).resolve().parents[3]
    lb = (ws / "src/rb_perception/rb_perception/lidar_ball.py").read_text(encoding="utf-8")
    assert "def pointcloud2_to_xyz" in lb, "共享实现应在 lidar_ball.py"
    con = (ws / "tools/console_app.py").read_text(encoding="utf-8")
    assert "from rb_perception.lidar_ball import pointcloud2_to_xyz" in con, \
        "控制台应复用共享实现，而不是自己再写一份"


def test_servo_turns_right_for_a_ball_on_right_and_remeasures():
    from types import SimpleNamespace
    mod = _load()
    runner = mod.Runner.__new__(mod.Runner)
    runner.pose = (0.0, 0.0, 0.0)
    runner.args = SimpleNamespace()
    runner.measure_ball_stable = lambda: (0.1, 1.0, 5)
    turns, moves = [], []
    runner.rotate_to = lambda yaw, timeout: turns.append(yaw) or True
    runner.goto = lambda *a, **kw: moves.append(a) or True
    runner.servo_to_ball(0.7, 1.5, 0.05, 1, 0.12, 12)
    assert turns == pytest.approx([-0.1])
    assert moves == []  # 必须重新量球，不能使用转动前的方位平移。


def test_disabled_field_move_does_not_clamp_uncalibrated_pose():
    mod = _load()
    runner = mod.Runner.__new__(mod.Runner)
    runner.pose = (1.5, 8.0, 0.0)
    runner.field_l, runner.field_w = 14.0, 7.5
    runner.goto = lambda *a, **kw: pytest.fail("禁用进场后仍产生移动")
    assert runner.move_into_field(0.0)


@pytest.mark.parametrize("pose_age,odom_age", [(0.6, 0.1), (0.1, 1.4)])
def test_stale_pose_or_lidar_requests_stop(pose_age, odom_age):
    from types import SimpleNamespace
    mod = _load()
    runner = mod.Runner.__new__(mod.Runner)
    runner.pose = (0.0, 0.0, 0.0)
    runner.pose_stamp, runner.odom_stamp = 100.0-pose_age, 100.0-odom_age
    runner.args = SimpleNamespace(pose_timeout=0.5)
    runner.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    stops = []
    runner.stop = lambda: stops.append(True)
    with pytest.raises(RuntimeError, match="时间戳过期"):
        runner.require_fresh_pose()
    assert stops == [True]


def test_old_detection_is_not_fresh_even_when_republished():
    from types import SimpleNamespace
    mod = _load()
    runner = mod.Runner.__new__(mod.Runner)
    runner.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    assert not runner.detection_fresh(98.5)
    assert runner.detection_fresh(99.9)
