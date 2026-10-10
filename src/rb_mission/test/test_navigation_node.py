"""Exercise the actual node control methods without a DDS/ROS installation.

Only imports/base/message boundaries are replaced; controller methods are compiled
from the production class unchanged. DDS integration remains a robot-side check.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from rb_mission.geometry import goto_command, clamp, wrap_pi
from rb_mission.navigation import ArrivalConfig, ArrivalGate, PoseFeedback, fresh


def load_node_class():
    source = Path(__file__).parents[1] / 'rb_mission' / 'mission_node.py'
    tree = ast.parse(source.read_text())
    retained = [n for n in tree.body if isinstance(n, ast.ClassDef)
                or (isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                    and n.targets[0].id.startswith(('P_', 'LAUNCH_')))]
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)] + retained, type_ignores=[])
    ns = dict(Node=object, math=math, ArrivalConfig=ArrivalConfig, ArrivalGate=ArrivalGate,
              PoseFeedback=PoseFeedback, fresh=fresh, clamp=clamp, wrap_pi=wrap_pi,
              goto_command=goto_command, SetParametersResult=lambda **kw: NS(**kw))
    exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), ns)
    return ns['MissionNode']


@pytest.fixture
def node():
    cls = load_node_class()
    n = cls.__new__(cls)
    n._tuning = cls._tuning_defaults()
    n._apply_tuning(n._tuning)
    n.ros_parameters = dict(n._tuning)
    n.get_parameter = lambda name: NS(value=n.ros_parameters[name])
    n.arrival, n.feedback = ArrivalGate(), PoseFeedback()
    n.now = 100.0
    n._now_seconds = lambda: n.now
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(n.now*1e9)))
    n.get_logger = lambda: NS(info=lambda *a: None)
    n.commands = []
    n.publish_cmd = lambda vx,vy,wz: n.commands.append((vx,vy,wz))
    n.avoid_enabled = False
    n.publish_zero = lambda: n.publish_cmd(0,0,0)
    n.publish_outputs = lambda: None
    n.last_pose = (0,0,0)
    n.has_pose, n.loc_ok, n.estop = True, True, False
    n.ball_count, n.max_ball_count = 0, 2
    n.last_loc_ok_time = n.now
    n.feedback.stamp = n.now
    n.feedback.linear_speed = n.feedback.angular_speed = 0.0
    n.phase, n.mission = 'GOTO', 'IDLE'
    n.goto_goal = (1,0,0,False)
    n.elapsed = lambda: 0
    n.nav_timeout_s = 90
    return n


def measurement(n, time, x, y=0, yaw=0, speed=0, angular=0):
    n.now = time
    n.last_pose = (x,y,yaw)
    n.feedback.stamp = time
    n.feedback.linear_speed, n.feedback.angular_speed = speed, angular
    n.last_loc_ok_time = time


def test_goto_keeps_target_until_stopped_and_confirmed(node):
    measurement(node,100,.94,speed=.15)
    node._phase_goto()
    assert node.goto_goal is not None and node.commands[-1] == (0,0,0)
    measurement(node,100.1,1.14,speed=.08)
    node._phase_goto()
    assert math.hypot(*node.commands[-1][:2]) <= .08
    assert node.goto_goal is not None
    measurement(node,100.2,.94)
    node._phase_goto()
    measurement(node,100.71,.94)
    node._phase_goto()
    assert node.goto_goal is None and node.phase == 'IDLE'


def test_stale_pose_or_status_stops_and_keeps_goal(node):
    goal = node.goto_goal
    node.now = 100.6
    node.tick()
    assert node.commands[-1] == (0,0,0) and node.goto_goal == goal
    measurement(node,100.7,0)
    node.last_loc_ok_time = 100
    node.tick()
    assert node.commands[-1] == (0,0,0)
    measurement(node,100.8,0)
    node.tick()
    assert node.commands[-1][0] > 0 and node.goto_goal == goal


def test_heading_must_be_correct_and_rotation_stopped(node):
    node.goto_goal = (1,0,math.pi/2,True)
    measurement(node,100,.95)
    node._phase_goto()
    assert node.commands[-1][2] > 0 and node.goto_goal is not None
    measurement(node,100.2,.95,yaw=math.pi/2,angular=.1)
    node._phase_goto()
    measurement(node,100.3,.95,yaw=math.pi/2)
    node._phase_goto()
    measurement(node,100.81,.95,yaw=math.pi/2)
    node._phase_goto()
    assert node.goto_goal is None


def test_confirm_timeout_faults_without_resuming_old_goal(node):
    measurement(node,100,.94,speed=.15)
    node._phase_goto()
    measurement(node,105.1,1.14,speed=.15)
    node._phase_goto()
    assert node.phase == 'FAULT' and node.goto_goal is None
    assert node.commands[-1] == (0,0,0)


def test_parameters_validate_whole_request_and_take_effect(node):
    invalid = [NS(name='navigation.velocity_window_s',value=.6)]
    assert not node.on_tuning_parameters(invalid).successful
    assert node.velocity_window_s == .15
    assert not node.on_tuning_parameters([NS(name='limits.max_linear',value=float('nan'))]).successful
    assert node.on_tuning_parameters([NS(name='limits.position_tolerance',value=.05),
                                      NS(name='navigation.settle_time_s',value=.8)]).successful
    assert node.pos_tol == .1  # validating callback has no side effects
    node.ros_parameters.update({'limits.position_tolerance':.05, 'navigation.settle_time_s':.8})
    node._refresh_tuning()
    assert node.pos_tol == .05 and node.settle_time_s == .8
    assert node.commands[-1] == (0,0,0)


def test_new_goal_resets_previous_confirmation(node):
    node.arrival.update(99,.06,0,0,0,ArrivalConfig())
    node.set_phase('GOTO','replacement target')
    assert node.arrival.stable_since is None


def test_sequential_parameter_validation_uses_accepted_server_values(node):
    node.ros_parameters['navigation.pose_timeout_s'] = .2
    # 上一次修改已接受但尚未 tick，不能用缓存中的旧 .5s 验证窗口。
    assert not node.on_tuning_parameters([
        NS(name='navigation.velocity_window_s', value=.3)]).successful


def test_nonzero_heading_command_rotates_once(node):
    measurement(node,100,0,yaw=math.pi/2)
    node._phase_goto()
    vx,vy,_ = node.commands[-1]
    assert abs(vx) < 1e-9 and vy == pytest.approx(-.4)
    # The chassis consumes these components directly as body velocities.
