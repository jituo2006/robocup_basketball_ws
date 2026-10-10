"""Exercise production state transitions with controlled service responses."""
from types import SimpleNamespace as NS
import pytest
from test_navigation_node import load_node_class


class Future:
    def __init__(self, done=False, success=True, error=None):
        self.ready, self.success, self.error = done, success, error

    def done(self):
        return self.ready

    def result(self):
        if self.error:
            raise self.error
        return NS(success=self.success)


@pytest.fixture
def mission():
    cls = load_node_class()
    n = cls.__new__(cls)
    n.now = 100.0
    n._now_seconds = lambda: n.now
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(n.now * 1e9)))
    n.get_logger = lambda: NS(info=lambda *a: None, warn=lambda *a: None)
    n.arrival = NS(reset=lambda: None)
    n.commands = []
    n.publish_zero = lambda: n.commands.append((0, 0, 0))
    n.detail = ""
    n.phase, n.mission = "LAUNCH", "SHOOT"
    n.require_zone = False
    n._localization_ready = lambda: True
    n.estop = False
    n.has_ball, n.ball_count = True, 1
    n.actions_done, n.actions_per_mission = 0, 2
    n.launch_called = False
    n.launch_future = n.launch_stop_future = n.launch_ack_time = None
    n.launch_hold_s = 1.5
    n.request, n.stop = Future(), Future()
    n._call_launcher = lambda: n.request
    n._stop_launcher = lambda: n.stop
    return n


def test_pending_launch_does_not_count_or_start_hold(mission):
    mission._phase_launch()
    mission.now += 20
    mission._phase_launch()
    assert mission.actions_done == 0 and mission.has_ball
    assert mission.launch_ack_time is None
    assert mission.commands[-1] == (0, 0, 0)


@pytest.mark.parametrize("mode", ["unavailable", "rejected", "exception"])
def test_launch_failure_never_counts(mission, mode):
    if mode == "unavailable":
        mission._call_launcher = lambda: None
    elif mode == "rejected":
        mission.request = Future(done=True, success=False)
    else:
        mission.request = Future(done=True, error=RuntimeError("lost service"))
    mission._phase_launch()
    if mission.phase == "LAUNCH":
        mission._phase_launch()
    assert mission.phase == "FAULT"
    assert mission.actions_done == 0 and mission.has_ball


def test_action_waits_for_ack_hold_and_stop_ack(mission):
    mission._phase_launch()
    mission.now += 10
    mission.request.ready = True
    mission._phase_launch()
    assert mission.launch_ack_time == 110
    mission.now = 111
    mission._phase_launch()
    assert mission.launch_stop_future is None
    mission.now = 111.6
    mission._phase_launch()
    assert mission.actions_done == 0 and mission.launch_stop_future is mission.stop
    mission._phase_launch()
    assert mission.actions_done == 0
    mission.stop.ready = True
    mission._phase_launch()
    assert mission.actions_done == 1 and not mission.has_ball
    assert mission.phase == "SEEK_BALL"
    assert mission._score_estimate() == -1


def test_stop_rejection_does_not_count(mission):
    mission._phase_launch()
    mission.request.ready = True
    mission._phase_launch()
    mission.now += 2
    mission._phase_launch()
    mission.stop.ready, mission.stop.success = True, False
    mission._phase_launch()
    assert mission.phase == "FAULT" and mission.actions_done == 0


def test_launch_timeout_faults_instead_of_retrying_shot(mission):
    mission.launch_called = True
    mission.retry_or_fault()
    assert mission.phase == "FAULT" and mission.actions_done == 0


def test_action_limit_prevents_another_service_request(mission):
    mission.actions_done = 2
    mission._call_launcher = lambda: pytest.fail("must not issue a third action")
    mission._phase_launch()
    assert mission.phase == "RETURN_HOME" and mission.actions_done == 2


def test_position_invalidated_after_request_faults_without_navigating(mission):
    mission._phase_launch()
    mission._zone_ok = lambda: False
    mission._phase_launch()
    assert mission.phase == "FAULT" and mission.actions_done == 0


@pytest.mark.parametrize("phase,estop", [("ESTOP", False), ("IDLE", True),
                                       ("GOTO", False), ("SEEK_BALL", False)])
def test_task_start_rejected_in_estop_or_active_control(mission, phase, estop):
    mission.phase, mission.estop = phase, estop
    response = mission.on_set_mission(NS(mission="PASS"), NS())
    assert not response.accepted and mission.phase == phase


def test_start_delay_is_stationary_and_does_not_reuse_other_mission_ball(mission):
    mission.phase = "IDLE"
    mission.start_delay_s = 8
    response = mission.on_set_mission(NS(mission="PASS"), NS())
    assert response.accepted and not mission.has_ball and mission.ball_count == 0
    mission.elapsed = lambda: 7.99
    mission._phase_start_delay()
    assert mission.phase == "START_DELAY" and mission.commands[-1] == (0, 0, 0)
    # 当前任务建立后重新输入的目标球信号可在延迟结束后使用。
    mission.on_has_ball(NS(data=True))
    mission.elapsed = lambda: 8
    mission._phase_start_delay()
    assert mission.phase == "NAV_TO_ZONE"


def test_task_cannot_arm_on_stale_localization(mission):
    mission.phase = "IDLE"
    mission._localization_ready = lambda: False
    response = mission.on_set_mission(NS(mission="PASS"), NS())
    assert not response.accepted and mission.phase == "IDLE"


def test_localization_loss_during_launch_stops_instead_of_waiting(mission):
    mission.launch_called = True
    mission.max_ball_count = 2
    mission._refresh_tuning = lambda: None
    mission._localization_ready = lambda: False
    mission.publish_outputs = lambda: None
    mission.tick()
    assert mission.phase == "FAULT" and mission.actions_done == 0


def test_delay_with_lost_localization_cannot_resume_late(mission):
    mission.phase = "START_DELAY"
    mission.start_delay_s = 8
    mission.elapsed = lambda: 8
    mission.max_ball_count = 2
    mission._refresh_tuning = lambda: None
    mission._localization_ready = lambda: False
    mission.publish_outputs = lambda: None
    mission.tick()
    assert mission.phase == "FAULT" and mission.commands[-1] == (0, 0, 0)
