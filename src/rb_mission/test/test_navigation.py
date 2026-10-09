import math
import pytest
from rb_mission.navigation import ArrivalConfig, ArrivalGate, PoseFeedback


def test_pass_through_is_not_arrival_and_drift_recovers():
    gate, cfg = ArrivalGate(), ArrivalConfig()
    assert gate.update(1, .08, 0, .15, 0, cfg) == 'settling'
    assert gate.update(1.1, .14, 0, .1, 0, cfg) == 'approach'
    assert gate.update(1.2, .11, 0, .02, 0, cfg) == 'approach'
    assert gate.update(1.3, .06, 0, 0, 0, cfg) == 'settling'
    assert gate.update(1.81, .06, 0, 0, 0, cfg) == 'complete'


def test_hysteresis_band_needs_correction_not_completion():
    gate, cfg = ArrivalGate(), ArrivalConfig()
    gate.update(1, .09, 0, 0, 0, cfg)
    assert gate.update(1.2, .11, 0, 0, 0, cfg) == 'correct'
    assert gate.update(2, .11, 0, 0, 0, cfg) == 'correct'


def test_stable_period_resets_on_motion_or_yaw_error():
    gate, cfg = ArrivalGate(), ArrivalConfig()
    gate.update(1, .06, 0, 0, 0, cfg)
    assert gate.update(1.4, .06, .2, 0, 0, cfg) == 'settling'
    gate.update(1.5, .06, 0, 0, .1, cfg)
    gate.update(1.6, .06, 0, 0, 0, cfg)
    assert gate.update(1.9, .06, 0, 0, 0, cfg) == 'settling'
    assert gate.update(2.11, .06, 0, 0, 0, cfg) == 'complete'


def test_republishing_old_measurement_does_not_advance_confirmation():
    gate, cfg = ArrivalGate(), ArrivalConfig()
    gate.update(1, .06, 0, 0, 0, cfg, sample_time=1)
    assert gate.update(2, .06, 0, 0, 0, cfg, sample_time=1) == 'settling'
    assert gate.update(2.1, .06, 0, 0, 0, cfg, sample_time=1.6) == 'complete'


def test_confirmation_deadline_survives_repeated_excursions():
    gate, cfg = ArrivalGate(), ArrivalConfig(settle_timeout_s=2)
    gate.update(1, .06, 0, .1, 0, cfg)
    gate.update(1.1, .14, 0, .1, 0, cfg)
    gate.update(2, .06, 0, .1, 0, cfg)
    assert gate.update(3, .14, 0, .1, 0, cfg) == 'timeout'


def test_feedback_rejects_stale_future_nan_and_repeated_samples():
    feedback = PoseFeedback()
    assert not feedback.update((0,0,0), 1, 2, .5, .15)
    assert not feedback.update((0,0,0), 3, 2, .5, .15)
    assert not feedback.update((math.nan,0,0), 2, 2, .5, .15)
    assert feedback.update((0,0,0), 2, 2, .5, .15)
    assert math.isinf(feedback.linear_speed)
    assert feedback.update((.04,0,0), 2.2, 2.2, .5, .15)
    assert feedback.linear_speed == pytest.approx(.2)
    assert not feedback.update((.04,0,0), 2.2, 2.3, .5, .15)
    assert feedback.linear_speed == pytest.approx(.2)
    assert feedback.update((.04,0,0), 3, 3, .5, .15)
    assert math.isinf(feedback.linear_speed)  # resumed data must reacquire speed


def test_feedback_wraps_yaw_at_pi():
    f = PoseFeedback()
    f.update((0,0,math.pi-.01), 1, 1, .5, .15)
    f.update((0,0,-math.pi+.01), 1.2, 1.2, .5, .15)
    assert f.angular_speed == pytest.approx(.1)


def test_feedback_reacquires_after_sim_clock_reset():
    f = PoseFeedback()
    f.update((0,0,0),100,100,.5,.15)
    f.update((0,0,0),100.2,100.2,.5,.15)
    assert f.update((0,0,0),1,1,.5,.15)
    assert math.isinf(f.linear_speed)
