"""User policy: elapsed time alone never cancels task/control state."""
from types import SimpleNamespace as NS
import pytest
from test_seek_observations import load, runner

def test_last_valid_pose_is_not_expired_and_never_requests_stop():
    mod=load(); run, clock=runner(mod, now=100000.)
    run.pose_stamp=1.
    run.stop=lambda: pytest.fail("time alone must not stop")
    assert run.require_fresh_pose() is False
    assert run.pose==(0.,0.,0.)

def test_old_but_valid_detection_timestamp_does_not_expire():
    mod=load(); run, clock=runner(mod, now=100000.)
    assert run.detection_fresh(1.)
    assert not run.detection_fresh(float("nan"))

def test_search_repeats_until_found_instead_of_ending_on_a_deadline():
    mod=load(); run, clock=runner(mod)
    found=NS(label="ball_basketball")
    sequence=iter([None,None,found])
    calls=[]
    run._find_ball_pass=lambda timeout: calls.append(timeout) or next(sequence)
    assert run.find_ball(.01) is found
    assert calls==[.01,.01,.01]

def test_explicit_pose_fault_still_stops():
    mod=load(); run, clock=runner(mod)
    run.motion_fault="invalid pose"
    stops=[];run.stop=lambda:stops.append(True)
    with pytest.raises(RuntimeError): run.require_fresh_pose()
    assert stops==[True]
