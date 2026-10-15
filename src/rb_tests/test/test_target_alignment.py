"""定位柱旋转方向、真实新帧计数、歧义目标及两个取球后的瞄准时机。"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS / 'tools'))
from match_round_route import RoundRoute, load_config
from visual_target_alignment import align_pillar, pillar_candidate, wrap
from test_match_round_route import fake_runner


def test_pillar_on_camera_right_commands_clockwise_rotation_only():
    r, events = fake_runner()
    r.pose = (1., 2., 0.)
    r.dets[0].bearing_rad = .08
    commands = []
    def publish(tw):
        commands.append((tw.linear.x, tw.linear.y, tw.angular.z))
        r.dets[0].bearing_rad = 0.
    r.publish_command = publish
    assert align_pillar(r, load_config()['pillar_alignment'], 0., emit=lambda msg: None)
    assert commands and commands[0][:2] == (0., 0.)
    assert -.15 <= commands[0][2] < 0


def test_cached_frame_does_not_count_as_three_stable_observations(monkeypatch):
    r, events = fake_runner(); r.pose = (1., 2., 0.)
    iterations=[0]
    def spin(seconds):
        iterations[0]+=1
        if iterations[0]>1000:
            raise KeyboardInterrupt
    r.spin=spin
    with pytest.raises(KeyboardInterrupt):
        align_pillar(r, load_config()['pillar_alignment'], 0., emit=lambda msg: None)
    assert iterations[0]>1000


def test_wrong_class_stale_or_multiple_pillars_are_never_chosen():
    r, events = fake_runner(); r.pose = (1., 2., 0.)
    cfg = load_config()['pillar_alignment']
    assert pillar_candidate(r, cfg, 0.) is r.dets[0]
    r.detection_fresh = lambda stamp: False
    assert pillar_candidate(r, cfg, 0.) is None
    r.detection_fresh = lambda stamp: True
    r.dets = [r.dets[0], r.dets[0]]
    assert pillar_candidate(r, cfg, 0.) is None
    r.dets = [NS(label='ball_basketball')]
    assert pillar_candidate(r, cfg, 0.) is None


def test_out_of_aim_sector_pillar_is_not_used():
    r, events = fake_runner(); r.pose = (1., 2., 0.)
    r.dets[0].bearing_rad = .8
    assert pillar_candidate(r, load_config()['pillar_alignment'], 0.) is None


def test_formal_release_does_not_send_commands_if_target_alignment_failed(monkeypatch):
    cfg = load_config(); cfg['release']['shoot']['commands'] = ['TEST_RELEASE']
    r, events = fake_runner()
    monkeypatch.setattr('match_round_route.align_pillar', lambda *args, **kw: False)
    plan = RoundRoute(r, cfg, 2, 'shoot', 'cycle', emit=lambda msg: None)
    assert not plan.action_point()
    assert not plan.release()
    assert not [e for e in events if e[0] == 'text']


@pytest.mark.parametrize('number', [1, 2, 3])
def test_whole_round_aims_each_ball_before_release_and_returns_home(number, monkeypatch):
    cfg = load_config(); r, events = fake_runner(); start = r.pose
    monkeypatch.setattr('match_round_route.align_pillar', lambda *args, **kw: events.append(('pillar',)) or True)
    plan = RoundRoute(r, cfg, number, 'shoot', 'cycle', release_kind='drop', emit=lambda msg: None)
    assert plan.execute()
    assert r.pose == pytest.approx(start)
    meaningful = [e[0] for e in events if e[0] in ('pillar', 'text')]
    assert meaningful == ['pillar', 'text', 'pillar', 'text']
