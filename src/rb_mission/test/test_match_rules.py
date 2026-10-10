"""Explicit rule constraints and observed-score examples from revised PDF p10."""
from pathlib import Path
import math
import pytest
import yaml
from rb_mission.match_rules import (
    ObservedAction, competition_gaps, score_round, validate_match_config,
)


@pytest.fixture
def cfg():
    return yaml.safe_load((Path(__file__).parents[1] / "config/mission.yaml").read_text())


@pytest.mark.parametrize("delay", [5, 8, 15])
def test_valid_start_delay(cfg, delay):
    cfg["timeouts"]["start_delay_s"] = delay
    validate_match_config(cfg)


@pytest.mark.parametrize("delay", [0, 4.99, 15.01, math.inf, math.nan, True, "8"])
def test_invalid_start_delay_is_rejected(cfg, delay):
    cfg["timeouts"]["start_delay_s"] = delay
    with pytest.raises(ValueError):
        validate_match_config(cfg)


@pytest.mark.parametrize("key,value", [("actions_per_mission", 3), ("max_ball_count", 3),
                                       ("max_ball_count", 2.5), ("actions_per_mission", True)])
def test_cannot_raise_rule_caps(cfg, key, value):
    cfg["rules"][key] = value
    with pytest.raises(ValueError):
        validate_match_config(cfg)


def test_swapped_roles_are_rejected(cfg):
    cfg["mission_labels"]["SHOOT"]["ball"] = "ball_basketball"
    with pytest.raises(ValueError):
        validate_match_config(cfg)


def test_pass_top_ring_full_score_and_time_cap():
    actions = [ObservedAction(released=True, in_pass_zone=True, pass_target=3)] * 2
    assert score_round("PASS", actions, entered=True, returned=True, remaining_s=100) == 140


def test_shoot_outside_three_point_full_score():
    actions = [ObservedAction(released=True, outside_shoot_line=True,
                              shot_result="basket", outside_three_point=True)] * 2
    assert score_round("SHOOT", actions, entered=True, returned=True, remaining_s=20) == 110


def test_release_location_contact_and_interference_penalties():
    actions = [ObservedAction(released=True, shot_result="contact"),
               ObservedAction(released=True, outside_shoot_line=True)]
    assert score_round("SHOOT", actions, interference_contacts=2) == 16


def test_no_return_time_points_without_both_observed_releases():
    assert score_round("PASS", [], returned=True, remaining_s=100) == 0
    assert score_round("PASS", [ObservedAction(released=True)],
                       returned=True, remaining_s=100) == 5


def test_time_points_are_whole_intervals_and_require_return():
    actions = [ObservedAction(released=True)] * 2
    assert score_round("PASS", actions, returned=True, remaining_s=9.99) == 16
    assert score_round("PASS", actions, returned=False, remaining_s=100) == 10


def test_unreleased_ball_does_not_score_as_hit():
    assert score_round("SHOOT", [ObservedAction(shot_result="basket")]) == 0


def test_rule_check_does_not_certify_hardware_by_boolean_flags(cfg):
    cfg["avoidance"]["enabled"] = True
    cfg["hardware_confirmed"] = True
    assert any("承重轮" in gap for gap in competition_gaps(cfg))
    assert any("碰撞" in gap for gap in competition_gaps(cfg))


def test_three_round_profiles_and_distinct_third_round_ball_counts():
    path = Path(__file__).parents[1] / "config/rules_2026_revised.yaml"
    rules = yaml.safe_load(path.read_text())
    assert set(rules["rounds"]) == {1, 2, 3}
    assert rules["rounds"][3]["PASS"]["three_point"] == [2, 2]
    assert rules["rounds"][3]["SHOOT"]["three_point"] == [4, 2]
    assert rules["rounds"][1]["preload_interpretation"] == "needs_referee_confirmation"
