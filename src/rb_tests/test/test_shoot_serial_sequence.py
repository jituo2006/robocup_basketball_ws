"""出球发送与发射后至少三秒复位时序；全部使用模拟串口。"""
import pytest
from test_match_round_route import fake_runner
from match_round_route import RoundRoute, load_config


@pytest.mark.parametrize('task', ['pass', 'shoot'])
def test_release_dispatches_checked_sequence_with_three_second_shot_minimum(task):
    cfg = load_config()
    r, _ = fake_runner()
    r.args.route_intake = True
    clock = [0.]
    sent = []
    r.spin = lambda delay: clock.__setitem__(0, clock[0]+delay)
    r.intake_text = lambda command: sent.append((clock[0], command)) or (True, 'sent')
    plan = RoundRoute(r, cfg, 2, task, 'collect', emit=lambda message: None)
    plan.target_aligned = True
    plan.heading_ready = True
    for _ in range(2):
        assert plan.release()
    expected = [(0., 'vesc_stop'), (0., 'vesc_home'), (0., 'vesc_shoot 3'),
                (3., 'vesc_stop'), (3., 'vesc_reset')]
    assert sent[:5] == expected
    assert sent[5:] == [(time+3., command) for time, command in expected]


def test_drop_does_not_execute_shoot_or_reset_sequence():
    cfg = load_config()
    r, events = fake_runner()
    r.args.route_intake = True
    plan = RoundRoute(r, cfg, 2, 'shoot', 'collect', release_kind='drop', emit=lambda message: None)
    assert plan.release()
    assert [event[1] for event in events if event[0] == 'text'] == cfg['drop_commands']
