"""自动传球固定时序；模拟串口，不运行实车。"""
import pytest
from test_match_round_route import fake_runner
from match_round_route import RoundRoute, load_config


@pytest.mark.parametrize('task', ['pass', 'shoot'])
def test_release_stops_chassis_then_sends_task_sequence_and_resumes(task):
    cfg = load_config()
    r, _ = fake_runner();r.args.route_intake=True
    clock=[0.];events=[]
    r.spin=lambda delay:clock.__setitem__(0,clock[0]+delay)
    r.stop=lambda:events.append((clock[0],'chassis_stop'))
    r.intake_text=lambda command:events.append((clock[0],command)) or (True,'sent')
    plan=RoundRoute(r,cfg,2,task,'collect',emit=lambda message:None)
    plan.target_aligned=True;plan.heading_ready=True
    if task=='pass':
        commands=['vesc_stop','vesc_home','vesc_shoot 3','vesc_stop','vesc_home','vesc_reset','vesc_stop']
        times=[0.,.3,.6,5.6,5.9,6.2,11.2];duration=11.2
    else:
        commands=['vesc_stop','vesc_home','vesc_shoot 3','vesc_stop','vesc_reset']
        times=[0.,0.,0.,3.,3.];duration=3.
    for cycle in range(2):
        events.clear()
        assert plan.release()
        events.append((clock[0],'next_route_motion'))
        assert [command for _,command in events]==['chassis_stop']+commands+['next_route_motion']
        assert [time for time,_ in events]==pytest.approx([cycle*duration]+[cycle*duration+v for v in times]+[(cycle+1)*duration])
        assert 'vesc_status' not in [command for _,command in events]


@pytest.mark.parametrize('failed_index',range(7))
def test_pass_write_failure_does_not_send_later_commands_or_resume(failed_index):
    cfg=load_config();r,_=fake_runner();r.args.route_intake=True
    sent=[]
    def send(command):
        sent.append(command)
        return len(sent)-1!=failed_index,'write result'
    r.intake_text=send
    plan=RoundRoute(r,cfg,2,'pass','collect',emit=lambda message:None)
    plan.heading_ready=True
    assert not plan.release()
    assert sent==cfg['release']['pass']['commands'][:failed_index+1]
    assert plan.last_error


def test_disabled_mechanism_skips_pass_serial_sequence():
    cfg=load_config();r,events=fake_runner();r.args.route_intake=False
    plan=RoundRoute(r,cfg,2,'pass','collect',emit=lambda message:None)
    assert plan.release()
    assert not any(event[0]=='text' for event in events)


def test_drop_does_not_execute_shoot_or_reset_sequence():
    cfg=load_config();r,events=fake_runner();r.args.route_intake=True
    plan=RoundRoute(r,cfg,2,'shoot','collect',release_kind='drop',emit=lambda message:None)
    assert plan.release()
    assert [event[1] for event in events if event[0]=='text']==cfg['drop_commands']
