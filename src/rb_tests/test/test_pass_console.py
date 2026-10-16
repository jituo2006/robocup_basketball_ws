"""六按键直接分发和结果透传；仅执行模拟命令，不调用 ROS/运动脚本。"""
import sys
import json
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from pass_console import PassRuns, command_for, make_app as production_app


class HealthyEquipment:
    """Only for script-dispatch tests; hardware policy has separate real-state tests."""
    def snapshot(self, boot=None):
        return dict(can_run=True, reasons=[],
                    lidar=dict(ready=True, reason='simulated ready'),
                    chassis=dict(ready=True, reason='simulated ready'))


def make_app(*args, **kwargs):
    return production_app(*args, health=HealthyEquipment(), **kwargs)


@pytest.mark.parametrize('side', ['A', 'B'])
@pytest.mark.parametrize('number,script', [(1, 'run_first_round.sh'), (2, 'run_round2.sh'), (3, 'run_round3.sh')])
def test_six_buttons_dispatch_original_scripts(side, number, script):
    command = command_for(side, number)
    assert command == ['bash', 'tools/' + script, '--start-point', side]


@pytest.mark.parametrize('code', [0, 7])
def test_api_returns_stdout_stderr_and_exact_exit_code(code):
    runs = PassRuns(lambda side, number: [sys.executable, '-c',
        f'import sys; print("stdout结果",flush=True); print("stderr结果",file=sys.stderr); sys.exit({code})'])
    client = make_app(runs).test_client()
    response = client.post('/api/pass/run', json={'side': 'B', 'round': 3})
    assert response.status_code == 202
    run_id = response.json['id']
    deadline = time.monotonic()+5
    while time.monotonic() < deadline:
        state = client.get('/api/pass/state', query_string={'id': run_id}).json
        if state['status'] != 'running':
            break
        time.sleep(.01)
    assert state['status'] == ('succeeded' if code == 0 else 'failed')
    assert state['returncode'] == code
    assert 'stdout结果' in state['lines'] and 'stderr结果' in state['lines']
    assert state['lines'][-1] == f'执行结束，退出码 {code}'
    assert client.get('/api/pass/state', query_string={'id': run_id, 'after': state['cursor']}).json['lines'] == []
    assert client.get('/api/pass/state').json['id'] == run_id


def test_page_has_only_six_action_buttons():
    from html.parser import HTMLParser
    class Buttons(HTMLParser):
        def __init__(self):
            super().__init__(); self.buttons = []
        def handle_starttag(self, tag, attrs):
            if tag == 'button': self.buttons.append(dict(attrs))
    client = make_app().test_client()
    parser = Buttons(); parser.feed(client.get('/').text)
    assert {(b['data-side'], b['data-round']) for b in parser.buttons} == {
        (side, str(number)) for side in ('A', 'B') for number in (1, 2, 3)}
    assert len(parser.buttons) == 6
    assert all('disabled' in b for b in parser.buttons)  # Initial status is unconfirmed.
    assert client.get('/api/pass/state').json['status'] == 'idle'


def test_missing_script_failure_is_returned():
    runs = PassRuns(lambda side, number: ['/nonexistent/robocup-test-command'])
    run_id = runs.start('A', 1)
    deadline = time.monotonic()+5
    while runs.snapshot(run_id)['status'] == 'running' and time.monotonic() < deadline:
        time.sleep(.01)
    assert runs.snapshot(run_id)['returncode'] == 127


def test_healthy_hardware_allows_switching_to_another_round():
    import threading
    class ControlledRuns(PassRuns):
        def _execute(self, run):
            event.wait(2)
    event = threading.Event()
    runs = ControlledRuns()
    client = make_app(runs).test_client()
    try:
        first = client.post('/api/pass/run', json={'side': 'A', 'round': 1})
        second = client.post('/api/pass/run', json={'side': 'B', 'round': 2})
        assert first.status_code == second.status_code == 202
        assert first.json['id'] != second.json['id']
    finally:
        event.set()


def test_backend_mapping_hot_reload_keeps_frontend_and_api_unchanged(tmp_path):
    config = tmp_path / 'actions.json'
    runs = PassRuns(lambda side, number: command_for(side, number, config))
    client = make_app(runs).test_client()
    original_page = client.get('/').data
    for name, code in [('old_script', 0), ('new_script', 9)]:
        # 同一个服务、同一个按钮请求，改后端脚本文件名和参数后立刻生效。
        script = tmp_path / f'{name}.py'
        script.write_text(f'import sys; print("{name}:", sys.argv[1]); sys.exit({code})')
        config.write_text(json.dumps({'A1': [sys.executable, str(script), name+'_argument']}))
        result = client.post('/api/pass/run', json={'side': 'A', 'round': 1})
        assert result.status_code == 202
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            state = client.get('/api/pass/state', query_string={'id': result.json['id']}).json
            if state['status'] != 'running':
                break
            time.sleep(.01)
        assert state['returncode'] == code
        assert f'{name}: {name}_argument' in state['lines']
        assert client.get('/').data == original_page


def test_script_content_updates_without_mapping_or_frontend_change(tmp_path):
    script = tmp_path / 'route.py'
    config = tmp_path / 'actions.json'
    config.write_text(json.dumps({'B2': [sys.executable, str(script)]}))
    runs = PassRuns(lambda side, number: command_for(side, number, config))
    for version in ('first', 'updated'):
        script.write_text(f'print("{version}")')
        run_id = runs.start('B', 2)
        deadline = time.monotonic()+5
        while runs.snapshot(run_id)['status'] == 'running' and time.monotonic() < deadline:
            time.sleep(.01)
        state = runs.snapshot(run_id)
        assert state['returncode'] == 0 and version in state['lines']


def wait_for_run(runs, run_id, status, timeout=8.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        value=runs.snapshot(run_id)
        if value['status']==status:return value
        time.sleep(.01)
    pytest.fail(str(runs.snapshot(run_id)))


def test_new_click_ends_old_script_before_new_one_starts(tmp_path):
    first=tmp_path/'first.py';second=tmp_path/'second.py';begun=tmp_path/'begun'
    first.write_text('import time\nfrom pathlib import Path\n'
                     f'Path({str(begun)!r}).write_text("started")\n'
                     'try: time.sleep(60)\nexcept KeyboardInterrupt: print("old exited",flush=True)\n')
    second.write_text('print("new B route",flush=True)')
    runs=PassRuns(lambda side,n:[sys.executable,str(first if side=='A' else second)])
    old=runs.start('A',1)
    try:
        deadline=time.monotonic()+3
        while not begun.exists() and time.monotonic()<deadline:time.sleep(.01)
        assert begun.exists()
        process=runs.runs[old]['process']
        new=runs.start('B',2)
        state=wait_for_run(runs,new,'succeeded')
        assert process.poll() is not None
        assert wait_for_run(runs,old,'superseded')['side']=='A'
        assert 'new B route' in state['lines']
        assert state['command']==[sys.executable,str(second)]
    finally:runs._stop_process(runs.runs[old]['process'])


def test_queued_stale_click_never_starts_its_script(tmp_path):
    flag=tmp_path/'old_started'
    runs=PassRuns(lambda side,n:[sys.executable,'-c',
        f'from pathlib import Path; Path({str(flag)!r}).touch()' if side=='A' else 'print("latest")'])
    with runs.execution_lock:
        old=runs.start('A',1);new=runs.start('B',3)
    wait_for_run(runs,new,'succeeded')
    wait_for_run(runs,old,'superseded')
    assert not flag.exists()


def test_route_pid_discovery_excludes_other_checkout_and_boot(tmp_path):
    from pass_console import workspace_route_pids, WS
    for pid,args,cwd in ((101,'python3\0tools/test_perimeter_ball.py\0',WS),
                         (102,'bash\0tools/boot.sh\0',WS),
                         (103,'python3\0tools/test_perimeter_ball.py\0',tmp_path)):
        proc=tmp_path/str(pid);proc.mkdir()
        (proc/'cmdline').write_bytes(args.encode());(proc/'cwd').symlink_to(cwd)
    assert workspace_route_pids(tmp_path)==[101]
