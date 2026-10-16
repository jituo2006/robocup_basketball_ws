"""Offline API, concurrency, finite-distance and shared-startup tests."""
import json
import subprocess
import sys
import threading
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from pass_console import PassRuns, make_app
from pass_health import PassHealth
from console_trip import ConsoleTasks

CONFIG=dict(distance_m=2.,speed_mps=.25,acceleration_mps2=.3,pause_s=1.,right_scale=1.,left_scale=1.)


def setup(lidar=False):
    clock=[100.]
    health=PassHealth(clock=lambda:clock[0],can_present=lambda:True)
    health.observe_chassis(json.dumps(dict(ready=True,dry_run=False)))
    health.observe_ready(True)
    if lidar:
        health.observe_session(json.dumps(dict(ready=True)))
        health.observe_localization(True)
    class FakeRuns(PassRuns):
        def _execute(self,run):
            pass
    runs=FakeRuns(lambda side,n:['fixture-only'])
    driver=['generation-1',None]
    tasks=ConsoleTasks(runs,health,CONFIG,driver=lambda:tuple(driver),clock=lambda:clock[0])
    published=[]
    tasks.attach(published.append)
    app=make_app(runs=runs,health=health,task='shoot',tasks=tasks)
    return app,tasks,health,clock,driver,published


def test_chassis_ready_without_lidar_enables_only_trip():
    app,tasks,health,clock,driver,published=setup()
    client=app.test_client()
    assert client.get('/api/shoot/state').json['hardware']['can_run'] is False
    assert client.get('/api/shoot/chassis-trip/state').json['can_run'] is True
    assert client.post('/api/shoot/run',json=dict(side='A',round=1)).status_code==409
    assert client.post('/api/shoot/chassis-trip/run',json={}).status_code==202
    assert published==[]


@pytest.mark.parametrize('fault',['board','dry_run','stale','driver','bridge','publisher'])
def test_real_faults_block_trip_with_reason(fault):
    app,tasks,health,clock,driver,published=setup()
    if fault=='board':health.observe_ready(False)
    if fault=='dry_run':health.observe_chassis(json.dumps(dict(ready=True,dry_run=True)))
    if fault=='stale':clock[0]+=3
    if fault=='driver':driver[:]=[None,'driver exited']
    if fault=='bridge':tasks.publish=None
    if fault=='publisher':tasks.other_publishers=True
    response=app.test_client().post('/api/shoot/chassis-trip/run',json={})
    assert response.status_code==409 and response.json['error']
    assert published==[] and tasks.motion.snapshot()['status']=='idle'


def test_both_directions_reject_other_tasks_until_end():
    app,tasks,health,clock,driver,published=setup(lidar=True)
    client=app.test_client()
    assert client.post('/api/shoot/run',json=dict(side='B',round=2)).status_code==202
    assert client.post('/api/shoot/run',json=dict(side='A',round=1)).status_code==409
    assert client.post('/api/shoot/chassis-trip/run',json={}).status_code==409
    assert client.post('/api/shoot/chassis-trip/stop',json={}).status_code==200
    assert tasks.activity()=='shoot' and published==[]
    with tasks.runs.lock:
        tasks.runs.runs[tasks.runs.latest]['status']='succeeded'
    assert client.post('/api/shoot/chassis-trip/run',json={}).status_code==202
    assert client.post('/api/shoot/run',json=dict(side='A',round=1)).status_code==409
    assert client.post('/api/shoot/chassis-trip/run',json={}).status_code==409
    tasks.tick()
    client.post('/api/shoot/chassis-trip/stop',json={})
    assert published[-1]==0 and tasks.activity()=='idle'
    tasks.tick()
    assert published==[0.,0.]


def test_multiclient_race_reserves_exactly_one_task():
    app,tasks,health,clock,driver,published=setup(lidar=True)
    barrier=threading.Barrier(20)
    results=[]
    def click(i):
        with app.test_client() as client:
            barrier.wait()
            response=client.post('/api/shoot/run' if i%2 else '/api/shoot/chassis-trip/run',
                                 json=dict(side='A',round=1) if i%2 else {})
            results.append(response.status_code)
    threads=[threading.Thread(target=click,args=(i,)) for i in range(20)]
    for thread in threads:thread.start()
    for thread in threads:thread.join(timeout=5)
    assert results.count(202)==1 and results.count(409)==19


def test_curve_completion_direction_distance_and_repeat():
    app,tasks,health,clock,driver,published=setup()
    tasks.start_trip()
    dt=.001
    right=left=0.
    for i in range(21000):
        clock[0]=100+i*dt
        before=len(published);tasks.tick()
        if len(published)>before:
            right+=min(published[-1],0)*dt;left+=max(published[-1],0)*dt
    assert right==pytest.approx(-2.,abs=.001)
    assert left==pytest.approx(2.,abs=.001)
    assert tasks.motion.snapshot()['status']=='succeeded' and published[-1]==0
    count=len(published);tasks.tick();assert len(published)==count
    health.observe_ready(True);health.observe_chassis(json.dumps(dict(ready=True,dry_run=False)))
    tasks.start_trip();assert tasks.motion.snapshot()['run_id']==2


@pytest.mark.parametrize('fault',['board_reset','driver_restart','driver_exit','publisher'])
def test_explicit_fault_ends_motion_and_never_resumes(fault):
    app,tasks,health,clock,driver,published=setup()
    tasks.start_trip();clock[0]+=.8;tasks.tick();assert published[-1]<0
    if fault=='board_reset':tasks.board_ready(False)
    if fault=='driver_restart':driver[0]='generation-2'
    if fault=='driver_exit':driver[:]=[None,'driver exited']
    tasks.tick(other_publishers=fault=='publisher')
    assert tasks.motion.snapshot()['status']=='failed' and published[-1]==0
    tasks.board_ready(True);driver[:]=['generation-3',None]
    count=len(published);tasks.tick();assert len(published)==count


def test_status_deadline_only_blocks_new_task_not_active_motion():
    app,tasks,health,clock,driver,published=setup()
    tasks.start_trip();clock[0]+=3;tasks.tick()
    assert tasks.motion.snapshot()['status']=='running' and published[-1]<0
    assert not tasks.snapshot()['chassis']['ready']


def test_idle_and_shooting_never_get_zero_from_trip_timer():
    app,tasks,health,clock,driver,published=setup(lidar=True)
    tasks.tick();tasks.start_shoot('A',1);tasks.tick(other_publishers=True)
    tasks.stop_trip();tasks.detach()
    assert published==[]


def test_retired_handover_is_gone_and_bad_json_cannot_start():
    app,tasks,health,clock,driver,published=setup()
    client=app.test_client()
    assert client.post('/api/console/prepare-fallback',json={}).status_code==410
    assert client.post('/api/shoot/chassis-trip/run',data='').status_code==400


def test_lidar_restarts_skip_chassis_cleanup_and_can_reconfiguration(tmp_path):
    source=(ROOT/'tools/boot.sh').read_text()
    body=source[source.index('console_start_chassis=true'):source.index('# ── ② 起整套')]
    calls=tmp_path/'calls'
    harness=tmp_path/'harness.sh'
    harness.write_text('''WS=/does-not-exist
ps(){ echo 0; }
systemctl(){ return 1; }
pgrep(){ [[ "$*" != *rb_chassis_node* ]] || echo 999999; }
kill(){ echo KILL >> "$CALLS"; }
ip(){ echo IP >> "$CALLS"; }
sleep(){ :; }
'''+body)
    result=subprocess.run(['bash',str(harness),'start_chassis:=false'],env={**__import__('os').environ,'CALLS':str(calls)},capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert not calls.exists()
    launch=(ROOT/'src/rb_bringup/launch/bringup_lidar_vision.launch.py').read_text()
    assert 'condition=IfCondition(LaunchConfiguration("start_chassis"))' in launch
    config=json.loads((ROOT/'tools/pass_boot.json').read_text())
    assert 'start_chassis:=false' in config['boot_argv']

def test_update_handover_freezes_admission_and_rejects_active_task():
    app,tasks,health,clock,driver,published=setup(lidar=True)
    client=app.test_client()
    assert client.post('/api/console/prepare-update',json={}).status_code==200
    assert client.post('/api/shoot/run',json=dict(side='A',round=1)).status_code==409
    assert client.post('/api/shoot/chassis-trip/run',json={}).status_code==409
    app,tasks,health,clock,driver,published=setup(lidar=True)
    client=app.test_client();tasks.start_trip()
    assert client.post('/api/console/prepare-update',json={}).status_code==409
    assert tasks.motion.snapshot()['status']=='running'
