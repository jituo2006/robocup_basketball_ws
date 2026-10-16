"""真实UI/API与六个入口的离线验证；绝不执行机器人路线。"""
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from pass_console import PassRuns, make_app, shoot_command_for
from match_round_route import load_config, search_plan

class Healthy:
    def snapshot(self, boot=None):
        return dict(can_run=True,reasons=[],lidar=dict(ready=True,reason='fixture'),chassis=dict(ready=True,reason='fixture'))

@pytest.mark.parametrize('side',['A','B'])
@pytest.mark.parametrize('number',[1,2,3])
def test_six_shoot_buttons_execute_fixed_wrappers(side,number):
    assert shoot_command_for(side,number)==['bash',f'tools/run_shoot_{side}{number}.sh']
    fake=PassRuns(lambda side,n:[sys.executable,'-c','print("simulated shooting")'])
    client=make_app(runs=fake,health=Healthy(),task='shoot').test_client()
    response=client.post('/api/shoot/run',json={'side':side,'round':number})
    assert response.status_code==202 and response.json['task']=='shoot'
    end=time.monotonic()+3
    while time.monotonic()<end:
        state=client.get('/api/shoot/state',query_string={'id':response.json['id']}).json
        if state['status']!='running':break
        time.sleep(.01)
    assert state['returncode']==0 and state['task']=='shoot'
    assert 'simulated shooting' in state['lines']


def test_shoot_hardware_guard_and_stale_passing_page():
    client=make_app(task='shoot').test_client()
    assert client.post('/api/shoot/run',json={'side':'B','round':3}).status_code==409
    assert client.post('/api/pass/run',json={'side':'B','round':3}).status_code==409
    assert '刷新' in client.post('/api/pass/run',json={'side':'B','round':3}).json['error']
    state=client.get('/api/shoot/state')
    assert state.json['status']=='idle' and state.json['task']=='shoot'
    assert 'hardware' in state.json and state.headers['Cache-Control']=='no-store'
    page=client.get('/').text
    assert '投篮控制' in page and '/api/shoot/run' in page and '/api/pass/run' not in page
    assert 'robocup-shoot-run' in page

@pytest.mark.parametrize('side',['A','B'])
@pytest.mark.parametrize('number',[1,2,3])
def test_actual_shell_wrappers_fix_task_side_first_ball_and_match(tmp_path,side,number):
    tools=tmp_path/'tools';tools.mkdir()
    for name in ['run_shoot_round.sh',f'run_shoot_{side}{number}.sh','match_round_route.py','round_return_budget.py','match_rounds.json','visual_target_alignment.py']:
        shutil.copy2(ROOT/'tools'/name,tools/name)
    # Replace the ROS entry with an argv recorder, then execute the real wrappers.
    (tools/'run_round_test.sh').write_text('#!/usr/bin/env bash\nexec python3 -c \'import sys,json;print(json.dumps(sys.argv[1:]))\' "$@"\n')
    result=subprocess.run(['bash',str(tools/f'run_shoot_{side}{number}.sh'),'--no-mechanism'],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    args=json.loads(result.stdout.splitlines()[-1])
    assert args==[str(number),'shoot','--start-point',side,'--round-first-ball','preloaded' if number==1 else 'search','--round-release-kind','match','--no-mechanism']
    result=subprocess.run(['bash',str(tools/f'run_shoot_{side}{number}.sh')],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    assert json.loads(result.stdout.splitlines()[-1])[-1]=='--mechanism'
    for override in ['--start-point=A','--round-first-ball=search','--round-task=pass','--round-release-kind=drop']:
        denied=subprocess.run(['bash',str(tools/f'run_shoot_{side}{number}.sh'),override],capture_output=True,text=True)
        assert denied.returncode==2

@pytest.mark.parametrize('side',['A','B'])
def test_shoot_scan_faces_ball_side_without_changing_pass(side):
    cfg=load_config(start_point=side)
    for p,yaw in search_plan(cfg,'inner',task='shoot'):
        assert math.cos(yaw)<-.99
    for p,yaw in search_plan(cfg,'inner',task='pass'):
        assert math.cos(yaw)>.99
    for p,yaw in search_plan(cfg,'arc',round_number=3,task='shoot'):
        radial=[p[i]-cfg['arc']['center'][i] for i in (0,1)]
        assert radial[0]*math.cos(yaw)+radial[1]*math.sin(yaw)>0


def test_real_shoot_ui_behaviour_and_endpoint_contract(tmp_path):
    fixture=ROOT/'src/rb_tests/test/shoot_trip_ui_check.js'
    result=subprocess.run(['node',str(fixture),str(ROOT/'tools/shoot_ui.html')],capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stdout+result.stderr
