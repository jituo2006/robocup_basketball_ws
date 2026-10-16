"""Opt-in real ROS transport test, isolated from all real robot topics and CAN."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
pytestmark=pytest.mark.skipif(os.environ.get('CONSOLE_TRIP_ROS_TEST')!='1',reason='isolated ROS fixture only')


def test_unified_monitor_completion_lidar_changes_and_stop(tmp_path):
    assert os.environ['ROS_DOMAIN_ID']=='177' and os.environ['ROS_LOCALHOST_ONLY']=='1'
    from console_trip import ConsoleTasks
    from pass_console import PassRuns,make_app
    from pass_health import PassHealth,RosHealthMonitor
    output=tmp_path/'twists.jsonl'
    lidar_on=tmp_path/'lidar-on'
    fake=tmp_path/'sim_board.py'
    fake.write_text('''import json,pathlib,sys,time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
rclpy.init()
node=rclpy.create_node('trip_simulated_board_only')
ready=node.create_publisher(Bool,'/chassis/ready',10)
status=node.create_publisher(String,'/chassis/status',10)
lidar=node.create_publisher(String,'/lidar/session',10)
location=node.create_publisher(Bool,'/localization/ok',10)
log=open(sys.argv[1],'w',buffering=1)
flag=pathlib.Path(sys.argv[2])
def command(m):log.write(json.dumps([time.monotonic(),m.linear.x,m.linear.y,m.angular.z])+'\\n')
def report():
 ready.publish(Bool(data=True))
 status.publish(String(data=json.dumps(dict(ready=True,dry_run=False))))
 lidar.publish(String(data=json.dumps(dict(ready=flag.exists(),reason='SIMULATED LIDAR'))))
 location.publish(Bool(data=flag.exists()))
sub=node.create_subscription(Twist,'/cmd_vel',command,10)
timer=node.create_timer(.05,report)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 log.close();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
''')
    board=subprocess.Popen([sys.executable,str(fake),str(output),str(lidar_on)],start_new_session=True)
    health=PassHealth(can_present=lambda:True)
    runs=PassRuns(lambda side,n:[sys.executable,'-c','print("SIMULATED ROUTE ONLY")'])
    driver=['simulated-generation',None]
    tasks=ConsoleTasks(runs,health,dict(distance_m=2.,speed_mps=.25,acceleration_mps2=.3,pause_s=1.,right_scale=1.,left_scale=1.),driver=lambda:tuple(driver))
    monitor=RosHealthMonitor(health,tasks=tasks)
    client=make_app(runs=runs,health=health,tasks=tasks,task='shoot').test_client()
    monitor.start()
    def wait(condition,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if condition():return
            time.sleep(.02)
        pytest.fail(repr(tasks.snapshot()))
    try:
        wait(lambda:tasks.snapshot()['can_run'],15)
        assert not health.snapshot()['can_run']
        assert client.post('/api/shoot/run',json=dict(side='A',round=1)).status_code==409
        assert client.post('/api/shoot/chassis-trip/run',json={}).status_code==202
        wait(lambda:tasks.motion.snapshot()['phase']=='右移',3)
        lidar_on.touch()
        wait(lambda:health.snapshot()['lidar']['ready'],3)
        assert client.post('/api/shoot/run',json=dict(side='B',round=2)).status_code==409
        lidar_on.unlink()
        wait(lambda:not health.snapshot()['lidar']['ready'],3)
        assert tasks.motion.snapshot()['status']=='running'
        wait(lambda:tasks.motion.snapshot()['status']=='succeeded',25)
        assert board.poll() is None
        samples=[json.loads(line) for line in output.read_text().splitlines()]
        right=left=0.
        for a,b in zip(samples,samples[1:]):
            assert a[1]==a[3]==0
            right+=min(a[2],0)*(b[0]-a[0]);left+=max(a[2],0)*(b[0]-a[0])
        assert right==pytest.approx(-2.,abs=.06) and left==pytest.approx(2.,abs=.06)
        assert samples[-1][2]==0
        assert client.post('/api/shoot/chassis-trip/run',json={}).status_code==202
        wait(lambda:tasks.motion.snapshot()['phase']=='右移',3)
        assert client.post('/api/shoot/chassis-trip/stop',json={}).status_code==200
        wait(lambda:json.loads(output.read_text().splitlines()[-1])[2]==0,2)
        assert tasks.motion.snapshot()['status']=='stopped'
        assert client.post('/api/shoot/chassis-trip/run',json={}).status_code==202
        wait(lambda:tasks.motion.snapshot()['phase']=='右移',3)
        driver[0]='simulated-new-driver-generation'
        wait(lambda:tasks.motion.snapshot()['status']=='failed',2)
        time.sleep(.1)
        assert json.loads(output.read_text().splitlines()[-1])[2]==0
        time.sleep(.1)
        assert tasks.motion.snapshot()['status']=='failed'
    finally:
        monitor.close()
        os.killpg(board.pid,signal.SIGINT)
        board.wait(timeout=5)
    assert not monitor.thread.is_alive()