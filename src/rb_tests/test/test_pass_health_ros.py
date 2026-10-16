"""Real ROS subscriptions in a separate localhost-only domain; no motion publishers."""
import json
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0,str(Path(__file__).parents[3]/'tools'))
from pass_health import PassHealth, RosHealthMonitor
from pass_console import PassRuns, make_app


def test_real_ros_status_bridge_and_backend_gate(monkeypatch):
    rclpy=pytest.importorskip('rclpy')
    from rclpy.context import Context
    from std_msgs.msg import Bool, String
    monkeypatch.setenv('ROS_DOMAIN_ID','177')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    context=Context(); rclpy.init(context=context)
    node=rclpy.create_node('pass_status_fixture',context=context)
    publishers={name:node.create_publisher(typ,topic,10) for name,typ,topic in (
        ('session',String,'/lidar/session'),('chassis',String,'/chassis/status'),
        ('ready',Bool,'/chassis/ready'),('localization',Bool,'/localization/ok'))}
    health=PassHealth(can_present=lambda:True)
    monitor=RosHealthMonitor(health)
    monitor.start()
    def publish(chassis_ready=True):
        publishers['session'].publish(String(data=json.dumps(dict(ready=True))))
        publishers['chassis'].publish(String(data=json.dumps(dict(ready=chassis_ready,dry_run=False,
                                                    reason='fixture ready' if chassis_ready else 'fixture fault'))))
        publishers['ready'].publish(Bool(data=chassis_ready))
        publishers['localization'].publish(Bool(data=True))
    def wait_for(value):
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            publish(value)
            time.sleep(.02)  # Publishers need no executor; monitor owns its own context/executor.
            if health.snapshot()['can_run'] is value:return
        pytest.fail(str(health.snapshot()))
    try:
        wait_for(True)
        runs=PassRuns(lambda *args:[sys.executable,'-c','print("ROS fixture round")'])
        client=make_app(runs,health=health).test_client()
        assert client.get('/api/hardware/state').json['can_run']
        assert client.post('/api/pass/run',json=dict(side='A',round=1)).status_code==202
        deadline=time.monotonic()+5
        while runs.snapshot()['status']=='running' and time.monotonic()<deadline:
            publish();time.sleep(.02)
        assert runs.snapshot()['status']=='succeeded'
        wait_for(False)
        for side in ('A','B'):
            for number in (1,2,3):
                assert client.post('/api/pass/run',json=dict(side=side,round=number)).status_code==409
    finally:
        monitor.close(); node.destroy_node(); context.shutdown()
