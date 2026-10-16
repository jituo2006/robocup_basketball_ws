"""执行真实管理器主循环，ROS、网口和进程边界替换；不连接机器人。"""
import importlib.util
from pathlib import Path
import sys
import struct
from types import ModuleType, SimpleNamespace as NS
import pytest

scripts=Path(__file__).parents[1]/'scripts'
sys.path.insert(0,str(scripts))
import lidar_session


@pytest.mark.parametrize('fault', ['link', 'imu_gap', 'odom_jump', 'mapping_exit',
                                 'slow_lidar', 'slow_mapping', 'sync_skew', 'batch_delay',
                                 'driver_silence'])
def test_manager_restarts_and_waits_for_new_stable_data(monkeypatch, fault):
    runtime=NS(t=100.,tick=0,events=[],messages=[],processes=[],callbacks={})
    def bad():return 102 < runtime.t < (104.3 if fault=='driver_silence' else (103.3 if fault=='imu_gap' else 102.7))
    class Proc:
        def __init__(self,cmd,**kwargs):
            assert kwargs['start_new_session']
            self.kind='mapping' if 'fast_lio' in cmd else 'driver'
            self.started=runtime.t
            self.pid=1000+len(runtime.processes);self.stopped=False
            runtime.processes.append(self);runtime.events.append(('start',self.kind,runtime.t))
        def poll(self):
            return 1 if self.stopped or self.kind=='mapping' and fault=='mapping_exit' and bad() else None
        def wait(self,timeout):self.stopped=True
    def kill(pid,sig):
        p=next(p for p in runtime.processes if p.pid==pid)
        p.stopped=True;runtime.events.append(('stop',p.kind,runtime.t))
    def active(kind):return any(p.kind==kind and p.poll() is None for p in runtime.processes)
    class Carrier:
        def __init__(self,*args):pass
        def read_text(self):return '0' if fault=='link' and bad() else '1'
    class Node:
        def create_publisher(self,*args):
            import json
            return NS(publish=lambda m:runtime.messages.append((runtime.t,json.loads(m.data))))
        def create_subscription(self,typ,topic,cb,qos,**kwargs):
            if topic=='/livox/lidar':
                assert kwargs.get('raw') is True
                def deliver(msg):
                    stamp=msg.header.stamp
                    cb(b'\x00\x01\x00\x00'+struct.pack('<iI',stamp.sec,stamp.nanosec))
                runtime.callbacks[topic]=deliver
            else:
                runtime.callbacks[topic]=cb
            return cb
        def create_timer(self,*args):pass
        def get_clock(self):return NS(now=lambda:NS(nanoseconds=int(runtime.t*1e9)))
        def get_logger(self):return NS(info=lambda *a:None,warn=lambda *a:None)
        def destroy_node(self):pass
    n=Node()
    def message(position=False, lidar=False):
        delay = 1.1 if fault=='sync_skew' and bad() else (.3 if fault=='batch_delay' else 0.)
        t=runtime.t-(delay if lidar else 0.)
        m=NS(header=NS(stamp=NS(sec=int(t),nanosec=int((t-int(t))*1e9))))
        if position:
            m.pose=NS(pose=NS(position=NS(x=500 if fault=='odom_jump' and bad() else 0,y=0,z=0)))
        return m
    def spin(*args,**kwargs):
        runtime.tick+=1;runtime.t=100+runtime.tick*.01
        if active('driver') and not (fault=='slow_lidar' and runtime.t<112) and not (fault=='driver_silence' and bad()):
            if not (fault=='imu_gap' and bad()):runtime.callbacks['/livox/imu'](message())
            if runtime.tick%10==0:runtime.callbacks['/livox/lidar'](message(lidar=True))
        slow = fault=='slow_mapping' and runtime.t<next(
            (p.started+12 for p in reversed(runtime.processes) if p.kind=='mapping'),float('inf'))
        if active('mapping') and not slow and runtime.tick%10==0:
            runtime.callbacks['/Odometry'](message(True))
    modules={
        'rclpy':dict(init=lambda:None,create_node=lambda *a:n,
                    ok=lambda:runtime.tick<(2300 if fault.startswith('slow_') else 650),
                    spin_once=spin,shutdown=lambda:None),
        'rclpy.qos':dict(qos_profile_sensor_data=object()),
        'livox_ros_driver2.msg':dict(CustomMsg=object),
        'nav_msgs.msg':dict(Odometry=object),
        'sensor_msgs.msg':dict(Imu=object),
        'std_msgs.msg':dict(String=lambda:NS(data='')),
    }
    for name,attrs in modules.items():
        m=ModuleType(name);m.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,m)
    monkeypatch.setattr(lidar_session,'Path',Carrier)
    monkeypatch.setattr(lidar_session,'startup_state',lambda *a:NS(ready=True,phase='waiting_inputs',reason='network ready'))
    monkeypatch.setattr(lidar_session.subprocess,'Popen',Proc)
    monkeypatch.setattr(lidar_session.os,'killpg',kill,raising=False)
    monkeypatch.setattr(lidar_session.signal,'SIGKILL',9,raising=False)
    monkeypatch.setattr(lidar_session,'time',NS(monotonic=lambda:runtime.t,time_ns=lambda:123))
    monkeypatch.setattr(sys,'argv',['lidar_session.py','--stable-seconds','.2'])
    lidar_session.main()
    starts=[t for action,kind,t in runtime.events if action=='start' and kind=='mapping']
    ready=[(t,s) for t,s in runtime.messages if s['ready']]
    if fault.startswith('slow_') or fault in ('batch_delay','driver_silence','imu_gap','sync_skew'):
        assert len(starts)==1
        assert sum(action=='start' and kind=='driver' for action,kind,t in runtime.events)==1
        assert ready
        if fault in ('driver_silence','imu_gap','sync_skew'):
            assert any(t<102 for t,s in ready) and any(t>=(104.3 if fault=='driver_silence' else 103.3 if fault=='imu_gap' else 102.7) for t,s in ready)
            assert ready[-1][1]['generation']==ready[0][1]['generation']
        if fault.startswith('slow_'):
            assert ready[0][0]>112
        assert all(p.stopped for p in runtime.processes)
        return
    assert len(starts)>=2
    assert ready and any(t<102 for t,s in ready) and any(t>103 for t,s in ready)
    driver_starts=sum(a=='start' and k=='driver' for a,k,t in runtime.events)
    if fault in ('sync_skew', 'imu_gap'):
        assert driver_starts==1  # 短暂不同步/短丢帧只重建估计，不重启驱动。
    if fault=='driver_silence':
        assert driver_starts>=2
    first_gen=ready[0][1]['generation']
    assert ready[-1][1]['generation']>first_gen
    assert all(s['cutoff']>=starts[-1] for t,s in ready if t>starts[-1])
    assert all(p.stopped for p in runtime.processes)
