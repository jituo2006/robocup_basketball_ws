"""Actual C++ node + ROS + PTY + simulated MCU; private ROS domains only."""
import os
import pty
import select
import subprocess
import threading
import time
import tty
from pathlib import Path
import sys
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from test_mechanism_control import FakeMCU, commands
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from mechanism_control import RosMechanismClient


@pytest.mark.parametrize('fragment',[False,True])
def test_real_serial_rx_checked_actions_and_refusals(tmp_path,fragment):
    domain=196+int(fragment)
    master,slave=pty.openpty()
    tty.setraw(master);tty.setraw(slave)
    context=Context();rclpy.init(args=[],context=context,domain_id=domain)
    node=Node('mechanism_pty_test',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    client=RosMechanismClient(node,lambda seconds:executor.spin_once(timeout_sec=seconds))
    executable=Path(get_package_prefix('rb_launcher'))/'lib/rb_launcher/rb_launcher_node'
    log=open(tmp_path/'node.log','w')
    process=subprocess.Popen([str(executable),'--ros-args','-p',f'port:={os.ttyname(slave)}',
        '-p','auto_reconnect:=false'],env=dict(os.environ,ROS_DOMAIN_ID=str(domain)),stdout=log,stderr=log)
    m=FakeMCU();stop=threading.Event();errors=[]
    def emit(kind,line,tx=None):
        if kind!='rx':return
        data=(line+'\r\n').encode()
        if fragment:
            os.write(master,data[:3]);time.sleep(.001);os.write(master,data[3:])
        else:os.write(master,data)
    m.emit=emit
    def emulate():
        buffer=b'';last=time.monotonic()
        try:
            while not stop.is_set():
                now=time.monotonic();m.pump(now-last);last=now
                if select.select([master],[],[],.003)[0]:
                    buffer+=os.read(master,4096)
                    while b'\n' in buffer:
                        line,buffer=buffer.split(b'\n',1)
                        m.send(line.decode().strip())
        except Exception as exc:
            if not stop.is_set():errors.append(exc)
    thread=threading.Thread(target=emulate,daemon=True);thread.start()
    try:
        deadline=time.monotonic()+8
        while not client.client.service_is_ready() or node.count_publishers('/rb_launcher/rx_event')<1:
            assert time.monotonic()<deadline, (tmp_path/'node.log').read_text()
            executor.spin_once(timeout_sec=.02)
        # Let DDS match both directions before issuing the first query.
        for _ in range(10):executor.spin_once(timeout_sec=.02)
        ok,message=client.controller.pickup(.05)
        assert ok,message+'\n'+(tmp_path/'node.log').read_text()[-2500:]
        assert m.position==0
        ok,message=client.controller.release(['vesc_stop','vesc_home','vesc_shoot 3','vesc_stop','vesc_reset'])
        assert ok,message+'\n'+(tmp_path/'node.log').read_text()[-2500:]
        assert commands(m).count('vesc_shoot 3')==1 and m.home==1
        before=len(m.sent)
        m.overrides['dji_move 1 90 5']=lambda:m.rx('ERR BUSY')
        ok,message=client.controller.pickup(0.)
        assert not ok and 'ERR BUSY' in message
        assert not any(c=='dji_move 1 0 5' for _,c in m.sent[before:])
        assert not any(c in ('dji_stop','vesc_stop') for _,c in m.sent[before:])
        # A partial reply blocks new motion, but never blocks a stop request.
        os.write(master,b'INCOMPLETE_REPLY ')
        for _ in range(5):executor.spin_once(timeout_sec=.02)
        before=len(m.sent)
        ok,message=client.raw_send('dji_move 1 90 5')
        assert not ok and '完整' in message
        assert len(m.sent)==before
        ok,message=client.raw_send('dji_stop')
        assert ok,message
        for _ in range(10):executor.spin_once(timeout_sec=.02)
        assert m.sent[-1][1]=='dji_stop'
        before=len(m.sent)
        ok,message=client.raw_send('dji_stop\nvesc_shoot 3')
        assert not ok
        for _ in range(5):executor.spin_once(timeout_sec=.01)
        assert len(m.sent)==before, 'multiline request reached the UART'
        assert not errors,errors
    finally:
        stop.set();thread.join(2)
        process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        log.close();executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)
        os.close(master);os.close(slave)
