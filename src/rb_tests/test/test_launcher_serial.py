"""Real launcher process + ROS service + PTY; never opens a robot device."""
import os
import pty
import select
import struct
import subprocess
import time
import zlib

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rb_msgs.srv import Launch


def test_actual_node_frames_stop_toggles_and_reconnect(tmp_path):
    master, slave = pty.openpty()
    port = tmp_path / 'launcher_test_port'
    context = Context()
    rclpy.init(args=[], context=context, domain_id=191)
    client_node = Node('launcher_serial_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client_node)
    client = client_node.create_client(Launch, '/rb_launcher/launch')
    executable = os.path.join(get_package_prefix('rb_launcher'), 'lib/rb_launcher/rb_launcher_node')
    env = dict(os.environ, ROS_DOMAIN_ID='191')
    log = open(tmp_path / 'launcher.log', 'w')
    process = subprocess.Popen([executable, '--ros-args', '-p', f'port:={port}',
                                '-p', 'reconnect_period_s:=0.1', '-p', 'send_rate_hz:=20.0'],
                               env=env, stdout=log, stderr=log)

    def call(action, speed=0, angle=0):
        request = Launch.Request(action=action, speed=speed, angle=angle)
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done(), 'launcher service timeout'
        return future.result()

    def drain():
        while select.select([master], [], [], .05)[0]:
            os.read(master, 4096)

    try:
        assert client.wait_for_service(timeout_sec=8)
        # Rejected request must not execute when the port later appears.
        assert not call(0, 3200, 71).success
        port.symlink_to(os.ttyname(slave))
        time.sleep(.4)
        assert not select.select([master], [], [], .2)[0]
        for action, cmd in [(0, 0x40), (1, 0x41), (4, 0x20), (5, 0x10)]:
            assert call(action, 3200 + action, 71 + action).success
            data = b''
            deadline = time.monotonic() + 2
            while len(data) < 11 and time.monotonic() < deadline:
                if select.select([master], [], [], .1)[0]:
                    data += os.read(master, 4096)
            assert len(data) >= 11
            frame = data[:11]
            assert frame[0] == ord('+') and frame[10] == ord('*')
            assert frame[1] == cmd
            assert struct.unpack('<HH', frame[2:6]) == (3200 + action, 71 + action)
            assert struct.unpack('<I', frame[6:10])[0] == zlib.crc32(frame[:6])
            if action in (4, 5):
                assert len(data) == 11
                assert not select.select([master], [], [], .2)[0], 'toggle repeated'
            assert call(7).success
            drain()
            assert not select.select([master], [], [], .15)[0], 'STOP still transmits'
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=5)
        log.close()
        executor.shutdown()
        client_node.destroy_node()
        rclpy.shutdown(context=context)
        os.close(master); os.close(slave)
