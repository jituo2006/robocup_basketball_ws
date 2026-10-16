"""Real rb_launcher + isolated ROS domain + PTY MCU. No real serial device."""
import os
import pty
import select
import subprocess
import threading
import time
import tty
from pathlib import Path
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from test_serial_actions import MCU, SHOOT
from serial_actions import RosSerialActions


@pytest.mark.parametrize('fragment', [False, True])
def test_real_ros_uart_retry_and_completion(tmp_path, fragment):
    domain = 198 + int(fragment)
    master, slave = pty.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    node = Node('serial_actions_pty_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = RosSerialActions(node, lambda s: executor.spin_once(timeout_sec=s))
    binary = Path(get_package_prefix('rb_launcher')) / 'lib/rb_launcher/rb_launcher_node'
    log = open(tmp_path / 'node.log', 'w')
    process = subprocess.Popen([
        str(binary), '--ros-args', '-p', f'port:={os.ttyname(slave)}', '-p', 'auto_reconnect:=false'],
        env=dict(os.environ, ROS_DOMAIN_ID=str(domain)), stdout=log, stderr=log)
    m = MCU()
    stop, errors = threading.Event(), []

    def emit(kind, line, tx=None):
        if kind != 'rx':
            return
        data = (line + '\r\n').encode()
        if fragment:
            os.write(master, data[:3])
            time.sleep(.001)
            os.write(master, data[3:])
        else:
            os.write(master, data)
    m.emit = emit
    attempts = [0]
    def delayed_up():
        attempts[0] += 1
        if attempts[0] < 3:
            m.rx('ERR BUSY')
        else:
            m.rx('ACK DJI_MOVE 1 MOVING')
            m.later(.2, lambda: m.rx('DONE'))
    m.overrides['dji_move 1 90 5'] = delayed_up

    def emulate():
        buffer = b''
        last = time.monotonic()
        try:
            while not stop.is_set():
                now = time.monotonic()
                m.pump(now - last)
                last = now
                if select.select([master], [], [], .002)[0]:
                    buffer += os.read(master, 4096)
                    while b'\n' in buffer:
                        raw, buffer = buffer.split(b'\n', 1)
                        command = raw.decode().strip()
                        m.sent.append((m.t, command))
                        m.reply(command)
        except Exception as exc:
            if not stop.is_set():
                errors.append(exc)
    thread = threading.Thread(target=emulate, daemon=True)
    thread.start()
    try:
        end = time.monotonic() + 8
        while not client.client.service_is_ready() or node.count_publishers('/rb_launcher/rx_event') < 1:
            assert time.monotonic() < end, (tmp_path / 'node.log').read_text()
            executor.spin_once(timeout_sec=.02)
        # Establish DDS matches before transmitting any simulated motor request.
        for _ in range(15):
            executor.spin_once(timeout_sec=.02)
        report = client.controller.pickup()
        assert report.confirmed, report.message
        up = [t for t, c in m.sent if c == 'dji_move 1 90 5']
        assert len(up) == 3
        assert all(.04 <= b-a <= .10 for a, b in zip(up, up[1:]))
        lower = next(t for t, c in m.sent if c == 'dji_move 1 0 5')
        assert lower - up[-1] >= 1.19
        report = client.controller.shoot(SHOOT)
        assert report.confirmed, report.message + '\n' + (tmp_path / 'node.log').read_text()[-2000:]
        vesc = [c for _, c in m.sent if c.startswith('vesc_') and c != 'vesc_status']
        assert vesc == SHOOT
        assert m.home == 0
        assert not errors, errors
    finally:
        stop.set()
        thread.join(2)
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        os.close(master)
        os.close(slave)
