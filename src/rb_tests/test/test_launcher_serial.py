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
    # 不传 fallback_ports：节点默认就是空（不启用回退），所以"端口不存在→拒绝"
    # 这条断言不会受机器上碰巧存在的 /dev/usb2ttl 之类节点影响。
    # （注：ROS 2 不接受 `-p fallback_ports:=[]` 这种空数组写法，会起不来。）
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


def test_missing_configured_port_falls_back_to_candidate(tmp_path):
    """配置的串口不存在时，应自动改用 fallback_ports 里第一个存在的。

    为什么单测这个：本机 udev 生成 /dev/usb2ttl，而上一届配置写 /dev/R1_usb2ttl
    —— 对不上时机构完全不能动，且日志只说"打开失败"，现场很难自己想到去改配置。
    回退逻辑保证"插上就能用"，找到哪个名字会在日志里说明。
    """
    master, slave = pty.openpty()
    real = os.ttyname(slave)              # 真实存在的设备，如 /dev/pts/5
    bogus = tmp_path / 'R1_usb2ttl'       # 配置里写的名字，不存在
    context = Context()
    rclpy.init(args=[], context=context, domain_id=192)
    client_node = Node('launcher_fallback_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client_node)
    client = client_node.create_client(Launch, '/rb_launcher/launch')
    executable = os.path.join(get_package_prefix('rb_launcher'), 'lib/rb_launcher/rb_launcher_node')
    env = dict(os.environ, ROS_DOMAIN_ID='192')
    log_path = tmp_path / 'launcher_fallback.log'
    log = open(log_path, 'w')
    # port 故意指向不存在的路径；fallback 指向真实 PTY
    process = subprocess.Popen([executable, '--ros-args',
                                '-p', f'port:={bogus}',
                                '-p', f'fallback_ports:=["{real}"]',
                                '-p', 'send_rate_hz:=20.0'],
                               env=env, stdout=log, stderr=log)
    try:
        assert client.wait_for_service(timeout_sec=8)
        # 启动时就应该已经回退成功 → 第一次请求就该被接受
        request = Launch.Request(action=0, speed=3000, angle=60)
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done(), 'launcher service timeout'
        assert future.result().success, '应回退到 PTY 并接受请求'

        data = b''
        deadline = time.monotonic() + 2
        while len(data) < 11 and time.monotonic() < deadline:
            if select.select([master], [], [], .1)[0]:
                data += os.read(master, 4096)
        assert len(data) >= 11, '回退后应能在 PTY 上看到帧'
        assert data[0] == ord('+') and data[10] == ord('*')
        assert data[1] == 0x40                  # SHOOT

        # 日志要说明用的是哪个设备（现场核对靠这句）
        log.flush()
        text = log_path.read_text(errors='replace')
        assert real in text, f'日志里应写明回退到 {real}'
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


def test_shipped_config_starts_node(tmp_path):
    """用**仓库里出厂的那份 launcher.yaml** 起节点，服务必须出现。

    为什么必须有这条：参数文件写错（例如 `fallback_ports: []` 这种空序列）会让
    节点在构造时抛 InvalidParameterValueException 直接 abort —— 表现为
    "机构服务不见了"，但 unit test 全绿。实测踩到过，所以用出厂配置兜一道。
    """
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path as _P
    # 优先用 install 里那份（节点实际读的就是它）；退回源码里那份
    cfg = _P(get_package_share_directory('rb_launcher')) / 'config' / 'launcher.yaml'
    if not cfg.is_file():
        from rb_tests.ws import WS
        cfg = WS / 'src' / 'rb_launcher' / 'config' / 'launcher.yaml'
    if not cfg.is_file():
        import pytest
        pytest.skip(f"找不到出厂配置 {cfg}")

    context = Context()
    rclpy.init(args=[], context=context, domain_id=193)
    client_node = Node('launcher_cfg_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client_node)
    client = client_node.create_client(Launch, '/rb_launcher/launch')
    executable = os.path.join(get_package_prefix('rb_launcher'), 'lib/rb_launcher/rb_launcher_node')
    env = dict(os.environ, ROS_DOMAIN_ID='193')
    log_path = tmp_path / 'launcher_cfg.log'
    log = open(log_path, 'w')
    # 用一个不存在的串口，避免碰真设备；节点应照常起来（只报 ok=false）
    process = subprocess.Popen([executable, '--ros-args',
                                '--params-file', str(cfg),
                                '-p', f'port:={tmp_path}/nonexistent'],
                               env=env, stdout=log, stderr=log)
    try:
        assert client.wait_for_service(timeout_sec=8), (
            '出厂配置下服务没起来 —— 多半是参数文件把节点搞崩了；'
            f'日志：{log_path.read_text(errors="replace")[-500:]}')
        assert process.poll() is None, '节点不该退出'
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
