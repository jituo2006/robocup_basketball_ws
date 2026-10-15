#!/usr/bin/env python3
"""管理 Livox/FAST-LIO 生命周期；断流后重启估计器，禁止跨断电间隔积分。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from lidar_health import LidarHealth, cdr_header_stamp


def stop_process(proc):
    if proc is None:
        return
    # ros2 launch 和它的节点在独立进程组内，退出时一起回收。
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=3.)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=2.)
    except ProcessLookupError:
        pass
    finally:
        # launch 父进程先退出时仍可能留下子节点；也回收同组残留。
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def main():
    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data

    parser = argparse.ArgumentParser()
    parser.add_argument('--lidar-root', default='/home/user/lidar_360')
    parser.add_argument('--interface', default='enp114s0')
    parser.add_argument('--stable-seconds', type=float, default=1.)
    args = parser.parse_args()
    if args.stable_seconds <= 0:
        parser.error('stable-seconds 必须大于零')
    rclpy.init()
    node = rclpy.create_node('lidar_session')
    pub = node.create_publisher(String, '/lidar/session', 10)
    health = LidarHealth(args.stable_seconds)
    driver = mapping = None
    generation = time.time_ns()
    ready = False
    cutoff = 0.
    serial = 0
    next_driver = next_mapping = 0.
    subscriptions = []

    def announce():
        msg = String()
        msg.data = json.dumps(dict(generation=generation, ready=ready, cutoff=cutoff))
        pub.publish(msg)

    def receive(name, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        position = None
        if name == 'odom':
            p = msg.pose.pose.position
            position = (p.x, p.y, p.z)
        health.observe(name, stamp, node.get_clock().now().nanoseconds*1e-9,
                       time.monotonic(), position)

    # 每帧约两万 CustomPoint，Python 展开数组实测耗时约 60～87ms。
    # 管理器只需要 Header；使用 raw 避免阻塞 IMU/里程计及会话心跳。
    def receive_lidar(data):
        try:
            stamp = cdr_header_stamp(data)
        except ValueError:
            return
        health.observe('lidar', stamp, node.get_clock().now().nanoseconds*1e-9,
                       time.monotonic())

    subscriptions.append(node.create_subscription(
        CustomMsg, '/livox/lidar', receive_lidar, qos_profile_sensor_data, raw=True))
    for name, typ, topic in [('imu', Imu, '/livox/imu'),
                             ('odom', Odometry, '/Odometry')]:
        subscriptions.append(node.create_subscription(
            typ, topic, lambda msg, name=name: receive(name, msg), qos_profile_sensor_data))
    node.create_timer(.1, announce)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            mono = time.monotonic()
            wall = node.get_clock().now().nanoseconds*1e-9
            try:
                link = Path('/sys/class/net', args.interface, 'carrier').read_text().strip() == '1'
            except OSError:
                link = False
            # Temporary delivery gaps only withdraw readiness; keep estimator state.
            fault = health.fault_serial != serial
            if ready and (not health.inputs_live(wall, mono) or not health.odom.live(wall, mono)):
                ready = False
                announce()
            if mapping is not None and (mapping.poll() is not None or not link or fault):
                ready = False
                generation += 1
                cutoff = 0.
                announce()  # 先让定位撤销旧帧，再回收估计进程。
                node.get_logger().warn('重置 FAST-LIO 会话：'+health.reason)
                stop_process(mapping)
                mapping = None
                health.reset_odom()
                next_mapping = time.monotonic()+1.
            serial = health.fault_serial
            if driver is not None and (driver.poll() is not None or not link):
                stop_process(driver)
                driver = None
                health.reset_inputs()
                next_driver = time.monotonic()+1.
            if link and driver is None and mono >= next_driver:
                driver = subprocess.Popen(['ros2', 'launch', 'livox_ros_driver2',
                                           'msg_MID360_launch.py'], start_new_session=True)
            if mapping is None and mono >= next_mapping and health.inputs_stable(wall, mono):
                health.reset_odom()
                mapping = subprocess.Popen(['ros2', 'launch', 'fast_lio', 'mapping.launch.py',
                    'config_path:='+args.lidar_root+'/fast_prop_ws/src/FAST_LIO_WITH_PROPAGATE/config',
                    'config_file:=mid360.yaml', 'rviz:=false'], start_new_session=True)
                node.get_logger().info('雷达/IMU 已连续稳定，启动全新 FAST-LIO 会话')
            if (mapping is not None and not ready and health.inputs_live(wall, mono)
                    and health.odom_stable(wall, mono)):
                cutoff = health.odom.stamp
                ready = True
                announce()
                node.get_logger().info('新里程计已稳定，定位可以接入')
    finally:
        ready = False
        if rclpy.ok():
            announce()
        stop_process(mapping)
        stop_process(driver)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
