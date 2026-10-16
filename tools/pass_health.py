"""Read-only equipment status and admission checks for new passing rounds."""
from __future__ import annotations

import json
from pathlib import Path
import threading
import time


class HardwareNotReady(RuntimeError):
    def __init__(self, hardware):
        self.hardware = hardware
        super().__init__('；'.join(hardware['reasons']) or '设备尚未就绪')


class PassHealth:
    # These deadlines only reject NEW starts. They never stop an active route
    # or expire a chassis velocity command.
    def __init__(self, clock=time.monotonic, max_age=2., can_present=None):
        self.clock, self.max_age = clock, max_age
        self.can_present = can_present or (lambda: Path('/sys/class/net/can0').exists())
        self.lock = threading.Lock()
        self.values = {}
        self.error = None

    def _save(self, name, value):
        with self.lock:
            self.values[name] = (value, self.clock())

    def set_error(self, error):
        with self.lock:
            self.error = str(error)

    def observe_session(self, text):
        try:
            data = json.loads(text)
            if not isinstance(data, dict) or type(data.get('ready')) is not bool:
                raise ValueError('缺少有效的ready状态')
            self._save('lidar', data)
        except (ValueError, TypeError) as exc:
            self._save('lidar', dict(ready=False, reason='雷达状态数据无效：'+str(exc)))

    def observe_chassis(self, text):
        try:
            data = json.loads(text)
            if (not isinstance(data, dict) or type(data.get('ready')) is not bool
                    or type(data.get('dry_run')) is not bool):
                raise ValueError('缺少有效的底盘诊断状态')
            self._save('chassis', data)
        except (ValueError, TypeError) as exc:
            self._save('chassis', dict(ready=False, dry_run=False, reason='底盘状态数据无效：'+str(exc)))

    def observe_ready(self, ready):
        self._save('board_ready', bool(ready))

    def observe_localization(self, ready):
        self._save('localization', bool(ready))

    def snapshot(self, boot=None):
        now = self.clock()
        with self.lock:
            values, error = dict(self.values), self.error

        def current(name):
            value, received = values.get(name, (None, None))
            return value if received is not None and 0 <= now-received <= self.max_age else None

        lidar, chassis = current('lidar'), current('chassis')
        if error:
            lidar_reason = chassis_reason = '设备状态监测不可用：'+error
            lidar_ok = chassis_ok = False
        else:
            power = (boot or {}).get('power') or {}
            lidar_ok = False
            if power.get('carrier') is False:
                lidar_reason = '雷达网口未连接，请检查雷达供电和网线'
            elif lidar is None:
                lidar_reason = ('雷达状态更新已中断，等待重新连接' if 'lidar' in values
                                else '等待雷达程序启动并报告状态')
            elif not lidar['ready']:
                lidar_reason = str(lidar.get('reason') or '雷达点云、IMU或里程计尚未就绪')
            elif current('localization') is not True:
                lidar_reason = '雷达数据已就绪，等待有效定位确认'
            else:
                lidar_ok, lidar_reason = True, '点云、IMU和定位均已就绪'

            chassis_ok = False
            if not self.can_present():
                chassis_reason = '未检测到CAN适配器can0，请检查USB-CAN连接'
            elif chassis is None:
                chassis_reason = ('底盘状态更新已中断，请检查底盘程序' if 'chassis' in values
                                  else '等待底盘程序与大疆板建立通信')
            elif chassis.get('dry_run'):
                chassis_reason = '底盘处于模拟模式，不能执行实车回合'
            elif not chassis['ready']:
                chassis_reason = str(chassis.get('reason') or '底盘握手尚未完成，请检查控制板、电调供电及CAN连接')
            elif current('board_ready') is not True:
                chassis_reason = '尚未收到底盘就绪确认，请检查控制板和CAN通信'
            else:
                chassis_ok, chassis_reason = True, '大疆板及四路电调已确认就绪'

        reasons = []
        if not lidar_ok: reasons.append('雷达：'+lidar_reason)
        if not chassis_ok: reasons.append('底盘：'+chassis_reason)
        return dict(can_run=lidar_ok and chassis_ok, reasons=reasons,
                    lidar=dict(ready=lidar_ok, reason=lidar_reason),
                    chassis=dict(ready=chassis_ok, reason=chassis_reason))


class RosHealthMonitor:
    """Shared status subscriptions and optional finite-motion command publisher."""
    def __init__(self, health, tasks=None):
        self.health = health
        self.tasks = tasks
        self.stop_event = threading.Event()
        self.thread = None

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, name='pass-equipment-status', daemon=True)
            self.thread.start()

    def _run(self):
        node = context = executor = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import Bool, String
            context = Context()
            rclpy.init(context=context)
            node = rclpy.create_node('pass_console_health', context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            def ready(message):
                self.health.observe_ready(message.data)
                if self.tasks is not None:
                    self.tasks.board_ready(message.data)
            if self.tasks is not None:
                from geometry_msgs.msg import Twist
                publisher = node.create_publisher(Twist, '/cmd_vel', 1)
                def publish(velocity):
                    message = Twist()
                    message.linear.y = velocity
                    publisher.publish(message)
                self.tasks.attach(publish)
                def motion_tick():
                    others = any(info.node_name != node.get_name() or info.node_namespace != node.get_namespace()
                                 for info in node.get_publishers_info_by_topic('/cmd_vel'))
                    self.tasks.tick(others)
                motion_timer = node.create_timer(.02, motion_tick)
            subscriptions = [
                node.create_subscription(String, '/lidar/session', lambda m: self.health.observe_session(m.data), 10),
                node.create_subscription(String, '/chassis/status', lambda m: self.health.observe_chassis(m.data), 10),
                node.create_subscription(Bool, '/chassis/ready', ready, 10),
                node.create_subscription(Bool, '/localization/ok', lambda m: self.health.observe_localization(m.data), 10),
            ]
            while not self.stop_event.is_set() and rclpy.ok(context=context):
                executor.spin_once(timeout_sec=.1)
        except Exception as exc:
            self.health.set_error(str(exc))
        finally:
            if self.tasks is not None:
                self.tasks.detach()
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.)
