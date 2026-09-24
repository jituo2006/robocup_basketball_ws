"""离线联调启动：不需要相机、雷达、CAN、机构，验证整条软件链路。

组成：
  fake_board.py         假底盘，把 /cmd_vel 积分成 /odom（让定位与任务能动起来）
  publish_test_image.py 反复发布合成测试图（让视觉有输入）
  rb_perception         真视觉节点
  rb_localization       真定位节点（2D EKF）
  rb_mission            真任务状态机
  rb_chassis(dry_run)   真底盘节点，只解算不下发 CAN

跑起来后：
  ros2 topic echo /mission/status
  ros2 service call /rb_mission/set_mission rb_msgs/srv/SetMission "{mission: PASS}"
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def _share(pkg, *parts):
    return str(Path(get_package_share_directory(pkg)).joinpath(*parts))


def generate_launch_description():
    # 夹具已移入 rb_tests 包（测试资产与主代码分离），用 ros2 run 拉起
    return LaunchDescription(
        [
            ExecuteProcess(cmd=["ros2", "run", "rb_tests", "fake_board"], output="screen"),
            ExecuteProcess(cmd=["ros2", "run", "rb_tests", "publish_test_image",
                                "--topic", "/camera/image_raw", "--rate", "5.0"],
                           output="screen"),

            Node(package="rb_perception", executable="perception_node", name="rb_perception",
                 output="screen",
                 parameters=[{"config_file": _share("rb_perception", "config", "perception.yaml"),
                              "image_topic": "/camera/image_raw",
                              # 合成测试图是**原始图**，所以这里必须关掉压缩订阅，
                              # 否则会去等 /camera/image_raw/compressed（没人发）。
                              "use_compressed": False,
                              "publish_debug_image": False}]),
            # 假底盘发 /odom，这里覆盖输入源
            Node(package="rb_localization", executable="localization_node", name="rb_localization",
                 output="screen",
                 parameters=[{"config_file": _share("rb_localization", "config", "localization.yaml"),
                              "odom_topic": "/odom",
                              "abs_pose_topic": ""}]),
            Node(package="rb_chassis", executable="rb_chassis_node", name="rb_chassis",
                 output="screen",
                 parameters=[_share("rb_chassis", "config", "chassis.yaml"),
                             {"dry_run": True}]),
            Node(package="rb_mission", executable="mission_node", name="rb_mission",
                 output="screen",
                 parameters=[{"config_file": _share("rb_mission", "config", "mission.yaml")}]),
        ]
    )
