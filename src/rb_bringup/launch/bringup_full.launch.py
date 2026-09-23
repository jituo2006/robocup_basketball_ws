"""整车启动：底盘 + 机构 + 视觉 + 定位 + 任务。

用法：
  ros2 launch rb_bringup bringup_full.launch.py
  ros2 launch rb_bringup bringup_full.launch.py dry_run:=true     # 只解算不下发 CAN

⚠️ 上车前请先跑 tools/preflight.sh 做一次预检。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share(pkg, *parts):
    return str(Path(get_package_share_directory(pkg)).joinpath(*parts))


def generate_launch_description():
    dry_run = LaunchConfiguration("dry_run")
    image_topic = LaunchConfiguration("image_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),

            # 底盘
            Node(package="rb_chassis", executable="rb_chassis_node", name="rb_chassis",
                 output="screen",
                 parameters=[_share("rb_chassis", "config", "chassis.yaml"),
                             {"dry_run": dry_run}]),

            # 机构
            Node(package="rb_launcher", executable="rb_launcher_node", name="rb_launcher",
                 output="screen",
                 parameters=[_share("rb_launcher", "config", "launcher.yaml")]),

            # 视觉
            Node(package="rb_perception", executable="perception_node", name="rb_perception",
                 output="screen",
                 parameters=[{"config_file": _share("rb_perception", "config", "perception.yaml"),
                              "image_topic": image_topic,
                              "publish_debug_image": True}]),

            # 定位
            Node(package="rb_localization", executable="localization_node", name="rb_localization",
                 output="screen",
                 parameters=[{"config_file": _share("rb_localization", "config", "localization.yaml")}]),

            # 任务
            Node(package="rb_mission", executable="mission_node", name="rb_mission",
                 output="screen",
                 parameters=[{"config_file": _share("rb_mission", "config", "mission.yaml")}]),
        ]
    )
