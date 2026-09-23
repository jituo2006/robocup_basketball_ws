from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("rb_localization")) / "config" / "localization.yaml"
    return LaunchDescription([
        # ⚠️ config 是"给节点自己读的配置文件"（含自定义顶层键），
        # 不能作为 ROS 参数文件直接传入（ROS 2 要求顶层是 node + ros__parameters）。
        Node(package="rb_localization", executable="localization_node",
             name="rb_localization", output="screen",
             parameters=[{"config_file": str(config)}]),
    ])
