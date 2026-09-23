from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("rb_mission")) / "config" / "mission.yaml"
    return LaunchDescription([
        # ⚠️ 同 localization：config 走 config_file 参数，不做 --params-file
        Node(package="rb_mission", executable="mission_node",
             name="rb_mission", output="screen",
             parameters=[{"config_file": str(config)}]),
    ])
