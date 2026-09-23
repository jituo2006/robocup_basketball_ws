from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("rb_launcher")) / "config" / "launcher.yaml"
    return LaunchDescription([
        Node(package="rb_launcher", executable="rb_launcher_node",
             name="rb_launcher", output="screen", parameters=[str(config)]),
    ])
