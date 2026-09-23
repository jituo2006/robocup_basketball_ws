from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("rb_chassis")) / "config" / "chassis.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="false",
                                  description="true = 只做运动学解算，不下发 CAN"),
            Node(
                package="rb_chassis",
                executable="rb_chassis_node",
                name="rb_chassis",
                output="screen",
                parameters=[str(config), {"dry_run": LaunchConfiguration("dry_run")}],
            ),
        ]
    )
