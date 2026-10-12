from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rb_camera"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="/dev/video0"),
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="720"),
            DeclareLaunchArgument("fps", default_value="30.0"),
            DeclareLaunchArgument(
                "config_file", default_value=str(share / "config" / "camera.yaml")
            ),
            Node(
                package="rb_camera",
                executable="camera_node",
                name="rb_camera",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("config_file"),
                        "device": LaunchConfiguration("device"),
                        "width": LaunchConfiguration("width"),
                        "height": LaunchConfiguration("height"),
                        "fps": LaunchConfiguration("fps"),
                    }
                ],
            ),
        ]
    )
