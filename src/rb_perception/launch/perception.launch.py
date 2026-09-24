from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rb_perception"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("use_compressed", default_value="true",
                                  description="true=订阅 JPEG 压缩图（原始大图投递只有 ~7fps）"),
            DeclareLaunchArgument("compressed_topic", default_value="/camera/image_raw/compressed"),
            DeclareLaunchArgument("publish_debug_image", default_value="true"),
            DeclareLaunchArgument(
                "config_file", default_value=str(share / "config" / "perception.yaml")
            ),
            Node(
                package="rb_perception",
                executable="perception_node",
                name="rb_perception",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("config_file"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "use_compressed": LaunchConfiguration("use_compressed"),
                        "compressed_topic": LaunchConfiguration("compressed_topic"),
                        "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                    }
                ],
            ),
        ]
    )
