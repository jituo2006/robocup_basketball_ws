"""Only rear camera + perception. Never launches chassis or a mission."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    camera_config = LaunchConfiguration("camera_config").perform(context)
    perception_config = LaunchConfiguration("perception_config").perform(context)
    with open(camera_config, encoding="utf-8") as fh:
        device = str((yaml.safe_load(fh) or {}).get("camera", {}).get("device", ""))
    if not device.startswith("/dev/v4l/by-id/"):
        raise RuntimeError("Set the rear camera's verified /dev/v4l/by-id/... device first")
    return [
        Node(package="rb_camera", executable="camera_node", name="rb_camera_rear",
             parameters=[{"config_file": camera_config}]),
        Node(package="rb_perception", executable="perception_node", name="rb_perception_rear",
             parameters=[{"config_file": perception_config,
                          "image_topic": "/camera_rear/image_raw",
                          "compressed_topic": "/camera_rear/image_raw/compressed",
                          "output_topic": "/perception_rear/detections",
                          "debug_image_topic": "/perception_rear/debug_image",
                          "precision_topic": "/perception_rear/ball_range"}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_config", default_value=os.path.join(
            get_package_share_directory("rb_camera"), "config", "camera_rear.yaml")),
        DeclareLaunchArgument("perception_config", default_value=os.path.join(
            get_package_share_directory("rb_perception"), "config", "perception_rear.yaml")),
        OpaqueFunction(function=start),
    ])
