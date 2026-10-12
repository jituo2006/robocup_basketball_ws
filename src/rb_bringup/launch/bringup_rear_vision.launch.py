from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def _share(pkg,*parts):
    return str(Path(get_package_share_directory(pkg)).joinpath(*parts))

def generate_launch_description():
    return LaunchDescription([
        Node(package='rb_camera', executable='camera_node', name='rb_camera_rear',
             output='screen', remappings=[('/rb_camera/ok','/rb_camera_rear/ok')],
             parameters=[{'config_file': _share('rb_camera','config','camera_rear.yaml')}]),
        Node(package='rb_perception', executable='perception_node', name='rb_perception_rear',
             output='screen', parameters=[{
                 'config_file': _share('rb_perception','config','perception_rear.yaml'),
                 'image_topic': '/camera_rear/image_raw',
                 'compressed_topic': '/camera_rear/image_raw/compressed',
                 'use_compressed': True, 'output_topic': '/perception/detections_rear',
                 'debug_image_topic': '/perception/debug_image_rear',
                 'publish_debug_image': False}]),
    ])
