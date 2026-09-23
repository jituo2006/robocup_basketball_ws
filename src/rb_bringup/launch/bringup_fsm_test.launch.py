"""只验证【决策 + 运动 + 定位】闭环，不依赖相机与视觉。

链路：
    mission → /cmd_vel → fake_board（积分成 /odom）→ localization（EKF）
        ↑                                                    │
        └────────── fake_detections（按位姿算检测）←──────────┘

用它可以完整走完 SEEK → ACQUIRE → NAV → ALIGN → LAUNCH → RETURN → DONE，
而不用等相机和球。视觉本身用 tools/make_test_image.py 单独验证。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def _share(pkg, *parts):
    return str(Path(get_package_share_directory(pkg)).joinpath(*parts))


def generate_launch_description():
    ws = "/home/user/robocup_basketball_ws"
    return LaunchDescription([
        ExecuteProcess(cmd=["python3", f"{ws}/tools/fake_board.py"], output="screen"),
        ExecuteProcess(cmd=["python3", f"{ws}/tools/fake_detections.py", "--rate", "10.0"],
                       output="screen"),
        # 假底盘发布的是 /odom（不是雷达的 /Odometry），所以这里覆盖输入话题。
        Node(package="rb_localization", executable="localization_node", name="rb_localization",
             output="screen",
             parameters=[{"config_file": _share("rb_localization", "config", "localization.yaml"),
                          "odom_topic": "/odom",
                          "abs_pose_topic": ""}]),
        Node(package="rb_mission", executable="mission_node", name="rb_mission",
             output="screen",
             parameters=[{"config_file": _share("rb_mission", "config", "mission.yaml")}]),
    ])
