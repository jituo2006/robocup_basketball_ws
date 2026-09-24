"""雷达 + 视觉定位的整车启动。

启动顺序（有时间依赖，故用 TimerAction 串起来）：
    ① Livox MID360 驱动        (ws_livox)
    ② FAST-LIO 建图/里程计      (fast_prop_ws)  → /Odometry
    ③ 视觉 / 底盘 / 机构 / 定位 / 任务
    ④ 可选：RViz 看建图效果

用法：
  # 先确认雷达网络（见 docs/05）
  ros2 launch rb_bringup bringup_lidar_vision.launch.py
  ros2 launch rb_bringup bringup_lidar_vision.launch.py rviz:=true
  ros2 launch rb_bringup bringup_lidar_vision.launch.py dry_run:=true   # 不含 CAN 输出

⚠️ 启动前必须做的两件事（详见 docs/05）：
  1. 把定位柱实际坐标填进 rb_localization/config/localization.yaml 的 landmarks
  2. 把开机摆车位置填进同文件的 odom_frame_conversion.start_pose
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIDAR_WS = "/home/user/lidar_360"


def _share(pkg, *parts):
    return str(Path(get_package_share_directory(pkg)).joinpath(*parts))


def _bash(script: str) -> list:
    return ["bash", "-c", script]


def generate_launch_description():
    dry_run = LaunchConfiguration("dry_run")
    image_topic = LaunchConfiguration("image_topic")
    use_compressed = LaunchConfiguration("use_compressed")
    camera_device = LaunchConfiguration("camera_device")
    rviz = LaunchConfiguration("rviz")
    domain = LaunchConfiguration("ros_domain_id")

    setup = f"export ROS_DOMAIN_ID={domain} && "

    # ① 雷达驱动
    livox = ExecuteProcess(
        cmd=_bash(
            "source /opt/ros/humble/setup.bash && "
            f"source {LIDAR_WS}/ws_livox/install/setup.bash && "
            f"{setup}"
            "exec ros2 launch livox_ros_driver2 msg_MID360_launch.py"
        ),
        output="screen", name="livox_driver",
    )

    # ② FAST-LIO（实车用的是带 IMU 递推的 fast_prop_ws）
    fastlio = ExecuteProcess(
        cmd=_bash(
            "source /opt/ros/humble/setup.bash && "
            f"source {LIDAR_WS}/ws_livox/install/setup.bash && "
            f"source {LIDAR_WS}/fast_prop_ws/install/setup.bash && "
            f"export ROS_DOMAIN_ID={domain} && "
            "exec ros2 launch fast_lio mapping.launch.py "
            f"config_path:={LIDAR_WS}/fast_prop_ws/src/FAST_LIO_WITH_PROPAGATE/config "
            "config_file:=mid360.yaml rviz:=false"
        ),
        output="screen", name="fastlio",
    )

    # ④ 可选 RViz
    rviz_proc = ExecuteProcess(
        cmd=_bash(
            "source /opt/ros/humble/setup.bash && "
            f"source {LIDAR_WS}/fast_prop_ws/install/setup.bash && "
            f"{setup}"
            f"exec rviz2 -d {LIDAR_WS}/fast_prop_ws/install/fast_lio/share/fast_lio/rviz/fastlio.rviz"
        ),
        output="screen", condition=IfCondition(rviz), name="rviz",
    )

    # ③ 其余节点
    rest = [
        Node(package="rb_camera", executable="camera_node", name="rb_camera",
             output="screen",
             parameters=[{"config_file": _share("rb_camera", "config", "camera.yaml"),
                          "device": camera_device}]),
        Node(package="rb_perception", executable="perception_node", name="rb_perception",
             output="screen",
             parameters=[{"config_file": _share("rb_perception", "config", "perception.yaml"),
                          "image_topic": image_topic,
                          "use_compressed": use_compressed,
                          "publish_debug_image": True}]),
        Node(package="rb_localization", executable="localization_node", name="rb_localization",
             output="screen",
             parameters=[{"config_file": _share("rb_localization", "config", "localization.yaml")}]),
        Node(package="rb_chassis", executable="rb_chassis_node", name="rb_chassis",
             output="screen",
             parameters=[_share("rb_chassis", "config", "chassis.yaml"),
                         {"dry_run": dry_run}]),
        Node(package="rb_launcher", executable="rb_launcher_node", name="rb_launcher",
             output="screen",
             parameters=[_share("rb_launcher", "config", "launcher.yaml")]),
        Node(package="rb_mission", executable="mission_node", name="rb_mission",
             output="screen",
             parameters=[{"config_file": _share("rb_mission", "config", "mission.yaml")}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true = 底盘只解算不下发 CAN"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("use_compressed", default_value="true",
                              description="原始大图消息投递只有 ~7fps，默认走 JPEG 压缩图"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("ros_domain_id", default_value="2"),

        livox,
        TimerAction(period=4.0, actions=[fastlio]),
        TimerAction(period=12.0, actions=rest),
        TimerAction(period=14.0, actions=[rviz_proc]),
    ])
