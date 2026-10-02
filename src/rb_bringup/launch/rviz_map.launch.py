"""实时点云地图 + 点击导航（RViz）。

干什么
------
把 FAST-LIO 的建图结果和机器人位姿画在 RViz 里，然后在图上**直接点一下**，
机器人就走到那个点 —— 不用再开别的窗口。

界面里有什么
------------
* 场地网格（1m 一格，方便估距离）
* `/Laser_map`      雷达累积地图（白）
* `/cloud_registered` 当前扫描（绿）
* `/localization/pose` 机器人位姿（红箭头 + 协方差椭圆）
* 固定坐标系 `field`（场地坐标系）

怎么用
------
    # 前提：雷达驱动 + FAST-LIO + rb_localization 都在跑
    ros2 launch rb_bringup rviz_map.launch.py

    # 然后在 RViz 工具栏选【2D Goal Pose】→ 在图上按住拖动出朝向 → 松手
    #   目标会发到 /goal_pose，rb_mission 收到就切到 GOTO 阶段开过去
    #
    # ⚠️ 只在 mission == IDLE 时接受（避免和自主任务抢 /cmd_vel）。
    #    正在跑任务时先切回 IDLE：
    #      ros2 service call /rb_mission/set_mission rb_msgs/srv/SetMission "{mission: IDLE}"

命令行代替点击（效果一样）：
    ros2 service call /rb_mission/goto_pose rb_msgs/srv/GotoPose \
        "{x: 5.0, y: 3.0, yaw: 0.0, align_yaw: false}"
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = str(Path(get_package_share_directory("rb_bringup")) / "rviz" / "map_goto.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=default_cfg,
                              description="RViz 配置路径"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_map",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
