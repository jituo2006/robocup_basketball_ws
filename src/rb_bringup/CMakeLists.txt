cmake_minimum_required(VERSION 3.8)
project(rb_bringup)
find_package(ament_cmake REQUIRED)
# rviz/ 里是"实时点云地图 + 点击导航"的配置（map_goto.rviz），必须一起装，
# 否则 rviz_map.launch.py 找不到文件。
# config/ 不在 Git 中，干净检出时不存在；只安装实际受版本控制的目录。
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
