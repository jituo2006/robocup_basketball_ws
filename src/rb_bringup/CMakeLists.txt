cmake_minimum_required(VERSION 3.8)
project(rb_bringup)
find_package(ament_cmake REQUIRED)
# rviz/ 里是"实时点云地图 + 点击导航"的配置（map_goto.rviz），必须一起装，
# 否则 rviz_map.launch.py 找不到文件。
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
ament_package()
