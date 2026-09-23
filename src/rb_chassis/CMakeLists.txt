cmake_minimum_required(VERSION 3.8)
project(rb_chassis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motor_lib REQUIRED)
find_package(Threads REQUIRED)

# 自带补丁版 bupt_can（防"旧速度帧积压→停车后突然窜车"的补丁 + 析构判空）
# 详见 third_party/README.md。不用系统 /usr/local 的旧版。
add_library(rb_can STATIC third_party/bupt_can.cpp)
target_include_directories(rb_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>)
target_link_libraries(rb_can PUBLIC Threads::Threads)

# ---- 底盘运动学库 ----
add_library(base_chassis src/base_chassis.cpp)
target_include_directories(base_chassis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(base_chassis rclcpp geometry_msgs nav_msgs)

add_library(omni_chassis src/omni_chassis.cpp)
target_include_directories(omni_chassis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(omni_chassis base_chassis motor_lib rb_can)
ament_target_dependencies(omni_chassis rclcpp geometry_msgs nav_msgs)

# ---- 底盘节点（参数化）----
add_executable(rb_chassis_node src/rb_chassis_node.cpp)
target_link_libraries(rb_chassis_node omni_chassis rb_can)
ament_target_dependencies(rb_chassis_node rclcpp geometry_msgs nav_msgs std_msgs)

install(TARGETS base_chassis omni_chassis
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS rb_chassis_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(base_chassis omni_chassis)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs motor_lib)
ament_package()
