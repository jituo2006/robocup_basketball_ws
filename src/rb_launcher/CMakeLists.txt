cmake_minimum_required(VERSION 3.8)
project(rb_launcher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rb_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# ---- 串口帧协议（移植自 ballrobot，未改动协议本身）----
add_library(serial_base_lib STATIC src/serial_port.cpp)
target_include_directories(serial_base_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(serial_base_lib Boost::system)

add_library(serial_R1_lib STATIC src/serial_port_R1.cpp)
target_link_libraries(serial_R1_lib serial_base_lib)

# ---- 机构控制节点 ----
add_executable(rb_launcher_node src/rb_launcher_node.cpp)
target_link_libraries(rb_launcher_node serial_R1_lib)
ament_target_dependencies(rb_launcher_node rclcpp std_msgs rb_msgs)

install(TARGETS rb_launcher_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
