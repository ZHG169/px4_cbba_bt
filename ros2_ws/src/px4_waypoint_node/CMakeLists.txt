cmake_minimum_required(VERSION 3.16)
project(px4_waypoint_node LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# 起飛並懸停（第 2 週再加航點）
add_executable(takeoff_hover src/takeoff_hover.cpp)
target_link_libraries(takeoff_hover rclcpp::rclcpp ${px4_msgs_TARGETS})

install(TARGETS takeoff_hover DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/hover_report.py DESTINATION lib/${PROJECT_NAME})

ament_package()
