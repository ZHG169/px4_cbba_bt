cmake_minimum_required(VERSION 3.16)
project(swarm_cbba LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_interfaces REQUIRED)
find_package(cbba_core REQUIRED)   # CBBA 核心、機間協定版本 2、正式指派流程

# 無人機和機器狗同一支：agent_type、能量模型由 params yaml 決定
add_executable(cbba_node src/cbba_node.cpp)
target_link_libraries(cbba_node cbba_core::cbba_core_core rclcpp::rclcpp ${swarm_interfaces_TARGETS})
install(TARGETS cbba_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
