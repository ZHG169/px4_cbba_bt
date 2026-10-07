cmake_minimum_required(VERSION 3.16)
project(ugv_cbba LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_interfaces REQUIRED)
find_package(uav_cbba REQUIRED)   # CBBA 核心、機間協定、節點骨架（和無人機共用同一份）

# 機器狗：接 /v60/robot_state、/v60/task_result
add_executable(ugv_cbba_node src/ugv_cbba_node.cpp)
target_link_libraries(ugv_cbba_node uav_cbba::uav_cbba_node_base ${swarm_interfaces_TARGETS})
install(TARGETS ugv_cbba_node DESTINATION lib/${PROJECT_NAME})

ament_package()
