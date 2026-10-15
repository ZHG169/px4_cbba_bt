cmake_minimum_required(VERSION 3.16)
project(uav_px4_bt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(cbba_core REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swarm_interfaces REQUIRED)

# BT 節點、CBF、位置廣播：不依賴 ROS（PX4、cbba_node 的介面由 UavPlatform 注入），可以不開模擬器測
add_library(${PROJECT_NAME}_core src/cbf.cpp src/motion.cpp src/flight_beacon.cpp src/bt_nodes.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC behaviortree_cpp::behaviortree_cpp cbba_core::cbba_core_core)

# 無人機的備用 BT 節點（取代 px4_waypoint_node/task_executor）
add_executable(uav_bt_node src/uav_bt_node.cpp src/px4_platform.cpp)
target_link_libraries(uav_bt_node ${PROJECT_NAME}_core rclcpp::rclcpp ament_index_cpp::ament_index_cpp
  ${px4_msgs_TARGETS} ${swarm_interfaces_TARGETS})

# 正式的無人機 BT（2026-10-13）：機器狗端（伯宇）task_manager 骨架＋無人機葉節點（skeleton/），樹在 trees/uav_mission.xml
add_executable(uav_task_manager_node src/uav_task_manager_node.cpp src/px4_platform.cpp)
target_link_libraries(uav_task_manager_node ${PROJECT_NAME}_core rclcpp::rclcpp ament_index_cpp::ament_index_cpp
  ${px4_msgs_TARGETS} ${std_msgs_TARGETS} ${swarm_interfaces_TARGETS})

install(TARGETS ${PROJECT_NAME}_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS uav_bt_node uav_task_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_cbf test_motion test_flight_beacon test_tree)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 120)
    target_link_libraries(${test_name} ${PROJECT_NAME}_core)
    target_compile_definitions(${test_name} PRIVATE
      TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/trees/main_uav_tree.xml")
  endforeach()
  # 骨架（skeleton/*.hpp 的 say() 用 rclcpp 的 logger，不用 rclcpp::init）
  ament_add_gtest(test_skeleton test/test_skeleton.cpp TIMEOUT 120)
  target_link_libraries(test_skeleton ${PROJECT_NAME}_core rclcpp::rclcpp)
  target_compile_definitions(test_skeleton PRIVATE
    UAV_MISSION_FILE="${CMAKE_CURRENT_SOURCE_DIR}/trees/uav_mission.xml")
endif()

ament_package()
