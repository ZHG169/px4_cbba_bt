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
find_package(swarm_interfaces REQUIRED)

# BT 節點、CPF、位置廣播：不依賴 ROS（PX4、cbba_node 的介面由 UavPlatform 注入），可以不開模擬器測
add_library(${PROJECT_NAME}_core src/cpf.cpp src/flight_beacon.cpp src/bt_nodes.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC behaviortree_cpp::behaviortree_cpp cbba_core::cbba_core_core)

# 無人機的備用 BT 節點（取代 px4_waypoint_node/task_executor）
add_executable(uav_bt_node src/uav_bt_node.cpp)
target_link_libraries(uav_bt_node ${PROJECT_NAME}_core rclcpp::rclcpp ament_index_cpp::ament_index_cpp
  ${px4_msgs_TARGETS} ${swarm_interfaces_TARGETS})

install(TARGETS ${PROJECT_NAME}_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS uav_bt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_cpf test_flight_beacon test_tree)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 120)
    target_link_libraries(${test_name} ${PROJECT_NAME}_core)
    target_compile_definitions(${test_name} PRIVATE
      TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/trees/main_uav_tree.xml")
  endforeach()
endif()

ament_package()
