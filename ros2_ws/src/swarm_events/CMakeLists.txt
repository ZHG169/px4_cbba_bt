cmake_minimum_required(VERSION 3.16)
project(swarm_events LANGUAGES CXX)

# L5 事件狀態機＋L6 任務生成（2026-11-23，docs/l5_l6_plan.md）：
#   swarm_events_core：不依賴 ROS 的函式庫（event_rules.yaml、範本展開）
#   event_manager：GCS 上的節點（A 批：/gcs/expand_incident，只回傳規劃結果；B 批：事件狀態機、自動派遣）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)   # operator_notices
find_package(swarm_interfaces REQUIRED)
find_package(swarm_cbba REQUIRED)   # swarm_cbba_registry：註冊表、場域、TaskSpec 檢查
find_package(cbba_core REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(swarm_events_core src/event_rules.cpp src/expander.cpp src/incident_manager.cpp)
target_include_directories(swarm_events_core PUBLIC include)
target_link_libraries(swarm_events_core PUBLIC swarm_cbba::swarm_cbba_registry cbba_core::cbba_core_core
  yaml-cpp::yaml-cpp)

add_executable(event_manager src/event_manager_node.cpp)
target_link_libraries(event_manager swarm_events_core rclcpp::rclcpp ${std_msgs_TARGETS} ${swarm_interfaces_TARGETS})
install(TARGETS event_manager DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_expander test/test_expander.cpp)
  target_link_libraries(test_expander swarm_events_core)
  target_compile_definitions(test_expander PRIVATE
    REGISTRY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../registry"
    SITE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../site-config/yard")
  ament_add_gtest(test_incident_manager test/test_incident_manager.cpp)
  target_link_libraries(test_incident_manager swarm_events_core)
  target_compile_definitions(test_incident_manager PRIVATE
    REGISTRY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../registry"
    SITE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../site-config/yard")
endif()

ament_package()
