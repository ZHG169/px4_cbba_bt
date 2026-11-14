cmake_minimum_required(VERSION 3.16)
project(swarm_cbba LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)   # cbba_path（UInt32MultiArray）
find_package(swarm_interfaces REQUIRED)
find_package(cbba_core REQUIRED)   # CBBA 核心、機間協定版本 2、正式指派流程
find_package(yaml-cpp REQUIRED)    # 2-1b 註冊表（registry/*.yaml，2026-11-14）

# 無人機和機器狗同一支：agent_type、能量模型由 params yaml 決定
add_executable(cbba_node src/cbba_node.cpp src/registry.cpp)
target_include_directories(cbba_node PRIVATE include)
target_link_libraries(cbba_node cbba_core::cbba_core_core rclcpp::rclcpp ${std_msgs_TARGETS}
  ${swarm_interfaces_TARGETS} yaml-cpp::yaml-cpp)
install(TARGETS cbba_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # 註冊表的載入與檢查：讀 repo 的 registry/（和 cbba_core/capabilities.hpp 比對）
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registry test/test_registry.cpp src/registry.cpp)
  target_include_directories(test_registry PRIVATE include)
  target_link_libraries(test_registry cbba_core::cbba_core_core yaml-cpp::yaml-cpp)
  target_compile_definitions(test_registry PRIVATE
    REGISTRY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../registry")
endif()

ament_package()
