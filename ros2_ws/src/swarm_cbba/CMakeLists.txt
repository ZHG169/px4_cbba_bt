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
find_package(rclcpp_action REQUIRED)   # fake_adapter（ExecuteStep.action 草案，2026-11-19）
find_package(geometry_msgs REQUIRED)

# 無人機和機器狗同一支：agent_type、能量模型由 params yaml 決定
add_executable(cbba_node src/cbba_node.cpp src/registry.cpp src/spec_gates.cpp src/spec_compose.cpp src/spec_validator.cpp)
target_include_directories(cbba_node PRIVATE include)
target_link_libraries(cbba_node cbba_core::cbba_core_core rclcpp::rclcpp ${std_msgs_TARGETS}
  ${swarm_interfaces_TARGETS} yaml-cpp::yaml-cpp)
# 假的 adapter（ExecuteStep.action 草案，2026-11-19）：照 actions.yaml 檢查、模擬執行，給對 Action 欄位與 BT 的 ExecuteSteps 測流程
add_executable(fake_adapter src/fake_adapter_node.cpp src/fake_executor.cpp src/registry.cpp src/spec_validator.cpp)
target_include_directories(fake_adapter PRIVATE include)
target_link_libraries(fake_adapter cbba_core::cbba_core_core rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS} ${swarm_interfaces_TARGETS} yaml-cpp::yaml-cpp)
install(TARGETS cbba_node fake_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # 註冊表的載入與檢查：讀 repo 的 registry/（和 cbba_core/capabilities.hpp 比對）、site-config/yard（場域設定）
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registry test/test_registry.cpp src/registry.cpp)
  target_include_directories(test_registry PRIVATE include)
  target_link_libraries(test_registry cbba_core::cbba_core_core yaml-cpp::yaml-cpp)
  target_compile_definitions(test_registry PRIVATE
    REGISTRY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../registry"
    SITE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../site-config/yard")
  # spec 的 wait_for → step-level gate（CB-4 第一批，2026-11-16）
  ament_add_gtest(test_spec_gates test/test_spec_gates.cpp src/spec_gates.cpp)
  target_include_directories(test_spec_gates PRIVATE include)
  target_link_libraries(test_spec_gates cbba_core::cbba_core_core yaml-cpp::yaml-cpp)
  # 角色 TaskSpec 的組合：識別、語意、空間寫進 spec 最上層（任務欄位補齊第一批，2026-11-17）
  ament_add_gtest(test_spec_compose test/test_spec_compose.cpp src/spec_compose.cpp)
  target_include_directories(test_spec_compose PRIVATE include)
  target_link_libraries(test_spec_compose cbba_core::cbba_core_core yaml-cpp::yaml-cpp)
  # TaskSpec 的檢查（任務欄位補齊 10，2026-11-18）：讀 repo 的 registry/、site-config/yard
  ament_add_gtest(test_spec_validator test/test_spec_validator.cpp src/spec_validator.cpp src/registry.cpp)
  target_include_directories(test_spec_validator PRIVATE include)
  target_link_libraries(test_spec_validator cbba_core::cbba_core_core yaml-cpp::yaml-cpp)
  target_compile_definitions(test_spec_validator PRIVATE
    REGISTRY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../registry"
    SITE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../site-config/yard")
  # 假 adapter 的模擬（ExecuteStep.action 草案，2026-11-19）：讀 repo 的 registry/、site-config/yard
  ament_add_gtest(test_fake_executor test/test_fake_executor.cpp src/fake_executor.cpp src/spec_validator.cpp src/registry.cpp)
  target_include_directories(test_fake_executor PRIVATE include)
  target_link_libraries(test_fake_executor cbba_core::cbba_core_core yaml-cpp::yaml-cpp)
  target_compile_definitions(test_fake_executor PRIVATE
    REGISTRY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../registry"
    SITE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../site-config/yard")
endif()

ament_package()
