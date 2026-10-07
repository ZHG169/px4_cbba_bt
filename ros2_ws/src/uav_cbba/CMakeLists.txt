cmake_minimum_required(VERSION 3.16)
project(uav_cbba LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 核心函式庫：不依賴 ROS，可以單獨編譯與測試
add_library(${PROJECT_NAME}_core
  src/energy_model.cpp
  src/scoring.cpp
  src/cbba_agent.cpp
  src/wire_header.cpp
  src/wire_agent_state.cpp
  src/wire_task_event.cpp
  src/wire_completion.cpp
  src/cbba_comm.cpp
  src/udp_link.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 離線模擬（測試與 cbba_sim 工具共用）
add_library(${PROJECT_NAME}_sim src/network_sim.cpp)
target_link_libraries(${PROJECT_NAME}_sim PUBLIC ${PROJECT_NAME}_core)

add_executable(cbba_sim tools/cbba_sim.cpp)
target_link_libraries(cbba_sim ${PROJECT_NAME}_sim)

set(UAV_CBBA_TESTS
  test_energy_model
  test_scoring
  test_rules
  test_agent
  test_lossy_network
  test_wire_header
  test_wire_agent_state
  test_wire_task_event
  test_wire_completion
  test_comm
)

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  # ---------- ROS 2 (colcon) ----------
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_sim
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  # ros2 run uav_cbba cbba_sim / plot_results.py
  install(TARGETS cbba_sim DESTINATION lib/${PROJECT_NAME})

  # 機上的 CBBA 節點：機間走 UDP，機內接 PX4 與 BT
  find_package(rclcpp REQUIRED)
  find_package(px4_msgs REQUIRED)
  find_package(swarm_interfaces REQUIRED)
  add_executable(cbba_node src/cbba_node.cpp)
  target_link_libraries(cbba_node ${PROJECT_NAME}_core rclcpp::rclcpp
    ${px4_msgs_TARGETS} ${swarm_interfaces_TARGETS})
  install(TARGETS cbba_node DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS scripts/plot_results.py DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY scenarios DESTINATION share/${PROJECT_NAME})
  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest QUIET)
    if(ament_cmake_gtest_FOUND)
      foreach(test_name ${UAV_CBBA_TESTS})
        ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 300)
        target_link_libraries(${test_name} ${PROJECT_NAME}_sim)
        target_compile_definitions(${test_name} PRIVATE
          RULE_CASES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/cbba_rule_cases.csv")
      endforeach()
    else()
      message(WARNING "找不到 ament_cmake_gtest，略過測試")
    endif()
  endif()

  ament_package()
else()
  # ---------- 沒有 ROS 的環境：直接用 CMake ----------
  include(CTest)
  if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
      include(FetchContent)
      FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
      )
      set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
      FetchContent_MakeAvailable(googletest)
    endif()
    foreach(test_name ${UAV_CBBA_TESTS})
      add_executable(${test_name} test/${test_name}.cpp)
      target_link_libraries(${test_name} ${PROJECT_NAME}_sim GTest::gtest_main)
      target_compile_definitions(${test_name} PRIVATE
        RULE_CASES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/cbba_rule_cases.csv")
      add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
  endif()
endif()
