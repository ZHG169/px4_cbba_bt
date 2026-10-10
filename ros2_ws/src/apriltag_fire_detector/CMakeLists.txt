cmake_minimum_required(VERSION 3.16)
project(apriltag_fire_detector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(apriltag REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swarm_interfaces REQUIRED)

# 偵測、座標轉換、confidence：不依賴 ROS
add_library(${PROJECT_NAME}_core src/fire_detector.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC apriltag::apriltag)

add_executable(fire_detector_node src/fire_detector_node.cpp)
target_link_libraries(fire_detector_node ${PROJECT_NAME}_core rclcpp::rclcpp
  ${px4_msgs_TARGETS} ${sensor_msgs_TARGETS} ${swarm_interfaces_TARGETS})

install(TARGETS ${PROJECT_NAME}_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS fire_detector_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fire_detector test/test_fire_detector.cpp TIMEOUT 60)
  target_link_libraries(test_fire_detector ${PROJECT_NAME}_core)
endif()

ament_package()
