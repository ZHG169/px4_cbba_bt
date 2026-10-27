cmake_minimum_required(VERSION 3.16)
project(swarm_zenoh NONE)

find_package(ament_cmake REQUIRED)

# 沒有要編譯的程式：設定產生器（Python）、白名單、說明。bridge 本身是 Eclipse 的 zenoh-bridge-ros2dds 執行檔（另外安裝）
install(PROGRAMS scripts/zenoh_bridge.py DESTINATION lib/${PROJECT_NAME} RENAME zenoh_bridge)
install(PROGRAMS scripts/selftest.py DESTINATION lib/${PROJECT_NAME} RENAME selftest)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_zenoh_bridge test/test_zenoh_bridge.py)
endif()

ament_package()
