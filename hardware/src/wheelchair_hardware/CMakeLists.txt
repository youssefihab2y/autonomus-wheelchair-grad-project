cmake_minimum_required(VERSION 3.8)
project(wheelchair_hardware)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/esp32_bridge.py
  scripts/keyboard_drive.py
  scripts/imu_attitude_test.py
  scripts/map_bridge.py
  scripts/goal_relay.py
  scripts/yolo_safety_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
