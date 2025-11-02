cmake_minimum_required(VERSION 3.8)
project(motion_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(motion_control src/motion_control.cpp)
ament_target_dependencies(motion_control rclcpp nav_msgs ackermann_msgs)

install(TARGETS
  motion_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
