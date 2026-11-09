cmake_minimum_required(VERSION 3.8)
project(painting_trajectory)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(painting_path src/painting_path.cpp)

ament_target_dependencies(painting_path
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

install(TARGETS painting_path
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
