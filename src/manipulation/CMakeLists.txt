cmake_minimum_required(VERSION 3.8)
project(manipulation)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(workflow_interfaces REQUIRED)

add_executable(manipulation_node
  src/manipulation_node.cpp
)

target_include_directories(manipulation_node PUBLIC
  include
)

ament_target_dependencies(manipulation_node
  control_msgs
  gazebo_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
  std_msgs
  trajectory_msgs
  workflow_interfaces
)

install(TARGETS manipulation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
