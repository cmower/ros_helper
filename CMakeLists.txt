cmake_minimum_required(VERSION 2.8.3)
project(ros_helper)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  visualization_msgs
  )

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
