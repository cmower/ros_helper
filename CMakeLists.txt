cmake_minimum_required(VERSION 2.8.3)
project(ros_helper)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  rospy
  visualization_msgs
  message_generation
  )

add_message_files(
  FILES
  MultiJoy.msg
  Keyboard.msg
  )

catkin_python_setup()

generate_messages( 
  DEPENDENCIES
  std_msgs
  sensor_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )
