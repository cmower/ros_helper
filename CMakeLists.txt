cmake_minimum_required(VERSION 2.8.3)
project(ros_helper)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
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
